#include "i2s_audio_speaker.h"

#ifdef USE_ESP32

#include <driver/i2s.h>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace i2s_audio {

static const size_t BUFFER_COUNT = 20;
static const size_t BUFFER_SIZE = 1024;

enum class TaskEventType : uint8_t {
  STARTING = 0,
  STARTED,
  STOPPING,
  STOPPED,
  WARNING = 255,
};

struct TaskEvent {
  TaskEventType type;
  esp_err_t err;
};

static const char *const TAG = "i2s_audio.speaker";

void I2SAudioSpeaker::setup() {
  ESP_LOGCONFIG(TAG, "Setting up I2S Audio Speaker...");

  this->buffer_ = RingBuffer::create(BUFFER_COUNT * BUFFER_SIZE);
  if (this->buffer_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create buffer queue");
    this->mark_failed();
    return;
  }

  this->event_queue_ = xQueueCreate(BUFFER_COUNT, sizeof(TaskEvent));
  if (this->event_queue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create event queue");
    this->mark_failed();
    return;
  }
}

void I2SAudioSpeaker::start() {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Cannot start audio, speaker failed to setup");
    return;
  }
  if (this->task_created_) {
    ESP_LOGW(TAG, "Called start while task has been already created.");
    return;
  }
  this->state_ = speaker::STATE_STARTING;
  this->buffer_.reset(); // always succeeds since there's no reader task
}
void I2SAudioSpeaker::start_() {
  if (this->task_created_) {
    return;
  }
  if (!this->parent_->try_lock()) {
    return;  // Waiting for another i2s component to return lock
  }

  xTaskCreate(I2SAudioSpeaker::player_task, "speaker_task", 8192, (void *) this, 1, &this->player_task_handle_);
  this->task_created_ = true;
}

void I2SAudioSpeaker::player_task(void *params) {
  I2SAudioSpeaker *this_speaker = (I2SAudioSpeaker *) params;

  TaskEvent event;
  event.type = TaskEventType::STARTING;
  xQueueSend(this_speaker->event_queue_, &event, portMAX_DELAY);

  i2s_driver_config_t config = {
      .mode = (i2s_mode_t) (this_speaker->i2s_mode_ | I2S_MODE_TX),
      .sample_rate = this_speaker->sample_rate_,
      .bits_per_sample = this_speaker->bits_per_sample_,
      .channel_format = this_speaker->channel_,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 4,
      .dma_buf_len = 1024,
      .use_apll = this_speaker->use_apll_,
      .tx_desc_auto_clear = true,
      .fixed_mclk = 0,
      .mclk_multiple = I2S_MCLK_MULTIPLE_256,
      .bits_per_chan = this_speaker->bits_per_channel_,
  };
#if SOC_I2S_SUPPORTS_DAC
  if (this_speaker->internal_dac_mode_ != I2S_DAC_CHANNEL_DISABLE) {
    config.mode = (i2s_mode_t) (config.mode | I2S_MODE_DAC_BUILT_IN);
  }
#endif

  esp_err_t err = i2s_driver_install(this_speaker->parent_->get_port(), &config, 0, nullptr);
  if (err != ESP_OK) {
    event.type = TaskEventType::WARNING;
    event.err = err;
    xQueueSend(this_speaker->event_queue_, &event, 0);
    event.type = TaskEventType::STOPPED;
    xQueueSend(this_speaker->event_queue_, &event, 0);
    while (true) {
      delay(10);
    }
  }

#if SOC_I2S_SUPPORTS_DAC
  if (this_speaker->internal_dac_mode_ == I2S_DAC_CHANNEL_DISABLE) {
#endif
    i2s_pin_config_t pin_config = this_speaker->parent_->get_pin_config();
    pin_config.data_out_num = this_speaker->dout_pin_;

    i2s_set_pin(this_speaker->parent_->get_port(), &pin_config);
#if SOC_I2S_SUPPORTS_DAC
  } else {
    i2s_set_dac_mode(this_speaker->internal_dac_mode_);
  }
#endif

  event.type = TaskEventType::STARTED;
  xQueueSend(this_speaker->event_queue_, &event, portMAX_DELAY);

  int32_t buffer[BUFFER_SIZE];
  uint8_t *buffer_start = reinterpret_cast<uint8_t *>(buffer);
  size_t buffer_size = sizeof(buffer);
  bool mono = this_speaker->channel_ == I2S_CHANNEL_FMT_ALL_LEFT;
  bool to32 = this_speaker->bits_per_sample_ > I2S_BITS_PER_SAMPLE_16BIT;
  for (int i = 0; i < mono + to32; i++) {
    buffer_size /= 2;
    buffer_start += buffer_size;
  }

  while (this_speaker->state_ != speaker::STATE_STOPPING) {
    size_t remaining = this_speaker->buffer_->read(buffer_start, buffer_size, this_speaker->timeout_ / portTICK_PERIOD_MS);
    if (remaining == 0 || remaining % 2 == 1) {
      break;  // Either received nothing, or reached an immediate stop signal.
    }

    const uint8_t *data = buffer_start;
    if (to32) {
      const int16_t *in = reinterpret_cast<const int16_t *>(data);
      int32_t *out = buffer;
      if (mono) {
        for (size_t i = 0; i < remaining; i += sizeof(int16_t)) {
          int32_t sample = static_cast<int32_t>(*in++) << 16;
          *out++ = sample;
          *out++ = sample;
        }
        data = reinterpret_cast<const uint8_t *>(buffer);
        remaining *= 4;
      } else {
        for (size_t i = 0; i < remaining; i += sizeof(int16_t)) {
          *out++ = static_cast<int32_t>(*in++) << 16;
        }
        data = reinterpret_cast<const uint8_t *>(buffer);
        remaining *= 2;
      }
    } else if (mono) {
      const int16_t *in = reinterpret_cast<const int16_t *>(data);
      int16_t *out = reinterpret_cast<int16_t *>(buffer);
      for (size_t i = 0; i < remaining; i += sizeof(int16_t)) {
        int16_t sample = *in++;
        *out++ = sample;
        *out++ = sample;
      }
      data = reinterpret_cast<const uint8_t *>(buffer);
      remaining *= 2;
    }

    while (remaining != 0) {
      size_t bytes_written;
      esp_err_t err =
          i2s_write(this_speaker->parent_->get_port(), data, remaining, &bytes_written, (32 / portTICK_PERIOD_MS));
      if (err != ESP_OK) {
        event = {.type = TaskEventType::WARNING, .err = err};
        if (xQueueSend(this_speaker->event_queue_, &event, 10 / portTICK_PERIOD_MS) != pdTRUE) {
          ESP_LOGW(TAG, "Failed to send WARNING event");
        }
        continue;
      }
      data += bytes_written;
      remaining -= bytes_written;
    }
  }

  event.type = TaskEventType::STOPPING;
  if (xQueueSend(this_speaker->event_queue_, &event, 10 / portTICK_PERIOD_MS) != pdTRUE) {
    ESP_LOGW(TAG, "Failed to send STOPPING event");
  }

  i2s_zero_dma_buffer(this_speaker->parent_->get_port());

  i2s_driver_uninstall(this_speaker->parent_->get_port());

  event.type = TaskEventType::STOPPED;
  if (xQueueSend(this_speaker->event_queue_, &event, 10 / portTICK_PERIOD_MS) != pdTRUE) {
    ESP_LOGW(TAG, "Failed to send STOPPED event");
  }

  while (true) {
    delay(10);
  }
}

void I2SAudioSpeaker::stop() { this->stop_(false); }

void I2SAudioSpeaker::finish() { this->stop_(true); }

void I2SAudioSpeaker::stop_(bool wait_on_empty) {
  if (this->is_failed())
    return;
  if (this->state_ == speaker::STATE_STOPPED)
    return;
  if (this->state_ == speaker::STATE_STARTING) {
    this->state_ = speaker::STATE_STOPPED;
    return;
  }
  // The reader task does this:
  //   check state -> check buffer size size -> maybe fall asleep -> handle data
  //                  \--------- atomic w.r.t. write() ---------/
  // If it is currently handling data, this will make the next state check stop:
  this->state_ = speaker::STATE_STOPPING;
  // If it has already checked the state but has not entered the atomic section
  // yet, this will ensure it does not fall asleep:
  this->buffer_->write_without_replacement("", 1);
  // And if it has already fallen asleep, this will wake it up:
  xTaskNotifyGive(this->player_task_handle_);
}

void I2SAudioSpeaker::watch_() {
  TaskEvent event;
  if (xQueueReceive(this->event_queue_, &event, 0) == pdTRUE) {
    switch (event.type) {
      case TaskEventType::STARTING:
        ESP_LOGD(TAG, "Starting I2S Audio Speaker");
        break;
      case TaskEventType::STARTED:
        ESP_LOGD(TAG, "Started I2S Audio Speaker");
        this->state_ = speaker::STATE_RUNNING;
        this->status_clear_warning();
        break;
      case TaskEventType::STOPPING:
        ESP_LOGD(TAG, "Stopping I2S Audio Speaker");
        break;
      case TaskEventType::STOPPED:
        this->state_ = speaker::STATE_STOPPED;
        vTaskDelete(this->player_task_handle_);
        this->task_created_ = false;
        this->player_task_handle_ = nullptr;
        this->parent_->unlock();
        this->buffer_->reset();
        ESP_LOGD(TAG, "Stopped I2S Audio Speaker");
        break;
      case TaskEventType::WARNING:
        ESP_LOGW(TAG, "Error writing to I2S: %s", esp_err_to_name(event.err));
        this->status_set_warning();
        break;
    }
  }
}

void I2SAudioSpeaker::loop() {
  switch (this->state_) {
    case speaker::STATE_STARTING:
      this->start_();
      [[fallthrough]];
    case speaker::STATE_RUNNING:
    case speaker::STATE_STOPPING:
      this->watch_();
      break;
    case speaker::STATE_STOPPED:
      break;
  }
}

size_t I2SAudioSpeaker::play(const uint8_t *data, size_t length) {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Cannot play audio, speaker failed to setup");
    return 0;
  }
  if (this->state_ == speaker::STATE_STOPPING) {
    return 0;
  }
  if (this->state_ == speaker::STATE_STOPPED) {
    this->start();
  }
  // Samples should be 2- or 4-byte long; round down to a sample boundary.
  return this->buffer_->write(data, length & ~static_cast<size_t>(1));
}

bool I2SAudioSpeaker::has_buffered_data() const { return this->buffer_->available() > 0; }

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
