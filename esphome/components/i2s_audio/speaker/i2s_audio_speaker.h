#pragma once

#ifdef USE_ESP32

#include "../i2s_audio.h"

#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "esphome/components/speaker/speaker.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"
#include "esphome/core/ring_buffer.h"

namespace esphome {
namespace i2s_audio {

class I2SAudioSpeaker : public I2SAudioOut, public speaker::Speaker, public Component {
 public:
  float get_setup_priority() const override { return esphome::setup_priority::LATE; }

  void setup() override;
  void loop() override;

  void set_timeout(uint32_t ms) { this->timeout_ = ms; }
  void set_dout_pin(uint8_t pin) { this->dout_pin_ = pin; }
#if SOC_I2S_SUPPORTS_DAC
  void set_internal_dac_mode(i2s_dac_mode_t mode) { this->internal_dac_mode_ = mode; }
#endif

  void start() override;
  void stop() override;
  void finish() override;

  size_t play(const uint8_t *data, size_t length) override;

  bool has_buffered_data() const override;

 protected:
  void start_();
  void stop_(bool wait_on_empty);
  void watch_();

  static void player_task(void *params);

  TaskHandle_t player_task_handle_{nullptr};
  QueueHandle_t event_queue_;
  std::unique_ptr<RingBuffer> buffer_;

  uint32_t timeout_{0};
  uint8_t dout_pin_{0};
  bool task_created_{false};

#if SOC_I2S_SUPPORTS_DAC
  i2s_dac_mode_t internal_dac_mode_{I2S_DAC_CHANNEL_DISABLE};
#endif
};

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
