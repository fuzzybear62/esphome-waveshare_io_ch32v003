#pragma once

#include "esphome/components/gpio_expander/cached_gpio.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace waveshare_io_ch32v003 {

class WaveshareIOCH32V003Component : public Component,
                                     public i2c::I2CDevice,
                                     public gpio_expander::CachedGpioExpander<uint8_t, 8> {
 public:
  // Explicit constructor for safe initialization
  WaveshareIOCH32V003Component();

  // --- ESPHome Lifecycle Methods ---
  void setup() override;
  void pin_mode(uint8_t pin, gpio::Flags flags);
  void loop() override;

  float get_setup_priority() const override;
  void dump_config() override;

  // --- Chip Specific Features ---
  uint16_t get_adc_value();
  uint8_t get_interrupt_status();
  void set_pwm_value(uint8_t value);

 protected:
  friend class WaveshareIOCH32V003GPIOPin;

  bool is_ready_; // Initialized in constructor
  
  // --- Hardware Implementation ---
  bool digital_read_hw(uint8_t pin) override;
  bool digital_read_cache(uint8_t pin) override;
  void digital_write_hw(uint8_t pin, bool value) override;

  float get_loop_priority() const override;

  // --- I2C Robustness Helpers ---
  bool write_register_with_retry_(uint8_t a_register, uint8_t value);
  bool read_registers_with_retry_(uint8_t a_register, uint8_t *data, uint8_t len);

  // --- Local State Masks ---
  uint8_t mode_mask_{0x00};    
  uint8_t output_mask_{0x00};  
  uint8_t input_mask_{0x00};   

  bool write_gpio_modes_();
  bool write_gpio_outputs_();
};

class WaveshareIOCH32V003GPIOPin : public GPIOPin, public Parented<WaveshareIOCH32V003Component> {
 public:
  void setup() override;
  void pin_mode(gpio::Flags flags) override;
  bool digital_read() override;
  void digital_write(bool value) override;
  std::string dump_summary() const override;

  void set_pin(uint8_t pin) { pin_ = pin; }
  void set_inverted(bool inverted) { inverted_ = inverted; }
  void set_flags(gpio::Flags flags);

  gpio::Flags get_flags() const override { return this->flags_; }

 protected:
  uint8_t pin_{255};
  bool inverted_{false};
  gpio::Flags flags_{gpio::FLAG_INPUT};
};

}  // namespace waveshare_io_ch32v003
}  // namespace esphome