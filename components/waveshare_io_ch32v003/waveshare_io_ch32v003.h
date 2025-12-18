#pragma once

#include "esphome/components/gpio_expander/cached_gpio.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace waveshare_io_ch32v003 {

/**
 * @brief Main component class for the Waveshare IO CH32V003 I2C Expander.
 *
 * This class handles the I2C communication, state caching, and logic
 * for the 8-bit GPIO expander. It inherits from CachedGpioExpander
 * to optimize bus traffic by caching read/write states.
 * * Capabilities:
 * - 8 GPIO pins (Input/Output)
 * - ADC Reading (10-bit)
 * - PWM Output
 * - Interrupt Status
 */
class WaveshareIOCH32V003Component : public Component,
                                     public i2c::I2CDevice,
                                     public gpio_expander::CachedGpioExpander<uint8_t, 8> {
 public:
  WaveshareIOCH32V003Component() = default;

  // --- ESPHome Lifecycle Methods ---
  void setup() override;
  void pin_mode(uint8_t pin, gpio::Flags flags);
  void loop() override;

  float get_setup_priority() const override;
  void dump_config() override;

  // --- Chip Specific Features ---
  
  /**
   * @brief Reads the ADC value from the board.
   * @return 10-bit ADC value (0-1023).
   */
  uint16_t get_adc_value();

  /**
   * @brief Reads the interrupt status register.
   * @return The status byte indicating interrupt source.
   */
  uint8_t get_interrupt_status();

  /**
   * @brief Sets the global PWM duty cycle.
   * @param value The PWM duty cycle (0-255).
   */
  void set_pwm_value(uint8_t value);

 protected:
  friend class WaveshareIOCH32V003GPIOPin;

  bool is_ready_{false};
  
  // --- Hardware Implementation (CachedGpioExpander overrides) ---
  
  /**
   * @brief Reads the physical state of a pin from the hardware.
   */
  bool digital_read_hw(uint8_t pin) override;

  /**
   * @brief Reads the state of a pin from the internal cache.
   */
  bool digital_read_cache(uint8_t pin) override;

  /**
   * @brief Writes a digital value to the physical hardware.
   */
  void digital_write_hw(uint8_t pin, bool value) override;

  float get_loop_priority() const override;

  // --- I2C Robustness Helpers ---
  
  /**
   * @brief Writes a register with retry logic to handle transient bus errors.
   */
  bool write_register_with_retry_(uint8_t a_register, uint8_t value);

  /**
   * @brief Reads registers with retry logic to handle transient bus errors.
   */
  bool read_registers_with_retry_(uint8_t a_register, uint8_t *data, uint8_t len);

  // --- Local State Masks (Source of Truth) ---
  // These shadow registers prevent unnecessary I2C reads/writes
  uint8_t mode_mask_{0x00};    // 0 = Input, 1 = Output (Default: Input for safety)
  uint8_t output_mask_{0x00};  // 0 = Low, 1 = High
  uint8_t input_mask_{0x00};   // Read cache

  // Helpers to commit masks to hardware
  bool write_gpio_modes_();
  bool write_gpio_outputs_();
};

/**
 * @brief Represents a single GPIO pin on the expansion board.
 *
 * This class proxies calls to the parent WaveshareIOCH32V003Component.
 */
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
  // Initialized to 255 to detect unconfigured state
  uint8_t pin_{255};
  bool inverted_{false};
  gpio::Flags flags_{gpio::FLAG_INPUT};
};

}  // namespace waveshare_io_ch32v003
}  // namespace esphome