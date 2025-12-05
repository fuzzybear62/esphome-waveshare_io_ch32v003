#pragma once

#include "esphome/components/output/float_output.h"
#include "esphome/core/component.h" // Ensure this is included
#include "waveshare_io_ch32v003.h"

namespace esphome {
namespace waveshare_io_ch32v003 {

/**
 * @brief PWM Output component for the Waveshare IO CH32V003 expansion board.
 *
 * This class implements the FloatOutput interface to provide percentage-based
 * control (0% - 100%) over the hardware PWM pins. It communicates with the
 * hardware via the parent WaveshareIOCH32V003Component.
 *
 * Inheritance:
 * - output::FloatOutput: Provides standard floating-point output logic.
 * - Component: Integrates with ESPHome's lifecycle (dump_config, etc.).
 * - Parented: specific template to access the main I2C hub instance.
 */

class WaveshareIOCH32V003Output : public output::FloatOutput, public Component, public Parented<WaveshareIOCH32V003Component> {
 public:
  /**
   * @brief Sets the safe operating range for the raw hardware PWM values.
   *
   * Maps the logical 0.0-1.0 output range to a specific subset of the
   * 0-255 hardware range.
   *
   * @param min_value Minimum hardware PWM duty cycle (0-255).
   * @param max_value Maximum hardware PWM duty cycle (0-255).
   */
  void set_pwm_safe_range(uint8_t min_value, uint8_t max_value);

  void dump_config() override;

 protected:
  /**
   * @brief Writes the logical state to the hardware.
   *
   * Called by the parent FloatOutput class. Converts the floating point
   * state (0.0 to 1.0) into the raw hardware value within the configured safe range.
   *
   * @param state The target level normalized between 0.0 and 1.0.
   */
  void write_state(float state) override;

  uint8_t pwm_min_value_{0};
  uint8_t pwm_max_value_{255};
};

}  // namespace waveshare_io_ch32v003
}  // namespace esphome