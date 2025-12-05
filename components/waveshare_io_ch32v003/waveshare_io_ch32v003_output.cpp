#include "waveshare_io_ch32v003_output.h"
#include "esphome/core/log.h"
#include <algorithm> // For std::clamp, std::swap
#include <cmath>     // For roundf

namespace esphome {
namespace waveshare_io_ch32v003 {

static const char *const TAG = "waveshare_io_ch32v003.output";

void WaveshareIOCH32V003Output::set_pwm_safe_range(uint8_t min_value, uint8_t max_value) {
  // Safe Range Validation
  // Automatically corrects invalid configurations (e.g., min=200, max=100)
  // to prevent undefined behavior during operation.
  if (min_value > max_value) {
    ESP_LOGW(TAG, "Invalid PWM safe range: min (%u) > max (%u). Swapping values.", min_value, max_value);
    std::swap(min_value, max_value);
  }
  this->pwm_min_value_ = min_value;
  this->pwm_max_value_ = max_value;
}

void WaveshareIOCH32V003Output::dump_config() {
  ESP_LOGCONFIG(TAG, "Waveshare IO PWM Output:");
  LOG_FLOAT_OUTPUT(this); // Prints inverted, min_power, max_power, etc.
  ESP_LOGCONFIG(TAG, "  PWM Safe Range: [%u, %u]", this->pwm_min_value_, this->pwm_max_value_);
}

void WaveshareIOCH32V003Output::write_state(float state) {
  // 1. Robustness Check: If the I2C hub has failed, abort immediately.
  if (this->parent_->is_failed()) {
    return;
  }

  // 2. Input Safety
  // Ensure the floating point state is strictly within [0.0, 1.0] before calculation.
  float safe_state = std::clamp(state, 0.0f, 1.0f);

  // 3. Precision Conversion
  // Use roundf instead of implicit truncation to ensure accuracy (e.g., 0.5 becomes 128, not 127).
  // Note: Inversion logic is already handled by FloatOutput::set_level before this method is called.
  uint8_t pwm_value = static_cast<uint8_t>(roundf(safe_state * 255.0f));

  // 4. Hardware Clamping (Safety)
  // Apply physical limits defined in YAML (e.g., max_power: 0.96 maps to pwm_max_value=247).
  uint8_t final_pwm_value = std::clamp(pwm_value, this->pwm_min_value_, this->pwm_max_value_);

  // 5. Verbose Logging
  // Useful for debugging brightness curves or verifying safety limits in real-time.
  ESP_LOGV(TAG, "PWM Write: state=%.3f -> raw=%u -> clamped=%u", safe_state, pwm_value, final_pwm_value);

  // 6. Delegate to Parent
  // Send the calculated value to the main component which manages the I2C bus.
  this->parent_->set_pwm_value(final_pwm_value);
}

}  // namespace waveshare_io_ch32v003
}  // namespace esphome