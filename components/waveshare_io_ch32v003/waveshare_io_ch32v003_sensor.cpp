#include "waveshare_io_ch32v003_sensor.h"
#include "esphome/core/log.h"
#include <algorithm> // For std::min
#include <cmath>     // For NAN, std::isnan

namespace esphome {
namespace waveshare_io_ch32v003 {

static const char *const TAG = "waveshare_io_ch32v003.sensor";

void WaveshareIOCH32V003Sensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Waveshare IO ADC Sensor...");
}

void WaveshareIOCH32V003Sensor::dump_config() {
  LOG_SENSOR("", "Waveshare IO ADC Sensor", this);
  ESP_LOGCONFIG(TAG, "  Reference Voltage: %.2fV", this->reference_voltage_);
  LOG_UPDATE_INTERVAL(this);
}

float WaveshareIOCH32V003Sensor::sample() {
  // 1. Robustness Check: If the I2C Hub is in an error state, abort reading.
  if (this->parent_->is_failed()) {
    return NAN;
  }

  // 2. Raw Reading (The parent handles I2C retries internally)
  uint16_t raw_value = this->parent_->get_adc_value();

  // Protection against anomalous values (glitches or firmware bugs) > 1023.
  // Ensures voltage calculation never exceeds reference_voltage_.
  raw_value = std::min<uint16_t>(raw_value, 1023);

  // 3. Conversion
  // The CH32V003 has a 10-bit ADC (0-1023).
  float voltage = (float(raw_value) / 1023.0f) * this->reference_voltage_;

  // Critical for tuning and verifying the voltage divider.
  ESP_LOGV(TAG, "ADC Sample: Raw=%u -> Voltage=%.3f V", raw_value, voltage);
  
  return voltage;
}

void WaveshareIOCH32V003Sensor::update() {
  // Execute the sampling logic
  float voltage = this->sample();

  // Error Handling:
  // If sample returns NAN (parent error), do not publish.
  // This preserves the last valid state in Home Assistant instead of sending 
  // false zeros or triggering incorrect automations.
  if (!std::isnan(voltage)) {
    this->publish_state(voltage);
  }
}

}  // namespace waveshare_io_ch32v003
}  // namespace esphome