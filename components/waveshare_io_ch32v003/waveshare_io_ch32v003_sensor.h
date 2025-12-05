#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "waveshare_io_ch32v003.h"

namespace esphome {
namespace waveshare_io_ch32v003 {

/**
 * @brief Voltage Sensor component for the Waveshare IO CH32V003 expansion board.
 *
 * This class reads the voltage from the expansion board's ADC.
 * It inherits from:
 * - sensor::Sensor: To publish state to Home Assistant/API.
 * - PollingComponent: To periodically trigger updates.
 * - voltage_sampler::VoltageSampler: To allow this sensor to be used as a raw source
 * by other components (e.g., resistive touch, NTC, etc.) that require an ADC backend.
 */
class WaveshareIOCH32V003Sensor : public sensor::Sensor,
                                  public PollingComponent,
                                  public voltage_sampler::VoltageSampler,
                                  public Parented<WaveshareIOCH32V003Component> {
 public:
  void set_reference_voltage(float reference_voltage) { this->reference_voltage_ = reference_voltage; }

  void setup() override;
  void update() override;
  void dump_config() override;
  
  /**
   * @brief Reads a single voltage sample from the hardware.
   * * Implementation of the VoltageSampler interface. 
   * Allows this component to be used as a source for other components (e.g. ntc, ct_clamp).
   * * @return The calculated voltage in Volts.
   */
  float sample() override;

 protected:
  // Default reference voltage, usually overridden by YAML configuration (e.g., 9.9V for specific board versions)
  float reference_voltage_{3.3f}; 
};

}  // namespace waveshare_io_ch32v003
}  // namespace esphome