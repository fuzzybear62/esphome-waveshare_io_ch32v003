/**
 * @file waveshare_io_ch32v003.cpp
 * @brief Implementation of the Waveshare IO CH32V003 Expansion Board Component.
 *
 * This file handles the low-level I2C communication, register management,
 * retry logic, and state management for GPIOs, ADC, and PWM features.
 */

#include "waveshare_io_ch32v003.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h" 

namespace esphome {
namespace waveshare_io_ch32v003 {

// --- Constants and Configuration ---
namespace {
  // Hardware Register Map
  // Addresses used to communicate with the CH32V003 firmware
  constexpr uint8_t IO_REG_DIRECTION = 0x02; // Register to set Input/Output direction
  constexpr uint8_t IO_REG_OUTPUT = 0x03;    // Register to write Output levels (High/Low)
  constexpr uint8_t IO_REG_INPUT = 0x04;     // Register to read Input levels
  constexpr uint8_t IO_REG_PWM = 0x05;       // Register to set PWM duty cycle
  constexpr uint8_t IO_REG_ADC = 0x06;       // Register to read 10-bit ADC value
  constexpr uint8_t IO_REG_INTERRUPT = 0x07; // Register for interrupt status
  
  // I2C Robustness Configuration
  constexpr uint8_t MAX_RETRIES = 3;         // Max attempts for I2C transactions
  constexpr uint32_t RETRY_DELAY_MS = 2;     // Delay between retries
  constexpr uint8_t MAX_PINS = 8;            // Total number of GPIO pins available
}

static const char *const TAG = "waveshare_io_ch32v003";

// --- Helpers for I2C Robustness (Retry Logic) ---

/**
 * @brief Writes a byte to a register with retry logic.
 * * Protects against transient I2C bus noise by attempting the write multiple times.
 * * @param a_register The target register address.
 * @param value The value to write.
 * @return true if successful, false after MAX_RETRIES.
 */
bool WaveshareIOCH32V003Component::write_register_with_retry_(uint8_t a_register, uint8_t value) {
  for (uint8_t i = 0; i < MAX_RETRIES; i++) {
    if (this->write_byte(a_register, value)) {
      if (i > 0) {
        ESP_LOGW(TAG, "I2C Write recovered after %d retry(ies) for reg 0x%02X", i, a_register);
      }
      return true;
    }
    delay(RETRY_DELAY_MS);
  }
  ESP_LOGE(TAG, "I2C Write failed after %d attempts for reg 0x%02X", MAX_RETRIES, a_register);
  return false;
}

/**
 * @brief Reads bytes from a register with retry logic.
 * * @param a_register The source register address.
 * @param data Pointer to the buffer where data will be stored.
 * @param len Number of bytes to read.
 * @return true if successful, false after MAX_RETRIES.
 */
bool WaveshareIOCH32V003Component::read_registers_with_retry_(uint8_t a_register, uint8_t *data, uint8_t len) {
  for (uint8_t i = 0; i < MAX_RETRIES; i++) {
    if (this->read_bytes(a_register, data, len)) {
      if (i > 0) {
        ESP_LOGW(TAG, "I2C Read recovered after %d retry(ies) for reg 0x%02X", i, a_register);
      }
      return true;
    }
    delay(RETRY_DELAY_MS);
  }
  return false;
}

// --- Setup and Loop ---

void WaveshareIOCH32V003Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Waveshare IO CH32V003...");
  ESP_LOGD(TAG, "Initializing hardware with Safe Mode (All Input/Low)");

  // SAFE LOGIC:
  // Initialize all pins as INPUT (0) to avoid short circuits at boot.
  // Previous code used 0xFF which meant OUTPUT HIGH - Dangerous!
  // We use 0x00 to ensure everything starts as Input/Low.
  this->mode_mask_ = 0xFF;
  this->output_mask_ = 0xFF;

  // IMPORTANT: We do NOT write to hardware here anymore.
  // We defer initialization to the first loop() call to ensure
  // the I2C bus is fully initialized by the ESPHome framework.
  
  // is_ready_ remains false until the first loop success.
}

void WaveshareIOCH32V003Component::loop() { 
  // 1. Deferred Initialization Logic
  // This block runs only once, the first time loop() is called and I2C is ready.
  if (!this->is_ready_) {
      ESP_LOGD(TAG, "Performing deferred hardware initialization...");
      
      bool step1 = this->write_gpio_modes_();
      bool step2 = this->write_gpio_outputs_();

      if (step1 && step2) {
          this->is_ready_ = true;
          ESP_LOGI(TAG, "Waveshare IO hardware synced and ready.");
          this->status_clear_warning();
      } else {
          // If I2C fails, we don't crash, but we warn and retry next loop.
          this->status_set_warning("Waiting for I2C bus to become ready...");
          return; 
      }
  }

  // 2. Standard Loop Logic
  // Clears the read cache at the start of every loop to ensure fresh data for this cycle.
  this->reset_pin_cache_(); 
}

// --- Pin Mode Management ---

/**
 * @brief Sets the mode (Input/Output) for a specific pin.
 * * @param pin The pin index (0-7).
 * @param flags GPIO flags indicating the desired mode.
 */
void WaveshareIOCH32V003Component::pin_mode(uint8_t pin, gpio::Flags flags) {
  if (pin >= MAX_PINS) return;

  // Correct bitwise logic: Update the internal shadow mask
  if (flags & gpio::FLAG_OUTPUT) {
    this->mode_mask_ |= (1 << pin);
  } else {
    // Safe default: Input
    this->mode_mask_ &= ~(1 << pin);
  }
  
  // Apply hardware change immediately to reflect the new configuration only when i2c is ready
  if (this->is_ready_) {
      this->write_gpio_modes_();
  }
}

// --- Hardware Operations (Low Level) ---

bool WaveshareIOCH32V003Component::write_gpio_modes_() {
  if (this->is_failed()) return false;
  return this->write_register_with_retry_(IO_REG_DIRECTION, this->mode_mask_);
}

bool WaveshareIOCH32V003Component::write_gpio_outputs_() {
  if (this->is_failed()) return false;
  return this->write_register_with_retry_(IO_REG_OUTPUT, this->output_mask_);
}

/**
 * @brief Reads the state of a digital pin from hardware.
 * * Uses a caching mechanism implicit in the input_mask_ updating logic.
 */
bool WaveshareIOCH32V003Component::digital_read_hw(uint8_t pin) {
  if (pin >= MAX_PINS) return false;

  if (this->is_failed()) {
    return (this->input_mask_ & (1 << pin)) != 0;
  }

  uint8_t data = 0;
  // Attempt hardware read (with Retry)
  // Reads the entire 8-bit port state at once
  if (this->read_registers_with_retry_(IO_REG_INPUT, &data, 1)) {
    this->input_mask_ = data;
    this->status_clear_warning();
  } else {
    this->status_set_warning("Failed to read GPIO inputs");
  }

  return (this->input_mask_ & (1 << pin)) != 0;
}

/**
 * @brief Writes a digital value to a pin.
 * * Updates the internal shadow register (output_mask_) and then writes to I2C.
 */
void WaveshareIOCH32V003Component::digital_write_hw(uint8_t pin, bool value) {
  if (pin >= MAX_PINS) return;
  if (this->is_failed()) return;

  // Atomic mask update
  if (value) {
    this->output_mask_ |= (1 << pin);
  } else {
    this->output_mask_ &= ~(1 << pin);
  }

  // DEBUG LOG: Track what we are writing
  ESP_LOGD(TAG, "Digital Write Pin %u -> %u (Mask: 0x%02X)", pin, value, this->output_mask_);

  // Hardware Write only if i2C bus is ready
  if (this->is_ready_) {
    if (!this->write_register_with_retry_(IO_REG_OUTPUT, this->output_mask_)) {
      this->status_set_warning("Failed to write GPIO output");
    } else {
      this->status_clear_warning();
    }
  }
}

bool WaveshareIOCH32V003Component::digital_read_cache(uint8_t pin) { 
  if (pin >= MAX_PINS) return false;
  return this->input_mask_ & (1 << pin); 
}

// --- Special Functions (ADC / PWM) ---

uint16_t WaveshareIOCH32V003Component::get_adc_value() {
  if (this->is_failed()) return 0;

  uint8_t data[2];
  // Read 2 bytes from ADC register (10-bit value)
  if (!this->read_registers_with_retry_(IO_REG_ADC, data, 2)) {
    return 0; 
  }
  // Combine bytes (Little Endian)
  uint16_t val = (data[1] << 8) | data[0];
  
  // DEBUG LOG: Track ADC readings
  ESP_LOGD(TAG, "ADC Read Raw: %u", val);
  
  return val;
}

uint8_t WaveshareIOCH32V003Component::get_interrupt_status() {
  if (this->is_failed()) return 0;
  uint8_t data = 0;
  this->read_registers_with_retry_(IO_REG_INTERRUPT, &data, 1);
  return data;
}

void WaveshareIOCH32V003Component::set_pwm_value(uint8_t value) {
  if (this->is_failed()) return;

  // DEBUG LOG: Track PWM writes
  ESP_LOGD(TAG, "Setting PWM HW Value: %u", value);

  uint8_t data[2] = {IO_REG_PWM, value};
  bool success = false;
  // Manual I2C write loop as we are writing a register+value pair explicitly
  for(uint8_t i=0; i<MAX_RETRIES; i++) {
    if (this->write_bytes(data[0], &data[1], 1)) {
        success = true;
        break;
    }
    delay(RETRY_DELAY_MS);
  }

  if (!success) {
      ESP_LOGW(TAG, "Failed to set PWM value");
  }
}

float WaveshareIOCH32V003Component::get_setup_priority() const { return setup_priority::DATA; }
float WaveshareIOCH32V003Component::get_loop_priority() const { return 9.0f; } 

// --- GPIOPin Implementation ---

void WaveshareIOCH32V003GPIOPin::setup() {
    if (this->pin_ == 255) {
        ESP_LOGE(TAG, "GPIO Pin not configured!");
        return;
    }
    this->parent_->pin_mode(this->pin_, this->flags_);
}

void WaveshareIOCH32V003GPIOPin::pin_mode(gpio::Flags flags) { 
    this->parent_->pin_mode(this->pin_, flags); 
}

bool WaveshareIOCH32V003GPIOPin::digital_read() { 
    // Reads from parent and applies inversion if configured
    return this->parent_->digital_read(this->pin_) ^ this->inverted_; 
}

void WaveshareIOCH32V003GPIOPin::digital_write(bool value) {
  // Writes to parent, applying inversion logic
  this->parent_->digital_write(this->pin_, value ^ this->inverted_);
}

std::string WaveshareIOCH32V003GPIOPin::dump_summary() const { 
    return str_sprintf("EXIO%u via WaveshareIO", pin_); 
}

void WaveshareIOCH32V003GPIOPin::set_flags(gpio::Flags flags) {
  flags_ = flags;
  this->parent_->pin_mode(this->pin_, flags);
}

}  // namespace waveshare_io_ch32v003
}  // namespace esphome