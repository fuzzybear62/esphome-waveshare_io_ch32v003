#include "waveshare_io_ch32v003.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h" 

namespace esphome {
namespace waveshare_io_ch32v003 {

namespace {
  constexpr uint8_t IO_REG_DIRECTION = 0x02; 
  constexpr uint8_t IO_REG_OUTPUT = 0x03;    
  constexpr uint8_t IO_REG_INPUT = 0x04;     
  constexpr uint8_t IO_REG_PWM = 0x05;       
  constexpr uint8_t IO_REG_ADC = 0x06;       
  constexpr uint8_t IO_REG_INTERRUPT = 0x07; 
  
  constexpr uint8_t MAX_RETRIES = 3;         
  constexpr uint32_t RETRY_DELAY_MS = 2;     
  constexpr uint8_t MAX_PINS = 8;            
}

static const char *const TAG = "waveshare_io_ch32v003";

// --- Constructor ---
WaveshareIOCH32V003Component::WaveshareIOCH32V003Component() {
  // Ensure the ready flag is false from the very beginning
  this->is_ready_ = false;
}

// --- Helpers for I2C Robustness ---

bool WaveshareIOCH32V003Component::write_register_with_retry_(uint8_t a_register, uint8_t value) {
  for (uint8_t i = 0; i < MAX_RETRIES; i++) {
    if (this->write_byte(a_register, value)) {
      if (i > 0) ESP_LOGW(TAG, "I2C Write recovered after %d retry(ies)", i);
      return true;
    }
    delay(RETRY_DELAY_MS);
  }
  ESP_LOGE(TAG, "I2C Write failed for reg 0x%02X", a_register);
  return false;
}

bool WaveshareIOCH32V003Component::read_registers_with_retry_(uint8_t a_register, uint8_t *data, uint8_t len) {
  for (uint8_t i = 0; i < MAX_RETRIES; i++) {
    if (this->read_bytes(a_register, data, len)) {
      if (i > 0) ESP_LOGW(TAG, "I2C Read recovered after %d retry(ies)", i);
      return true;
    }
    delay(RETRY_DELAY_MS);
  }
  return false;
}

// --- Setup and Loop ---

void WaveshareIOCH32V003Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Waveshare IO CH32V003...");
  
  // Initialize internal masks to a safe state (Input/Low).
  // IMPORTANT: Do NOT write to I2C here. We wait for the first loop().
  this->mode_mask_ = 0x00;
  this->output_mask_ = 0x00;
  
  // is_ready_ remains false until the loop() successfully syncs with hardware.
}

void WaveshareIOCH32V003Component::loop() { 
  // --- Deferred Initialization ---
  // Runs only once, when the I2C bus is guaranteed to be ready.
  if (!this->is_ready_) {
    ESP_LOGD(TAG, "Performing deferred hardware initialization...");
    
    // Attempt to write the accumulated configurations to hardware
    bool step1 = this->write_gpio_modes_();
    bool step2 = this->write_gpio_outputs_();

    if (step1 && step2) {
      this->is_ready_ = true;
      ESP_LOGI(TAG, "Waveshare IO hardware synced and ready.");
      this->status_clear_warning();
    } else {
      // If it fails, we do not crash; we warn and retry on the next loop iteration.
      this->status_set_warning("Waiting for I2C bus...");
      return; 
    }
  }

  // --- Standard Loop ---
  this->reset_pin_cache_(); 
}

void WaveshareIOCH32V003Component::dump_config() {
  ESP_LOGCONFIG(TAG, "WaveshareIO CH32V003:");
  LOG_I2C_DEVICE(this)
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with IO Expander failed!");
  }
}

// --- Pin Mode Management ---

void WaveshareIOCH32V003Component::pin_mode(uint8_t pin, gpio::Flags flags) {
  if (pin >= MAX_PINS) return;

  // Update only the internal mask
  if (flags & gpio::FLAG_OUTPUT) {
    this->mode_mask_ |= (1 << pin);
  } else {
    this->mode_mask_ &= ~(1 << pin);
  }
  
  // Write to hardware ONLY if deferred initialization is complete
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

bool WaveshareIOCH32V003Component::digital_read_hw(uint8_t pin) {
  if (pin >= MAX_PINS) return false;

  if (this->is_failed()) {
    return (this->input_mask_ & (1 << pin)) != 0;
  }

  uint8_t data = 0;
  if (this->read_registers_with_retry_(IO_REG_INPUT, &data, 1)) {
    this->input_mask_ = data;
    this->status_clear_warning();
  } else {
    this->status_set_warning("Failed to read GPIO inputs");
  }

  return (this->input_mask_ & (1 << pin)) != 0;
}

void WaveshareIOCH32V003Component::digital_write_hw(uint8_t pin, bool value) {
  if (pin >= MAX_PINS) return;
  if (this->is_failed()) return;

  if (value) {
    this->output_mask_ |= (1 << pin);
  } else {
    this->output_mask_ &= ~(1 << pin);
  }

  ESP_LOGD(TAG, "Digital Write Pin %u -> %u", pin, value);

  // Write to hardware ONLY if deferred initialization is complete
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

// --- Special Functions ---

uint16_t WaveshareIOCH32V003Component::get_adc_value() {
  if (this->is_failed()) return 0;
  uint8_t data[2];
  if (!this->read_registers_with_retry_(IO_REG_ADC, data, 2)) return 0; 
  return (data[1] << 8) | data[0];
}

uint8_t WaveshareIOCH32V003Component::get_interrupt_status() {
  if (this->is_failed()) return 0;
  uint8_t data = 0;
  this->read_registers_with_retry_(IO_REG_INTERRUPT, &data, 1);
  return data;
}

void WaveshareIOCH32V003Component::set_pwm_value(uint8_t value) {
  if (this->is_failed()) return;
  uint8_t data[2] = {IO_REG_PWM, value};
  for(uint8_t i=0; i<MAX_RETRIES; i++) {
    if (this->write_bytes(data[0], &data[1], 1)) return;
    delay(RETRY_DELAY_MS);
  }
  ESP_LOGW(TAG, "Failed to set PWM value");
}

float WaveshareIOCH32V003Component::get_setup_priority() const { return setup_priority::DATA; }
float WaveshareIOCH32V003Component::get_loop_priority() const { return 9.0f; } 

// --- GPIOPin Implementation ---

void WaveshareIOCH32V003GPIOPin::setup() {
    if (this->pin_ == 255) return;
    this->parent_->pin_mode(this->pin_, this->flags_);
}

void WaveshareIOCH32V003GPIOPin::pin_mode(gpio::Flags flags) { 
    this->parent_->pin_mode(this->pin_, flags); 
}

bool WaveshareIOCH32V003GPIOPin::digital_read() { 
    return this->parent_->digital_read(this->pin_) ^ this->inverted_; 
}

void WaveshareIOCH32V003GPIOPin::digital_write(bool value) {
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