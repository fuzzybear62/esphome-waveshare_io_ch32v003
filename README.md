# Waveshare IO CH32V003 Component for ESPHome

A custom ESPHome component for the **Waveshare IO Expansion Board** based on the **CH32V003** RISC-V microcontroller.

**Hardware Support:**
This component has been specifically developed for and tested on the **Waveshare ESP32-S3 7" Display version B (1024x600)**.

## Installation

Add the following to your ESPHome `yaml` configuration to pull the component directly from GitHub:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/fuzzybear62/esphome-waveshare_io_ch32v003
      ref: main
    components: [ waveshare_io_ch32v003 ]
```

## Configuration

### 1\. Main Hub Setup

Configure the I2C bus and the main component hub.
**Note:** The default I2C address for the IO expander on this board is `0x24`.

```yaml
i2c:
  sda: GPIO8  # Adjust to your board's specific I2C pins
  scl: GPIO9
  scan: true
  id: bus_a

waveshare_io_ch32v003:
  id: waveshare_io_hub
  i2c_id: bus_a
  address: 0x24
```

### 2\. Display Backlight (PWM)

The following configuration sets up the backlight control using the PWM capabilities of the CH32V003. It includes safe limits to protect the hardware and inversion settings required for this specific display panel.

```yaml
output:
  - platform: waveshare_io_ch32v003
    id: wave_io_pwm_output
    inverted: true
    zero_means_zero: true
    waveshare_io_ch32v003_id: waveshare_io_hub
    # Hardware safety limits (0-255)
    safe_pwm_levels:
      min_value: 0
      max_value: 247

light:
  - platform: monochromatic
    output: wave_io_pwm_output
    name: "Display Backlight"
    icon: mdi:lightbulb-on
    id: display_backlight
    restore_mode: ALWAYS_ON
    default_transition_length: 0ms
```

### 3\. Backlight Power Switch (GPIO)

Some revisions of the board allow toggling the backlight power completely via GPIO.

```yaml
switch:
  - platform: gpio
    name: "Backlight Switch"
    id: backlight_switch
    internal: true                
    pin:
      waveshare_io_ch32v003: waveshare_io_hub
      number: 2
      mode: OUTPUT
    restore_mode: ALWAYS_ON 
```

### 4\. Battery Voltage Sensor (ADC)

The CH32V003 can read the battery voltage. The configuration below assumes the standard voltage divider present on the Waveshare board (10-bit ADC, 3.3V reference, 3:1 divider).

```yaml
sensor:
  - platform: waveshare_io_ch32v003
    name: "Battery level"
    waveshare_io_ch32v003_id: waveshare_io_hub
    id: wave_adc
    # Calculation: value = adc * 3 * 3.3V / 1023.0
    # Therefore, reference_voltage is set to 9.9 (3 * 3.3)
    reference_voltage: 9.9 
```

## Hardware Notes

  * **I2C Address**: The default address for this board is **0x24**.
  * **Backlight**: The backlight PWM on this board is typically inverted.
  * **Voltage Reference**: The voltage divider requires a reference voltage calculation of roughly `9.9V` to output the correct battery voltage.

## Troubleshooting

  * **Communication Failed**: Ensure you are using address `0x24`. If issues persist, check `i2c` logs to see if the device is detected on a scan.
  * **Flickering**: If the backlight flickers, ensure `zero_means_zero: true` is enabled in the output configuration.

## License

[MIT License](https://www.google.com/search?q=LICENSE)
