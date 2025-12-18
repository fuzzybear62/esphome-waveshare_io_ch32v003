"""
Waveshare IO CH32V003 PWM Output Component for ESPHome.

This component handles the PWM output functionality for the Waveshare IO expansion board.
It allows configuring pins as floating-point outputs with support for both
percentage-based power levels and raw hardware PWM levels (0-255).
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import output
from esphome.const import (
    CONF_ID, 
    CONF_MIN_POWER, 
    CONF_MAX_POWER
)
from . import (
    CONF_WAVESHARE_IO_CH32V003_ID,
    WaveshareIOCH32V003Component,
    waveshare_io_ch32v003_ns,
)

DEPENDENCIES = ["waveshare_io_ch32v003"]

# --- Class Definition ---
# Defines the C++ class hierarchy.
# Inherits from FloatOutput for standard output entity behavior.
# Inherits from Component to handle setup() and loop() lifecycle methods.
WaveshareIOCH32V003Output = waveshare_io_ch32v003_ns.class_(
    "WaveshareIOCH32V003Output",
    output.FloatOutput,
    cg.Component, 
    cg.Parented.template(WaveshareIOCH32V003Component),
)

# --- Constants ---
CONF_SAFE_PWM_LEVELS = "safe_pwm_levels"
CONF_MIN_VALUE = "min_value"
CONF_MAX_VALUE = "max_value"

def validate_safe_levels(cfg):
    """
    Validates that the minimum PWM value is strictly less than the maximum.
    """
    if CONF_SAFE_PWM_LEVELS in cfg:
        sl = cfg[CONF_SAFE_PWM_LEVELS]
        if sl[CONF_MIN_VALUE] >= sl[CONF_MAX_VALUE]:
            raise cv.Invalid(
                f"safe_pwm_levels: min_value ({sl[CONF_MIN_VALUE]}) must be strictly less than max_value ({sl[CONF_MAX_VALUE]})"
            )
    return cfg

# --- Configuration Schema ---
# Extends the standard Float Output schema.
# Adds requirements for the parent component linkage.
# Supports two modes of configuration:
# 1. Standard: min_power and max_power as percentages (0.0 - 1.0).
# 2. Raw: safe_pwm_levels with explicit 0-255 integer limits.
CONFIG_SCHEMA = cv.All(
    output.FLOAT_OUTPUT_SCHEMA.extend(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(WaveshareIOCH32V003Output),
            
            # Reference to the main Hub component
            cv.Required(CONF_WAVESHARE_IO_CH32V003_ID): cv.use_id(
                WaveshareIOCH32V003Component
            ),
            
            # Standard Mode (Percentage based)
            cv.Optional(CONF_MIN_POWER, default=0.0): cv.percentage,
            cv.Optional(CONF_MAX_POWER, default=1.0): cv.percentage,
            
            # Raw Mode (Hardware 8-bit precision)
            cv.Optional(CONF_SAFE_PWM_LEVELS): cv.Schema({
                cv.Required(CONF_MIN_VALUE): cv.int_range(min=0, max=255),
                cv.Required(CONF_MAX_VALUE): cv.int_range(min=0, max=255),
            }),
        }
    ),
    validate_safe_levels
)

async def to_code(config):
    """
    Code generation logic for the PWM output.
    """
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_parented(var, config[CONF_WAVESHARE_IO_CH32V003_ID])
    
    # Registers the variable as a standard Output entity
    await output.register_output(var, config)
    
    # Registers the variable as a Component to ensure setup() and dump_config() are called
    await cg.register_component(var, config)
    
    # Determine PWM limits: Priority is given to raw 'safe_pwm_levels' if defined.
    # Otherwise, converts standard percentage power to 0-255 range.
    if CONF_SAFE_PWM_LEVELS in config:
        raw_config = config[CONF_SAFE_PWM_LEVELS]
        min_pwm = raw_config[CONF_MIN_VALUE]
        max_pwm = raw_config[CONF_MAX_VALUE]
    else:
        min_pwm = int(config[CONF_MIN_POWER] * 255)
        max_pwm = int(config[CONF_MAX_POWER] * 255)
    
    # Set the calculated safe range in the C++ object
    cg.add(var.set_pwm_safe_range(min_pwm, max_pwm))

    # --- ADDED FOR MODULAR COMPILATION ---
    # Only compile the output source file if this component is actually used.
    cg.add_library("waveshare_io_ch32v003_output", None, ["waveshare_io_ch32v003_output.cpp"])