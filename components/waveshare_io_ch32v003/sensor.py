"""
Waveshare IO CH32V003 Voltage Sensor Component for ESPHome.

This component implements a voltage sensor for the Waveshare IO expansion board.
It acts as a PollingComponent to periodically read voltage and implements the
VoltageSampler interface, allowing it to be used by other components that need
voltage readings (e.g., ADC buttons).
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, voltage_sampler
from esphome.const import (
    CONF_ID,
    CONF_REFERENCE_VOLTAGE,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
)
from . import (
    CONF_WAVESHARE_IO_CH32V003_ID,
    WaveshareIOCH32V003Component,
    waveshare_io_ch32v003_ns,
)

# No explicit DEPENDENCIES here; we rely on AUTO_LOAD in __init__.py

# --- Class Definition ---
# Defines the C++ class hierarchy.
# - sensor.Sensor: Base class for publishing float values.
# - cg.PollingComponent: base class for components that update periodically.
# - voltage_sampler.VoltageSampler: Interface for on-demand voltage sampling.
WaveshareIOCH32V003Sensor = waveshare_io_ch32v003_ns.class_(
    "WaveshareIOCH32V003Sensor",
    sensor.Sensor,
    cg.PollingComponent,
    voltage_sampler.VoltageSampler,
    cg.Parented.template(WaveshareIOCH32V003Component),
)

# --- Configuration Schema ---
CONFIG_SCHEMA = (
    sensor.sensor_schema(
        WaveshareIOCH32V003Sensor,
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            # Link to the main Hub component (Parent)
            cv.Required(CONF_WAVESHARE_IO_CH32V003_ID): cv.use_id(
                WaveshareIOCH32V003Component
            ),
            # Reference voltage for the voltage divider.
            # Default is usually set to the max range (e.g., 9.9V or 3.3V depending on hardware).
            cv.Optional(CONF_REFERENCE_VOLTAGE, default="9.9V"): cv.All(
                cv.voltage, 
                cv.Range(min=0.1)
            ),
        }
    )
    # Adds the polling interval configuration (default: 60s)
    .extend(cv.polling_component_schema("60s"))
)

async def to_code(config):
    """
    Code logic for the voltage sensor.
    """
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_parented(var, config[CONF_WAVESHARE_IO_CH32V003_ID])
    
    # Register as a component (required for setup() and loop() / polling)
    await cg.register_component(var, config)
    
    # Register as a sensor (required for Home Assistant state publication)
    await sensor.register_sensor(var, config)

    # Note: voltage_sampler.register_voltage_sampler(...) is NOT called here.
    # The C++ inheritance mechanism handles the casting automatically.

    # Set the configured reference voltage for calculation
    cg.add(var.set_reference_voltage(config[CONF_REFERENCE_VOLTAGE]))

    # --- ADDED FOR MODULAR COMPILATION ---
    # Only compile the sensor source file if this component is actually used.
    cg.add_library("waveshare_io_ch32v003_sensor", None, ["waveshare_io_ch32v003_sensor.cpp"])