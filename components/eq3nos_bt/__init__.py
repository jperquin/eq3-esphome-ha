import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import (
    ble_client,
    sensor,
    binary_sensor,
    switch,
    text_sensor,
    number,
    select,
)

from esphome.components.time import RealTimeClock
from esphome.const import CONF_ID, CONF_NAME

# --------------------------------------------------
# AUTO LOAD
# --------------------------------------------------
AUTO_LOAD = ["sensor", "binary_sensor", "switch", "number", "select", "text_sensor"]

# --------------------------------------------------
# NAMESPACE & CLASSES
# --------------------------------------------------
eq3ns_ns = cg.esphome_ns.namespace("eq3nos_bt")

EQ3NOS = eq3ns_ns.class_("EQ3NOS", cg.Component)
EQ3TargetNumber = eq3ns_ns.class_("EQ3TargetNumber", number.Number)

EQ3BoostSwitch = eq3ns_ns.class_("EQ3BoostSwitch", switch.Switch, cg.Component)
EQ3OffSwitch   = eq3ns_ns.class_("EQ3OffSwitch",   switch.Switch, cg.Component)
EQ3LockSwitch  = eq3ns_ns.class_("EQ3LockSwitch",  switch.Switch, cg.Component)
EQ3ModeSelect  = eq3ns_ns.class_("EQ3ModeSelect",  select.Select, cg.Component)

# --------------------------------------------------
# CONFIG KEYS
# --------------------------------------------------
CONF_TIME_ID       = "time_id"
CONF_MAC_ADDRESS   = "mac_address"
CONF_BLE_CLIENT_ID = "ble_client_id"

CONF_SERIAL     = "eq3_serial"
CONF_FIRMWARE   = "eq3_firmware"
CONF_VALVE_POS  = "valve_position"
CONF_CUR_TEMP   = "current_temperature"
CONF_TGT_TEMP   = "target_temperature"
CONF_BATTERY    = "battery_low"
CONF_WATCHDOG   = "watchdog"
CONF_RECOVERY   = "recovery_counter"
CONF_SETPOINT   = "setpoint"
CONF_BOOST      = "boost"
CONF_OFF        = "toff"
CONF_LOCK       = "lock"
CONF_MODE       = "mode"

# --------------------------------------------------
# HELPER
# --------------------------------------------------
def full_name(conf, suffix: str) -> str:
    return f"{conf[CONF_NAME]} {suffix}"

# --------------------------------------------------
# SCHEMA
# --------------------------------------------------
EQ3NOS_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(EQ3NOS),

    cv.Required(CONF_NAME): cv.string,
    cv.Required(CONF_MAC_ADDRESS): cv.mac_address,
    cv.Required(CONF_BLE_CLIENT_ID): cv.use_id(ble_client.BLEClient),
    cv.Optional(CONF_TIME_ID): cv.use_id(RealTimeClock),

    cv.Optional(CONF_SERIAL):   text_sensor.text_sensor_schema(),
    cv.Optional(CONF_FIRMWARE): text_sensor.text_sensor_schema(),

    cv.Optional(CONF_VALVE_POS): sensor.sensor_schema(
        unit_of_measurement="%",
        accuracy_decimals=0,
    ),
    cv.Optional(CONF_CUR_TEMP): sensor.sensor_schema(
        unit_of_measurement="°C",
        accuracy_decimals=1,
    ),
    cv.Optional(CONF_TGT_TEMP): sensor.sensor_schema(
        unit_of_measurement="°C",
        accuracy_decimals=1,
    ),
    cv.Optional(CONF_RECOVERY): sensor.sensor_schema(
        accuracy_decimals=0,
    ),

    cv.Optional(CONF_BATTERY):  binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_WATCHDOG): binary_sensor.binary_sensor_schema(),

    cv.Optional(CONF_SETPOINT): number.number_schema(EQ3TargetNumber),
    cv.Optional(CONF_BOOST):    switch.switch_schema(EQ3BoostSwitch),
    cv.Optional(CONF_OFF):      switch.switch_schema(EQ3OffSwitch),
    cv.Optional(CONF_LOCK):     switch.switch_schema(EQ3LockSwitch),
    cv.Optional(CONF_MODE):     select.select_schema(EQ3ModeSelect),
})

CONFIG_SCHEMA = cv.ensure_list(EQ3NOS_SCHEMA)

# --------------------------------------------------
# TO_CODE
# --------------------------------------------------
async def to_code(config):

    for conf in config:
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)

        client = await cg.get_variable(conf[CONF_BLE_CLIENT_ID])
        cg.add(var.set_ble_client_parent(client))
        cg.add(var.set_mac_address(cg.RawExpression(f"\"{conf[CONF_MAC_ADDRESS]}\"")))
        cg.add(var.set_name(conf[CONF_NAME]))

        if CONF_TIME_ID in conf:
            time_obj = await cg.get_variable(conf[CONF_TIME_ID])
            cg.add(var.set_time_source(time_obj))

        # ---- TEXT ----
        if CONF_SERIAL in conf:
            conf[CONF_SERIAL][CONF_NAME] = full_name(conf, "Serial")
            s = await text_sensor.new_text_sensor(conf[CONF_SERIAL])
            cg.add(var.set_serial_sensor(s))

        if CONF_FIRMWARE in conf:
            conf[CONF_FIRMWARE][CONF_NAME] = full_name(conf, "Firmware")
            s = await text_sensor.new_text_sensor(conf[CONF_FIRMWARE])
            cg.add(var.set_firmware_sensor(s))

        # ---- SENSORS ----
        for key, label, setter in [
            (CONF_VALVE_POS, "Valve Position", var.set_valve_position_sensor),
            (CONF_CUR_TEMP,  "Current Temperature", var.set_current_temperature_sensor),
            (CONF_TGT_TEMP,  "Target Temperature", var.set_target_temperature_sensor),
            (CONF_RECOVERY,  "Recovery Counter", var.set_recovery_counter_sensor),
        ]:
            if key in conf:
                conf[key][CONF_NAME] = full_name(conf, label)
                sens = await sensor.new_sensor(conf[key])
                cg.add(setter(sens))

        # ---- BINARY ----
        for key, label, setter in [
            (CONF_BATTERY,  "Battery Low", var.set_battery_low_sensor),
            (CONF_WATCHDOG, "Watchdog",    var.set_watchdog_sensor),
        ]:
            if key in conf:
                conf[key][CONF_NAME] = full_name(conf, label)
                sens = await binary_sensor.new_binary_sensor(conf[key])
                cg.add(setter(sens))

        # ---- NUMBER ----
        if CONF_SETPOINT in conf:
            conf[CONF_SETPOINT][CONF_NAME] = full_name(conf, "Setpoint")
            num = cg.new_Pvariable(conf[CONF_SETPOINT][CONF_ID])
            await number.register_number(
                num,
                conf[CONF_SETPOINT],
                min_value=5.0,
                max_value=30.0,
                step=0.5,
            )
            cg.add(var.set_target_number(num))

        # ---- SWITCHES ----
        for key, label, setter in [
            (CONF_BOOST, "Boost", var.set_boost_switch),
            (CONF_OFF,   "Off",   var.set_off_switch),
            (CONF_LOCK,  "Lock",  var.set_lock_switch),
        ]:
            if key in conf:
                conf[key][CONF_NAME] = full_name(conf, label)
                sw = await switch.new_switch(conf[key])
                cg.add(sw.set_parent(var))
                cg.add(setter(sw))

        # ---- MODE ----
        if CONF_MODE in conf:
            conf[CONF_MODE][CONF_NAME] = full_name(conf, "Mode")
            sel = await select.new_select(
                conf[CONF_MODE],
                options=["auto", "manual", "vacation"],
            )
            cg.add(sel.set_parent(var))
            cg.add(var.set_mode_select(sel))
