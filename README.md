# EQ3 BLE valves for ESPHome

Maintained fork of [i2nos-giu/eq3-esphome-ha](https://github.com/i2nos-giu/eq3-esphome-ha), preserving the original MIT license and attribution. The component controls EQ3 BLE thermostatic valves from an ESP32 and exposes their controls and diagnostics through ESPHome.

## Fork changes

The first commit imports the component already used on our test bench, including ESPHome compatibility changes, bounded status copying, characteristic reuse, setpoint feedback and vacation-time handling.

The recovery changes address an identification request timing out during BLE discovery and a timeout forwarding a previous status buffer as a new reply:

- Allow 45 seconds for connection/discovery, followed by a separate 15-second reply deadline. A 75-second application deadline also covers queue delays.
- Clear response buffers on every transaction and failure. Failures are reported directly instead of manufacturing a reply with an appended `0xFF`.
- Accept notifications only for the active connection, characteristic and expected response type.
- Keep the transport busy until disconnect completes, with a 10-second fallback.
- Retry failed identification after 60 seconds; preserve the hourly identification and five-minute status/time polls.
- Decode exactly ten serial characters after the four-byte identification header. Preserve the existing decimal firmware value format.
- Use a separate polling timer per valve instance.

Entity schema and names are unchanged. Keep your existing ESPHome node name, IDs, entity names and MQTT settings when changing the component source. An existing retained identification value can remain visible until a new valid response arrives.

## Use this fork

Add this to the existing device configuration, replacing the old source for `eq3nos_bt`:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/jperquin/eq3-esphome-ha
      ref: fix/ble-identification-recovery
    components: [eq3nos_bt]
    refresh: 0s
```

During bench development the branch can be refreshed on every build. For deployment, pin `ref` to the reviewed full commit SHA so builds remain reproducible.

The component lives in `components/eq3nos_bt`. Complete compile-only examples are in `tests/fixtures/esp32.yaml` and `tests/fixtures/esp32-s3.yaml`; their network settings and valve address are placeholders.

## Request identification or status

With an EQ3 component whose ID is `valve_01`, queue a serial/firmware query:

```yaml
button:
  - platform: template
    name: "Query TRV identification"
    on_press:
      - lambda: |-
          id(valve_01).control_.request_identification();
```

`request_identification()` returns whether the request was queued, suppressing duplicates queued through this method. It sends protocol command `0x00`; it does not change the valve setpoint. A valid response publishes serial and firmware together. Failed identification schedules a retry.

To queue a current status request using protocol command `0x03`:

```cpp
id(valve_01).control_.applic_queuer(
    {esphome::eq3nos_bt::AppCommandType::READ_STATUS, false, 0});
```

The regular time/status poll also uses `0x03`, with the synchronized time attached. Neither query reads an actual room-temperature measurement from the TRV.

## Validation and limits

Native regression tests cover separate connection/reply deadlines, timer wraparound, retry timing, serial length, trailers and malformed identification frames:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -I components/eq3nos_bt tests/session_test.cpp -o /tmp/eq3-session-test
/tmp/eq3-session-test
esphome compile tests/fixtures/esp32.yaml
esphome compile tests/fixtures/esp32-s3.yaml
```

CI runs these checks with ESPHome 2026.10.0 and ESP-IDF. Compilation is separate from hardware validation: the recovery changes still need a flash-and-observe test with an actual valve, including an interrupted BLE session and successful identification retry.

The inherited implementation uses fixed EQ3 GATT handles. The inherited Off control and room-temperature behavior are outside this recovery change. Firmware updates, MQTT broker changes and fleet migration are also outside its scope.
