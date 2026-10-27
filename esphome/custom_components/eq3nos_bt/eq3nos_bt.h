#pragma once
#include "eq3nos_log.h"
#include "eq3nos_bt_entities.h"
#include "eq3nos_ble.h"
#include "eq3nos_control.h"
#include "eq3_target_number.h"
#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/select/select.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/**
 * @file
 * @brief ESPHome custom component for EQ3 BLE thermostatic valves.
 */

/** @defgroup eq3_component 1. ESPHome integration
 *  @brief Entities and component lifecycle (Home Assistant <-> ESPHome).
 */

/** @defgroup eq3_ble 2. BLE transport
 *  @brief Connection management, notifications, read/write operations and retries.
 */

/** @defgroup eq3_protocol 3. EQ3 protocol
 *  @brief EQ3 frame encoding/decoding and command definitions.
 */


namespace esphome {
namespace eq3nos_bt {

/**
 * @brief Main ESPHome component for EQ3 valves.
 * @ingroup eq3_component
 */
class EQ3NOS  
  : public ble_client::BLEClientNode, 
    public Component {

 public:      
		void setup() override;
		void loop() override;    
        
        /**
        * @brief Handle BLE client events from ESP-IDF.
        * @ingroup eq3_ble
        * @note Main BLE entry point used to drive connection and notifications.
        */
		void gattc_event_handler(esp_gattc_cb_event_t event,
			 esp_gatt_if_t gatt_if,
			 esp_ble_gattc_cb_param_t *param) override;

        // ----- Basic identification -----
		void set_name(const std::string &name) { name_ = name; }
		void set_mac_address(const std::string &mac) { mac_address_ = mac; }
		
        ble_client::BLEClient* ble() { return ble_client::BLEClientNode::parent_; } 

        // eq3-bench patch D (26-9-2026): getter voor de vacation-temperatuur
        // (laatst bevestigde target); fallback-filters zitten in de aanroeper.
        float get_target_temperature() const { return target_temperature; }

        // ----- Entity bindings (wiring from Python/YAML) -----
        // These setters are intentionally undocumented (simple pointer wiring).
        void set_serial_sensor(text_sensor::TextSensor *s) { eq3_serial_sensor_ = s; }
        void set_firmware_sensor(text_sensor::TextSensor *s) { eq3_firmware_sensor_ = s; }
        void set_watchdog_sensor(binary_sensor::BinarySensor *sens) { watchdog_sensor_ = sens; }

        void set_valve_position_sensor(sensor::Sensor *sensor) { valve_position_sensor_ = sensor;}
        void set_current_temperature_sensor(sensor::Sensor *sensor) {current_temperature_sensor_ = sensor;}	
        void set_target_temperature_sensor(sensor::Sensor *sensor) { target_temperature_sensor_ = sensor;}
        void set_battery_low_sensor(binary_sensor::BinarySensor *sensor) { battery_low_sensor_ = sensor;}
        void set_recovery_counter_sensor(sensor::Sensor *sens) { recovery_counter_sensor_ = sens; }

        void set_target_number(EQ3TargetNumber *num) { 
		    target_number_ = num;
            num->set_parent(this);
        }

        void set_mode_select(EQ3ModeSelect *sel);
		void set_lock_switch( EQ3LockSwitch *sw) { lock_switch_ = sw; }
		void set_boost_switch(EQ3BoostSwitch *sw) { boost_switch_ = sw; }
		void set_off_switch(EQ3OffSwitch *sw) { off_switch_ = sw; }

        // ----- Public commands (actions) -----
        /**
        * @brief Set the target temperature on the valve.
        * @param celsius Target temperature in °C.
        * @note BLE transmission may fail due to timeout/disconnect; state sync may be deferred.
        * @ingroup eq3_protocol
        */
        void set_target_temperature(float celsius);

        /// @brief Request boost mode on/off. 
        /// @ingroup eq3_protocol 
		void request_boost(bool state);

        /// @brief Request off mode on/off. 
        /// @ingroup eq3_protocol
		void request_off(bool state);

        // ----- Publishing to HA/ESPHome entities -----
        void publish_eq3_identification( std::string &sn,  std::string &fw);
		void publish_base_field(uint8_t vp, float tt, Eq3Mode mode, bool lm, bool bt, bool ws, bool bl);
        void publish_recovery_counter_sensor(uint16_t rc_);

        // ----- Internal messaging -----
        void send_command_to_ble_transport(const ConnCommand &cmd);
		void send_replay_to_control(const std::vector<uint8_t> &msg);

        // ----- Time -----
        void set_time_source(esphome::time::RealTimeClock *t) {this->time_source_ = t; }
        esphome::ESPTime get_eq3_time();

        // Expose control (if used externally)
        EQ3Control control_;

protected:
        // Identity / state
		std::string name_;
		std::string mac_address_;
		std::string serial_number_;
		std::string eq3_firmware_;

         // Measurements/state snapshot
        uint8_t valve_percent{0};
        float current_temperature{0.0f};
        float target_temperature{20.0f};

        Eq3Mode mode_status{};
        bool lock_mode{};
        bool windows{};
        bool boost{};
        bool battery_low{};

        bool watchdog_state_{false};
        uint32_t recovery_counter_{0};       
		float setpoint_;

	    // Bound entities
        text_sensor::TextSensor *eq3_serial_sensor_{nullptr};	
		text_sensor::TextSensor *eq3_firmware_sensor_{nullptr};
        binary_sensor::BinarySensor *watchdog_sensor_{nullptr};   
        binary_sensor::BinarySensor *battery_low_sensor_{nullptr};  
		sensor::Sensor *valve_position_sensor_{nullptr};
		sensor::Sensor *current_temperature_sensor_{nullptr};
   		sensor::Sensor *target_temperature_sensor_{nullptr};
        sensor::Sensor *recovery_counter_sensor_{nullptr};      

        EQ3ModeSelect *mode_select_{nullptr};
        EQ3LockSwitch *lock_switch_{nullptr};
		EQ3TargetNumber *target_number_{nullptr};
		EQ3BoostSwitch *boost_switch_{nullptr};
		EQ3OffSwitch *off_switch_{nullptr};
        
        // Time
		esphome::time::RealTimeClock *time_source_{nullptr};

        void module_init();

private:
		EQ3NOSBLE ble_transport_;
        bool module_to_init = true;
        bool ble_client_init_error = false;
      
};

}  // namespace eq3nos_bt
}  // namespace esphome
