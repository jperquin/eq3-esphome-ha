#include "eq3nos_bt.h"
#include "esphome/core/log.h"
#include "eq3nos_types.h"

namespace esphome {
namespace eq3nos_bt {

static const char *TAG = "eq3nos_bt";

// =================== EQ3NOS ===================
void EQ3NOS::setup() {

    this->ble_client_init_error = true;
    if (ble_client::BLEClientNode::parent_ == nullptr) {
        this->ble_client_init_error = true;
    } else 
        this->ble_client_init_error = false;

    // Registro il nodo sul BLE client
    this->set_ble_client_parent(ble_client::BLEClientNode::parent_);
    ble_client::BLEClientNode::parent_->register_ble_node(this);

    control_.set_parent(this);
    control_.app_task_init();

    // Inizializzo connessione
    ble_transport_.set_parent(this);      // parent per callback di connessione
    ble_transport_.con_task_init();
    
    this->module_to_init = true;
}

// Set flag for init from main loop
void EQ3NOS::module_init() {
    //delay(3000);
    this->module_to_init = false;
    if (ble_client_init_error)
        ESP_LOGE(TAG, "Parent BLE not set!");   
    EQ3_D(TAG, "📡 EQ3NOS: setup complited"); 
}

// Main loop
void EQ3NOS::loop() {
    if(this->module_to_init == true)
        module_init();
    
    static uint32_t last_attempt = 0;
    
	if (millis() - last_attempt < 1000) return; // timer 1 secondo
    last_attempt = millis();
   
    this->control_.increment_timer_applic();
    this->ble_transport_.increment_timer_connect();
    
    this->control_.app_task();
    this->ble_transport_.connect_task();
}

void EQ3NOS::send_command_to_ble_transport(const ConnCommand &cmd ) {  
		//this->get_eq3_time();
    EQ3_D(TAG, "Received command from application");
    bool error = ble_transport_.command_queuer(cmd);  
    EQ3_D(TAG, "%s command to connection.", error? "queued": "not queued" ); 
}

void EQ3NOS::send_replay_to_control(const std::vector<uint8_t> &msg) {
    EQ3_D(TAG, "Received replay from connection");
    this->control_.handle_incoming_message(msg);
    EQ3_D(TAG, "replay received from application" ); 
}

// =================== MODE SELECTOR ===================
void EQ3ModeSelect::control(const std::string &value) {

    if (!parent_) {
        ESP_LOGW(TAG, "Mode select parent not set!");
        return;
    } 
    if (value == "auto") {
        parent_->control_.cmd_set_mode(Eq3Mode::AUTO);
    } else if (value == "manual") {
        parent_->control_.cmd_set_mode(Eq3Mode::MANUAL);
    } else if (value == "vacation") {
        parent_->control_.cmd_set_mode(Eq3Mode::VACATION);
    }
    //mode_select_->publish_state(value);
}

// =================== LOOK SWITCH ===================
void EQ3LockSwitch::write_state(bool state) {
    if (!parent_) {
        ESP_LOGW(TAG, "Lock switch parent not set!");
        return;
    }
   parent_->control_.cmd_lock(state);
   this->publish_state(state);
}

// =================== BOOST SWITCH ===================
void EQ3BoostSwitch::write_state(bool state) {
   // ESP_LOGI(TAG, "Boost switch changed state: %s", state ? "ON" : "OFF");
    if (!parent_) {
        ESP_LOGW(TAG, "Boost switch parent not set!");
        return;
    }
    parent_->control_.cmd_boost(state);
    this->publish_state(state);
}

// =================== OFF SWITCH ===================
void EQ3OffSwitch::write_state(bool state) {
  EQ3_D(TAG, "Off switch changed state: %s", state ? "ON" : "OFF");
  if (!parent_) {
    ESP_LOGW(TAG, "Off switch parent not set!");
    return;
  }

  //parent_->request_off(state);
}

void EQ3NOS::set_mode_select(EQ3ModeSelect *sel) {
   EQ3_D(TAG, "Set mode select");
  mode_select_ = sel;
}

void EQ3NOS::request_boost(bool state) {
  EQ3_D(TAG, " Cosa devo fare qui? Request BOOST state: %s", state ? "ON" : "OFF");

}

// ================= SET TARGET TEMPERATURE =================
void EQ3NOS::set_target_temperature(float celsius) {
    celsius = clamp(celsius, 5.0f, 30.0f);
    uint8_t target = static_cast<uint8_t>(roundf(celsius * 2.0f));
    control_.send_setpoint_(target);

    EQ3_D("EQ3", "Setpoint %.1f °C -> TT = 0x%02X (%d)",
         celsius, target, target);
}

// =========== Handle BLE client events from ESP-IDF ===========
void EQ3NOS::gattc_event_handler  (esp_gattc_cb_event_t event,
                                esp_gatt_if_t gatt_if,
                                esp_ble_gattc_cb_param_t *param) {
    ble_transport_.on_ble_event(event, gatt_if, param);
}

void EQ3NOS::publish_eq3_identification(std::string &sn, std::string &fw) {
    this->serial_number_ = sn;
    this->eq3_firmware_ = fw;
	
   if (eq3_serial_sensor_ != nullptr)
        eq3_serial_sensor_->publish_state(this->serial_number_);
   if (eq3_firmware_sensor_ != nullptr)
        eq3_firmware_sensor_->publish_state(this->eq3_firmware_);
}        

void EQ3NOS::publish_recovery_counter_sensor(uint16_t rc_) {
    if (recovery_counter_sensor_ != nullptr) {
        recovery_counter_sensor_->publish_state(rc_);
    }
}

void EQ3NOS::publish_base_field(uint8_t vp, float tt,  Eq3Mode mode, bool lm, bool bt, bool ws, bool bl){
    this->valve_percent = vp;
    //this->current_temperature = ct;
    this->target_temperature = tt;
    this->mode_status = mode;
    this->lock_mode = lm;
    this->boost = bt;
    this->windows = ws;
	this->battery_low = bl;

    if (mode_select_ != nullptr) {
        switch (mode) {
            case Eq3Mode::AUTO:
                mode_select_->publish_state("auto");
            break;
            case Eq3Mode::MANUAL:
                mode_select_->publish_state("manual");
            break;
            case Eq3Mode::VACATION:
                mode_select_->publish_state("vacation");
            break;
        }
    }

    if (valve_position_sensor_ != nullptr) {
        valve_position_sensor_->publish_state( this->valve_percent);
    } 
    if (target_temperature_sensor_ != nullptr) {
        target_temperature_sensor_->publish_state(this->target_temperature);
    }   
  //  if (current_temperature_sensor_ != nullptr) {
  //      current_temperature_sensor_->publish_state(this->current_temperature);
  //  }

    if (lock_switch_ != nullptr) {
        lock_switch_->publish_state(this->lock_mode);
    }

    if (boost_switch_ != nullptr) {
        boost_switch_->publish_state(bt);
    }

    if (battery_low_sensor_ != nullptr) {
        battery_low_sensor_->publish_state(this->battery_low);
    }
    
    // eq3-bench patch A (26-9-2026): number echo is niet meer one-shot.
    // Elke bevestigde status-frame publiceert de target naar de number,
    // zodat MQTT-discovery state_topic == sensor/target state_topic en de
    // HA-side discovery-remap niet meer nodig is.
    if (target_number_ != nullptr) {
        target_number_->publish_state(this->target_temperature);
    }

    watchdog_state_ = !watchdog_state_;
    if (watchdog_sensor_ != nullptr) {
        watchdog_sensor_->publish_state(watchdog_state_);
    }
}

esphome::ESPTime EQ3NOS::get_eq3_time(){
	auto now = time_source_->now();
	EQ3_D(TAG, "Time now: %04d-%02d-%02d %02d:%02d:%02d",
         now.year, now.month, now.day_of_month,
         now.hour, now.minute, now.second);
         return now;
}

}  // namespace eq3nos_bt
}  // namespace esphome
