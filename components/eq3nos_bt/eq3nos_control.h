#pragma once

#include <cstdint>
#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/core/log.h"
#include <sstream>
#include <iomanip>
#include "eq3nos_adapters.h"
#include "eq3nos_session.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/time/real_time_clock.h"

/**
 * @file
 * @brief EQ3 protocol and application control layer.
 *
 * Implements:
 * - A small application state machine that schedules commands and parses responses.
 * - EQ3 protocol message building/parsing (setpoint, boost, lock, time sync, status read).
 * - A FreeRTOS queue used to serialize commands.
 *
 * @ingroup eq3_protocol
 */

/// @brief Seconds between periodic status updates.
#define STATUSTIMER 3600

/// @brief Seconds between periodic information queries.
#define NORMALQUERY 300


/**
 * @brief Operating mode reported by the EQ3 valve.
 * @ingroup eq3_protocol
 */
enum class Eq3Mode : uint8_t {
    // Keep values aligned to the EQ3 protocol specification.
    // Add brief docs per value only if not obvious.
    AUTO = 0,       ///< Automatic/scheduled mode.
    MANUAL = 1,     ///< Manual setpoint mode.
    VACATION = 2,   ///< Vacation setpoint mode.
};
                    
namespace esphome {
namespace eq3nos_bt {

class EQ3NOS;   // forward declaration

/**
 * @brief Internal application state machine steps.
 * @note These are not EQ3 protocol values; they represent this component's workflow.
 */
enum class ApplicationStatus {
    APP_WAIT,           ///< Idle state.
    GET_SERIAL_NUMBER,  ///< Request device identification.
    NOW_UPDATE_TIME,    ///< Send time sync to the valve.
    READ_STATUS,         ///< Request base status/telemetry.    
    SET_MODE_AUTO,
    SET_MODE_MANUAL,
    SET_MODE_VACATION,
    SET_LOCK_SWITCH,  
    GET_CURR_TEMPERATURE,
    SET_TARGET_TEMPERATURE,
    SET_BOOST_SWITCH,
};

/** @brief High-level commands enqueued by the component/user actions. */
enum class AppCommandType {
    GET_INFO,
    UPDATE_TIME,
    SET_AUTO,
    SET_MANUAL,
    SET_VACATION,
    SET_LOCK,
    SET_TARGET_TEMP,
    SET_BOOST,
    GET_CURR_TEMP,
    READ_STATUS
};

/**
 * @brief EQ3 "set time" command payload.
 * @note Packed structure sent over BLE; all fields are BCD/decimal as specified by EQ3.
 */
#pragma pack(push, 1)
struct Eq3SetTime {
    uint8_t cmd;     ///< Command ID (0x03).
    uint8_t year;    ///< Year % 100.
    uint8_t month;   ///< 1–12.
    uint8_t day;     ///< 1–31.
    uint8_t hour;    ///< 0–23.
    uint8_t minute;  ///< 0–59.
    uint8_t second;  ///< 0–59.
};
#pragma pack(pop)

/**
 * @brief Parsed base status response from the valve.
 * @note Field meanings are derived from observed EQ3 BLE frames.
 */
#pragma pack(push, 1)
struct Eq3StatusResponse {
    uint8_t replaycode;         ///< Typically 0x02.
    uint8_t always1;            ///< Typically 0x1.
    uint8_t mode;               ///< Raw mode value (see Eq3Mode).
    uint8_t valve_percent;      ///< Valve position in percent (0–100).
    uint8_t unknown;            ///< Unknown/reserved.
    uint8_t target_temp_x2;     ///< Target temperature * 2 (e.g. 21.0°C -> 42).
    uint8_t vm_day;             ///< Vacation mode day.
    uint8_t vm_year;            ///< Vacation mode year.
    uint8_t vm_month;           ///< Vacation mode month.
    uint8_t vm_time;            ///< Vacation mode time.
    uint8_t temperature_ow;     ///< Window-open temperature (encoding depends on device).
    uint8_t comfort_temperature;///< Comfort temperature (device encoding).
    uint8_t eco_temperature;    ///< Eco temperature (device encoding).
    uint8_t temperature_offset; ///< Temperature offset (device encoding).
};
#pragma pack(pop)

struct ApplCommand {
    AppCommandType type;
    bool status;  // per eventuali comandi ON/OFF
    uint8_t value; // Per temperatura
};

/**
 * @brief EQ3 application control and protocol dispatcher.
 *
 * Owns a command queue and an internal state machine that:
 * - accepts high-level commands (set mode, setpoint, boost, lock, time sync)
 * - serializes BLE requests
 * - parses incoming messages and updates cached valve status
 *
 * The actual BLE I/O is performed by the parent component (EQ3NOS).
 *
 * @ingroup eq3_protocol
 */
class EQ3Control {
    public:
        /// @brief Bind the parent component used for BLE send/publish operations.
        void set_parent(EQ3NOS *parent);

        /// @brief Initialize FreeRTOS queue and internal task state.
        void app_task_init();

        /**
        * @brief Feed an incoming BLE message to the protocol parser.
        * @param msg Raw BLE payload.
        */
        void handle_incoming_message(const std::vector<uint8_t> &msg);

        /**
        * @brief Enqueue an application command to be executed by the state machine.
        * @return true if enqueued, false if queue is full or not initialized.
        */
        bool applic_queuer(ApplCommand cmd);

        /// @brief Periodic control task (called by parent loop or task runner).
        void app_task();

        /// @brief Force a status update workflow.
        void update_status();
        bool request_identification();
        void transport_failed(const char *reason);

        // High-level command helpers     
        void cmd_set_mode(Eq3Mode mode);
        void cmd_lock(bool state);
        void cmd_boost(bool state); 
        void send_setpoint_(uint8_t target);

        void increment_timer_applic();

    protected:  
        bool rcvd_get_info();
        uint8_t eq3_valve_status_;
        void set_eq3_time();
        
	private:

		EQ3NOS *parent_{nullptr};   // puntatore al "padre" (EQ3NOS)	
		QueueHandle_t application_queue_{nullptr};
		ApplicationStatus application_status_ = ApplicationStatus::APP_WAIT;
        Eq3StatusResponse  valve_base_status_;
        int app_dequeuer(); 
        void send_mode(Eq3Mode mode);   
        void send_target_temp(uint8_t temp);
        void send_lock(bool state);
        void send_boost(bool state);
        void rcvd_base_status(ApplicationStatus tipo);
        void get_info();
		void get_current_temp();
        void parse_msg();
		
		static constexpr size_t MAX_APP_QUEUE = 20; // numero max comandi in coda
		
		//bool ask_info_= false;	// 
	    //bool rcv_info_ = false; // flag di attesa risposta
	    
		bool app_task_busy_ = false; 		
		uint32_t timer_appl = 0;
		uint32_t timer_ora = 0;
		uint32_t timer_query = 0;
        uint32_t timer_recovery = 0;
        uint32_t command_started_ms_{0};
        bool identification_queued_{false};
        IdentificationRetry identification_retry_;
        uint32_t recovery_counter = 0;
        std::vector<uint8_t> local_msg;
        int local_msg_size = 15;
        bool rcv_msg_ready = false;
		  
};

}  // namespace eq3nos_bt
}  // namespace esphome
