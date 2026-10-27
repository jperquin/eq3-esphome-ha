#pragma once

#include <vector>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esphome/components/ble_client/ble_client.h"
#include "eq3nos_types.h"

/**
 * @file
 * @brief BLE transport layer for EQ3 valves.
 *
 * This module encapsulates all BLE-related operations for EQ3 devices:
 * - connection lifecycle (connect/disconnect)
 * - GATTC event handling and notification enabling (CCCD)
 * - sending raw messages and forwarding notifications to the application layer
 * - command serialization via a FreeRTOS queue + watchdog
 *
 * The EQ3 protocol parsing/encoding is handled by the protocol layer (EQ3Control).
 *
 * @ingroup eq3_ble
 */

#define DISCONNECT_NOW 3 // disconnecti dopo 3 sec
#define WATCHDOG_TIMEOUT 15 // Errore dopo 15 sec senza risposta BLE

namespace esphome {
namespace eq3nos_bt {

class EQ3NOS;  // forward declaration

/**
 * @brief Internal BLE session state machine.
 * @note These are transport-layer states (not EQ3 protocol states).
 * @ingroup eq3_ble
 */
enum class ConnectionStatus : uint8_t {
    WAIT_STATE,
    START_SESSION,
    CONNECTION_REQUESTED,
    CONNECTED_SUCCEFULLY,
    WAIT_RESOLV,
    AFTER_CONNECTED,
    SEND_COMMAND,
    WAIT_REPLAY,
    SEND_REPLAY_TO_APP,
    WAIT_TO_DISCONNECT,
    CONNECT_ERROR,
    GENERIC_DISCONNECT,
};

/**
 * @brief BLE transport implementation for EQ3 valves.
 *
 * This class serializes BLE operations and exposes a minimal interface used by the
 * parent component (EQ3NOS) and by the protocol/application layer.
 *
 * Notes:
 * - EQ3 service discovery may fail; known handles are assigned manually after search complete.
 * - Notifications are enabled via CCCD after connection.
 *
 * @ingroup eq3_ble
 */
class EQ3NOSBLE {
public:
    /// @brief Bind the parent component used to send messages upstream (EQ3NOS).
    void set_parent(EQ3NOS *parent);

    /**
     * @brief Handle ESP-IDF GATTC events.
     *
     * Main BLE callback entry point. Handles service resolution completion and
     * notification reception.
     *
     * @param event ESP-IDF GATTC event type.
     * @param gatt_if GATT client interface.
     * @param param Event parameters.
     */
    void on_ble_event(esp_gattc_cb_event_t event,
	    esp_gatt_if_t gatt_if,
        esp_ble_gattc_cb_param_t *param);

    /**
     * @brief Assign EQ3 GATT handles manually when discovery is unreliable.
     *
     * EQ3 valves may fail standard service discovery. This function assigns known handles
     * for write/notify characteristics and CCCD, and marks the transport as resolved.
     *
     * @note Handles are device/firmware dependent; these values come from observed logs.
     */
      void on_services_resolved_manual(); 

     /// @brief Initialize the BLE transport task state and create the command queue.
	void con_task_init();
    
     /// @brief Main transport task loop (dequeue command, process session, watchdog).
    void connect_task();

     /**
     * @brief Enqueue a transport command to be executed by the BLE task.
     * @param cmd Command descriptor (type + payload).
     * @return true if the command was queued, false if queue unavailable/full.
     */
    bool command_queuer(const ConnCommand &cmd);

    void increment_timer_connect(); // Timer per gestione task
  
   
    std::vector<uint8_t> last_notify_data_;
	std::vector<uint8_t> local_copy;

private:
    EQ3NOS *parent_{nullptr};
    QueueHandle_t ble_transport_queue_{nullptr};
    esphome::esp32_ble_client::BLECharacteristic *write_ch_ = nullptr;
	esphome::esp32_ble_client::BLECharacteristic *notify_ch_ = nullptr;

     /**
     * @brief Enable notifications after a successful connection.
     *
     * Registers for notifications and writes the CCCD descriptor to enable notify on the
     * EQ3 notify characteristic.
     */
    void after_connected_();

    void wait_to_reconnect_set();
    /**
     * @brief Watchdog for stuck BLE sessions.
     *
     * If the task is busy for too long, it resets the session and sends a synthetic error
     * reply upstream to unblock the application layer.
     */
    void connectionWatchdog();
     /**
     * @brief Forward the last received notification payload to the application layer.
     *
     * Sends the locally buffered notification payload to the parent component, which will
     * dispatch it to the protocol parser.
     */
    void send_replay_to_app();
     /**
     * @brief Send the currently staged raw command to the valve (write characteristic).
     *
     * @note Requires an active BLE connection and a resolved write handle.
     */
    void send_command();
    int command_dequeuer(); 
    void session_Processor();
	void session_RAW(const uint8_t *data, size_t len);
    ConnectionStatus connection_status_;
    uint8_t temp_cmd_data[16];
    size_t temp_cmd_len = 0;
	uint32_t timer_connect = 0;
    uint32_t ble_transport_watchdog = 0;
    bool resolved = false;
    bool rcv_replay = false;
    bool conn_task_busy_ = false;
    uint32_t disconnect_delay = 0;
    void set_task_busy();
    void reset_task_busy();
    std::vector<uint8_t> command_buffer_;
    static constexpr size_t MAX_CONN_QUEUE = 20; // numero max comandi in coda
    uint16_t service_handle_{0};
    uint16_t write_handle_{0};
    uint16_t notify_handle_{0};
    uint16_t cccd_handle_{0}; 
};

}  // namespace eq3nos
}  // namespace esphome
