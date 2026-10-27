#include "eq3nos_log.h"
#include "eq3nos_ble.h"
#include "eq3nos_bt.h"
#include "esphome/core/log.h"

namespace esphome {
namespace eq3nos_bt {

static const char *TAG = "eq3nos_ble";


// Helper per accedere ai protected di BLEClientBase
struct FriendClient : public esphome::esp32_ble_client::BLEClientBase {
    using esphome::esp32_ble_client::BLEClientBase::gattc_if_;
    using esphome::esp32_ble_client::BLEClientBase::conn_id_;
    using esphome::esp32_ble_client::BLEClientBase::remote_bda_;
};

// ============ Handle ESP-IDF GATTC events  ====================
void EQ3NOSBLE::on_ble_event(esp_gattc_cb_event_t event,
                                 esp_gatt_if_t gatt_if,
                                 esp_ble_gattc_cb_param_t *param) {
	switch (event) {
		case ESP_GATTC_CONNECT_EVT: {break;} //EQ3_D(TAG, "I2NOS --> Callback - ESP_GATTC_CONNECT_EVT");
		case ESP_GATTC_OPEN_EVT: {break;} //EQ3_D(TAG, "I2NOS --> Callback - ESP_GATTC_OPEN_EVT");
		case ESP_GATTC_SEARCH_RES_EVT: {break;} //EQ3_D(TAG, "I2NOS --> Callback - ESP_GATTC_OPEN_EVT");
		case ESP_GATTC_CFG_MTU_EVT: {break;} //EQ3_D(TAG, "MTU configurato: %d (status=%d)", param->cfg_mtu.mtu, param->cfg_mtu.status);
		case ESP_GATTC_REG_FOR_NOTIFY_EVT: {break;} //EQ3_D(TAG, "status=%d handle=0x%04x", param->reg_for_notify.handle);
		case ESP_GATTC_WRITE_DESCR_EVT: {break;} //EQ3_D(TAG, "Descriptor scritto (handle=%04X)", param->write.handle);
		case ESP_GATTC_WRITE_CHAR_EVT: {break;} //EQ3_D(TAG, "WRITE_CHAR_EVT");
		case ESP_GATTC_SEARCH_CMPL_EVT: {
			this->on_services_resolved_manual();// Evita discovery manuale: usiamo gli handle noti
			break;		
		}		
		case ESP_GATTC_NOTIFY_EVT: {
			//EQ3_D(TAG, "ricevuto i dati li ruoto per elaborazione, handle=0x%04X len=%d", param->notify.handle, param->notify.value_len);
			this->last_notify_data_.assign(param->notify.value,	param->notify.value + param->notify.value_len);
			local_copy = this->last_notify_data_;
			this->last_notify_data_.clear();
			this->rcv_replay = true;
			break;
		}
		
		default: {
		 	EQ3_D(TAG, "-----gattc_event_handler: unhandled event=%d", event);
		break;
		}
	}
}

void EQ3NOSBLE::send_replay_to_app() { this->parent_->send_replay_to_control(local_copy); }

void  EQ3NOSBLE::on_services_resolved_manual() {
 // Handle noti del servizio EQ3 (dati dal log proxy)
	this->write_handle_  = 0x0411;  // characteristic di scrittura
	this->notify_handle_ = 0x0421;  // characteristic di notifica
	this->cccd_handle_   = 0x0430;  // descriptor CCCD
		
     // Crea oggetti "virtuali" per coerenza con il codice esistente
     // eq3-bench patch E (26-9-2026): allocate once, reuse afterwards.
     // HEAD allocated two BLECharacteristic objects on every SEARCH_CMPL_EVT
     // (i.e. per BLE session) and never freed them.
    if (this->write_ch_ == nullptr) {
        this->write_ch_ = new esphome::esp32_ble_client::BLECharacteristic();
        this->write_ch_->handle = write_handle_;
    }
    if (this->notify_ch_ == nullptr) {
        this->notify_ch_ = new esphome::esp32_ble_client::BLECharacteristic();
        this->notify_ch_->handle = notify_handle_;
    }
    
    this->resolved = true;

   // EQ3_D(TAG, "Caratteristiche inizializzate manualmente: write=0x%04X notify=0x%04X",
    //         this->write_ch_->handle, this->notify_ch_->handle);
    return;
}

void EQ3NOSBLE::after_connected_() {

    auto *cli = this->parent_->ble();
    if (!cli) {
        ESP_LOGE(TAG, "BLEClient unavailable");
        return;
    }

    // Serve ancora FriendClient solo per remote_bda_
    auto *base = static_cast<FriendClient *>(
        static_cast<esphome::esp32_ble_client::BLEClientBase *>(cli)
    );

    // 1) Registriamo la notifica
    esp_err_t err = esp_ble_gattc_register_for_notify(
        cli->get_gattc_if(),      // come send_command
        base->remote_bda_,        // unico campo non pubblico
        this->notify_handle_
    );

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "register_for_notify failed: %s", esp_err_to_name(err));
        return;
    }

    // 2) Attiviamo il CCCD
    uint8_t notify_en[2] = {0x01, 0x00};

    err = esp_ble_gattc_write_char_descr(
        cli->get_gattc_if(),      // come send_command
        cli->get_conn_id(),       // come send_command
        this->cccd_handle_,
        sizeof(notify_en),
        notify_en,
        ESP_GATT_WRITE_TYPE_RSP,
        ESP_GATT_AUTH_REQ_NONE
    );

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "write_char_descr failed: %s", esp_err_to_name(err));
    } 
}

void EQ3NOSBLE::con_task_init(){

    if (!ble_transport_queue_) 
        ESP_LOGE(TAG, "Failed to create connection queue");
    if (ble_transport_queue_ == nullptr) { 
        ble_transport_queue_ = xQueueCreate(MAX_CONN_QUEUE, sizeof(ConnCommand));
    } 
   this->connection_status_ = ConnectionStatus::WAIT_STATE;
} 

// Timer of this task
void EQ3NOSBLE::increment_timer_connect() { this->timer_connect++; this->ble_transport_watchdog++;}

// This Task main loop 
void EQ3NOSBLE::connect_task(){	
	
	this->command_dequeuer();
	this->session_Processor();
    this->connectionWatchdog();
}

// Async command queuer for task ble connection
bool EQ3NOSBLE::command_queuer(const ConnCommand &cmd) {

    EQ3_D(TAG, "Connect task received command type=%d, len=%d from application task",
        (int)cmd.type, 
        (int)cmd.length);

    if (!ble_transport_queue_) {
        ESP_LOGE(TAG, "Connection queue unavailable");
        return false;
    }
    
   // for (int i = 0; i < cmd.length; i++) {
   //     EQ3_D(TAG, " connect_queuer: data[%d] = 0x%02X", i, cmd.data[i]);
   // }
    ConnCommand copy = cmd;
    if (xQueueSendToBack(ble_transport_queue_, &copy, pdMS_TO_TICKS(10)) != pdTRUE) {
        EQ3_D(TAG, "Fail to queue command");
        return false;
    }
    EQ3_D(TAG, "Command queued");
    return true;
}

// Async command dequeuer for task ble connection
int  EQ3NOSBLE::command_dequeuer(){	

    ConnCommand cmd;
    bool conncommand_received = false;
    if( !this->conn_task_busy_){
        conncommand_received = false;
		if (xQueueReceive(ble_transport_queue_, &cmd, 0) == pdTRUE) { 
			conncommand_received = true;
		}
		
		if(conncommand_received){
            EQ3_D(TAG, "Connection task command dequeued");
		    conncommand_received = false;
		    set_task_busy();
            if(cmd.type == ConnCommandType::SEND_RAW)	{ 
                session_RAW(cmd.data,cmd.length);
            }	  
		}		
    }     
 	return 0;
}

// send raw data to the valve
void EQ3NOSBLE::session_RAW(const uint8_t *data, size_t len) {
    if (len > sizeof(temp_cmd_data))
        len = sizeof(temp_cmd_data);
   
    memcpy(temp_cmd_data, data, len);
    temp_cmd_len = len;
    this->connection_status_ = ConnectionStatus::START_SESSION;
    this->timer_connect = 0;
    this->ble_transport_watchdog = 0;


    /*std::string hex; char buf[6];
    for (auto b : temp_cmd_data) {snprintf(buf, sizeof(buf), "%02X ", b); hex += buf;}
    EQ3_D(TAG, "Session_raw: %s", hex.c_str()); */	
    
}

// State machine
void  EQ3NOSBLE::session_Processor(){	

    bool now_connected = parent_->ble()->connected();
    switch(this->connection_status_) {
        case ConnectionStatus::SEND_COMMAND: 
        case ConnectionStatus::AFTER_CONNECTED: 
        case ConnectionStatus::WAIT_RESOLV:
        case ConnectionStatus::CONNECTED_SUCCEFULLY:
        case ConnectionStatus::WAIT_REPLAY:
        case ConnectionStatus::SEND_REPLAY_TO_APP: 
        case ConnectionStatus::WAIT_TO_DISCONNECT:         
             if(!now_connected) 
                connection_status_ = ConnectionStatus::CONNECT_ERROR;
            break; 
        case ConnectionStatus::WAIT_STATE:
            break;
        default:
            break;
    }
    
    switch(this->connection_status_){
        case ConnectionStatus::WAIT_TO_DISCONNECT: 
        		//parent_->ble()->disconnect(); // avvio tentativo 
        		//reset_task_busy(); //per sicurezza 
        		//connection_status_ = ConnectionStatus::WAIT_STATE;
            this->disconnect_delay++; 
            if(disconnect_delay > DISCONNECT_NOW) {
                parent_->ble()->disconnect(); // avvio tentativo 
                reset_task_busy(); //per sicurezza 
                EQ3_D(TAG, "Disconnect delay expired");
                connection_status_ = ConnectionStatus::WAIT_STATE;
           }
        break;
        case ConnectionStatus::SEND_REPLAY_TO_APP: 
            this->send_replay_to_app(); 
            connection_status_ = ConnectionStatus::WAIT_TO_DISCONNECT; 
            this->reset_task_busy();      
          //  EQ3_D(TAG, "Risposta inviata"); 
        break;
        case ConnectionStatus::WAIT_REPLAY:      
            if(this->rcv_replay)
                connection_status_ = ConnectionStatus::SEND_REPLAY_TO_APP;
            //EQ3_D(TAG, "Attendo risposta");
        break;
    	case ConnectionStatus::SEND_COMMAND:      
            this->send_command();
            connection_status_ = ConnectionStatus::WAIT_REPLAY;
            //EQ3_D(TAG, "Invio comando");
        break;
    	case ConnectionStatus::AFTER_CONNECTED:  
            this->after_connected_();
            connection_status_ = ConnectionStatus::SEND_COMMAND; 
           // EQ3_D(TAG, "Abitito notify");
        break;
        case ConnectionStatus::WAIT_RESOLV:
            if(this->resolved)
                connection_status_ = ConnectionStatus::AFTER_CONNECTED;
            //EQ3_D(TAG, "Wait per resolve");
        break; 
        case ConnectionStatus::CONNECTED_SUCCEFULLY:
            connection_status_ = ConnectionStatus::WAIT_RESOLV;
           // EQ3_D(TAG, "Connessione attiva");
        break;
        case ConnectionStatus::CONNECTION_REQUESTED:
            if(now_connected)
                connection_status_ = ConnectionStatus::CONNECTED_SUCCEFULLY;
           // EQ3_D(TAG, "Wait per connessione attiva");
        break;
        case ConnectionStatus::START_SESSION:
        	  //EQ3_D(TAG, "Start Session");
            this->resolved = false;
            this->rcv_replay = false;
            this->disconnect_delay = 0;
            if(!now_connected){
                parent_->ble()->connect(); // avvio tentativo
                connection_status_ = ConnectionStatus::CONNECTION_REQUESTED;
            } else
            	connection_status_ = ConnectionStatus::SEND_COMMAND;
            //EQ3_D(TAG, "Avvio connessione");
        break;
         case ConnectionStatus::GENERIC_DISCONNECT:
            reset_task_busy();
            connection_status_ = ConnectionStatus::WAIT_STATE;
            EQ3_D(TAG, "Disconnesso");
        break;
        case ConnectionStatus::CONNECT_ERROR:
            reset_task_busy();
            connection_status_ = ConnectionStatus::WAIT_STATE;
            EQ3_D(TAG, "Connection error");
        break;
       
        case ConnectionStatus::WAIT_STATE:
            this->disconnect_delay = 0;
        break;
        default:
        break;
    }
}

// Invio Comando alla valvola
void EQ3NOSBLE::send_command() {

    // Verifica write_ch_
    if (!this->write_ch_) {
        ESP_LOGW(TAG, "write channel unavailable");
        return;
    }
    bool now_connected = parent_->ble()->connected();
    if (!now_connected) {
        ESP_LOGW(TAG, "Not connected, command ignored");
        return;
    }

    // Validazione lunghezzadisconnect_delay
    if (this->temp_cmd_len == 0 || this->temp_cmd_len > 23) {
        ESP_LOGW(TAG, "Command lenght error (%i)", temp_cmd_len);
        return;
    }

    // Otteniamo il BLEClient tramite il parent EQ3BT
    auto *cli = this->parent_->ble();
    if (!cli) {
        ESP_LOGE(TAG, "BLEClient unavailable");
        return;
    }

    // Eseguiamo la scrittura sulla characteristic
    esp_err_t err = esp_ble_gattc_write_char(
        cli->get_gattc_if(),
        cli->get_conn_id(),
        this->write_ch_->handle,
        temp_cmd_len,
        (uint8_t *)temp_cmd_data,
        ESP_GATT_WRITE_TYPE_RSP,
        ESP_GATT_AUTH_REQ_NONE
    );

    // Logging
    if (err == ESP_OK) {
        
        std::string hex;
        for (size_t i = 0; i < temp_cmd_len; i++) {
            char buf[4];
            sprintf(buf, " %02X", temp_cmd_data[i]);
            hex += buf;
        }
        EQ3_D(TAG, "send_command - handle=0x%04X len=%d data:%s",
            this->write_ch_->handle, (int)temp_cmd_len, hex.c_str());
       
    } else {
        ESP_LOGW(TAG, "Send command error (err=%d)", err);
    }
}

// 
void EQ3NOSBLE::connectionWatchdog() {
    
    if(this->conn_task_busy_) {
        if(this->ble_transport_watchdog > WATCHDOG_TIMEOUT) {
            EQ3_D(TAG, "WATCHDOG_TIMEOUT");
            reset_task_busy();
            connection_status_ = ConnectionStatus::WAIT_STATE;  
            local_copy.push_back(0xFF);
            this->send_replay_to_app(); 
            this->timer_connect = 0;
            this->ble_transport_watchdog = 0;
        }
    }
}
/*
*
*/
void  EQ3NOSBLE::set_task_busy() {this->conn_task_busy_ = true;}
void  EQ3NOSBLE::reset_task_busy() {this->conn_task_busy_ = false;}

/*
*
*/
void EQ3NOSBLE::set_parent(EQ3NOS *parent) {
    this->parent_ = parent;
}


}  // namespace eq3nos
}  // namespace esphome
