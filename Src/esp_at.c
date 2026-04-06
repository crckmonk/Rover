/*
* ESP8266 driver. For now handles MAVLink message processing.
*/
#include "esp_at.h"
#include <string.h>
#include <stdio.h>
#include "printf.h"
#include "mcutils.h"
#include "common/mavlink.h"
#include "control.h"
#include <stdlib.h>



static volatile uint8_t          manual_ctrl_pending = 0;
static mavlink_manual_control_t  pending_manual_ctrl;

/* * Debug utils*/
static const char* ESP_StatusToString(ESP_Status_t status) {
    switch (status) {
        case ESP_OK:          return "OK";
        case ESP_ERROR:       return "ERROR";
        case ESP_BUSY:        return "BUSY";
        case ESP_NO_RESPONSE: return "NO_RESPONSE";
        case ESP_TIMEOUT:     return "TIMEOUT";
        default:                  return "UNKNOWN";
    }
}


/* * PARAMETER DUMMY */

typedef struct {
    char     name[17];
    float    value;
    uint8_t  type;      
} RoverParam_t;

static RoverParam_t params[] = {
    {"SYSID_THISMAV", 1.0f, MAV_PARAM_TYPE_INT32},
};


#define PARAM_COUNT (sizeof(params) / sizeof(params[0]))

void MAVLink_SendParamValue(uint16_t index) {
    if (index >= PARAM_COUNT) return;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_param_value_pack(
        Rover_GetId(), MAV_COMP_ID_AUTOPILOT1, &msg,
        params[index].name,   // param_id (char[16])
        params[index].value,  // param_value (float)
        params[index].type,   // param_type
        PARAM_COUNT,          // param_count — total number
        index                 // param_index — this one's index
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    ESP_QueueMAVLink(&tx_queue, buf, len);
}


#define ESP_CMD(dev, cmd, timeout, step_name) do { \
    DEBUG_PRINTF(DBG_VERBOSE,"[ESP] Step: %s\r\n", step_name); \
    ESP_Status_t _status = ESP_SendATCommand(dev, cmd, timeout); \
    if (_status != ESP_OK) { \
        DEBUG_PRINTF(DBG_VERBOSE,"[ESP] FAILED at '%s': %s\r\n", step_name, ESP_StatusToString(_status)); \
        return ESP_ERROR; \
    } \
} while(0)


static Queue_t tx_queue = {0};
static Queue_t rx_queue = {0};




static uint8_t ESP_QueueMAVLink(Queue_t* queue, uint8_t *data, uint16_t len) {
    return Queue_Enqueue(queue, data, len);
}

static ESP_Status_t ESP_SendMAVLink(ESP_Handler_t *dev, uint8_t *data, uint32_t len) {
    uint8_t cmd[32];
    snprintf((char*)cmd, sizeof(cmd), "AT+CIPSEND=%d", (int)len);
    
    if (ESP_SendATCommand(dev, cmd, 200) != ESP_OK) {
        DEBUG_PRINTF(DBG_ERROR, "[ESP] CIPSEND failed\r\n");
        return ESP_ERROR;
    }
    
    UART_SendData(dev->uart_buffers, data, len);
    
    dev->response_received = 0;
    dev->response_status = ESP_TIMEOUT;
    
    uint32_t tickstart = HAL_GetTick();
    while ((HAL_GetTick() - tickstart) < 500) {
        ESP_ProccessAvailableBytes(dev);
        if (dev->response_received) {
            return dev->response_status;
        }
    }
    
    return ESP_TIMEOUT;
}

static ESP_Status_t ESP_ProcessTxQueue(ESP_Handler_t *dev) {
    QueueItem_t* item = Queue_Peek(&tx_queue);
    if (item == NULL) {
        return ESP_OK; 
    }
    
    ESP_Status_t result = ESP_SendMAVLink(dev, item->data, item->len);
    if (result == ESP_OK) {
        Queue_Dequeue(&tx_queue);
    } else {
        DEBUG_PRINTF(DBG_ERROR, "[ESP] TX Send failed\r\n");
    }
    return result;
}


ESP_Status_t ESP_SendString(ESP_Handler_t* dev, const uint8_t* str){
    UART_SendData(dev->uart_buffers, (uint8_t*)str, strlen(str));
    return ESP_OK;
}

ESP_Status_t ESP_FlushRx(ESP_Handler_t* dev){
    dev->rx_index = 0;
    memset(dev->rx_buffer, 0, sizeof(dev->rx_buffer));
    if(dev->rx_index == 0 && dev->rx_buffer[0] == 0){
        return ESP_OK;
    }  
    return ESP_ERROR;
}

void ESP_Init(ESP_Handler_t* dev, 
                         UART_HandleTypeDef* huart,UART_Buffers_t *uart_buffers) {  
    dev->huart = huart;
    dev->uart_buffers = uart_buffers;  
    dev->rx_index = 0;
    dev->state = ESP_STATE_IDLE;
    dev->response_received = 0;
    dev->response_status = ESP_OK;
    dev->ssid = NULL;
    dev->password = NULL;
    memset(dev->rx_buffer, 0, sizeof(dev->rx_buffer));
}

/**
 * @brief Send AT command to ESP. Command string shouldn't include \r\\n
 * @param dev 
 * @param cmd 
 * @param timeout 
 * @return 
 */
ESP_Status_t ESP_SendATCommand(ESP_Handler_t *dev, const uint8_t* cmd, uint32_t timeout) {
    dev->response_received = 0;
    dev->response_status = ESP_TIMEOUT;
    dev->rx_index = 0;
    
    ESP_FlushRx(dev);

    uint32_t drain_start = HAL_GetTick();
    while (dev->state == ESP_STATE_IPD_DATA && (HAL_GetTick() - drain_start) < 50) {
        ESP_ProccessAvailableBytes(dev);
    }


  uint8_t tx_buff[ESP_MAX_BUFFER_SIZE];

  dev->state = ESP_STATE_AWAIT_RESPONSE;
  if (strlen(cmd) > 0) {
    DEBUG_PRINTF(DBG_VERBOSE, "[ESP] TX: %s\r\n", cmd);
    snprintf((char*)tx_buff, ESP_MAX_BUFFER_SIZE, "%s\r\n", cmd);
    if(UART_SendData(dev->uart_buffers, tx_buff, strlen((const char *)tx_buff)) != UART_OK){
        DEBUG_PRINTF(DBG_ERROR, "[ESP] UART_ERROR Sending a command\r\n");
        return ESP_ERROR;
    }
  }


  uint32_t tickstart = HAL_GetTick();
  while ((HAL_GetTick() - tickstart) < timeout) 
  {
        ESP_ProccessAvailableBytes(dev);

    if (dev->response_received){
        DEBUG_PRINTF(DBG_VERBOSE,"[ESP] Response: %s (status=%d)\r\n", dev->rx_buffer, dev->response_status);
            dev->state = ESP_STATE_IDLE;
            return dev->response_status;
        }
    }
    HAL_Delay(1);   
    dev->state = ESP_STATE_IDLE;
    return ESP_TIMEOUT;
}


void MAVLink_SendAutopilotVersion(void) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint64_t capabilities =
        MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
        MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
        MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED;

    mavlink_msg_autopilot_version_pack(
        Rover_GetId(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        capabilities,   // capabilities bitmask
        1,              // flight_sw_version
        0,              // middleware_sw_version
        0,              // os_sw_version
        0,              // board_version
        NULL,           // flight_custom_version (8 bytes, NULL = zeros)
        NULL,           // middleware_custom_version
        NULL,           // os_custom_version
        0,              // vendor_id
        0,              // product_id
        0,               // uid
        0               // uid2
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    ESP_QueueMAVLink(&tx_queue, buf, len);
}


static void MAVLink_SendCmdAck(uint16_t command, uint8_t result) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    mavlink_msg_command_ack_pack(
        Rover_GetId(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        command,
        result,
        0, 0, 0, 0
    );
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    ESP_QueueMAVLink(&tx_queue, buf, len);
}


static void handle_command_long(ESP_Handler_t *dev, mavlink_message_t* msg) {
    /* TODO: Add handlers for more commands*/
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(msg, &cmd);
    DEBUG_PRINTF(DBG_INFO, "[ESP] [MAV] Handling command: %d\r\n", cmd.command);
    if (cmd.target_system != Rover_GetId()) {
        DEBUG_PRINTF(DBG_INFO, "[ESP] [MAV] Cmd wrong target system\r\n");
        return;
    }
    uint8_t result = MAV_RESULT_FAILED;
    switch(cmd.command) {
        case MAV_CMD_REQUEST_MESSAGE: {
                uint32_t msg_id = (uint32_t)cmd.param1;
                DEBUG_PRINTF(DBG_INFO, "[ESP] [MAV] Message requested: %d\r\n", cmd.param1);
                switch(msg_id){
                    case 148: // AUTOPILOT_VERSION
                        MAVLink_SendCmdAck(cmd.command, MAV_RESULT_ACCEPTED);
                        MAVLink_SendAutopilotVersion();
                        break;
                    case 300: // PROTOCOL_VERSION
                        MAVLink_SendCmdAck(cmd.command, MAV_RESULT_ACCEPTED);
                        //SendProtocolVersion
                        break;
                    case 259: 
                            MAVLink_SendCmdAck(cmd.command, MAV_RESULT_UNSUPPORTED);
                            break;
                    default:
                        MAVLink_SendCmdAck(cmd.command, MAV_RESULT_UNSUPPORTED);
                        break;
                }
        }
        case MAV_CMD_COMPONENT_ARM_DISARM: {
                DEBUG_PRINTF(DBG_INFO, "[ESP] [MAV] Handling ARM/DISARM\r\n");
                bool arm = (cmd.param1 > 0.5f);
            
            if (arm) {
                Rover_Arm();
                result = MAV_RESULT_ACCEPTED;
                DEBUG_PRINTF(DBG_INFO, "[ESP] [MAV] Armed\r\n");
            } else {
                Rover_Disarm();
                result = MAV_RESULT_ACCEPTED;
                DEBUG_PRINTF(DBG_INFO, "[ESP] [MAV] Disarmed\r\n");
            }
            break;
        }   
        case MAV_CMD_DO_SET_MODE: {
            result = MAV_RESULT_ACCEPTED;
            break;
        }
    }
    if(result == MAV_RESULT_ACCEPTED){
        MAVLink_SendCmdAck(cmd.command, result);
    }
    
}


static void ESP_HandleMAVLinkMsg(ESP_Handler_t *dev, mavlink_message_t *msg){
    DEBUG_PRINTF(DBG_VERBOSE, "[ESP] Received MAVLink message ID: %d\r\n", msg->msgid);
            switch(msg->msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(msg, &hb);
                    DEBUG_PRINTF(DBG_INFO, "[ESP] Heartbeat from sysid=%d compid=%d type=%d\r\n",
                           msg->sysid, msg->compid, hb.type);
                    break;
                    }
                case MAVLINK_MSG_ID_COMMAND_LONG: {
                    handle_command_long(dev, msg);
                    break;
                    }
                case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                    DEBUG_PRINTF(DBG_INFO,"[ESP] GCS requesting parameter list\r\n");
                    break;
                    }
                case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
                    DEBUG_PRINTF(DBG_INFO,"[ESP] GCS requesting data stream\r\n");
                    break;
                }
                case MAVLINK_MSG_ID_MANUAL_CONTROL: {
                    mavlink_manual_control_t manual;
                    mavlink_msg_manual_control_decode(msg, &manual);
                    if (manual.target != Rover_GetId()) {break;}
                    pending_manual_ctrl = manual;
                    manual_ctrl_pending = 1;
                    break;
                    }
                default:
                    DEBUG_PRINTF(DBG_INFO, "[ESP]  Unknown message ID: %d\r\n", msg->msgid);
                    break;
            }
}


static void ESP_HandleIPDData(ESP_Handler_t *dev, uint8_t *data, uint16_t len) {
    mavlink_message_t msg;
    mavlink_status_t status;
    DEBUG_PRINTF(DBG_VERBOSE, "[ESP] Received %d bytes: ", len);
     if(DEBUG_LEVEL>=DBG_VERBOSE){
         for (uint16_t i = 0; i < len; i++) {
             printf("%02X ", data[i]);
         }
     }
    DEBUG_PRINTF(DBG_VERBOSE, "\r\n\r\n");
    for (uint16_t i = 0; i < len; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
            /* * Check if data is MAVLink message*/
            ESP_QueueMAVLink(&rx_queue, (uint8_t*)&msg, sizeof(mavlink_message_t));
        }
    }
}


ESP_Status_t ESP_WifiStationConnect(ESP_Handler_t* dev) {
/* 
* Start ESP8266 in UDP SoftAP mode. Credentials are provided in the dev struct 
*/
    DEBUG_PRINTF(DBG_INFO,"[ESP] === Starting ESP8266 in station mode ===\r\n");
    UART_FlushRx(dev->uart_buffers);  

    ESP_CMD(dev, "AT+RST", 5000, "Reset");
    HAL_Delay(4000);
    UART_FlushRx(dev->uart_buffers);  
    ESP_CMD(dev, "ATE0", 2000, "Disable Echo");
    ESP_CMD(dev, "AT+CWMODE=1", 2000, "Set AP Mode");
   // ESP_SendCommand(dev, "AT+CIPSTA=\"192.168.0.100\",\"192.168.0.1\",\"255.255.255.0\"", 3000);
    char cmd[128];
    if (dev->ssid != NULL && dev->password != NULL) {
        snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", dev->ssid, dev->password);
    } else {
        DEBUG_PRINTF(DBG_VERBOSE, "[ESP] SSID and PASS not provided. Returning...\r\n");
        return ESP_ERROR;    
    }
    ESP_CMD(dev, cmd, 10000, "Connecting...");
    HAL_Delay(5000);
    DEBUG_PRINTF(DBG_VERBOSE, "[ESP] Connected. Checking IP\r\n");
    ESP_SendATCommand(dev, "AT+CIFSR",3000);
    ESP_SendATCommand(dev, "AT+CIPSTART=\"UDP\",\"192.168.0.255\",14550,14550", 3000);
    
    dev->state = ESP_STATE_IDLE;
    
    DEBUG_PRINTF(DBG_INFO,"[ESP] === Init Complete ===\r\n");
    return ESP_OK;
}



ESP_Status_t ESP_UDPSoftAP(ESP_Handler_t* dev) {
/* 
* Start ESP8266 in UDP SoftAP mode. Credentials are provided in the dev struct 
*/
    DEBUG_PRINTF(DBG_INFO,"[ESP] === Starting ESP8266 Init ===\r\n");
        UART_FlushRx(dev->uart_buffers);  

    ESP_CMD(dev, "AT+RST", 5000, "Reset");
    HAL_Delay(4000);
    UART_FlushRx(dev->uart_buffers);  
    ESP_CMD(dev, "ATE0", 2000, "Disable Echo");
    DEBUG_PRINTF(DBG_VERBOSE,"[ESP] Step: Close existing connections");

    
    ESP_SendATCommand(dev, "AT+CIPCLOSE",3000);
    ESP_CMD(dev, "AT+CWMODE=2", 2000, "Set AP Mode");

    char cmd[128];
    if (dev->ssid != NULL && dev->password != NULL) {
        snprintf(cmd, sizeof(cmd), "AT+CWSAP=\"%s\",\"%s\",6,3", dev->ssid, dev->password);
    } else {
        snprintf(cmd, sizeof(cmd), "AT+CWSAP=\"RoverAP\",\"\",6,0");
    }

    ESP_CMD(dev, cmd, 2000, "Configure SoftAP");
    ESP_SendATCommand(dev, "AT+CIPSTART=\"UDP\",\"192.168.4.255\",14550,14550", 3000);
    
    DEBUG_PRINTF(DBG_VERBOSE,"[ESP] RX: %s\r\n", dev->rx_buffer);
    DEBUG_PRINTF(DBG_VERBOSE, "Verifying AP...\r\n");
    
    ESP_SendATCommand(dev, "AT+CWSAP?", 1000);
    DEBUG_PRINTF(DBG_VERBOSE, "AP Config: %s\r\n", dev->rx_buffer);
    
    ESP_SendATCommand(dev, "AT+CIPAP?", 1000);
    DEBUG_PRINTF(DBG_INFO, "IP Config: %s\r\n", dev->rx_buffer);

    ESP_SendATCommand(dev, "AT+CWDHCP=0,1", 2000);  
    HAL_Delay(1000);  

    ESP_SendATCommand(dev, "AT+CWDHCP?", 1000);   
    DEBUG_PRINTF(DBG_INFO, "DHCP Status: %s", dev->rx_buffer);
    dev->state = ESP_STATE_IDLE;
    
    DEBUG_PRINTF(DBG_INFO,"[ESP] === Init Complete ===\r\n");
    return ESP_OK;
}

static ESP_Status_t ESP_ProcessByte(ESP_Handler_t* dev, uint8_t byte) {
    static uint16_t ipd_received = 0;
    static uint8_t ipd_data[ESP_MAX_BUFFER_SIZE];
    static ESP_IPDHeader_t current_ipd;
    switch(dev->state) {
        case ESP_STATE_IDLE:
        case ESP_STATE_AWAIT_RESPONSE:
            if (byte == '\n') {
                dev->rx_buffer[dev->rx_index] = '\0';
                // if (dev->rx_index > 0) {
                //     for(uint8_t i =0;i<dev->rx_index;i++){
                //         printf("%x ",dev->rx_buffer[i]);
                //     }
                //     printf("\r\n");
                //     //DEBUG_PRINTF(DEBUG_VERBOSE,"[ESP_RX] %s\r\n", dev->rx_buffer);
                // }
                if (strcmp((const char*)dev->rx_buffer, "OK") == 0) {
                    dev->response_status = ESP_OK;
                    dev->response_received = 1;
                }
                if (strcmp((const char*)dev->rx_buffer, "ready") == 0) {
                    dev->response_status = ESP_OK;
                    dev->response_received = 1;
                }
                else if (strcmp((const char*)dev->rx_buffer, "SEND OK") == 0) {
                    dev->response_status = ESP_OK;
                    dev->response_received = 1;
                }
                else if (strcmp((const char*)dev->rx_buffer, "SEND FAIL") == 0) {
                    dev->response_status = ESP_ERROR;
                    dev->response_received = 1;
                }
                else if (strcmp((const char*)dev->rx_buffer, "ERROR") == 0) {
                    dev->response_status = ESP_ERROR;
                    dev->response_received = 1;
                }
                else if (strncmp((const char*)dev->rx_buffer, "busy", 4) == 0) {
                    dev->response_status = ESP_BUSY;
                    dev->response_received = 1;
                }
                else if (strncmp((const char*)dev->rx_buffer, "CONNECT", 7) == 0) {
                    dev->response_status = ESP_OK;
                    dev->response_received = 1;
                }                
                dev->rx_index = 0;
            }
            else if (byte == ':' && strncmp((const char*)dev->rx_buffer, "+IPD,", 5) == 0) {
                dev->rx_buffer[dev->rx_index] = '\0';
                const char* ptr = (const char*)dev->rx_buffer + 5; 
                current_ipd.length = (uint16_t)atoi(ptr);
                DEBUG_PRINTF(DBG_VERBOSE, "[ESP] IPD Header: length=%d\r\n", current_ipd.length);
                if (current_ipd.length > 0 && current_ipd.length < sizeof(ipd_data)) {
                    dev->state = ESP_STATE_IPD_DATA;
                    ipd_received = 0;
                } else {
                    DEBUG_PRINTF(DBG_ERROR, "[ESP] Invalid IPD length: %d\r\n", current_ipd.length);
                }
                dev->rx_index = 0;
            }
            else if (byte == '>') {
                dev->response_status = ESP_OK;
                dev->response_received = true;
                dev->rx_index = 0;
            }
            else if (byte != '\r') {  
                if (dev->rx_index < sizeof(dev->rx_buffer) - 1) {
                    dev->rx_buffer[dev->rx_index++] = byte;
                }
            }
            break;            
        case ESP_STATE_IPD_DATA:
            if (ipd_received < sizeof(ipd_data)) {
                ipd_data[ipd_received++] = byte;
            }
            
            if (ipd_received >= current_ipd.length) {
                ESP_HandleIPDData(dev, ipd_data, current_ipd.length);
                dev->state = ESP_STATE_IDLE;
                ipd_received = 0;
            }
            break;
    }
    return ESP_OK;
}

static void ESP_ProcessRxQueue(ESP_Handler_t *dev){
    QueueItem_t* item = Queue_Peek(&rx_queue);
    if (item) {
        mavlink_message_t* msg = (mavlink_message_t*)item->data;
        ESP_HandleMAVLinkMsg(dev, msg);
        Queue_Dequeue(&rx_queue);
    }
}

void ESP_ProccessAvailableBytes(ESP_Handler_t *dev){
    // if(processing_rx_bytes){
    //     return;
    // }
    // processing_rx_bytes =1;
    while(UART_RxDataAvailable(dev->uart_buffers)){
        uint8_t byte = UART_GetByte(dev->uart_buffers);
        ESP_ProcessByte(dev,byte);
    }
    // processing_rx_bytes = 0;
}

void ESP_MainLoop(ESP_Handler_t* dev) {
   uint8_t retry_counter = 0;
   ESP_Status_t result;
   ESP_ProccessAvailableBytes(dev);
   ESP_ProcessRxQueue(dev);
   if(manual_ctrl_pending){
    manual_ctrl_pending = 0;
    Rover_ProcessManualCtrl(&pending_manual_ctrl);
   }
   do {
    result = ESP_ProcessTxQueue(dev);
    retry_counter++;
   } while (result != ESP_OK || retry_counter < 6);
}


void MAVLink_SendHeartbeat(ESP_Handler_t *dev) {
/*
* Keeping this a direct send to keep timing
*/
    mavlink_message_t msg;
    uint8_t attempt_count = 0;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack(
        Rover_GetId(),           
        MAV_COMP_ID_AUTOPILOT1, // Component ID
        &msg,
        MAV_TYPE_GROUND_ROVER,  // Vehicle type
        MAV_AUTOPILOT_GENERIC,
        Rover_GetMode(),
        0,                      // custom_mode
        Rover_GetState()
    );
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    while (attempt_count < 5){
        if (ESP_SendMAVLink(dev, buf,len) != ESP_OK){
            return;
        } else {
            HAL_Delay(50);
            attempt_count++;
        }
    }
}

void MAVLink_SendSysStatus(ESP_Handler_t* dev) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    // System status
    mavlink_msg_sys_status_pack(
        Rover_GetId(), MAV_COMP_ID_AUTOPILOT1, &msg,
        0,              // onboard_control_sensors_present
        0,              // onboard_control_sensors_enabled
        0,              // onboard_control_sensors_health
        500,            // load (0.1%)
        Rover_GetVBat(),  // voltage_battery (mV)
        0.1 * 100,   // current_battery (cA)
        100,       // battery_remaining (%)
        0, 0, 0, 0, 0, 0, 0, 0,0
    );
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    ESP_QueueMAVLink(&tx_queue, buf, len);
}
