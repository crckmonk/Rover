#include "rover_mavlink.h"


static ESP_Handler_t* esp_dev = NULL;

static volatile uint8_t          manual_ctrl_pending = 0;
static mavlink_manual_control_t  pending_manual_ctrl;

static Queue_t tx_queue = {0};
static Queue_t rx_queue = {0};




static MAVLink_Param_t params[] = {
    {"SYSID_THISMAV", 1.0f, MAV_PARAM_TYPE_INT32}
};


void MAVLink_Init(ESP_Handler_t* dev) {
    esp_dev = dev;
    Queue_Init(&tx_queue);
    Queue_Init(&rx_queue);
}


static uint8_t MAVLink_QueueMessage(Queue_t* queue,uint8_t *data, uint16_t len) {
    return Queue_Enqueue(queue, data, len);
}

uint8_t MAVLink_QueueRxMessage( uint8_t *data, uint16_t len) {
    return Queue_Enqueue(&rx_queue, data, len);
}

uint8_t MAVLink_QueueTxMessage(uint8_t *data, uint16_t len) {
    return Queue_Enqueue(&tx_queue, data, len);
}



void MAVLink_SendParamValue(uint16_t index) {
    if (index >= COUNT(params)) return;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_param_value_pack(
        Rover_GetId(), MAV_COMP_ID_AUTOPILOT1, &msg,
        params[index].name,   // param_id (char[16])
        params[index].value,  // param_value (float)
        params[index].type,   // param_type
        COUNT(params),          // param_count — total number
        index                 // param_index — this one's index
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    MAVLink_QueueMessage(&tx_queue, buf, len);
}





HAL_StatusTypeDef MAVLink_ProcessTXQueue(void) {
    QueueItem_t* item = Queue_Peek(&tx_queue);
    if (item == NULL) {
        return HAL_OK; 
    }
    
    if (ESP_SendMessage(esp_dev, item->data, item->len) == ESP_OK) {
        Queue_Dequeue(&tx_queue);
        return HAL_OK;
    } else {
        DEBUG_PRINTF(DBG_ERROR, "[ESP] TX Send failed\r\n");
    }
    return HAL_ERROR;
}

void MAVLink_ProcessRxQueue(void){
    QueueItem_t* item = Queue_Peek(&rx_queue);
    if (item) {
        mavlink_message_t* msg = (mavlink_message_t*)item->data;
        MAVLink_HandleMessage(msg);
        Queue_Dequeue(&rx_queue);
    }
}


void MAVLink_HandleCommandLong(mavlink_message_t* msg) {
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
    MAVLink_QueueMessage(&tx_queue, buf, len);
}

void MAVLink_SendCmdAck(uint16_t command, uint8_t result) {
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
    MAVLink_QueueMessage(&tx_queue, buf, len);
}


void MAVLink_HandleMessage( mavlink_message_t *msg){
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
                    MAVLink_HandleCommandLong(msg);
                    break;
                    }
                case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                    DEBUG_PRINTF(DBG_INFO,"[ESP] GCS requesting parameter list\r\n");
                        for (uint16_t i = 0; i <  COUNT(params); i++) {
                            MAVLink_SendParamValue(i);
                            HAL_Delay(10);
                        }
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

void MAVLink_MainLoop(void) {
  uint8_t retry_counter = 0;
  HAL_StatusTypeDef result;
  MAVLink_ProcessRxQueue();
  if (manual_ctrl_pending) {
    manual_ctrl_pending = 0;
    Rover_ProcessManualCtrl(&pending_manual_ctrl);
  }
  do {
    result = MAVLink_ProcessTXQueue();
    retry_counter++;
    HAL_Delay(5);
  } while (result != HAL_OK || retry_counter < 5);
}

void MAVLink_SendHeartbeat(void) {
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
        if (ESP_SendMessage(esp_dev, buf,len) != ESP_OK){
            return;
        } else {
            HAL_Delay(50);
            attempt_count++;
        }
    }
}

void MAVLink_SendSysStatus(void) {
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
    MAVLink_QueueMessage(&tx_queue, buf, len);
}
