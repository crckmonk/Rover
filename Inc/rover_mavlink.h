#ifndef ROVER_MAVLINK_H
#define ROVER_MAVLINK_H

#include "stm32f4xx_hal.h"
#include "queue.h"
#include "common/mavlink.h"
#include "mcutils.h"
#include "esp_at.h"
#include "control.h"




typedef struct {
    char     name[17];
    float    value;
    uint8_t  type;      
} MAVLink_Param_t;

typedef struct ESP_Handler_t ESP_Handler_t;

void MAVLink_Init(ESP_Handler_t* dev);

uint8_t MAVLink_QueueRxMessage(uint8_t* data, uint16_t len);

uint8_t MAVLink_QueueTxMessage(uint8_t* data, uint16_t len);

void MAVLink_SendParamValue(uint16_t index);

HAL_StatusTypeDef MAVLink_ProcessTXQueue(void);

void MAVLink_ProcessRxQueue(void);

void MAVLink_HandleCommandLong(mavlink_message_t* msg);

void MAVLink_SendAutopilotVersion(void);

void MAVLink_SendCmdAck(uint16_t command, uint8_t result);

void MAVLink_HandleMessage(mavlink_message_t* msg);

void MAVLink_SendStatusText(uint8_t severity, const char* text, uint8_t comp_id);

void MAVLink_MainLoop(void);

void MAVLink_SendHeartbeat(void);

void MAVLink_SendSysStatus(void);

#endif
