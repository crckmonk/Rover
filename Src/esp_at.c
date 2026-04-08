/*
* ESP8266 driver. For now handles MAVLink message processing.
*/
#include "esp_at.h"
#include <string.h>
#include <stdio.h>
#include "printf.h"
#include "common/mavlink.h"
#include <stdlib.h>
#include "rover_mavlink.h"
#include "mcutils.h"

/* * Debug utils*/
static const char* ESP_StatusToString(ESP_Status_t status) {
    switch (status) {
        case ESP_OK:          return "OK";
        case ESP_ERROR:       return "ERROR";
        case ESP_BUSY:        return "BUSY";
        case ESP_TIMEOUT:     return "TIMEOUT";
        default:                  return "UNKNOWN";
    }
}


#define ESP_CMD(dev, cmd, timeout, step_name) do { \
    DEBUG_PRINTF(DBG_VERBOSE,"[ESP] Step: %s\r\n", step_name); \
    ESP_Status_t _status = ESP_SendATCommand(dev, cmd, timeout); \
    if (_status != ESP_OK) { \
        DEBUG_PRINTF(DBG_VERBOSE,"[ESP] FAILED at '%s': %s\r\n", step_name, ESP_StatusToString(_status)); \
        return ESP_ERROR; \
    } \
} while(0)

/* * PARAMETER DUMMY */




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
            MAVLink_QueueRxMessage((uint8_t*)&msg, sizeof(mavlink_message_t));
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


ESP_Status_t ESP_SendMessage(ESP_Handler_t *dev, uint8_t *data, uint32_t len) {
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

   ESP_ProccessAvailableBytes(dev);
}