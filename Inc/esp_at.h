#ifndef ESP_AT_H
#define ESP_AT_H

#include "stm32f4xx_hal.h"
#include "usart.h"


#define ESP_MAX_BUFFER_SIZE 512


#define		ESP_WAIT_SHORT			1000
#define		ESP_WAIT_MED			5000
#define		ESP_WAIT_LONG			15000

typedef struct UART_Buffers_t UART_Buffers_t;

typedef enum {
    ESP_OK = HAL_OK,
    ESP_ERROR = HAL_ERROR,
    ESP_BUSY = HAL_BUSY,
    ESP_TIMEOUT = HAL_TIMEOUT
} ESP_Status_t;


typedef enum {
    ESP_STATE_IDLE,
    ESP_STATE_AWAIT_RESPONSE,
    ESP_STATE_IPD_DATA
} ESP_State_t;

typedef struct {
    uint8_t link_id;
    uint16_t length;
    char remote_ip[16];
    uint16_t remote_port;
} ESP_IPDHeader_t;

typedef struct ESP_Handler_t{
    UART_HandleTypeDef* huart;
    volatile UART_Buffers_t *uart_buffers;
    
    ESP_State_t state;

    uint8_t* ssid;
    uint8_t* password;

    uint8_t response_received;
    ESP_Status_t response_status;
    size_t rx_index;
    uint8_t rx_buffer[ESP_MAX_BUFFER_SIZE];

} ESP_Handler_t;

ESP_Status_t ESP_SendString(ESP_Handler_t* dev, const uint8_t* str);
void ESP_Init(ESP_Handler_t* dev, UART_HandleTypeDef* huart, UART_Buffers_t* uart_buffers);
ESP_Status_t ESP_SendATCommand(ESP_Handler_t* dev, const uint8_t* cmd, uint32_t timeout);
ESP_Status_t ESP_SendMessage(ESP_Handler_t* dev, uint8_t* data, uint32_t len);
void ESP_ProccessAvailableBytes(ESP_Handler_t* dev);
ESP_Status_t ESP_WifiStationConnect(ESP_Handler_t* dev);
ESP_Status_t ESP_UDPSoftAP(ESP_Handler_t* dev);
void ESP_MainLoop(ESP_Handler_t* dev);

#endif /* ESP_AT_H */