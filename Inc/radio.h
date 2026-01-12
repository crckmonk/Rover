/* radio.h - High level RF communication functions
*/
#ifndef RADIO_H
#define RADIO_H

#include "nrf24.h"
#include "control.h"
#include <string.h>


uint8_t Radio_InitNRF24(NRF24L01* dev, SPI_HandleTypeDef *hspi, uint8_t* txAddress, NRF24_TXRX_STATE state);
uint8_t Radio_NRF24RxMainLoop(NRF24L01 *dev, command_packet *dataPacket, command_packet *rxCmdPacket);


#endif