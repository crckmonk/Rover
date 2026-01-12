#include "radio.h"


uint8_t Radio_InitNRF24(NRF24L01* dev, SPI_HandleTypeDef *hspi, uint8_t* txAddress, NRF24_TXRX_STATE state) {
  uint8_t status = 0;
  dev->spi = hspi;
  dev->DATA_RATE = NRF24_DATA_RATE_1MBPS;
  dev->RX_ADDRESS = txAddress;
  dev->TX_ADDRESS = txAddress;
  dev->RF_CHANNEL = NRF24_CHANNEL;
  dev->PayloadLength = NRF24_PAYLOAD_LENGTH;
  dev->RetransmitCount = 10;
  dev->RetransmitDelay = 15;
  dev->TX_POWER = NRF24_TX_PWR_0dBm;
  dev->CRC_WIDTH = NRF24_CRC_WIDTH_1B;
  dev->ADDR_WIDTH = NRF24_ADDR_WIDTH_5;
  dev->NRF24_CSN_GPIOx = NRF24_CSN_GPIO_Port;
  dev->NRF24_CSN_GPIO_PIN = NRF24_CSN_Pin;
  dev->NRF24_CE_GPIOx = NRF24_CE_GPIO_Port;
  dev->NRF24_CE_GPIO_PIN = NRF24_CE_Pin;
  dev->NRF24_IRQ_GPIOx = NRF24_IRQ_GPIO_Port;
  dev->NRF24_IRQ_GPIO_PIN = NRF24_IRQ_Pin;
  dev->NRF24_IRQn = EXTI1_IRQn;
  dev->NRF24_IRQ_preempt_priority = 5;
  dev->NRF24_IRQ_sub_priority = 0;
  dev->STATE = state;
  dev->BUSY_FLAG = state == NRF24_STATE_TX ? 0 : 1;
  NRF24_Init(dev);
  NRF24_SetDynamicPayload(dev, 1);
  NRF24_EnableDynamicPayloadPipes(dev);
  NRF24_EnableAckPayload(dev, 1);
  NRF24_FlushRX(dev);
  NRF24_FlushTX(dev);
  NRF24_ClearInterrupts(dev);
  NRF24_ReadRegister(dev, NRF24_STATUS, &status);
  return status; 
}

uint8_t Radio_NRF24RxMainLoop(NRF24L01* dev, command_packet* dataPacket, command_packet *rxCmdPacket ){
    uint8_t status = 0;
    uint8_t cmdBuffer[NRF24_PAYLOAD_LENGTH];
    cmdBuffer[0] = dataPacket->packet_type;
    cmdBuffer[1] = dataPacket->left_motors_speed;
    cmdBuffer[2] = dataPacket->right_motors_speed;
    cmdBuffer[3] = dataPacket->buttons;

    if(dev->IRQ_FLAG == 1) { // RX Data Ready Interrupt
        NRF24_PullPacket(dev, cmdBuffer);
        NRF24_WriteAckPayload(dev, 0, cmdBuffer, NRF24_PAYLOAD_LENGTH);
        dev->IRQ_FLAG = 0;
    }
    if(cmdBuffer[0]){
        rxCmdPacket->packet_type = cmdBuffer[0];
        rxCmdPacket->left_motors_speed = cmdBuffer[1];
        rxCmdPacket->right_motors_speed = cmdBuffer[2];
        rxCmdPacket->buttons = cmdBuffer[3];
    }
    status = dev->LAST_STATUS;
    return status;
}

