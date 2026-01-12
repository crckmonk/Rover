#ifndef NRF24L01_H
#define NRF24L01_H

#include "nrf24_conf.h"
#include "stm32f4xx_hal.h"
/* Registers */
#define NRF24_CONFIG		0x00
#define NRF24_EN_AA		0x01
#define NRF24_EN_RXADDR	0x02
#define NRF24_SETUP_AW	0x03
#define NRF24_SETUP_RETR	0x04
#define NRF24_RF_CH		0x05
#define NRF24_RF_SETUP	0x06
#define NRF24_STATUS		0x07
#define NRF24_OBSERVE_TX	0x08
#define NRF24_CD			0x09
#define NRF24_RX_ADDR_P0	0x0A
#define NRF24_RX_ADDR_P1	0x0B
#define NRF24_RX_ADDR_P2	0x0C
#define NRF24_RX_ADDR_P3	0x0D
#define NRF24_RX_ADDR_P4	0x0E
#define NRF24_RX_ADDR_P5	0x0F
#define NRF24_TX_ADDR		0x10
#define NRF24_RX_PW_P0	0x11
#define NRF24_RX_PW_P1	0x12
#define NRF24_RX_PW_P2	0x13
#define NRF24_RX_PW_P3	0x14
#define NRF24_RX_PW_P4	0x15
#define NRF24_RX_PW_P5	0x16
#define NRF24_FIFO_STATUS	0x17
#define NRF24_DYNPD		0x1C
#define NRF24_FEATURE		0x1D

/* Commands */
#define NRF24_CMD_R_REGISTER			0x00
#define NRF24_CMD_W_REGISTER			0x20
#define NRF24_CMD_R_RX_PAYLOAD		0x61
#define NRF24_CMD_W_TX_PAYLOAD		0xA0
#define NRF24_CMD_FLUSH_TX			0xE1
#define NRF24_CMD_FLUSH_RX			0xE2
#define NRF24_CMD_REUSE_TX_PL			0xE3
#define NRF24_CMD_ACTIVATE			0x50
#define NRF24_CMD_R_RX_PL_WID			0x60
#define NRF24_CMD_W_ACK_PAYLOAD		0xA8
#define NRF24_CMD_W_TX_PAYLOAD_NOACK	0xB0
#define NRF24_CMD_NOP					0xFF

#define NRF24_SPI_TIMEOUT	10000
#define NRF24_PAYLOAD_LENGTH   8 // 1B ~ 32B
typedef enum{
	NRF24_DATA_RATE_250KBPS=1,
	NRF24_DATA_RATE_1MBPS=0,
	NRF24_DATA_RATE_2MBPS=2
} NRF24_DATA_RATE;

typedef enum{
	NRF24_TX_PWR_M18dBm=0,
	NRF24_TX_PWR_M12dBm=1,
	NRF24_TX_PWR_M6dBm=2,
	NRF24_TX_PWR_0dBm=3
} NRF24_TX_PWR;

typedef enum{
	NRF24_ADDR_WIDTH_3=1,
	NRF24_ADDR_WIDTH_4=2,
	NRF24_ADDR_WIDTH_5=3
} NRF24_ADDR_WIDTH;

typedef enum{
	NRF24_CRC_WIDTH_1B=0,
	NRF24_CRC_WIDTH_2B=1
} NRF24_CRC_WIDTH;

typedef enum{
	NRF24_STATE_RX=1,
	NRF24_STATE_TX=0
} NRF24_TXRX_STATE;

typedef struct {
	SPI_HandleTypeDef* spi;
	NRF24_DATA_RATE 	DATA_RATE;
	uint8_t			RF_CHANNEL;
	uint8_t			PayloadLength;
	uint8_t			RetransmitCount;
	uint8_t			RetransmitDelay;
	NRF24_TX_PWR		TX_POWER;
	uint8_t*		RX_ADDRESS;
	uint8_t*		TX_ADDRESS;
	NRF24_CRC_WIDTH	CRC_WIDTH;
	NRF24_ADDR_WIDTH	ADDR_WIDTH;
	NRF24_TXRX_STATE	STATE;
	uint8_t			BUSY_FLAG;
	uint8_t 		LAST_STATUS;
	volatile uint8_t IRQ_FLAG;

	uint8_t*		RX_BUFFER;
	uint8_t*		TX_BUFFER;

	GPIO_TypeDef*	NRF24_CSN_GPIOx;	// CSN pin
	uint16_t		NRF24_CSN_GPIO_PIN;

	GPIO_TypeDef*	NRF24_CE_GPIOx;	// CE pin
	uint16_t		NRF24_CE_GPIO_PIN;

	GPIO_TypeDef*	NRF24_IRQ_GPIOx;	// IRQ pin
	uint16_t		NRF24_IRQ_GPIO_PIN;
	IRQn_Type		NRF24_IRQn;
	uint8_t			NRF24_IRQ_preempt_priority;
	uint8_t			NRF24_IRQ_sub_priority;

} NRF24L01;

typedef enum{
	NRF24_OK,
	NRF24_ERROR
} NRF24_RESULT;


/* Initialization routine */
NRF24_RESULT NRF24_Init(NRF24L01* dev);

void NRF24_CE_ENABLE(NRF24L01* dev);
void NRF24_CE_DISABLE(NRF24L01* dev);
void NRF24_CS_ENABLE(NRF24L01* dev);
void NRF24_CS_DISABLE(NRF24L01* dev);

/* EXTI Interrupt Handler */
uint8_t NRF24_IRQ_Handler(NRF24L01* dev);

/* Blocking Data Sending / Receiving FXs */
NRF24_RESULT NRF24_SendPacket(NRF24L01* dev,uint8_t* data);
NRF24_RESULT NRF24_ReceivePacket(NRF24L01* dev,uint8_t* data);

/* Non-Blocking Data Sending / Receiving FXs */
NRF24_RESULT NRF24_PushPacket(NRF24L01* dev,uint8_t* data);
NRF24_RESULT NRF24_PullPacket(NRF24L01* dev,uint8_t* data);

/* LOW LEVEL STUFF (you don't have to look in here...)*/
NRF24_RESULT NRF24_SendCommand(NRF24L01* dev, uint8_t cmd, uint8_t* tx,uint8_t* rx,uint8_t len);
/* CMD */
NRF24_RESULT NRF24_ReadRegister(NRF24L01* dev,uint8_t reg, uint8_t* data);
NRF24_RESULT NRF24_WriteRegister(NRF24L01* dev,uint8_t reg, uint8_t* data);
NRF24_RESULT NRF24_ReadRXPayload(NRF24L01* dev,uint8_t* data);
NRF24_RESULT NRF24_WriteTXPayload(NRF24L01* dev,uint8_t* data);
NRF24_RESULT NRF24_FlushTX(NRF24L01* dev);
NRF24_RESULT NRF24_FlushRX(NRF24L01* dev);

/* RF_SETUP */
NRF24_RESULT NRF24_SetDataRate(NRF24L01* dev,NRF24_DATA_RATE rate);
NRF24_RESULT NRF24_SetTXPower(NRF24L01* dev,NRF24_TX_PWR pwr);
NRF24_RESULT NRF24_SetCCW(NRF24L01* dev,uint8_t activate);

/* STATUS */
NRF24_RESULT NRF24_ClearInterrupts(NRF24L01* dev);

/* RF_CH */
NRF24_RESULT NRF24_SetRFChannel(NRF24L01* dev,uint8_t ch);

/* SETUP_RETR */
NRF24_RESULT NRF24_SetRetransmittionCount(NRF24L01* dev,uint8_t count);
NRF24_RESULT NRF24_SetRetransmittionDelay(NRF24L01* dev,uint8_t delay);

/* SETUP_AW */
NRF24_RESULT NRF24_SetAddressWidth(NRF24L01* dev,NRF24_ADDR_WIDTH width);

/* EN_RXADDR */
NRF24_RESULT NRF24_EnableRXPipe(NRF24L01* dev,uint8_t pipe);

/* EN_AA */
NRF24_RESULT NRF24_EnableAutoAcknowledgement(NRF24L01* dev,uint8_t pipe);

/* CONFIG */
NRF24_RESULT NRF24_EnableCRC(NRF24L01* dev,uint8_t activate);
NRF24_RESULT NRF24_SetCRCWidth(NRF24L01* dev,NRF24_CRC_WIDTH width);
NRF24_RESULT NRF24_PowerUp(NRF24L01* dev,uint8_t powerUp);
NRF24_RESULT NRF24_RXTXControl(NRF24L01* dev,NRF24_TXRX_STATE rx);
NRF24_RESULT NRF24_EnableRXDataReadyIRQ(NRF24L01* dev,uint8_t activate);
NRF24_RESULT NRF24_EnableTXDataSentIRQ(NRF24L01* dev,uint8_t activate);
NRF24_RESULT NRF24_EnableMaxRetransmitIRQ(NRF24L01* dev,uint8_t activate);

/* RX_ADDR_P0 */
NRF24_RESULT NRF24_SetRXAddress_P0(NRF24L01* dev,uint8_t* address);	// 5bytes of address

/* TX_ADDR */
NRF24_RESULT NRF24_SetTXAddress(NRF24L01* dev,uint8_t* address);	// 5bytes of address

/* RX_PW_P0 */
NRF24_RESULT NRF24_SetRXPayloadWidth_P0(NRF24L01* dev,uint8_t width);

/* FEATURE */
NRF24_RESULT NRF24_SetDynamicPayload(NRF24L01* dev,uint8_t activate);
NRF24_RESULT NRF24_EnableDynamicPayloadPipes(NRF24L01* dev);
NRF24_RESULT NRF24_EnableAckPayload(NRF24L01* dev, uint8_t activate);
NRF24_RESULT NRF24_WriteAckPayload(NRF24L01* dev, uint8_t pipe, uint8_t* data, uint8_t len);
#endif /* NRF24L01_H */
