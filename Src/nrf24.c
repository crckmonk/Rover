#include "nrf24.h"

void NRF24_CS_ENABLE(NRF24L01* dev) {
	HAL_GPIO_WritePin(dev->NRF24_CSN_GPIOx, dev->NRF24_CSN_GPIO_PIN,
			GPIO_PIN_RESET);
}

void NRF24_CS_DISABLE(NRF24L01* dev) {
	HAL_GPIO_WritePin(dev->NRF24_CSN_GPIOx, dev->NRF24_CSN_GPIO_PIN, GPIO_PIN_SET);
}

void NRF24_CE_ENABLE(NRF24L01* dev) {
	HAL_GPIO_WritePin(dev->NRF24_CE_GPIOx, dev->NRF24_CE_GPIO_PIN, GPIO_PIN_SET);
}

void NRF24_CE_DISABLE(NRF24L01* dev) {
	HAL_GPIO_WritePin(dev->NRF24_CE_GPIOx, dev->NRF24_CE_GPIO_PIN, GPIO_PIN_RESET);
}

NRF24_RESULT NRF24_SetupGPIO(NRF24L01* dev) {

	GPIO_InitTypeDef GPIO_InitStructure;

	// CE pin
	GPIO_InitStructure.Pin = dev->NRF24_CE_GPIO_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
	GPIO_InitStructure.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(dev->NRF24_CE_GPIOx, &GPIO_InitStructure);
	// end CE pin

	// IRQ pin
	GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Pin = dev->NRF24_IRQ_GPIO_PIN;
	HAL_GPIO_Init(dev->NRF24_IRQ_GPIOx, &GPIO_InitStructure);

	/* Enable and set EXTI Line Interrupt to the given priority */
	HAL_NVIC_SetPriority(dev->NRF24_IRQn, dev->NRF24_IRQ_preempt_priority,
			dev->NRF24_IRQ_sub_priority);
	HAL_NVIC_EnableIRQ(dev->NRF24_IRQn);
	// end IRQ pin

	NRF24_CS_DISABLE(dev);
	NRF24_CE_DISABLE(dev);

	return NRF24_OK;
}

NRF24_RESULT NRF24_Init(NRF24L01* dev) {

	NRF24_SetupGPIO(dev);

	NRF24_PowerUp(dev, 1);

	uint8_t config=0;

	while((config&2)==0){	// wait for powerup
		NRF24_ReadRegister(dev,NRF24_CONFIG,&config);
	}

	NRF24_SetRXPayloadWidth_P0(dev, dev->PayloadLength);
	if(dev->RX_ADDRESS) // If not the default address is used 
		NRF24_SetRXAddress_P0(dev, dev->RX_ADDRESS);
	if(dev->TX_ADDRESS)
		NRF24_SetTXAddress(dev, dev->TX_ADDRESS);
	NRF24_EnableRXDataReadyIRQ(dev, 1);
	NRF24_EnableTXDataSentIRQ(dev,1);
	NRF24_EnableMaxRetransmitIRQ(dev,1);
	NRF24_EnableCRC(dev, 1);
	NRF24_SetCRCWidth(dev, dev->CRC_WIDTH);
	if(!dev->ADDR_WIDTH)
		return NRF24_ERROR;
	
	NRF24_SetAddressWidth(dev, dev->ADDR_WIDTH);
	NRF24_SetRFChannel(dev, dev->RF_CHANNEL);
	NRF24_SetDataRate(dev, dev->DATA_RATE);
	NRF24_SetRetransmittionCount(dev, dev->RetransmitCount);
	NRF24_SetRetransmittionDelay(dev, dev->RetransmitDelay);

	//NRF24_EnableRXPipe(dev, 0); ENANBLED BY DEFAULT
	//NRF24_EnableAutoAcknowledgement(dev, 0); ENANBLED BY DEFAULT FOR ALL PIPES

	NRF24_ClearInterrupts(dev);

	NRF24_RXTXControl(dev, NRF24_STATE_RX);

	NRF24_FlushRX(dev);

	return NRF24_OK;
}

NRF24_RESULT NRF24_SendCommand(NRF24L01* dev, uint8_t cmd, uint8_t* tx, uint8_t* rx,
		uint8_t len) {
	uint8_t myTX[len + 1];
	uint8_t myRX[len + 1];
	myTX[0] = cmd;

	int i = 0;
	for (i = 0; i < len; i++) {
		myTX[1 + i] = tx[i];
		myRX[i] = 0;
	}

	NRF24_CS_ENABLE(dev);
	if (HAL_SPI_TransmitReceive(dev->spi, myTX, myRX, 1 + len, NRF24_SPI_TIMEOUT)
			!= HAL_OK) {
		return NRF24_ERROR;
	}

	for (i = 0; i < len; i++) {
		rx[i] = myRX[1 + i];
	}

	NRF24_CS_DISABLE(dev);

	return NRF24_OK;
}

uint8_t NRF24_IRQ_Handler(NRF24L01* dev) {
	uint8_t status = 0;
	if (NRF24_ReadRegister(dev, NRF24_STATUS, &status) != NRF24_OK) {
		return 0;
	}

	if ((status & (1 << 6))) {	// RX FIFO Interrupt
		uint8_t fifo_status = 0;
		NRF24_CE_DISABLE(dev);
		NRF24_WriteRegister(dev, NRF24_STATUS, &status);
		NRF24_ReadRegister(dev, NRF24_FIFO_STATUS, &fifo_status);
		if ((fifo_status & 1) == 0) {
			NRF24_ReadRXPayload(dev, dev->RX_BUFFER);
			status |= 1 << 6;
			NRF24_WriteRegister(dev, NRF24_STATUS, &status);
			//NRF24_FlushRX(dev);
			dev->BUSY_FLAG=0;
			dev->IRQ_FLAG |= NRF24_IRQ_RX_DR;
		}
		NRF24_CE_ENABLE(dev);
	}
	if ((status & (1 << 5))) {	// TX Data Sent Interrupt
		status |= 1 << 5;	// clear the interrupt flag
		NRF24_CE_DISABLE(dev);
		NRF24_RXTXControl(dev, NRF24_STATE_RX);
		NRF24_CE_ENABLE(dev);
		NRF24_WriteRegister(dev, NRF24_STATUS, &status);
		dev->BUSY_FLAG=0;
		dev->IRQ_FLAG |= NRF24_IRQ_TX_DS;
	}
	if ((status & (1 << 4))) {	// MaxRetransmits reached
		status |= 1 << 4;

		NRF24_FlushTX(dev);
		NRF24_CE_DISABLE(dev);
		NRF24_RXTXControl(dev, NRF24_STATE_RX);
		NRF24_CE_ENABLE(dev);
		NRF24_WriteRegister(dev, NRF24_STATUS, &status);
		dev->BUSY_FLAG=0;
		dev->IRQ_FLAG |= NRF24_IRQ_MAX_RT;
	}
	return status;
}

NRF24_RESULT NRF24_ReadRegister(NRF24L01* dev, uint8_t reg, uint8_t* data) {
	uint8_t tx = 0;
	if (NRF24_SendCommand(dev, NRF24_CMD_R_REGISTER | reg, &tx, data, 1)
			!= NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_WriteRegister(NRF24L01* dev, uint8_t reg, uint8_t* data) {
	uint8_t rx = 0;
	if (NRF24_SendCommand(dev, NRF24_CMD_W_REGISTER | reg, data, &rx, 1)
			!= NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_ReadRXPayload(NRF24L01* dev, uint8_t* data) {
	uint8_t tx[dev->PayloadLength];
	if (NRF24_SendCommand(dev, NRF24_CMD_R_RX_PAYLOAD, tx, data, dev->PayloadLength)
			!= NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_WriteTXPayload(NRF24L01* dev, uint8_t* data) {
	uint8_t rx[dev->PayloadLength];
	if (NRF24_SendCommand(dev, NRF24_CMD_W_TX_PAYLOAD, data, rx, dev->PayloadLength)
			!= NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_FlushTX(NRF24L01* dev) {
	uint8_t rx = 0;
	uint8_t tx = 0;
	if (NRF24_SendCommand(dev, NRF24_CMD_FLUSH_TX, &tx, &rx, 0) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_FlushRX(NRF24L01* dev) {
	uint8_t rx = 0;
	uint8_t tx = 0;
	if (NRF24_SendCommand(dev, NRF24_CMD_FLUSH_RX, &tx, &rx, 0) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_SetDataRate(NRF24L01* dev, NRF24_DATA_RATE rate) {
	uint8_t reg = 0;
	if (NRF24_ReadRegister(dev, NRF24_RF_SETUP, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	if (rate & 1) {	// low bit set
		reg |= 1 << 5;
	} else {	// low bit clear
		reg &= ~(1 << 5);
	}

	if (rate & 2) {	// high bit set
		reg |= 1 << 3;
	} else {	// high bit clear
		reg &= ~(1 << 3);
	}
	if (NRF24_WriteRegister(dev, NRF24_RF_SETUP, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_SetTXPower(NRF24L01* dev, NRF24_TX_PWR pwr) {
	uint8_t reg = 0;
	if (NRF24_ReadRegister(dev, NRF24_RF_SETUP, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	reg &= 0xF9;	// clear bits 1,2
	reg |= pwr << 1;	// set bits 1,2
	if (NRF24_WriteRegister(dev, NRF24_RF_SETUP, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_SetCCW(NRF24L01* dev, uint8_t activate) {
	uint8_t reg = 0;
	if (NRF24_ReadRegister(dev, NRF24_RF_SETUP, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}

	if (activate) {
		reg |= 0x80;
	} else {
		reg &= 0x7F;
	}

	if (NRF24_WriteRegister(dev, NRF24_RF_SETUP, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_ClearInterrupts(NRF24L01* dev) {
	uint8_t reg = 0;
	if (NRF24_ReadRegister(dev, NRF24_STATUS, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}

	reg |= 7 << 4;	// setting bits 4,5,6

	if (NRF24_WriteRegister(dev, NRF24_STATUS, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_SetRFChannel(NRF24L01* dev, uint8_t ch) {
	ch &= 0x7F;
	uint8_t reg = 0;
	if (NRF24_ReadRegister(dev, NRF24_RF_CH, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}

	reg |= ch;	// setting channel

	if (NRF24_WriteRegister(dev, NRF24_RF_CH, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_SetDynamicPayload(NRF24L01* dev,uint8_t activate) {
	uint8_t reg = 0;
	if (NRF24_ReadRegister(dev, NRF24_FEATURE, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}

	if (activate) {
		reg |= 1 << 2;
	}else {
		reg &= ~(1 << 2 );
	}

	if (NRF24_WriteRegister(dev, NRF24_FEATURE, &reg) != NRF24_OK) {
			return NRF24_ERROR;
	}



	return NRF24_OK;
}

NRF24_RESULT NRF24_EnableDynamicPayloadPipes(NRF24L01* dev) {
	uint8_t regValue = 0b00111111;

	if (NRF24_WriteRegister(dev, NRF24_DYNPD, &regValue) != NRF24_OK) {
			return NRF24_ERROR;
	}

	return NRF24_OK;
}

NRF24_RESULT NRF24_SetRetransmittionCount(NRF24L01* dev, uint8_t count) {
	count &= 0x0F;
	uint8_t reg = 0;
	if (NRF24_ReadRegister(dev, NRF24_SETUP_RETR, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}

	reg &= 0xF0;	// clearing bits 0,1,2,3
	reg |= count;	// setting count

	if (NRF24_WriteRegister(dev, NRF24_SETUP_RETR, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_SetRetransmittionDelay(NRF24L01* dev, uint8_t delay) {
	delay &= 0x0F;
	uint8_t reg = 0;
	if (NRF24_ReadRegister(dev, NRF24_SETUP_RETR, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}

	reg &= 0x0F;	// clearing bits 1,2,6,7
	reg |= delay << 4;	// setting delay

	if (NRF24_WriteRegister(dev, NRF24_SETUP_RETR, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_SetAddressWidth(NRF24L01* dev, NRF24_ADDR_WIDTH width) {
	uint8_t reg = 0;
	if (NRF24_ReadRegister(dev, NRF24_SETUP_AW, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}

	reg &= 0x03;	// clearing bits 0,1
	reg |= width;	// setting delay

	if (NRF24_WriteRegister(dev, NRF24_SETUP_AW, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_EnableRXPipe(NRF24L01* dev, uint8_t pipe) {
	uint8_t reg = 0;
	if (NRF24_ReadRegister(dev, NRF24_EN_RXADDR, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}

	reg |= 1 << pipe;

	if (NRF24_WriteRegister(dev, NRF24_EN_RXADDR, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_EnableAutoAcknowledgement(NRF24L01* dev, uint8_t pipe) {
	uint8_t reg = 0;
	if (NRF24_ReadRegister(dev, NRF24_EN_AA, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}

	reg |= 1 << pipe;

	if (NRF24_WriteRegister(dev, NRF24_EN_AA, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_EnableCRC(NRF24L01* dev, uint8_t activate) {
	uint8_t reg = 0;
	if (NRF24_ReadRegister(dev, NRF24_CONFIG, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}

	if (activate) {
		reg |= 1 << 3;
	} else {
		reg &= ~(1 << 3);
	}

	if (NRF24_WriteRegister(dev, NRF24_CONFIG, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_SetCRCWidth(NRF24L01* dev, NRF24_CRC_WIDTH width) {
	uint8_t reg = 0;
	if (NRF24_ReadRegister(dev, NRF24_CONFIG, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}

	if (width == NRF24_CRC_WIDTH_2B) {
		reg |= 1 << 2;
	} else {
		reg &= ~(1 << 3);
	}

	if (NRF24_WriteRegister(dev, NRF24_CONFIG, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_PowerUp(NRF24L01* dev, uint8_t powerUp) {
	uint8_t reg = 0;
	if (NRF24_ReadRegister(dev, NRF24_CONFIG, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}

	if (powerUp) {
		reg |= 1 << 1;
	} else {
		reg &= ~(1 << 1);
	}

	if (NRF24_WriteRegister(dev, NRF24_CONFIG, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_RXTXControl(NRF24L01* dev, NRF24_TXRX_STATE rx) {
	uint8_t reg = 0;
	if (NRF24_ReadRegister(dev, NRF24_CONFIG, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}

	if (rx) {
		reg |= 1;
	} else {
		reg &= ~(1);
	}

	if (NRF24_WriteRegister(dev, NRF24_CONFIG, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	dev->STATE = rx;
	return NRF24_OK;
}

NRF24_RESULT NRF24_EnableRXDataReadyIRQ(NRF24L01* dev, uint8_t activate) {
	uint8_t reg = 0;
	if (NRF24_ReadRegister(dev, NRF24_CONFIG, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}

	if (!activate) {
		reg |= 1 << 6;
	} else {
		reg &= ~(1 << 6);
	}

	if (NRF24_WriteRegister(dev, NRF24_CONFIG, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_EnableTXDataSentIRQ(NRF24L01* dev, uint8_t activate) {
	uint8_t reg = 0;
	if (NRF24_ReadRegister(dev, NRF24_CONFIG, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	if (!activate) {
		reg |= 1 << 5;
	} else {
		reg &= ~(1 << 5);
	}
	if (NRF24_WriteRegister(dev, NRF24_CONFIG, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_EnableMaxRetransmitIRQ(NRF24L01* dev, uint8_t activate) {
	uint8_t reg = 0;
	if (NRF24_ReadRegister(dev, NRF24_CONFIG, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	if (!activate) {
		reg |= 1 << 4;
	} else {
		reg &= ~(1 << 4);
	}
	if (NRF24_WriteRegister(dev, NRF24_CONFIG, &reg) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_SetRXAddress_P0(NRF24L01* dev, uint8_t* address) {
	uint8_t rx[5];
	if (NRF24_SendCommand(dev, NRF24_CMD_W_REGISTER | NRF24_RX_ADDR_P0, address, rx,
			5) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_SetTXAddress(NRF24L01* dev, uint8_t* address) {
	uint8_t rx[5];
	if (NRF24_SendCommand(dev, NRF24_CMD_W_REGISTER | NRF24_TX_ADDR, address, rx, 5)
			!= NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_SetRXPayloadWidth_P0(NRF24L01* dev, uint8_t width) {
	width &= 0x3F;
	if (NRF24_WriteRegister(dev, NRF24_RX_PW_P0, &width) != NRF24_OK) {
		return NRF24_ERROR;
	}
	return NRF24_OK;
}

NRF24_RESULT NRF24_SendPacket(NRF24L01* dev, uint8_t* data) {

	dev->BUSY_FLAG = 1;

	NRF24_CE_DISABLE(dev);
	NRF24_RXTXControl(dev, NRF24_STATE_TX);
	NRF24_WriteTXPayload(dev, data);
	NRF24_CE_ENABLE(dev);

	while (dev->BUSY_FLAG == 1);	// wait for end of transmittion

	return NRF24_OK;
}

NRF24_RESULT NRF24_ReceivePacket(NRF24L01* dev, uint8_t* data) {

	dev->BUSY_FLAG = 1;

	NRF24_CE_DISABLE(dev);
	NRF24_RXTXControl(dev, NRF24_STATE_RX);
	NRF24_CE_ENABLE(dev);

	while (dev->BUSY_FLAG == 1);	// wait for reception

	int i = 0;
	for (i = 0; i < dev->PayloadLength; i++) {
		data[i] = dev->RX_BUFFER[i];
	}

	return NRF24_OK;
}

NRF24_RESULT NRF24_PushPacket(NRF24L01* dev, uint8_t* data) {

	if(dev->BUSY_FLAG==1){
		NRF24_FlushTX(dev);
	}else{
		dev->BUSY_FLAG = 1;
	}
	NRF24_CE_DISABLE(dev);
	NRF24_RXTXControl(dev, NRF24_STATE_TX);
	NRF24_WriteTXPayload(dev, data);
	NRF24_CE_ENABLE(dev);

	return NRF24_OK;
}

NRF24_RESULT NRF24_PullPacket(NRF24L01* dev, uint8_t* data) {

	int i = 0;
	for (i = 0; i < dev->PayloadLength; i++) {
		data[i] = dev->RX_BUFFER[i];
	}

	return NRF24_OK;
}

NRF24_RESULT NRF24_EnableAckPayload(NRF24L01* dev, uint8_t activate) {
    uint8_t reg = 0;
    if (NRF24_ReadRegister(dev, NRF24_FEATURE, &reg) != NRF24_OK) {
        return NRF24_ERROR;
    }

    if (activate) {
        reg |= 1 << 1;  // EN_ACK_PAY bit
		reg |= 1 << 2;  // EN_DPL bit
    } else {
        reg &= ~(1 << 1);
		reg &= ~(1 << 2);
    }

    if (NRF24_WriteRegister(dev, NRF24_FEATURE, &reg) != NRF24_OK) {
        return NRF24_ERROR;
    }
    return NRF24_OK;
}

NRF24_RESULT NRF24_WriteAckPayload(NRF24L01* dev, uint8_t pipe, uint8_t* data, uint8_t len) {
    uint8_t rx[32];
    if (len > 32) len = 32;
    
    // Command: W_ACK_PAYLOAD | pipe (0xA8 | pipe)
if (NRF24_SendCommand(dev, NRF24_CMD_W_ACK_PAYLOAD | (pipe & 0x07), data, rx, len) != NRF24_OK) {
    return NRF24_ERROR;
    }
    return NRF24_OK;
}