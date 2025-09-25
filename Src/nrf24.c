/*
 *  nrf24l01_plus.c
 *
 *  Created on: 2021. 7. 20.
 *      Author: mokhwasomssi
 * 
 */


#include "nrf24.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;
#define NRF24_SPI                     (&hspi1)


static void cs_high()
{
    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);
}

static void cs_low()
{
    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET);
}

static void ce_high()
{
    HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_SET);
}

static void ce_low()
{
    HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_RESET);
}

static uint8_t read_register(uint8_t reg)
{
    uint8_t command = NRF24_CMD_R_REGISTER | reg;
    uint8_t status;
    uint8_t read_val;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24_SPI, &command, &status, 1, 2000);
    HAL_SPI_Receive(NRF24_SPI, &read_val, 1, 2000);
    cs_high();

    return read_val;
}

static uint8_t write_register(uint8_t reg, uint8_t value)
{
    uint8_t command = NRF24_CMD_W_REGISTER | reg;
    uint8_t status;
    uint8_t write_val = value;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24_SPI, &write_val, 1, 2000);
    cs_high();

    return write_val;
}


/* nRF24L01+ Main Functions */
void nrf24_rx_init(channel MHz, air_data_rate bps)
{
    nrf24_reset();

    nrf24_prx_mode();
    nrf24_power_up();

    nrf24_rx_set_payload_widths(NRF24_PAYLOAD_LENGTH);

    nrf24_set_rf_channel(MHz);
    nrf24_set_rf_air_data_rate(bps);
    nrf24_set_rf_tx_output_power(_0dBm);

    nrf24_set_crc_length(1);
    nrf24_set_address_widths(5);

    nrf24_auto_retransmit_count(3);
    nrf24_auto_retransmit_delay(250);
    
    ce_high();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,RESET);
}

void nrf24_tx_init(channel MHz, air_data_rate bps)
{
    nrf24_reset();

    nrf24_ptx_mode();
    nrf24_power_up();

    nrf24_set_rf_channel(MHz);
    nrf24_set_rf_air_data_rate(bps);
    nrf24_set_rf_tx_output_power(_0dBm);

    nrf24_set_crc_length(1);
    nrf24_set_address_widths(5);

    nrf24_auto_retransmit_count(3);
    nrf24_auto_retransmit_delay(250);

    ce_high();

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,RESET);

}

void nrf24_rx_receive(uint8_t* rx_payload)
{
    //printf("RECEIVING\r\n");
    nrf24_read_rx_fifo(rx_payload);
    //printf("CLEARING RX \r\n");
    nrf24_clear_rx_dr();
    //printf("Receive done \r\n");
    nrf24_flush_rx_fifo();
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void nrf24_tx_transmit(uint8_t* tx_payload)
{
    nrf24_write_tx_fifo(tx_payload);
}

void nrf24_tx_irq()
{
    uint8_t tx_ds = nrf24_get_status();
    tx_ds &= 0x20;

    if(tx_ds)
    {   
        // TX_DS
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        nrf24_clear_tx_ds();
    }

    else
    {
        // MAX_RT
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
        nrf24_clear_max_rt();
    }
}

/* nRF24L01+ Sub Functions */
void nrf24_reset()
{
    // Reset pins
    cs_high();
    ce_low();

    // Reset registers
    write_register(NRF24_REG_CONFIG, 0x08);
    write_register(NRF24_REG_EN_AA, 0x3F);
    write_register(NRF24_REG_EN_RXADDR, 0x03);
    write_register(NRF24_REG_SETUP_AW, 0x03);
    write_register(NRF24_REG_SETUP_RETR, 0x03);
    write_register(NRF24_REG_RF_CH, 0x02);
    write_register(NRF24_REG_RF_SETUP, 0x07);
    write_register(NRF24_REG_STATUS, 0x7E);
    write_register(NRF24_REG_RX_PW_P0, 0x00);
    write_register(NRF24_REG_RX_PW_P0, 0x00);
    write_register(NRF24_REG_RX_PW_P1, 0x00);
    write_register(NRF24_REG_RX_PW_P2, 0x00);
    write_register(NRF24_REG_RX_PW_P3, 0x00);
    write_register(NRF24_REG_RX_PW_P4, 0x00);
    write_register(NRF24_REG_RX_PW_P5, 0x00);
    write_register(NRF24_REG_FIFO_STATUS, 0x11);
    write_register(NRF24_REG_DYNPD, 0x00);
    write_register(NRF24_REG_FEATURE, 0x00);

    // Reset FIFO
    nrf24_flush_rx_fifo();
    nrf24_flush_tx_fifo();
}

void nrf24_prx_mode()
{
    uint8_t new_config = read_register(NRF24_REG_CONFIG);
    new_config |= 1 << 0;

    write_register(NRF24_REG_CONFIG, new_config);
}

void nrf24_ptx_mode()
{
    uint8_t new_config = read_register(NRF24_REG_CONFIG);
    new_config &= 0xFE;

    write_register(NRF24_REG_CONFIG, new_config);
}

uint8_t nrf24_read_rx_fifo(uint8_t* rx_payload)
{
    uint8_t command = NRF24_CMD_R_RX_PAYLOAD;
    uint8_t status;
    cs_low();
    //printf("Sending RX_PAYLOAD CMD\r\n");
    HAL_SPI_Transmit(NRF24_SPI, &command, 1, 1000);
    status = HAL_SPI_Receive(NRF24_SPI, rx_payload, NRF24_PAYLOAD_LENGTH, 1000);
        cs_high();
    //printf("READ FIFO WAIT 1ms\r\n");
    if (status == HAL_OK)
        printf("Receive OK\r\n");
    else
        printf("Receive ERROR: %u\r\n",status);
    return status;
}

uint8_t nrf24_write_tx_fifo(uint8_t* tx_payload)
{
    uint8_t command = NRF24_CMD_W_TX_PAYLOAD;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24_SPI, &command, &status, 1, 1000);
    HAL_SPI_Transmit(NRF24_SPI, tx_payload, NRF24_PAYLOAD_LENGTH, 1000);
    cs_high(); 

    return status;
}

void nrf24_flush_rx_fifo()
{
    uint8_t command = NRF24_CMD_FLUSH_RX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24_SPI, &command, &status, 1, 1000);
    cs_high();
}

void nrf24_flush_tx_fifo()
{
    uint8_t command = NRF24_CMD_FLUSH_TX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24_SPI, &command, &status, 1, 1000);
    cs_high();
}

uint8_t nrf24_get_status()
{
    uint8_t command = NRF24_CMD_NOP;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24_SPI, &command, &status, 1, 1000);
    cs_high(); 

    return status;
}

uint8_t nrf24_get_fifo_status()
{
    return read_register(NRF24_REG_FIFO_STATUS);
}

void nrf24_rx_set_payload_widths(widths bytes)
{
    write_register(NRF24_REG_RX_PW_P0, bytes);
}

void nrf24_clear_rx_dr()
{
    uint8_t new_status = nrf24_get_status();
    new_status |= 0x40;
    write_register(NRF24_REG_STATUS, new_status); // Only RX_DR bit set
}

void nrf24_clear_tx_ds()
{
    uint8_t new_status = nrf24_get_status();
    new_status |= 0x20;

    write_register(NRF24_REG_STATUS, new_status);     
}

void nrf24_clear_max_rt()
{
    uint8_t new_status = nrf24_get_status();
    new_status |= 0x10;

    write_register(NRF24_REG_STATUS, new_status); 
}

void nrf24_power_up()
{
    uint8_t new_config = read_register(NRF24_REG_CONFIG);
    new_config |= 1 << 1;

    write_register(NRF24_REG_CONFIG, new_config);
}

void nrf24_power_down()
{
    uint8_t new_config = read_register(NRF24_REG_CONFIG);
    new_config &= 0xFD;

    write_register(NRF24_REG_CONFIG, new_config);
}

void nrf24_set_crc_length(length bytes)
{
    uint8_t new_config = read_register(NRF24_REG_CONFIG);
    
    switch(bytes)
    {
        // CRCO bit in CONFIG resiger set 0
        case 1:
            new_config &= 0xFB;
            break;
        // CRCO bit in CONFIG resiger set 1
        case 2:
            new_config |= 1 << 2;
            break;
    }

    write_register(NRF24_REG_CONFIG, new_config);
}

void nrf24_set_address_widths(widths bytes)
{
    write_register(NRF24_REG_SETUP_AW, bytes - 2);
}

void nrf24_auto_retransmit_count(count cnt)
{
    uint8_t new_setup_retr = read_register(NRF24_REG_SETUP_RETR);
    
    // Reset ARC register 0
    new_setup_retr |= 0xF0;
    new_setup_retr |= cnt;
    write_register(NRF24_REG_SETUP_RETR, new_setup_retr);
}

void nrf24_auto_retransmit_delay(delay us)
{
    uint8_t new_setup_retr = read_register(NRF24_REG_SETUP_RETR);

    // Reset ARD register 0
    new_setup_retr |= 0x0F;
    new_setup_retr |= ((us / 250) - 1) << 4;
    write_register(NRF24_REG_SETUP_RETR, new_setup_retr);
}

void nrf24_set_rf_channel(channel MHz)
{
	uint16_t new_rf_ch = MHz - 2400;
    write_register(NRF24_REG_RF_CH, new_rf_ch);
}

void nrf24_set_rf_tx_output_power(output_power dBm)
{
    uint8_t new_rf_setup = read_register(NRF24_REG_RF_SETUP) & 0xF9;
    new_rf_setup |= (dBm << 1);

    write_register(NRF24_REG_RF_SETUP, new_rf_setup);
}

void nrf24_set_rf_air_data_rate(air_data_rate bps)
{
    // Set value to 0
    uint8_t new_rf_setup = read_register(NRF24_REG_RF_SETUP) & 0xD7;
    
    switch(bps)
    {
        case _1Mbps: 
            break;
        case _2Mbps: 
            new_rf_setup |= 1 << 3;
            break;
        case _250kbps:
            new_rf_setup |= 1 << 5;
            break;
    }
    write_register(NRF24_REG_RF_SETUP, new_rf_setup);
}