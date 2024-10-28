/*
Library:					NRF24L01/NRF24L01+
Written by:					Mohamed Yaqoob (MYaqoobEmbedded YouTube Channel)
Date Written:				10/11/2018
Last modified:				28/10/2024 by fectec
Description:				This is an STM32 device driver library for the NRF24L01 Nordic Radio transceiver, using STM HAL libraries

References:					This library was written based on the Arduino NRF24 Open-Source library by J. Coliz and the NRF24 datasheet
									- https:// github.com/maniacbug/RF24
									- https:// www.sparkfun.com/datasheets/Components/SMD/nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf
										
*	Copyright (C) 2018 - M. Yaqoob
	This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
	of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.

	This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
	or indirectly by this software, read more about this on the GNU General Public License.
*/

#include "nRF24.h"

// Variables

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define _BOOL(x) (((x)>0) ? 1:0)

// Library variables

static uint64_t pipe0_reading_address;
static bool ack_payload_available; 			// Whether there is an AckPayload waiting
static uint8_t ack_payload_length; 			// Dynamic size of pending AckPayload
static uint8_t payload_size; 				// Fixed size of payloads
static bool dynamic_payloads_enabled; 		// Whether dynamic payloads are enabled
static bool p_variant; 						// False for RF24L01 and true for RF24L01P
static bool wide_band; 						// 2Mbs data rate in use

// NRF24L01 pins and handles

// CE and CSN pins

static GPIO_TypeDef			*NRF24_PORT;
static uint16_t				NRF24_CSN_PIN;
static uint16_t				NRF24_CE_PIN;

// SPI handle

static SPI_HandleTypeDef NRF24_hspi;

// Debug UART handle

static UART_HandleTypeDef NRF24_huart;

void NRF24_DelayMicroSeconds(uint32_t uSec)
{
	uint32_t uSecVar = uSec;
	uSecVar = uSecVar* ((SystemCoreClock / 1000000) / 3);
	while(uSecVar--);
}

// 1. Chip Select function

void NRF24_CSN(int mode)
{
	if(mode) HAL_GPIO_WritePin(NRF24_PORT, NRF24_CSN_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(NRF24_PORT, NRF24_CSN_PIN, GPIO_PIN_RESET);
}

// 2. Chip Enable

void NRF24_CE(int level)
{
	if(level) HAL_GPIO_WritePin(NRF24_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(NRF24_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}

// 3. Read single byte from a register

uint8_t NRF24_Read_Register(uint8_t reg)
{
	uint8_t spiBuf[3];
	uint8_t retData;

	// Put CSN low

	NRF24_CSN(0);

	// Transmit register address

	spiBuf[0] = reg&0x1F;
	HAL_SPI_Transmit(&NRF24_hspi, spiBuf, 1, 100);

	// Receive data

	HAL_SPI_Receive(&NRF24_hspi, &spiBuf[1], 1, 100);
	retData = spiBuf[1];

	// Bring CSN high

	NRF24_CSN(1);
	return retData;
}

// 4. Read multiple bytes register

void NRF24_Read_Register_N(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t spiBuf[3];

	// Put CSN low

	NRF24_CSN(0);

	// Transmit register address

	spiBuf[0] = reg&0x1F;

	HAL_SPI_Transmit(&NRF24_hspi, spiBuf, 1, 100);

	// Receive data

	HAL_SPI_Receive(&NRF24_hspi, buf, len, 100);

	// Bring CSN high

	NRF24_CSN(1);
}

// 5. Write single byte register

void NRF24_Write_Register(uint8_t reg, uint8_t value)
{
	uint8_t spiBuf[3];

	// Put CSN low

	NRF24_CSN(0);

	// Transmit register address and data

	spiBuf[0] = reg|0x20;
	spiBuf[1] = value;
	HAL_SPI_Transmit(&NRF24_hspi, spiBuf, 2, 100);

	// Bring CSN high

	NRF24_CSN(1);
}
// 6. Write multiple bytes register

void NRF24_Write_Register_N(uint8_t reg, const uint8_t* buf, uint8_t len)
{
	uint8_t spiBuf[3];

	// Put CSN low

	NRF24_CSN(0);

	// Transmit register address and data

	spiBuf[0] = reg|0x20;
	HAL_SPI_Transmit(&NRF24_hspi, spiBuf, 1, 100);
	HAL_SPI_Transmit(&NRF24_hspi, (uint8_t*)buf, len, 100);

	// Bring CSN high

	NRF24_CSN(1);
}

// 7. Write transmit payload

void NRF24_Write_Payload(const void* buf, uint8_t len)
{
	uint8_t wrPayloadCmd;

	// Bring CSN low

	NRF24_CSN(0);

	// Send Write TX payload command followed by pbuf data

	wrPayloadCmd = CMD_W_TX_PAYLOAD;
	HAL_SPI_Transmit(&NRF24_hspi, &wrPayloadCmd, 1, 100);
	HAL_SPI_Transmit(&NRF24_hspi, (uint8_t *)buf, len, 100);

	// Bring CSN high

	NRF24_CSN(1);
}

// 8. Read receive payload

void NRF24_Read_Payload(void* buf, uint8_t len)
{
	uint8_t cmdRxBuf;

	// Get data length using payload size

	uint8_t data_len = MIN(len, NRF24_GetPayloadSize());

	// Read data from RX payload buffer

	NRF24_CSN(0);
	cmdRxBuf = CMD_R_RX_PAYLOAD;
	HAL_SPI_Transmit(&NRF24_hspi, &cmdRxBuf, 1, 100);
	HAL_SPI_Receive(&NRF24_hspi, buf, data_len, 100);
	NRF24_CSN(1);
}

// 9. Flush TX buffer

void NRF24_Flush_TX(void)
{
	NRF24_Write_Register(CMD_FLUSH_TX, 0xFF);
}

// 10. Flush RX buffer

void NRF24_Flush_RX(void)
{
	NRF24_Write_Register(CMD_FLUSH_RX, 0xFF);
}

// 11. Get status register value

uint8_t NRF24_Get_Status(void)
{
	uint8_t statReg;
	statReg = NRF24_Read_Register(REG_STATUS);
	return statReg;
}

// 12. Begin function

void NRF24_Begin(GPIO_TypeDef *NRF_PORT, uint16_t NRF_CSN_PIN, uint16_t NRF_CE_PIN, SPI_HandleTypeDef NRF_SPI)
{
	// Copy SPI handle variable

	memcpy(&NRF24_hspi, &NRF_SPI, sizeof(NRF_SPI));

	// Copy pins and port variables

	NRF24_PORT = NRF_PORT;
	NRF24_CSN_PIN = NRF_CSN_PIN;
	NRF24_CE_PIN = NRF_CE_PIN;
	
	// Put pins to idle state

	NRF24_CSN(1);
	NRF24_CE(0);

	// 5 ms initial delay

	HAL_Delay(5);
	
	// Soft reset registers default values

	NRF24_Write_Register(0x00, 0x08);
	NRF24_Write_Register(0x01, 0x3f);
	NRF24_Write_Register(0x02, 0x03);
	NRF24_Write_Register(0x03, 0x03);
	NRF24_Write_Register(0x04, 0x03);
	NRF24_Write_Register(0x05, 0x02);
	NRF24_Write_Register(0x06, 0x0f);
	NRF24_Write_Register(0x07, 0x0e);
	NRF24_Write_Register(0x08, 0x00);
	NRF24_Write_Register(0x09, 0x00);

	uint8_t pipeAddrVar[6];

	pipeAddrVar[4]=0xE7; pipeAddrVar[3]=0xE7; pipeAddrVar[2]=0xE7; pipeAddrVar[1]=0xE7; pipeAddrVar[0]=0xE7; 
	NRF24_Write_Register_N(0x0A, pipeAddrVar, 5);

	pipeAddrVar[4]=0xC2; pipeAddrVar[3]=0xC2; pipeAddrVar[2]=0xC2; pipeAddrVar[1]=0xC2; pipeAddrVar[0]=0xC2; 
	NRF24_Write_Register_N(0x0B, pipeAddrVar, 5);

	NRF24_Write_Register(0x0C, 0xC3);
	NRF24_Write_Register(0x0D, 0xC4);
	NRF24_Write_Register(0x0E, 0xC5);
	NRF24_Write_Register(0x0F, 0xC6);

	pipeAddrVar[4]=0xE7; pipeAddrVar[3]=0xE7; pipeAddrVar[2]=0xE7; pipeAddrVar[1]=0xE7; pipeAddrVar[0]=0xE7; 

	NRF24_Write_Register_N(0x10, pipeAddrVar, 5);
	NRF24_Write_Register(0x11, 0);
	NRF24_Write_Register(0x12, 0);
	NRF24_Write_Register(0x13, 0);
	NRF24_Write_Register(0x14, 0);
	NRF24_Write_Register(0x15, 0);
	NRF24_Write_Register(0x16, 0);
	
	NRF24_Activate_CMD();
	NRF24_Write_Register(0x1c, 0);
	NRF24_Write_Register(0x1d, 0);
	NRF24_Print_Radio_Settings();


	// Initialize retries 15 and delay 1250 us

	NRF24_SetRetries(15, 15);

	// Initialize PA level to max (0dB)

	NRF24_SetPALevel(RF24_PA_0dB);

	// Initialize data rate to 2Mbps

	NRF24_SetDataRate(RF24_2MBPS);

	// Initialize CRC length to 16-bit (2 bytes)

	NRF24_SetCRCLength(RF24_CRC_16);

	// Disable dynamic payload

	NRF24_DisableDynamicPayloads();

	// Set payload size

	NRF24_SetPayloadSize(32);
	
	// Reset status register

	NRF24_Reset_Status();

	// Initialize channel to 52

	NRF24_SetChannel(52);

	// Disable AutoAck

	NRF24_SetAutoAck(false);

	// Flush buffers

	NRF24_Flush_TX();
	NRF24_Flush_RX();
	
	NRF24_PowerDown();

}

// 13. Listen on open pipes for reading (Must call NRF24_OpenReadingPipe() first)

void NRF24_StartListening(void)
{
    // Power up and set to RX mode

    NRF24_Write_Register(REG_CONFIG, NRF24_Read_Register(REG_CONFIG) | (1UL << 1) | (1UL << 0));

    // Restore pipe 0 address if exists

    if (pipe0_reading_address)
        NRF24_Write_Register_N(REG_RX_ADDR_P0, (uint8_t *)(&pipe0_reading_address), 5);

    // Flush buffers

    NRF24_Flush_TX();
    NRF24_Flush_RX();

    // Set CE HIGH to start listening

    NRF24_CE(1);

    // Wait for 130 uSec for the radio to come on

    NRF24_DelayMicroSeconds(150);
}

// 14. Stop listening (essential before any write operation)

void NRF24_StopListening(void)
{
    NRF24_CE(0);
    NRF24_Flush_TX();
    NRF24_Flush_RX();
}

// 15. Write (transmit data), returns true if successfully sent

bool NRF24_Write(const void* buf, uint8_t len)
{
    bool retStatus;

    // Start writing

    NRF24_Reset_Status();
    NRF24_StartWrite(buf, len);

    // Data monitor

    uint8_t observe_tx;
    uint8_t status;
    uint32_t sent_at = HAL_GetTick();
    const uint32_t timeout = 10;		// ms to wait for timeout

    do
    {
        NRF24_Read_Register_N(REG_OBSERVE_TX, &observe_tx, 1);

        // Get status register

        status = NRF24_Get_Status();
    }
    while (!(status & (_BV(BIT_TX_DS) | _BV(BIT_MAX_RT)) && (HAL_GetTick() - sent_at < timeout)));

    // printConfigReg();
    // printStatusReg();

    bool tx_ok, tx_fail;
    NRF24_What_Happened(&tx_ok, &tx_fail, &ack_payload_available);
    retStatus = tx_ok;

    if (ack_payload_available)
    {
        ack_payload_length = NRF24_GetDynamicPayloadSize();
    }

    // Power down

    NRF24_Available();
    NRF24_Flush_TX();

    return retStatus;
}

// 16. Check for available data to read

bool NRF24_Available(void)
{
    return NRF24_AvailablePipe(NULL);
}

// 17. Read received data

bool NRF24_Read(void* buf, uint8_t len)
{
    NRF24_Read_Payload(buf, len);
    uint8_t rxStatus = NRF24_Read_Register(REG_FIFO_STATUS) & _BV(BIT_RX_EMPTY);
    NRF24_Flush_RX();
    NRF24_GetDynamicPayloadSize();

    return rxStatus;
}

// 18. Open TX pipe for writing (cannot perform this while listening, has to call NRF24_StopListening)

void NRF24_OpenWritingPipe(uint64_t address)
{
    NRF24_Write_Register_N(REG_RX_ADDR_P0, (uint8_t *)(&address), 5);
    NRF24_Write_Register_N(REG_TX_ADDR, (uint8_t *)(&address), 5);

    const uint8_t max_payload_size = 32;
    NRF24_Write_Register(REG_RX_PW_P0, MIN(payload_size, max_payload_size));
}

// 19. Open reading pipe

void NRF24_OpenReadingPipe(uint8_t number, uint64_t address)
{
    if (number == 0) pipe0_reading_address = address;

    if (number <= 6)
    {
        if (number < 2)
        {
            // Address width is 5 bytes

            NRF24_Write_Register_N(NRF24_ADDR_REGS[number], (uint8_t *)(&address), 5);
        }
        else
        {
            NRF24_Write_Register_N(NRF24_ADDR_REGS[number], (uint8_t *)(&address), 1);
        }

        // Write payload size

        NRF24_Write_Register(RF24_RX_PW_PIPE[number], payload_size);

        // Enable pipe

        NRF24_Write_Register(REG_EN_RXADDR, NRF24_Read_Register(REG_EN_RXADDR) | _BV(number));
    }
}

// 20. Set transmit retries (rf24_Retries_e) and delay

void NRF24_SetRetries(uint8_t delay, uint8_t count)
{
    NRF24_Write_Register(REG_SETUP_RETR, (delay & 0xf) << BIT_ARD | (count & 0xf) << BIT_ARC);
}

// 21. Set RF channel frequency

void NRF24_SetChannel(uint8_t channel)
{
    const uint8_t max_channel = 127;
    NRF24_Write_Register(REG_RF_CH, MIN(channel, max_channel));
}

// 22. Set payload size

void NRF24_SetPayloadSize(uint8_t size)
{
    const uint8_t max_payload_size = 32;
    payload_size = MIN(size, max_payload_size);
}

// 23. Get payload size

uint8_t NRF24_GetPayloadSize(void)
{
    return payload_size;
}

// 24. Get dynamic payload size, of latest packet received

uint8_t NRF24_GetDynamicPayloadSize(void)
{
    return NRF24_Read_Register(CMD_R_RX_PL_WID);
}

// 25. Enable payload on acknowledge packet

void NRF24_EnableAckPayload(void)
{
    // Need to enable dynamic payload and AckPayload together

    NRF24_Write_Register(REG_FEATURE, NRF24_Read_Register(REG_FEATURE) | _BV(BIT_EN_ACK_PAY) | _BV(BIT_EN_DPL));

    if (!NRF24_Read_Register(REG_FEATURE))
    {
        NRF24_Activate_CMD();
        NRF24_Write_Register(REG_FEATURE, NRF24_Read_Register(REG_FEATURE) | _BV(BIT_EN_ACK_PAY) | _BV(BIT_EN_DPL));
    }

    // Enable dynamic payload on pipes 0 & 1

    NRF24_Write_Register(REG_DYNPD, NRF24_Read_Register(REG_DYNPD) | _BV(BIT_DPL_P1) | _BV(BIT_DPL_P0));
}

// 26. Enable dynamic payloads

void NRF24_EnableDynamicPayloads(void)
{
    // Enable dynamic payload through feature register

    NRF24_Write_Register(REG_FEATURE, NRF24_Read_Register(REG_FEATURE) | _BV(BIT_EN_DPL));

    if (!NRF24_Read_Register(REG_FEATURE))
    {
        NRF24_Activate_CMD();
        NRF24_Write_Register(REG_FEATURE, NRF24_Read_Register(REG_FEATURE) | _BV(BIT_EN_DPL));
    }

    // Enable dynamic payload on all pipes

    NRF24_Write_Register(REG_DYNPD, NRF24_Read_Register(REG_DYNPD) | _BV(BIT_DPL_P5) | _BV(BIT_DPL_P4) | _BV(BIT_DPL_P3) | _BV(BIT_DPL_P2) | _BV(BIT_DPL_P1) | _BV(BIT_DPL_P0));

    dynamic_payloads_enabled = true;
}

void NRF24_DisableDynamicPayloads(void)
{
    NRF24_Write_Register(REG_FEATURE, NRF24_Read_Register(REG_FEATURE) & ~(_BV(BIT_EN_DPL)));

    // Disable for all pipes

    NRF24_Write_Register(REG_DYNPD, 0);
    dynamic_payloads_enabled = false;
}

// 27. Check if module is NRF24L01+ or normal module

bool NRF24_IsNRF_Plus(void)
{
    return p_variant;
}

// 28. Set Auto Ack for all

void NRF24_SetAutoAck(bool enable)
{
    if (enable)
        NRF24_Write_Register(REG_EN_AA, 0x3F);
    else
        NRF24_Write_Register(REG_EN_AA, 0x00);
}

// 29. Set Auto Ack for certain pipe

void NRF24_SetAutoAckPipe(uint8_t pipe, bool enable)
{
    if (pipe <= 6)
    {
        uint8_t en_aa = NRF24_Read_Register(REG_EN_AA);

        if (enable)
        {
            en_aa |= _BV(pipe);
        }
        else
        {
            en_aa &= ~_BV(pipe);
        }

        NRF24_Write_Register(REG_EN_AA, en_aa);
    }
}

// 30. Set transmit power level

void NRF24_SetPALevel(rf24_pa_dbm_e level)
{
    uint8_t setup = NRF24_Read_Register(REG_RF_SETUP);
    setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));

    // Switch uses RAM (Evil!)

    if (level == RF24_PA_0dB)
    {
        setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));
    }
    else if (level == RF24_PA_m6dB)
    {
        setup |= _BV(RF_PWR_HIGH);
    }
    else if (level == RF24_PA_m12dB)
    {
        setup |= _BV(RF_PWR_LOW);
    }
    else if (level == RF24_PA_m18dB)
    {
        // Nothing
    }
    else if (level == RF24_PA_ERROR)
    {
        // On error, go to maximum PA

        setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));
    }

    NRF24_Write_Register(REG_RF_SETUP, setup);
}

// 31. Get transmit power level

rf24_pa_dbm_e NRF24_GetPALevel(void)
{
    rf24_pa_dbm_e result = RF24_PA_ERROR;
    uint8_t power = NRF24_Read_Register(REG_RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));

    // Switch uses RAM (Evil!)

    if (power == (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)))
    {
        result = RF24_PA_0dB;
    }
    else if (power == _BV(RF_PWR_HIGH))
    {
        result = RF24_PA_m6dB;
    }
    else if (power == _BV(RF_PWR_LOW))
    {
        result = RF24_PA_m12dB;
    }
    else
    {
        result = RF24_PA_m18dB;
    }

    return result;
}

// 32. Set data rate (250 Kbps, 1Mbps, 2Mbps)

bool NRF24_SetDataRate(rf24_datarate_e speed)
{
    bool result = false;
    uint8_t setup = NRF24_Read_Register(REG_RF_SETUP);

    // HIGH and LOW '00' is 1Mbs - Our default

    wide_band = false;
    setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    if (speed == RF24_250KBPS)
    {
        // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
        // Making it '10'.

        wide_band = false;
        setup |= _BV(RF_DR_LOW);
    }
    else
    {
        // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
        // Making it '01'

        if (speed == RF24_2MBPS)
        {
            wide_band = true;
            setup |= _BV(RF_DR_HIGH);
        }
        else
        {
            // 1Mbs

            wide_band = false;
        }
    }

    NRF24_Write_Register(REG_RF_SETUP, setup);

    // Verify our result

    if (NRF24_Read_Register(REG_RF_SETUP) == setup)
    {
        result = true;
    }
    else
    {
        wide_band = false;
    }

    return result;
}

// 33. Get data rate

rf24_datarate_e NRF24_GetDataRate(void)
{
    rf24_datarate_e result;
    uint8_t dr = NRF24_Read_Register(REG_RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    // Switch uses RAM (Evil!)
    // Order matters in our case below

    if (dr == _BV(RF_DR_LOW))
    {
    	// '10' = 250KBPS

        result = RF24_250KBPS;
    }
    else if (dr == _BV(RF_DR_HIGH))
    {
        // '01' = 2MBPS

        result = RF24_2MBPS;
    }
    else
    {
        // '00' = 1MBPS

        result = RF24_1MBPS;
    }

    return result;
}

// 34. Set CRC length (disable, 8-bits or 16-bits)

void NRF24_SetCRCLength(rf24_crclength_e length)
{
    uint8_t config = NRF24_Read_Register(REG_CONFIG) & ~(_BV(BIT_CRCO) | _BV(BIT_EN_CRC));

    // Switch uses RAM

    if (length == RF24_CRC_DISABLED)
    {
        // Do nothing, we turned it off above
    }
    else if (length == RF24_CRC_8)
    {
        config |= _BV(BIT_EN_CRC);
    }
    else
    {
        config |= _BV(BIT_EN_CRC);
        config |= _BV(BIT_CRCO);
    }

    NRF24_Write_Register(REG_CONFIG, config);
}

// 35. Get CRC length

rf24_crclength_e NRF24_GetCRCLength(void)
{
    rf24_crclength_e result = RF24_CRC_DISABLED;
    uint8_t config = NRF24_Read_Register(REG_CONFIG) & (_BV(BIT_CRCO) | _BV(BIT_EN_CRC));

    if (config & _BV(BIT_EN_CRC))
    {
        if (config & _BV(BIT_CRCO))
            result = RF24_CRC_16;
        else
            result = RF24_CRC_8;
    }

    return result;
}

// 36. Disable CRC

void RF24_DisableCRC(void)
{
    uint8_t disable = NRF24_Read_Register(REG_CONFIG) & ~_BV(BIT_EN_CRC);
    NRF24_Write_Register(REG_CONFIG, disable);
}

// 37. Power up

void NRF24_PowerUp(void)
{
    NRF24_Write_Register(REG_CONFIG, NRF24_Read_Register(REG_CONFIG) | _BV(BIT_PWR_UP));
}

// 38. Power down

void NRF24_PowerDown(void)
{
    NRF24_Write_Register(REG_CONFIG, NRF24_Read_Register(REG_CONFIG) & ~_BV(BIT_PWR_UP));
}

// 39. Check if data are available and on which pipe (use this for multiple RX pipes)

bool NRF24_AvailablePipe(uint8_t* pipe_num)
{
    uint8_t status = NRF24_Get_Status();

    bool result = (status & _BV(BIT_RX_DR));

    if (result)
    {
        // If the caller wants the pipe number, include that

        if (pipe_num)
            *pipe_num = (status >> BIT_RX_P_NO) & 0x7;

        // Clear the status bit

        NRF24_Write_Register(REG_STATUS, _BV(BIT_RX_DR));

        // Handle AckPayload receipt

        if (status & _BV(BIT_TX_DS))
        {
            NRF24_Write_Register(REG_STATUS, _BV(BIT_TX_DS));
        }
    }

    return result;
}

// 40. Start write (for IRQ mode)

void NRF24_StartWrite(const void* buf, uint8_t len)
{
    // Transmitter power-up

    NRF24_CE(0);
    NRF24_Write_Register(REG_CONFIG, (NRF24_Read_Register(REG_CONFIG) | _BV(BIT_PWR_UP)) & ~_BV(BIT_PRIM_RX));
    NRF24_CE(1);
    NRF24_DelayMicroSeconds(150);

    // Send the payload

    NRF24_Write_Payload(buf, len);

    // Enable TX for 15 us

    NRF24_CE(1);
    NRF24_DelayMicroSeconds(15);
    NRF24_CE(0);
}

// 41. Write AckPayload

void NRF24_WriteAckPayload(uint8_t pipe, const void* buf, uint8_t len)

{
	const uint8_t* current = (uint8_t *)buf;
	const uint8_t max_payload_size = 32;
	uint8_t data_len = MIN(len,max_payload_size);

	NRF24_CSN(0);
	NRF24_Write_Register_N(CMD_W_ACK_PAYLOAD | ( pipe & 0x7 ) , current, data_len);
	NRF24_CSN(1);
}

// 42. Check if an AckPayload is available

bool NRF24_IsAckPayloadAvailable(void)
{
	bool result = ack_payload_available;
	ack_payload_available = false;
	return result;
}

// 43. Check interrupt flags

void NRF24_What_Happened(bool *tx_ok, bool *tx_fail, bool *rx_ready)

{
	uint8_t status = NRF24_Get_Status();
	*tx_ok = 0;
	NRF24_Write_Register(REG_STATUS,_BV(BIT_RX_DR) | _BV(BIT_TX_DS) | _BV(BIT_MAX_RT) );

	// Report to the user what happened

	*tx_ok = status & _BV(BIT_TX_DS);
	*tx_fail = status & _BV(BIT_MAX_RT);
	*rx_ready = status & _BV(BIT_RX_DR);
}

// 44. Test if there is a carrier on the previous listening period (useful to check for interference)

bool NRF24_Test_Carrier(void)
{
	return NRF24_Read_Register(REG_CD) & 1;
}


// 45. Test if a signal carrier exists (=> -64dB), only for NRF24L01+

bool NRF24_Test_RPD(void)
{
	return NRF24_Read_Register(REG_RPD) & 1;
}

// 46. Reset status

void NRF24_Reset_Status(void)
{
	NRF24_Write_Register(REG_STATUS,_BV(BIT_RX_DR) | _BV(BIT_TX_DS) | _BV(BIT_MAX_RT) );
}

// 47. Activate CMD

void NRF24_Activate_CMD(void)
{
	uint8_t cmdRxBuf[2];

	// Read data from RX payload buffer

	NRF24_CSN(0);
	cmdRxBuf[0] = CMD_ACTIVATE;
	cmdRxBuf[1] = 0x73;
	HAL_SPI_Transmit(&NRF24_hspi, cmdRxBuf, 2, 100);
	NRF24_CSN(1);
}

// 48. Get AckPayload size

uint8_t NRF24_GetAckPayloadSize(void)
{
	return ack_payload_length;
}

// Debug function prototypes

// 1. Print radio settings

void NRF24_Print_Radio_Settings(void)
{
	uint8_t reg8Val;
	char uartTxBuf[100];
	sprintf(uartTxBuf, "\r\n**********************************************\r\n");
	HAL_UART_Transmit(&NRF24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	// a) Get CRC settings - Configuration Register

	reg8Val = NRF24_Read_Register(0x00);

	if(reg8Val & (1 << 3))
	{
		if(reg8Val & (1 << 2)) sprintf(uartTxBuf, "CRC:\r\n		Enabled, 2 Bytes \r\n");
		else sprintf(uartTxBuf, "CRC:\r\n		Enabled, 1 Byte \r\n");
	}
	else
	{
		sprintf(uartTxBuf, "CRC:\r\n		Disabled \r\n");
	}

	HAL_UART_Transmit(&NRF24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);


	// b) AutoAck on pipes

	reg8Val = NRF24_Read_Register(0x01);

	sprintf(uartTxBuf, "ENAA:\r\n		P0:	%d\r\n		P1:	%d\r\n		P2:	%d\r\n		P3:	%d\r\n		P4:	%d\r\n		P5:	%d\r\n",
		_BOOL(reg8Val&(1<<0)), _BOOL(reg8Val&(1<<1)), _BOOL(reg8Val&(1<<2)), _BOOL(reg8Val&(1<<3)), _BOOL(reg8Val&(1<<4)), _BOOL(reg8Val&(1<<5)));
	HAL_UART_Transmit(&NRF24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	// c) Enabled RX addresses

	reg8Val = NRF24_Read_Register(0x02);

	sprintf(uartTxBuf, "EN_RXADDR:\r\n		P0:	%d\r\n		P1:	%d\r\n		P2:	%d\r\n		P3:	%d\r\n		P4:	%d\r\n		P5:	%d\r\n",
		_BOOL(reg8Val&(1<<0)), _BOOL(reg8Val&(1<<1)), _BOOL(reg8Val&(1<<2)), _BOOL(reg8Val&(1<<3)), _BOOL(reg8Val&(1<<4)), _BOOL(reg8Val&(1<<5)));
	HAL_UART_Transmit(&NRF24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	// d) Address width

	reg8Val = NRF24_Read_Register(0x03)&0x03;
	reg8Val +=2;

	sprintf(uartTxBuf, "SETUP_AW:\r\n		%d bytes \r\n", reg8Val);
	HAL_UART_Transmit(&NRF24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	// e) RF channel

	reg8Val = NRF24_Read_Register(0x05);

	sprintf(uartTxBuf, "RF_CH:\r\n		%d CH \r\n", reg8Val&0x7F);
	HAL_UART_Transmit(&NRF24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	// f) Data rate & RF_PWR

	reg8Val = NRF24_Read_Register(0x06);

	if(reg8Val & (1 << 3)) sprintf(uartTxBuf, "Data Rate:\r\n		2Mbps \r\n");
	else sprintf(uartTxBuf, "Data Rate:\r\n		1Mbps \r\n");

	HAL_UART_Transmit(&NRF24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	reg8Val &= (3 << 1);
	reg8Val = (reg8Val>>1);

	if(reg8Val == 0) sprintf(uartTxBuf, "RF_PWR:\r\n		-18dB \r\n");
	else if(reg8Val == 1) sprintf(uartTxBuf, "RF_PWR:\r\n		-12dB \r\n");
	else if(reg8Val == 2) sprintf(uartTxBuf, "RF_PWR:\r\n		-6dB \r\n");
	else if(reg8Val == 3) sprintf(uartTxBuf, "RF_PWR:\r\n		0dB \r\n");

	HAL_UART_Transmit(&NRF24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	// g) RX pipes addresses

	uint8_t pipeAddrs[6];

	NRF24_Read_Register_N(0x0A, pipeAddrs, 5);

	sprintf(uartTxBuf, "RX_Pipe0 Addrs:\r\n		%02X,%02X,%02X,%02X,%02X  \r\n", pipeAddrs[4], pipeAddrs[3], pipeAddrs[2],pipeAddrs[1],pipeAddrs[0]);
	HAL_UART_Transmit(&NRF24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	NRF24_Read_Register_N(0x0A+1, pipeAddrs, 5);

	sprintf(uartTxBuf, "RX_Pipe1 Addrs:\r\n		%02X,%02X,%02X,%02X,%02X  \r\n", pipeAddrs[4], pipeAddrs[3], pipeAddrs[2],pipeAddrs[1],pipeAddrs[0]);
	HAL_UART_Transmit(&NRF24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	NRF24_Read_Register_N(0x0A+2, pipeAddrs, 1);

	sprintf(uartTxBuf, "RX_Pipe2 Addrs:\r\n		xx,xx,xx,xx,%02X  \r\n", pipeAddrs[0]);
	HAL_UART_Transmit(&NRF24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	NRF24_Read_Register_N(0x0A+3, pipeAddrs, 1);

	sprintf(uartTxBuf, "RX_Pipe3 Addrs:\r\n		xx,xx,xx,xx,%02X  \r\n", pipeAddrs[0]);
	HAL_UART_Transmit(&NRF24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	NRF24_Read_Register_N(0x0A+4, pipeAddrs, 1);

	sprintf(uartTxBuf, "RX_Pipe4 Addrs:\r\n		xx,xx,xx,xx,%02X  \r\n", pipeAddrs[0]);
	HAL_UART_Transmit(&NRF24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	NRF24_Read_Register_N(0x0A+5, pipeAddrs, 1);

	sprintf(uartTxBuf, "RX_Pipe5 Addrs:\r\n		xx,xx,xx,xx,%02X  \r\n", pipeAddrs[0]);
	HAL_UART_Transmit(&NRF24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	sprintf(uartTxBuf, "**********************************************\r\n");
	HAL_UART_Transmit(&NRF24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
}
