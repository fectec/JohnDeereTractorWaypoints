/*
 * nRF24.h
 */

#ifndef INC_NRF24_H_
#define INC_NRF24_H_

// Includes

#include "main.h"
#include "nRF24L01.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

// TypeDefs

// 1. Power amplifier function, NRF24_SetPALevel()

typedef enum {
	RF24_PA_m18dB = 0,
	RF24_PA_m12dB,
	RF24_PA_m6dB,
	RF24_PA_0dB,
	RF24_PA_ERROR
} rf24_pa_dbm_e;

// 2. NRF24_SetDataRate() input

typedef enum {
	RF24_1MBPS = 0,
	RF24_2MBPS,
	RF24_250KBPS
} rf24_datarate_e;

// 3. NRF24_SetCRCLength() input

typedef enum {
	RF24_CRC_DISABLED = 0,
	RF24_CRC_8,
	RF24_CRC_16
} rf24_crclength_e;

// 4. Pipe address registers

static const uint8_t NRF24_ADDR_REGS[7] = {
	REG_RX_ADDR_P0,
	REG_RX_ADDR_P1,
	REG_RX_ADDR_P2,
	REG_RX_ADDR_P3,
	REG_RX_ADDR_P4,
	REG_RX_ADDR_P5,
	REG_TX_ADDR
};

// 5. RX_PW_Px registers addresses

static const uint8_t RF24_RX_PW_PIPE[6] = {
		REG_RX_PW_P0,
		REG_RX_PW_P1,
		REG_RX_PW_P2,
		REG_RX_PW_P3,
		REG_RX_PW_P4,
		REG_RX_PW_P5
};

// Function prototypes

void NRF24_DelayMicroSeconds(uint32_t uSec);

// 1. Chip Select function

void NRF24_CSN(int mode);

// 2. Chip Enable

void NRF24_CE(int level);

// 3. Read single byte from a register

uint8_t NRF24_Read_Register(uint8_t reg);

// 4. Read multiple bytes register

void NRF24_Read_Register_N(uint8_t reg, uint8_t *buf, uint8_t len);

// 5. Write single byte register

void NRF24_Write_Register(uint8_t reg, uint8_t value);

// 6. Write multiple bytes register

void NRF24_Write_Register_N(uint8_t reg, const uint8_t* buf, uint8_t len);

// 7. Write transmit payload

void NRF24_Write_Payload(const void* buf, uint8_t len);

// 8. Read receive payload

void NRF24_Read_Payload(void* buf, uint8_t len);

// 9. Flush TX buffer

void NRF24_Flush_TX(void);

// 10. Flush RX buffer

void NRF24_Flush_RX(void);

// 11. Get status register value

uint8_t NRF24_Get_Status(void);

// 12. Begin function

void NRF24_Begin(GPIO_TypeDef *NRF_PORT, uint16_t NRF_CSN_PIN, uint16_t NRF_CE_PIN, SPI_HandleTypeDef NRF_SPI);

// 13. Listen on open pipes for reading (Must call NRF24_OpenReadingPipe() first)

void NRF24_StartListening(void);

// 14. Stop listening (essential before any write operation)

void NRF24_StopListening(void);

// 15. Write (transmit data), returns true if successfully sent

bool NRF24_Write(const void* buf, uint8_t len);

// 16. Check for available data to read

bool NRF24_Available(void);

// 17. Read received data

bool NRF24_Read(void* buf, uint8_t len);

// 18. Open TX pipe for writing (cannot perform this while listening, has to call NRF24_StopListening)

void NRF24_OpenWritingPipe(uint64_t address);

// 19. Open reading pipe

void NRF24_OpenReadingPipe(uint8_t number, uint64_t address);

// 20 set transmit retries (rf24_Retries_e) and delay

void NRF24_SetRetries(uint8_t delay, uint8_t count);

// 21. Set RF channel frequency

void NRF24_SetChannel(uint8_t channel);

// 22. Set payload size

void NRF24_SetPayloadSize(uint8_t size);

// 23. Get payload size

uint8_t NRF24_GetPayloadSize(void);

// 24. Get dynamic payload size, of latest packet received

uint8_t NRF24_GetDynamicPayloadSize(void);

// 25. Enable payload on acknowledge packet

void NRF24_EnableAckPayload(void);

// 26. Enable dynamic payloads

void NRF24_EnableDynamicPayloads(void);

void NRF24_DisableDynamicPayloads(void);

// 27. Check if module is NRF24L01+ or normal module

bool NRF24_IsNRF_Plus(void);

// 28. Set Auto Ack for all

void NRF24_SetAutoAck(bool enable);

// 29. Set Auto Ack for certain pipe

void NRF24_SetAutoAckPipe(uint8_t pipe, bool enable);

// 30. Set transmit power level

void NRF24_SetPALevel(rf24_pa_dbm_e level);

// 31. Get transmit power level

rf24_pa_dbm_e NRF24_GetPALevel(void);

// 32. Set data rate (250 Kbps, 1Mbps, 2Mbps)

bool NRF24_SetDataRate(rf24_datarate_e speed);

// 33. Get data rate

rf24_datarate_e NRF24_GetDataRate(void);

// 34. Set CRC length (disable, 8-bits or 16-bits)

void NRF24_SetCRCLength(rf24_crclength_e length);

// 35. Get CRC length

rf24_crclength_e NRF24_GetCRCLength(void);

// 36. Disable CRC

void RF24_DisableCRC(void);

// 37. Power up

void NRF24_PowerUp(void);

// 38. Power down

void NRF24_PowerDown(void);

// 39. Check if data are available and on which pipe (use this for multiple RX pipes)

bool NRF24_AvailablePipe(uint8_t* pipe_num);

// 40. Start write (for IRQ mode)

void NRF24_StartWrite(const void* buf, uint8_t len);

// 41. Write AckPayload

void NRF24_WriteAckPayload(uint8_t pipe, const void* buf, uint8_t len);

// 42. Check if an AckPayload is available

bool NRF24_IsAckPayloadAvailable(void);

// 43. Check interrupt flags

void NRF24_What_Happened(bool *tx_ok, bool *tx_fail, bool *rx_ready);

// 44. Test if there is a carrier on the previous listening period (useful to check for interference)

bool NRF24_Test_Carrier(void);

// 45. Test if a signal carrier exists (=> -64dB), only for NRF24L01+

bool NRF24_Test_RPD(void);

// 46. Reset status

void NRF24_Reset_Status(void);

// 47. Activate CMD

void NRF24_Activate_CMD(void);

// 48. Get AckPayload size

uint8_t NRF24_GetAckPayloadSize(void);

// Debug function prototypes

// 1. Print radio settings

void NRF24_Print_Radio_Settings(void);

// 2. Print status

void NRF24_Print_Status_Reg(void);

// 3. Print configurations

void NRF24_Print_Config_Reg(void);

// 4. Initialize variables

void NRF24_Debug_UART_Init(UART_HandleTypeDef NRF24_UART);

// 5. FIFO Status

void NRF24_Print_FIFO_Status(void);

#endif /* INC_NRF24_H_ */
