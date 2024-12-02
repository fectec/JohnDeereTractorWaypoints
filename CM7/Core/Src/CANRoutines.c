/*
 * CANRoutine.c
 */

#include "main.h"
#include "CANRoutines.h"
#include "printf.h"

uint16_t CAN_ReceiveEncoder(FDCAN_RxHeaderTypeDef *RxHeader)
{
	uint8_t RxData[64];  // Buffer to store received data

	// Attempt to get a message from FIFO0

	if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, RxHeader, RxData) == HAL_OK)
	{
		// DEBUG: Print the message ID (identifier)

		// printf("\r\n %lx @ ", RxHeader->Identifier);

		// Determine the data length (Data Length Code - DLC)

		uint8_t dataLength = (RxHeader->DataLength >> 16) & 0x0F;

		// DEBUG: Print valid data bytes based on DLC

		/*for (int i = 0; i < dataLength; i++)
		{
			printf(" 0x%x", RxData[i]);
		}*/

		// Combine the bytes into a number

		if (dataLength == 2)
		{
			// Combine RxData[1] (high byte) and RxData[0] (low byte)

			uint16_t encoderValue = (RxData[1] << 8) | RxData[0];

			// DEBUG: Print the numerical encoder value after the raw data

			// printf(" | Encoder Value: %d", encoderValue);
		}

		// DEBUG: printf("\n\r");
	}
}
