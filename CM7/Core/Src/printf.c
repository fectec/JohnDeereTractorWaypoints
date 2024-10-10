/*
 * printf.c
 */

#include "main.h"
#include "printf.h"

/**
 * @brief Retarget the C library printf function to USART.
 *
 * This function is used to transmit a single character over UART
 * when the printf function is called. It takes the character to be
 * transmitted as input, sends it via the USART peripheral, and
 * returns the transmitted character.
 *
 * @param ch The character to be transmitted.
 * @return The transmitted character.
 */

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
