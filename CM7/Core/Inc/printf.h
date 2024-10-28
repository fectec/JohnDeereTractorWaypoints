/*
 * printf.h
 *
 * This module provides functionality to retarget the standard C library
 * printf function to transmit output over USART. It defines the necessary
 * prototypes and UART handler for UART communication.
 */

#ifndef INC_PRINTF_H_
#define INC_PRINTF_H_

// Includes

#include "main.h"
#include <stdio.h>

// Extern variables

extern UART_HandleTypeDef huart3;	// UART handle for USART communication

// Function prototypes

#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

#endif /* INC_PRINTF_H_ */
