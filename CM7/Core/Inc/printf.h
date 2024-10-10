/*
 * printf.h
 */

#ifndef INC_PRINTF_H_
#define INC_PRINTF_H_

extern UART_HandleTypeDef huart2;

#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

#endif /* INC_PRINTF_H_ */
