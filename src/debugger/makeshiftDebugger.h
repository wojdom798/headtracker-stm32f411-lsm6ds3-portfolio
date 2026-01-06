#ifndef MAKESHIFT_DEBUGGER_H
#define MAKESHIFT_DEBUGGER_H

/**
 * @file makeshiftDebugger.h
 * 
 * stm32f303re:
 * baud rate (default): 115200 bps @ 72 MHz
 *
 * stm32f411:
 * baud rate: 576000 bps @ 96 MHz
 *
 * "\n\r" vs "\t\r"
 */

#include <stdint.h>
// #include "stm32f303xe.h"
#include "stm32f411xe.h"


#define DBG_USART (USART2)


#define INT_TO_ASCII_BUFFER_SIZE (50U)

// Usart 2 (nucleo)
#define DBG_USART_GPIO_EN (RCC_AHBENR_GPIOAEN)
#define DBG_USART_GPIO (GPIOA)
#define DBG_USART_TX (2U) // PA2
#define DBG_USART_RX (3U) // PA3

void setupDbgHardware(void);
void debuggerWriteByte(USART_TypeDef* USARTx, uint8_t byte);
void debuggerWriteStr(USART_TypeDef* USARTx, char* str);
void convertIntToAscii(char* buffer, uint16_t bufSize, uint64_t value);
void convertIntToAsciiHex(char* buffer, uint16_t bufSize, uint8_t optionFlags, uint64_t value);

void debuggerPrintValue(
	char* buffer,
	uint16_t bufSize,
	uint16_t optionFlags,
	char* prefix,
	char* postfix,
	uint64_t value
);

void dbgPrintHex(char* prefix, uint64_t value, char* postfix);
void dbgPrintDec(char* prefix, uint64_t value, char* postfix);

#endif
