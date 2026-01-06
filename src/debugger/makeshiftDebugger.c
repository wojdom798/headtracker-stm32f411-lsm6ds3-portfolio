#include "makeshiftDebugger.h"

char intToAsciiBuffer[INT_TO_ASCII_BUFFER_SIZE];

void setupDbgHardware(void)
{
    // USART2 GPIO setup
    // supply clock to port A (TX = PA2, RX = PA3)
    // RCC->AHBENR |= DBG_USART_GPIO_EN;
    // // clear mode before assigning it the proper value
    // DBG_USART_GPIO->MODER &= ~((0x3 << (DBG_USART_TX * 2)) | (0x3 << (DBG_USART_RX * 2)));
    // // set mode to alternate function (0b10 = 0x2) for RX and TX pins
    // DBG_USART_GPIO->MODER |= ((0x2 << (DBG_USART_TX * 2)) | (0x2 << (DBG_USART_RX * 2)));
    // // Port Output Type Register set to 0 for TX and RX pins (output push-pull)
    // DBG_USART_GPIO->OTYPER &= ~((1 << DBG_USART_TX) | (1 << DBG_USART_RX));
    // // clear Port Output Speed Register for TX and RX pins (default value = low speed)
    // DBG_USART_GPIO->OSPEEDR &= ~((0x3 << (DBG_USART_TX * 2)) | (0x3 << (DBG_USART_RX * 2)));
    // // set GPIO speed for TX and RX to medium
    // DBG_USART_GPIO->OSPEEDR |= ((0x1 << (DBG_USART_TX * 2)) | (0x1 << (DBG_USART_RX * 2)));
    // // clear pull-up/pull-down register for TX and RX pins
    // // -> no pull-up, pull-down
    // DBG_USART_GPIO->PUPDR &= ~((0x3 << (DBG_USART_TX * 2)) | (0x3 << (DBG_USART_RX * 2)));
    // // AFR[0] = AFR Low Register (pins 0..7)
    // // AFR[1] = AFR High Register (pins 8..15)
    // // clear alternate function (set to zero) for USART2 RX and TX pins
    // DBG_USART_GPIO->AFR[0] &= ~((0xF << (DBG_USART_TX * 4)) | (0xF << (DBG_USART_RX * 4)));
    // // select alternate function AF7 for USART2 RX and TX pins
    // // AF7 for PA2 means USART2_TX, AF7 for PA3 is USART2_RX
    // DBG_USART_GPIO->AFR[0] |= ((0x7 << (DBG_USART_TX * 4)) | (0x7 << (DBG_USART_RX * 4)));

    // // USART2 setup
    // RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    // // USART2 clock source selection
    // // clear to default value (PCLK as USART2 clock source)
    // RCC->CFGR3 &= ~(0x3 << (RCC_CFGR3_USART2SW_Pos)); 
    // // System clock (SYSCLK) selected as USART2 clock (0b01)
    // RCC->CFGR3 |= (0x1 << (RCC_CFGR3_USART2SW_Pos));
    // // M[1:0] = 00: 1 Start bit, 8 data bits, n stop bits
    // USART2->CR1 &= ~( USART_CR1_M1 | USART_CR1_M0 );
    // // USARTx_CR1_OVER8 = 0 => Oversampling by 16
    // USART2->CR1 &= ~(USART_CR1_OVER8);
    // // set baud rate USART2_BRR
    // USART2->BRR = 625; // 115200 bps @ 72 MHz
    // // 1 stop bit
    // USART2->CR2 &= ~(0x3 << USART_CR2_STOP_Pos);

    // // USART2 interrupt setup
    // // transmission complete interrupt enable
    // // USART2->CR1 |= USART_CR1_TCIE;
    // // receive data register not empty interrupt enable
    // USART2->CR1 |= USART_CR1_RXNEIE;

    // // Enables USART2
    // USART2->CR1 |= USART_CR1_TE; // enable transmitter
    // USART2->CR1 |= USART_CR1_RE; // enable receiver
    // USART2->CR1 |= USART_CR1_UE; // enable USART

    // // USART2 Interrupts
    // NVIC_SetPriority(USART2_IRQn, 0x03);
    // NVIC_EnableIRQ(USART2_IRQn);
}


void debuggerWriteByte(USART_TypeDef* USARTx, uint8_t byte)
{
    // USARTx->TDR = (uint32_t) byte;
    // while (!(USARTx->ISR & USART_ISR_TC)) continue;

    USARTx->DR = (uint32_t) byte;
    while (!(USARTx->SR & USART_SR_TC))
    {
        continue;
    }
}


void debuggerWriteStr(USART_TypeDef* USARTx, char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
    {
        debuggerWriteByte(USARTx, (uint8_t) str[i]);
        i++;
    }
}


void convertIntToAscii(char* buffer, uint16_t bufSize, uint64_t value)
{
    uint16_t i = 0;
    uint64_t valueCopy = value;

    if (value == 0)
    {
        buffer[0] = 0x30;
        buffer[1] = '\0';
    }
    else
    {
        while (valueCopy > 0)
        {
            i++;
            valueCopy /= 10;
        }
        buffer[i] = '\0';
        while (i > 0)
        {
            i--;
            buffer[i] = (value % 10) + 0x30;
            value /= 10;
        }
    }
}

// uint8_t optionFlags = 0b 00ba pppp
// a=1, then apply padding "pppp"
// b=1, then add 0x prefix
// pppp = 1, pad with 0s to one byte
// pppp = 2, pad with 0s to two bytes (half-word)
void convertIntToAsciiHex(char* buffer, uint16_t bufSize, uint8_t optionFlags, uint64_t value)
{
	uint32_t i = 0;
	uint32_t padd = 0;
	uint64_t varCopy = value;

	if (optionFlags & 0x20)
	{
		padd = 2;
		buffer[0] = '0';
		buffer[1] = 'x';
	}

	if (value == 0)
	{
		buffer[0 + padd] = '0';
		buffer[1 + padd] = '\0';
		return;
	}


	while (varCopy > 0)
	{
		i++;
		varCopy /= 16;
	}

	if ((optionFlags & 0x10) && ((optionFlags & 0x0F) != 0x00))
	{
		if ((optionFlags & 0x0F) == 0x01U) // pad to 1 byte
		{
			padd += 4U - (i % 4U);
			buffer[padd - 1] = '0';
		}
		else
		{

		}
	}

	varCopy = value;
	buffer[i + padd] = '\0';

	while (i > 0)
	{
		i--;
		if ((varCopy % 16) < 10)
		{
			buffer[i + padd] = (varCopy % 16) + 0x30;
		}
		else
		{
			buffer[i + padd] = (varCopy % 16) + 0x37;
		}

		varCopy /= 16;
	}
}

/*
 * uint16_t optionFlags: 0b 00ba pppp 0000 000s
 * bit "s" (number system):
 * s = 0: decimal
 * s = 1: hexadecimal
 *
 * if hexadecimal, then bits 0b 00ba pppp apply:
 * a=1, then apply padding "pppp" (not implemented yet)
 * b=1, then add 0x prefix
 */
void debuggerPrintValue(
	char* buffer,
	uint16_t bufSize,
	uint16_t optionFlags,
	char* prefix,
	char* postfix,
	uint64_t value
)
{
	if (optionFlags & 0x0001) // hexadecimal
	{
		debuggerWriteStr(DBG_USART, prefix);
		convertIntToAsciiHex(
			buffer,
			bufSize,
			(uint8_t)(optionFlags >> 8U),
			value
		);
		debuggerWriteStr(DBG_USART, buffer);
		debuggerWriteStr(DBG_USART, postfix);


	}
	else // decimal
	{
		debuggerWriteStr(DBG_USART, prefix);
		convertIntToAscii(buffer, bufSize, value);
		debuggerWriteStr(DBG_USART, buffer);
		debuggerWriteStr(DBG_USART, postfix);
	}
}


// received data
void USART2_IRQHandler(void)
{
    // if (USART2->ISR & USART_ISR_RXNE)
    // {
    //     // [...]
    //     uint8_t readChar = USART2->RDR;
    // }
    if (USART2->SR & USART_SR_RXNE)
    {
        // [...]
        uint8_t readChar = USART2->DR;
    }
}


void dbgPrintHex(char* prefix, uint64_t value, char* postfix)
{
    debuggerPrintValue(
        intToAsciiBuffer,
        INT_TO_ASCII_BUFFER_SIZE,
        0x2001,
        prefix,
        postfix,
        value
    );
}


void dbgPrintDec(char* prefix, uint64_t value, char* postfix)
{
    debuggerPrintValue(
        intToAsciiBuffer,
        INT_TO_ASCII_BUFFER_SIZE,
        0x0000,
        prefix,
        postfix,
        value
    );
}