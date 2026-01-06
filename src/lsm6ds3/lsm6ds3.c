#include <stdint.h>
#include "stm32f411xe.h"
#include "utils.h"
#include "lsm6ds3.h"


uint8_t lsm6ds3IrqReadActive = 0;


void setUpLsm6ds3Gpios(void)
{
    // lsm6ds3 CS pin setup
	RCC->AHB1ENR |= LSM6DS3_CS_GPIOEN; // IOP(C)EN
	LSM6DS3_CS_GPIO->MODER &= ~(0x3 << (LSM6DS3_CS_PIN * 2));
    // general purpose output mode
	LSM6DS3_CS_GPIO->MODER |= (0x1 << (LSM6DS3_CS_PIN * 2));
	LSM6DS3_CS_GPIO->OTYPER &= ~(1 << LSM6DS3_CS_PIN); // output push-pull
	LSM6DS3_CS_GPIO->OSPEEDR &= ~(0x3 << (LSM6DS3_CS_PIN * 2)); // reset
	LSM6DS3_CS_GPIO->OSPEEDR |= (0x3 << (LSM6DS3_CS_PIN * 2)); // high speed
	LSM6DS3_CS_GPIO->PUPDR &= ~(0x3 << (LSM6DS3_CS_PIN * 2)); // no pull-up, no pull-down


    // disable lsm6ds3 SPI
    LSM6DS3_CS_HIGH();
}


uint8_t lsm6ds3ReadWhoAmI(SPI_TypeDef* spi, uint8_t* readByteIrq)
{
    // 0x0F = the "who am I" register
    // 0x80 = set MSb to 1 (read register, 0 = write to register)
    uint8_t whoAmIRegAddress = 0x0F | 0x80;
    uint8_t dummyByte = 0x00;

    // enable lsm6ds3 SPI
    LSM6DS3_CS_LOW();

    while (!(spi->SR & SPI_SR_TXE))
    {
        continue;
    }

    *(volatile uint8_t *) &spi->DR = whoAmIRegAddress;

    while (!(spi->SR & SPI_SR_TXE))
    {
        continue;
    }

    *(volatile uint8_t *) &spi->DR = dummyByte;

    // this flag should be set to 0 by interrupt service routine
    lsm6ds3IrqReadActive = 1;

    // wait for data to be read (IRQ)
    while (lsm6ds3IrqReadActive)
    {
        continue;
    }


    // disable lsm6ds3 SPI
    LSM6DS3_CS_HIGH();

    return *readByteIrq;
}


uint8_t lsm6ds3ReadSingleRegisterSPI(
    SPI_TypeDef* spi, uint8_t registerAddress, uint8_t* readByteIrq
)
{
    // enable lsm6ds3 SPI
    LSM6DS3_CS_LOW();


    while (!(spi->SR & SPI_SR_TXE))
    {
        continue;
    }

    // 0x80: MSb=1 indicates read from lsm6ds3 register
    *(volatile uint8_t *) &spi->DR = registerAddress | 0x80;


    while (!(spi->SR & SPI_SR_TXE))
    {
        continue;
    }

    *(volatile uint8_t *) &spi->DR = 0x00; // dummy write


    // this flag should be set to 0 by interrupt service routine
    lsm6ds3IrqReadActive = 1;

    // wait for data to be read (IRQ)
    while (lsm6ds3IrqReadActive)
    {
        continue;
    }

    // disable lsm6ds3 SPI
    LSM6DS3_CS_HIGH();


    return *readByteIrq;
}


void lsm6ds3ReadRegistersSPI(
    SPI_TypeDef* spi,
    uint8_t* readByteIrq,
    uint8_t firstRegisterAddr,
    uint8_t* dataBuffer,
    uint8_t numRegisters
)
{
    // enable lsm6ds3 SPI
    LSM6DS3_CS_LOW();


    while (!(spi->SR & SPI_SR_TXE))
    {
        continue;
    }

    // 0x80: MSb=1 indicates read from lsm6ds3 register
    *(volatile uint8_t *) &spi->DR = firstRegisterAddr | 0x80;


    for (uint8_t i = 0; i < numRegisters; i++)
    {
        while (!(spi->SR & SPI_SR_TXE))
        {
            continue;
        }

        *(volatile uint8_t *) &spi->DR = 0x00; // dummy write


        // this flag should be set to 0 by interrupt service routine
        lsm6ds3IrqReadActive = 1;

        // wait for data to be read (IRQ)
        while (lsm6ds3IrqReadActive)
        {
            continue;
        }


        dataBuffer[i] = *readByteIrq;
    }
    

    // disable lsm6ds3 SPI
    LSM6DS3_CS_HIGH();
}


void lsm6ds3WriteSingleRegisterSPI(
    SPI_TypeDef* spi, uint8_t registerAddress, uint8_t dataByte
)
{
    // enable lsm6ds3 SPI
    LSM6DS3_CS_LOW();

    while (!(spi->SR & SPI_SR_TXE))
    {
        continue;
    }

    *(volatile uint8_t *) &spi->DR = registerAddress;


    // this flag should be set to 0 by interrupt service routine
    lsm6ds3IrqReadActive = 1;
    // wait for data to be read (IRQ)
    while (lsm6ds3IrqReadActive)
    {
        continue;
    }

    while (!(spi->SR & SPI_SR_TXE))
    {
        continue;
    }

    *(volatile uint8_t *) &spi->DR = dataByte;


    // this flag should be set to 0 by interrupt service routine
    lsm6ds3IrqReadActive = 1;
    // wait for data to be read (IRQ)
    while (lsm6ds3IrqReadActive)
    {
        continue;
    }


    // disable lsm6ds3 SPI
    LSM6DS3_CS_HIGH();
}