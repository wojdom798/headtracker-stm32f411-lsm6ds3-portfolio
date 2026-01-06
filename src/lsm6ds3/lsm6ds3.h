#ifndef _LSM6DS3_H
#define _LSM6DS3_H


#define LSM6DS3_CS_GPIOEN (RCC_AHB1ENR_GPIOBEN)
#define LSM6DS3_CS_GPIO (GPIOB)
#define LSM6DS3_CS_PIN (0U)

#define LSM6DS3_CS_HIGH() (LSM6DS3_CS_GPIO->ODR |= (1 << LSM6DS3_CS_PIN))
#define LSM6DS3_CS_LOW()  (LSM6DS3_CS_GPIO->ODR &= ~(1 << LSM6DS3_CS_PIN))


#define LSM6DS3_WHO_AM_I (0x0FU) // should read 0x69
// Linear acceleration sensor control register 1 (r/w)
#define LSM6DS3_CTRL1_XL (0x10U)
// Angular rate sensor control register 2 (r/w)
#define LSM6DS3_CTRL2_G  (0x11U)


// Internal Temperature Sensor Output Registers
#define LSM6DS3_OUT_TEMP_L (0x20U)
#define LSM6DS3_OUT_TEMP_H (0x21U)

// Gyroscope Output Registers
#define OUTx_L_G (0x22U) // X
#define OUTx_H_G (0x23U)
#define OUTy_L_G (0x24U) // Y
#define OUTy_H_G (0x25U)
#define OUTz_L_G (0x26U) // Z
#define OUTz_H_G (0x27U)

// Accelerometer Output Registers
#define OUTx_L_XL (0x28U) // X
#define OUTx_H_XL (0x29U)
#define OUTy_L_XL (0x2AU) // Y
#define OUTy_H_XL (0x2BU)
#define OUTz_L_XL (0x2CU) // Z
#define OUTz_H_XL (0x2DU)



void setUpLsm6ds3Gpios(void);
uint8_t lsm6ds3ReadWhoAmI(SPI_TypeDef* spi, uint8_t* readByteIrq);

uint8_t lsm6ds3ReadSingleRegisterSPI(
    SPI_TypeDef* spi, uint8_t registerAddress, uint8_t* readByteIrq
);

void lsm6ds3ReadRegistersSPI(
    SPI_TypeDef* spi,
    uint8_t* readByteIrq,
    uint8_t firstRegisterAddr,
    uint8_t* dataBuffer,
    uint8_t numRegisters
);

void lsm6ds3WriteSingleRegisterSPI(
    SPI_TypeDef* spi, uint8_t registerAddress, uint8_t dataByte
);

#endif