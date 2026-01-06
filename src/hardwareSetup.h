#ifndef _HARDWARE_SETUP_H
#define _HARDWARE_SETUP_H

#include <stdint.h>
#include "stm32f411xe.h"


// ----------------------------------------------------------------------------
//                         Miscellaneous Defines
// ----------------------------------------------------------------------------
// #define DBG_USART (USART2)
#define PCF8574_I2C_WRITE_ADDR (0x40)
#define PCF8574_I2C_READ_ADDR (0x41)


// ----------------------------------------------------------------------------
//                      Clock Config Select Defines
// ----------------------------------------------------------------------------
#define SYS_CLOCK_SELECTED_CONFIG (SYS_CLOCK_CONFIG_USB)
#define SYS_CLOCK_CONFIG_NO_USB (1U)
#define SYS_CLOCK_CONFIG_USB (2U)


// ----------------------------------------------------------------------------
//                      Should Setup Peripheral
// ----------------------------------------------------------------------------
#define SHOULD_SET_UP_I2C1 (0U)
#define SHOULD_SET_UP_I2C2 (0U)
#define SHOULD_SET_UP_USART6 (0U)
#define SHOULD_SET_UP_USART2 (1U)


// ----------------------------------------------------------------------------
//                      USART6 Defines
// ----------------------------------------------------------------------------
// USART6 has shared pins with USB, so I have to ditch it
#define USART6_SHOULD_ENABLE (0) // 0 = don't enable, 1 = enable

#define USART6_TX_GPIO_CLKEN (RCC_AHB1ENR_GPIOAEN)
#define USART6_TX_GPIO (GPIOA)
#define USART6_TX_PIN (11)

#define USART6_RX_GPIO_CLKEN (RCC_AHB1ENR_GPIOAEN)
#define USART6_RX_GPIO (GPIOA)
#define USART6_RX_PIN (12)

// ----------------------------------------------------------------------------
//                      USART2 Defines
// ----------------------------------------------------------------------------
#define USART2_TX_GPIO_CLKEN (RCC_AHB1ENR_GPIOAEN)
#define USART2_TX_GPIO (GPIOA)
#define USART2_TX_PIN (2)

#define USART2_RX_GPIO_CLKEN (RCC_AHB1ENR_GPIOAEN)
#define USART2_RX_GPIO (GPIOA)
#define USART2_RX_PIN (3)

// ----------------------------------------------------------------------------
//                      I2C2 Defines
// ----------------------------------------------------------------------------
#define I2C2_SDA_GPIOEN (RCC_AHB1ENR_GPIOBEN)
#define I2C2_SDA_GPIO (GPIOB)
#define I2C2_SDA_PIN (3)

#define I2C2_SCL_GPIOEN (RCC_AHB1ENR_GPIOBEN)
#define I2C2_SCL_GPIO (GPIOB)
#define I2C2_SCL_PIN (10)

// ----------------------------------------------------------------------------
//                      I2C1 Defines
// ----------------------------------------------------------------------------
// #define I2C1_SDA_GPIOEN (RCC_AHB1ENR_GPIOBEN)
// #define I2C1_SDA_GPIO (GPIOB)
// #define I2C1_SDA_PIN (7)

// #define I2C1_SCL_GPIOEN (RCC_AHB1ENR_GPIOBEN)
// #define I2C1_SCL_GPIO (GPIOB)
// #define I2C1_SCL_PIN (6)

#define I2C1_SDA_GPIOEN (RCC_AHB1ENR_GPIOBEN)
#define I2C1_SDA_GPIO (GPIOB)
#define I2C1_SDA_PIN (9)

#define I2C1_SCL_GPIOEN (RCC_AHB1ENR_GPIOBEN)
#define I2C1_SCL_GPIO (GPIOB)
#define I2C1_SCL_PIN (8)


// ----------------------------------------------------------------------------
//                      SPI 1 Defines
// ----------------------------------------------------------------------------
#define SPI1_SCK_GPIOEN   (RCC_AHB1ENR_GPIOAEN)
#define SPI1_SCK_GPIO     (GPIOA)
#define SPI1_SCK_PIN      (5)
#define SPI1_SCK_AF_NUM   (5) // alternate function number

#define SPI1_MISO_GPIOEN  (RCC_AHB1ENR_GPIOAEN)
#define SPI1_MISO_GPIO    (GPIOA)
#define SPI1_MISO_PIN     (6)
#define SPI1_MISO_AF_NUM  (5) // alternate function number

#define SPI1_MOSI_GPIOEN  (RCC_AHB1ENR_GPIOAEN)
#define SPI1_MOSI_GPIO    (GPIOA)
#define SPI1_MOSI_PIN     (7)
#define SPI1_MOSI_AF_NUM  (5) // alternate function number


// ----------------------------------------------------------------------------
//                       Function Prototypes
// ----------------------------------------------------------------------------
void stm32f411SetupPeripherals(void);

void setUpVoltageScaling(void);
void setUpSystemClock(void);
void setUpUsart2(void);
void setUpUsart6(void);
void enableInterruptsUSART6(uint8_t priority);
void setUpTimer11(void);
void enableInterruptsTimer11(uint8_t priority);
void setUpTimer2(void);
void enableInterruptsTimer2(uint8_t priority);
void setUpI2c2(void);
void setUpI2c1(void);
void setUpF411BlackpillHardware(void);
void enableInterruptsF411BlackpillBtn(uint8_t priority);

void setUpSpi1(void);
void enableInterruptsSpi1(uint8_t priority);

#endif
