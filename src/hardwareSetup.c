#include "hardwareSetup.h"
#include "main.h"
#include "makeshiftDebugger.h"


void setUpVoltageScaling(void)
{
    // 0b11 = Scale 1 mode (<= 100 MHz)
    PWR->CR |= (0x3U << PWR_CR_VOS_Pos);
}


void setUpSystemClock(void)
{
    #if (SYS_CLOCK_SELECTED_CONFIG == SYS_CLOCK_CONFIG_NO_USB)
    #warning "SYS_CLOCK_SELECTED_CONFIG = NO USB"
    // set Flash Latency to 3 (3 WS - wait states)
    FLASH->ACR &= ~(0xFUL << FLASH_ACR_LATENCY_Pos); // make sure latency = 0 (reset)
    FLASH->ACR |= (0x3UL << FLASH_ACR_LATENCY_Pos); // set latency to 3 WS


    // setup APB1 prescaler (APB1 max freq = 50 MHz)
    RCC->CFGR &= ~(RCC_CFGR_PPRE1_Msk); // reset PRE1[2:0]
    
    // PRE1[2:0] = 0b100 (0x4); AHB clock divided by 2
    // 100 MHz ÷ 2 = 50 MHz
    // RCC->CFGR |= (0x4UL << RCC_CFGR_PPRE1_Pos);
    
    // PRE1[2:0] = 0b101 (0x5); AHB clock divided by 4
    // 100 MHz ÷ 4 = 25 MHz
    RCC->CFGR |= (0x5UL << RCC_CFGR_PPRE1_Pos);

    #define ABP1_FREQ_MHZ (25) // this one's used for i2c FREQ


    // wait a couple of cycles for the changes to take effect
    for (uint8_t i = 0; i < 16; i++)
    {
        asm("nop");
    }


    RCC->CR |= RCC_CR_HSEON; // enable HSE
    // wait for HSE oscillator to get stable
    while (!(RCC->CR & RCC_CR_HSERDY))
    {
        continue;
    }

    // make sure that PLL and PLLI2S are disabled before changing PLL source
    RCC->CR &= ~(RCC_CR_PLLON);
    RCC->CR &= ~(RCC_CR_PLLI2SON);

    // select HSE as input for the PLL (change PLL source)
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

    // configure PLL
    // 1. PLLM (initial division of HSE clock)
    // for HSE = 25 MHz, PLLM will divide it to provide 1 MHz for VCO
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM); // reset PLLM
    RCC->PLLCFGR |= (0x19UL << RCC_PLLCFGR_PLLM_Pos); // PLLM = 0x19 = 25
    // 2. PLLN (multiply VCO)
    // multiply 1 MHz times 200 to get 200 MHz
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN); // reset PLLN
    RCC->PLLCFGR |= (0xC8UL << RCC_PLLCFGR_PLLN_Pos); // PLLN = 0xC8 = 200
    // 3. PLLP (divide multiplied VCO for system clock; at least by 2)
    // divide 200 MHz VCO by 2 to get 100 MHz as system clock
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP); // reset PLLP
    // PLLP[1:0] = 0b00; PLLP = 2
    // PLLP[1:0] = 0b01; PLLP = 4
    // PLLP[1:0] = 0b10; PLLP = 6
    // PLLP[1:0] = 0b11; PLLP = 8
    RCC->PLLCFGR |= (0x0UL << RCC_PLLCFGR_PLLP_Pos); // PLLP[1:0] = 0b00; PLLP = 2

    // enable PLL
    RCC->CR |= (RCC_CR_PLLON);
    // wait for PLL to lock
    while (!(RCC->CR & RCC_CR_PLLRDY))
    {
        continue;
    }

    // set PLL_clk as System Clock
    RCC->CFGR &= ~(RCC_CFGR_SW_Msk); // reset SW[1:0] (default value, HSI)
    RCC->CFGR |= (0x2UL << RCC_CFGR_SW_Pos); // select PLL as system clock

    // debug - HSE as system clock
    // RCC->CFGR &= ~(RCC_CFGR_SW_Msk); // reset SW[1:0]
    // RCC->CFGR |= (0x1UL << RCC_CFGR_SW_Pos); // select HSE as system clock


    RCC->CR &= ~RCC_CR_HSION; // disable HSI

    #elif (SYS_CLOCK_SELECTED_CONFIG == SYS_CLOCK_CONFIG_USB)
    #warning "SYS_CLOCK_SELECTED_CONFIG = USB"
    //-------------------------------------------------------------------
    //                       Clock Config For USB
    // 
    // sys_clk = 96 MHz
    // AHB1 = 48 MHz
    // USB = 48 MHz
    //-------------------------------------------------------------------
    // set Flash Latency to 3 (3 WS - wait states)
    FLASH->ACR &= ~(0xFUL << FLASH_ACR_LATENCY_Pos); // make sure latency = 0
    FLASH->ACR |= (0x3UL << FLASH_ACR_LATENCY_Pos); // set latency to 3 WS


    // setup APB1 prescaler (APB1 max freq = 50 MHz)
    RCC->CFGR &= ~(RCC_CFGR_PPRE1_Msk); // reset PRE1[2:0]
    
    // PRE1[2:0] = 0b100 (0x4); AHB clock divided by 2
    // 96 MHz ÷ 2 = 48 MHz
    RCC->CFGR |= (0x4UL << RCC_CFGR_PPRE1_Pos);
    
    // PRE1[2:0] = 0b101 (0x5); AHB clock divided by 4
    // 96 MHz ÷ 4 = 24 MHz
    // RCC->CFGR |= (0x5UL << RCC_CFGR_PPRE1_Pos);

    #define ABP1_FREQ_MHZ (48) // this one's used for i2c FREQ


    // wait a couple of cycles for the changes to take effect
    for (uint8_t i = 0; i < 16; i++)
    {
        asm("nop");
    }


    RCC->CR |= RCC_CR_HSEON; // enable HSE
    // wait for HSE oscillator to get stable
    while (!(RCC->CR & RCC_CR_HSERDY))
    {
        continue;
    }

    // make sure that PLL and PLLI2S are disabled before changing PLL source
    RCC->CR &= ~(RCC_CR_PLLON);
    RCC->CR &= ~(RCC_CR_PLLI2SON);

    // select HSE as input for the PLL (change PLL source)
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

    // ----------------------------------------------
    // configure PLL
    // ----------------------------------------------
    // 1. PLLM (initial division of HSE clock)
    // for HSE = 25 MHz, PLLM will divide it to provide 1 MHz for VCO
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM); // reset PLLM
    RCC->PLLCFGR |= (0x19UL << RCC_PLLCFGR_PLLM_Pos); // PLLM = 0x19 = 25
    // 2. PLLN (multiply VCO)
    // multiply 1 MHz times 192 to get 192 MHz
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN); // reset PLLN
    RCC->PLLCFGR |= (0xC0UL << RCC_PLLCFGR_PLLN_Pos); // PLLN = 0xC0 = 192
    // 3. PLLP (divide multiplied VCO for system clock; at least by 2)
    // divide 192 MHz VCO by 2 to get 96 MHz as system clock
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP); // reset PLLP
    // PLLP[1:0] = 0b00; PLLP = 2
    // PLLP[1:0] = 0b01; PLLP = 4
    // PLLP[1:0] = 0b10; PLLP = 6
    // PLLP[1:0] = 0b11; PLLP = 8
    RCC->PLLCFGR |= (0x0UL << RCC_PLLCFGR_PLLP_Pos); // PLLP[1:0] = 0b00; PLLP = 2

    // 4. PLLQ (USB clock prescaler)
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLQ); // reset PLLQ
    // WRONG (usb_clk = 192 MHz / 2 = 96 MHz)
    // RCC->PLLCFGR |= (0x2UL << RCC_PLLCFGR_PLLQ_Pos); // PLLQ[3:0] = 0b0010; PLLQ = 2
    RCC->PLLCFGR |= (0x4UL << RCC_PLLCFGR_PLLQ_Pos); // PLLQ[3:0] = 0b0100; PLLQ = 4
    // ----------------------------------------------
    // end: configure PLL
    // ----------------------------------------------

    // enable PLL
    RCC->CR |= (RCC_CR_PLLON);
    // wait for PLL to lock
    while (!(RCC->CR & RCC_CR_PLLRDY))
    {
        continue;
    }

    // set PLL_clk as System Clock
    RCC->CFGR &= ~(RCC_CFGR_SW_Msk); // reset SW[1:0] (default value, HSI)
    RCC->CFGR |= (0x2UL << RCC_CFGR_SW_Pos); // select PLL as system clock

    RCC->CR &= ~RCC_CR_HSION; // disable HSI
    #endif // SYS_CLOCK_SELECTED_CONFIG
}


void setUpUsart2(void)
{
    // USART GPIO setup
    // (1) TX
    // enable TX GPIO clock (RCC)
    RCC->AHB1ENR |= USART2_TX_GPIO_CLKEN;
    // clear mode before assigning it the proper value
    USART2_TX_GPIO->MODER &= ~(0x3 << (USART2_TX_PIN * 2));
    // set mode to alternate function mode (0b10 = 0x2)
    USART2_TX_GPIO->MODER |= (0x2 << (USART2_TX_PIN * 2));
    // Port Output Type Register set to 0 for TX pin (output push-pull)
    USART2_TX_GPIO->OTYPER &= ~(1 << USART2_TX_PIN);
    // clear Port Output Speed Register for TX pin (default value = low speed)
    USART2_TX_GPIO->OSPEEDR &= ~(0x3 << (USART2_TX_PIN * 2));
    // set GPIO speed for TX to medium
    USART2_TX_GPIO->OSPEEDR |= (0x1 << (USART2_TX_PIN * 2));
    // clear pull-up/pull-down register for TX pin
    // -> no pull-up, pull-down
    USART2_TX_GPIO->PUPDR &= ~(0x3 << (USART2_TX_PIN * 2));
    // AFR[0] = AFR Low Register (pins 0..7)
    // AFR[1] = AFR High Register (pins 8..15)
    // clear alternate function (set to zero) for TX pin
    USART2_TX_GPIO->AFR[0] &= ~(0xF << (USART2_TX_PIN * 4));
    // select alternate function AF7 for TX pin
    // AF7 = USART2_TX for PA2
    USART2_TX_GPIO->AFR[0] |= (0x7 << (USART2_TX_PIN * 4));


    // (2) RX
    // enable RX GPIO clock (RCC)
    RCC->AHB1ENR |= USART2_RX_GPIO_CLKEN;
    // clear mode before assigning it the proper value
    USART2_RX_GPIO->MODER &= ~(0x3 << (USART2_RX_PIN * 2));
    // set mode to alternate function mode (0b10 = 0x2)
    USART2_RX_GPIO->MODER |= (0x2 << (USART2_RX_PIN * 2));
    // Port Output Type Register set to 0 for RX pin (output push-pull)
    USART2_RX_GPIO->OTYPER &= ~(1 << USART2_RX_PIN);
    // clear Port Output Speed Register for RX pin (default value = low speed)
    USART2_RX_GPIO->OSPEEDR &= ~(0x3 << (USART2_RX_PIN * 2));
    // set RX pin speed to medium
    USART2_RX_GPIO->OSPEEDR |= (0x1 << (USART2_RX_PIN * 2));
    // clear pull-up/pull-down register for RX pin
    // -> no pull-up, pull-down
    USART2_RX_GPIO->PUPDR &= ~(0x3 << (USART2_RX_PIN * 2));
    // AFR[0] = AFR Low Register (pins 0..7)
    // AFR[1] = AFR High Register (pins 8..15)
    // clear alternate function (set to zero) for RX pin
    USART2_RX_GPIO->AFR[0] &= ~(0xF << (USART2_RX_PIN * 4));
    // select alternate function AF7 for RX pin
    // AF7 = USART2_RX for PA3
    USART2_RX_GPIO->AFR[0] |= (0x7 << (USART2_RX_PIN * 4));


    // Enable USART clock (RCC)
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    // make sure USART is disabled
    USART2->CR1 &= ~(USART_CR1_UE);

    // M = 0: 1 Start bit, 8 data bits, n Stop bit
    USART2->CR1 &= ~(USART_CR1_M);
    // 1 stop bit
    USART2->CR2 &= ~(0x3 << USART_CR2_STOP_Pos);
    // USARTx_CR1_OVER8 = 0 => Oversampling by 16
    USART2->CR1 &= ~(USART_CR1_OVER8);

    // reset BRR
    USART2->BRR = 0x00000000UL;
    // Tx/Rx baud = (f_CK) / (8 * (2 - OVER8) * USARTDIV)
    // OVER8 = 0
    // Tx/Rx baud = (f_CK) / (16 * USARTDIV)
    // USARTDIV = f_CK / (16 * baud)
    
    // 115200 bps @ 100 MHz
    // (100000000)/(16*115200) ~= 54.25
    // USART6->BRR |= (54 << USART_BRR_DIV_Mantissa_Pos);
    // USART6->BRR |= (4 << USART_BRR_DIV_Fraction_Pos); // 4/16 = 0.25

    // 921600 bps @ 100 MHz
    // (100000000)/(16*921600) ~= 6.78
    // USART2->BRR |= (6 << USART_BRR_DIV_Mantissa_Pos);
    // USART2->BRR |= (12 << USART_BRR_DIV_Fraction_Pos); // 12/16 = 0.75

    // 921600 bps @ 25 MHz
    // (25000000)/(16*921600) ~= 1.69
    // USART2->BRR |= (1 << USART_BRR_DIV_Mantissa_Pos);
    // USART2->BRR |= (11 << USART_BRR_DIV_Fraction_Pos); // 11/16 = 0.6875

    // 576000 bps @ 25 MHz
    // (25000000)/(16*576000) ~= 2.71
    // USART2->BRR |= (2 << USART_BRR_DIV_Mantissa_Pos);
    // USART2->BRR |= (11 << USART_BRR_DIV_Fraction_Pos); // 11/16 = 0.6875

    // 576000 bps @ 48 MHz
    // (48000000)/(16*576000) ~= 5.21
    USART2->BRR |= (5 << USART_BRR_DIV_Mantissa_Pos);
    // 0.21 * 16 = 3
    USART2->BRR |= (3 << USART_BRR_DIV_Fraction_Pos); // 3/16 = 0.1875


    // USART2 interrupt setup
    // received data register not empty interrupt enable
    // USART2->CR1 |= USART_CR1_RXNEIE;

    USART2->CR1 |= USART_CR1_TE; // enable transmitter
    USART2->CR1 |= USART_CR1_RE; // enable receiver
    // enable USART2
    USART2->CR1 |= (USART_CR1_UE);
}


void setUpUsart6(void)
{
    // USART GPIO setup
    // (1) USART6_TX
    // enable TX GPIO clock (RCC)
    RCC->AHB1ENR |= USART6_TX_GPIO_CLKEN;
    // clear mode before assigning it the proper value
    USART6_TX_GPIO->MODER &= ~(0x3 << (USART6_TX_PIN * 2));
    // set mode to alternate function mode (0b10 = 0x2)
    USART6_TX_GPIO->MODER |= (0x2 << (USART6_TX_PIN * 2));
    // Port Output Type Register set to 0 for TX pin (output push-pull)
    USART6_TX_GPIO->OTYPER &= ~(1 << USART6_TX_PIN);
    // clear Port Output Speed Register for TX pin (default value = low speed)
    USART6_TX_GPIO->OSPEEDR &= ~(0x3 << (USART6_TX_PIN * 2));
    // set GPIO speed for TX to medium
    USART6_TX_GPIO->OSPEEDR |= (0x1 << (USART6_TX_PIN * 2));
    // clear pull-up/pull-down register for TX pin
    // -> no pull-up, pull-down
    USART6_TX_GPIO->PUPDR &= ~(0x3 << (USART6_TX_PIN * 2));
    // AFR[0] = AFR Low Register (pins 0..7)
    // AFR[1] = AFR High Register (pins 8..15)
    // clear alternate function (set to zero) for TX pin
    USART6_TX_GPIO->AFR[1] &= ~(0xF << ((USART6_TX_PIN - 8U) * 4));
    // select alternate function AF8 for TX pin
    // AF8 = USART6_TX for PA11
    USART6_TX_GPIO->AFR[1] |= (0x8 << ((USART6_TX_PIN - 8U) * 4));


    // (2) USART6_RX
    // enable RX GPIO clock (RCC)
    RCC->AHB1ENR |= USART6_RX_GPIO_CLKEN;
    // clear mode before assigning it the proper value
    USART6_RX_GPIO->MODER &= ~(0x3 << (USART6_RX_PIN * 2));
    // set mode to alternate function mode (0b10 = 0x2)
    USART6_RX_GPIO->MODER |= (0x2 << (USART6_RX_PIN * 2));
    // Port Output Type Register set to 0 for RX pin (output push-pull)
    USART6_RX_GPIO->OTYPER &= ~(1 << USART6_RX_PIN);
    // clear Port Output Speed Register for RX pin (default value = low speed)
    USART6_RX_GPIO->OSPEEDR &= ~(0x3 << (USART6_RX_PIN * 2));
    // set RX pin speed to medium
    USART6_RX_GPIO->OSPEEDR |= (0x1 << (USART6_RX_PIN * 2));
    // clear pull-up/pull-down register for RX pin
    // -> no pull-up, pull-down
    USART6_RX_GPIO->PUPDR &= ~(0x3 << (USART6_RX_PIN * 2));
    // AFR[0] = AFR Low Register (pins 0..7)
    // AFR[1] = AFR High Register (pins 8..15)
    // clear alternate function (set to zero) for RX pin
    USART6_RX_GPIO->AFR[1] &= ~(0xF << ((USART6_RX_PIN - 8U) * 4));
    // select alternate function AF8 for RX pin
    // AF8 = USART6_RX for PA12
    USART6_RX_GPIO->AFR[1] |= (0x8 << ((USART6_RX_PIN - 8U) * 4));


    // Enable USART6 clock (RCC)
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    // make sure USART6 is disabled
    USART6->CR1 &= ~(USART_CR1_UE);

    // M = 0: 1 Start bit, 8 data bits, n Stop bit
    USART6->CR1 &= ~(USART_CR1_M);
    // 1 stop bit
    USART6->CR2 &= ~(0x3 << USART_CR2_STOP_Pos);
    // USARTx_CR1_OVER8 = 0 => Oversampling by 16
    USART6->CR1 &= ~(USART_CR1_OVER8);

    // reset BRR
    USART6->BRR = 0x00000000UL;
    // Tx/Rx baud = (f_CK) / (8 * (2 - OVER8) * USARTDIV)
    // OVER8 = 0
    // Tx/Rx baud = (f_CK) / (16 * USARTDIV)
    // USARTDIV = f_CK / (16 * baud)
    
    // 115200 bps @ 100 MHz
    // (100000000)/(16*115200) ~= 54.25
    // USART6->BRR |= (54 << USART_BRR_DIV_Mantissa_Pos);
    // USART6->BRR |= (4 << USART_BRR_DIV_Fraction_Pos); // 4/16 = 0.25

    // 921600 bps @ 100 MHz
    // (100000000)/(16*921600) ~= 6.78
    USART6->BRR |= (6 << USART_BRR_DIV_Mantissa_Pos);
    USART6->BRR |= (12 << USART_BRR_DIV_Fraction_Pos); // 12/16 = 0.75


    // USART6 interrupt setup
    // received data register not empty interrupt enable
    USART6->CR1 |= USART_CR1_RXNEIE;

    USART6->CR1 |= USART_CR1_TE; // enable transmitter
    USART6->CR1 |= USART_CR1_RE; // enable receiver
    // enable USART6
    USART6->CR1 |= (USART_CR1_UE);
}


void enableInterruptsUSART6(uint8_t priority)
{
    NVIC_SetPriority(USART6_IRQn, priority);
    NVIC_EnableIRQ(USART6_IRQn);
}


void setUpTimer11(void)
{
    RCC->APB2ENR |= (RCC_APB2ENR_TIM11EN); // enable clock for Timer 11

    TIM11->CR1 &= ~(TIM_CR1_CEN); // disable counter

    RCC->APB2RSTR |= (RCC_APB2RSTR_TIM11RST); // reset Timer 11
    RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM11RST); // deassert Timer 11 reset

    // 100 MHz ÷ 5000 = 20 kHz
    // TIM11->PSC = 5000 - 1;
    // TIM11->ARR = 10 - 1; // (2 kHz) 500 μs (per interrupt)

    // 96 MHz ÷ 960 = 100 kHz (10 μs)
    // TIM11->PSC = 960 - 1;
    // TIM11->ARR = 10 - 1; // (10 kHz) 100 μs (per interrupt)

    // 96 MHz ÷ 480 = 200 kHz (5 μs)
    TIM11->PSC = 480 - 1;
    TIM11->ARR = 10 - 1; // (20 kHz) 50 μs (per interrupt)

    TIM11->EGR |= TIM_EGR_UG; // re-initialize the counter
    TIM11->DIER |= TIM_DIER_UIE; // enable update interrupt
    TIM11->CR1 |= (TIM_CR1_CEN); // enable counter
}


void enableInterruptsTimer11(uint8_t priority)
{
    NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, priority);
    NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
}


void setUpTimer2(void)
{
    //*************************************************************************
    //                         Setup Timer 2
    //
    // IMPORTANT
    // since sys_clk = 100 MHz, and APB1 is prescaled by 2 (50 MHz)
    // then all timers get 2x APB1 clock
    // if APB1 was not prescaled, then its timers would get 1x APB1 clock
    //*************************************************************************
    RCC->APB1RSTR |= (RCC_APB1RSTR_TIM2RST); // reset Timer 2
    RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM2RST); // deassert Timer 2 reset
    RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN); // enable clock for Timer 2

    TIM2->CR1 &= ~(TIM_CR1_CEN); // disable counter

    TIM2->CR1 &= ~(TIM_CR1_DIR); // DIR = 0; upcounting

    // 100 MHz ÷ 5000 = 20 kHz
    // TIM2->PSC = 5000 - 1;
    // TIM2->ARR = 10 - 1; // (2 kHz) 500 μs (per interrupt)

    // 96 MHz ÷ 4800 = 20 kHz
    TIM2->PSC = 4800 - 1;
    TIM2->ARR = 10 - 1; // (20 kHz ÷ 10 = 2 kHz) 500 μs (per interrupt)

    TIM2->EGR |= TIM_EGR_UG; // re-initialize the counter
    TIM2->DIER |= TIM_DIER_UIE; // update interrupt enable
    TIM2->CR1 |= (TIM_CR1_CEN); // enable counter
}


void enableInterruptsTimer2(uint8_t priority)
{
    NVIC_SetPriority(TIM2_IRQn, priority);
    NVIC_EnableIRQ(TIM2_IRQn);
}


void setUpI2c2(void)
{
    // 1. Disable I2C
    I2C2->CR1 &= ~I2C_CR1_PE;


    // 2. Reset I2C Clock
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
    asm("nop");
    asm("nop");
    asm("nop");
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
    asm("nop");
    asm("nop");
    asm("nop");
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    asm("nop");
    asm("nop");
    asm("nop");


    // 3. Set I2C Clock Source (HSI)
    // there's no neeed for that; all APB1 peripherals get their clock directly
    // from the bus on stm32f411


    // 4. Set up I2C timing
    // 4.1. peripheral clock frequency (must be set to APB1 frequency)
    I2C2->CR2 &= ~(I2C_CR2_FREQ_Msk); // reset FREQ[5:0]
    I2C2->CR2 |= (0x32UL << I2C_CR2_FREQ_Pos); // 50 MHz
    // 4.2. Clock control register (CCR)
    I2C2->CCR &= ~I2C_CCR_FS; // F/S = Sm mode
    // for Sm mode (Standard mode, up to 100 kHz)
    // CCR = 0.5 * (f_pclk/f_scl)
    // where f_pclk = APB1 clock frequency
    // f_scl = SCL frequency (100 kHz for Sm mode)
    // CCR = 250 (0xFA)
    I2C2->CCR &= ~(I2C_CCR_CCR_Msk); // reset CCR[11:0]
    I2C2->CCR |= (0xFAUL << I2C_CCR_CCR_Pos); // CCR[11:0] = 250
    // 4.3. Rise time register (TRISE)
    // Trise[5:0] = (T_rise_max / T_pclk1) + 1
    // T_rise_max = max rise time for particular mode (1000 ns for Sm mode)
    // T_pclk1 = APB1 period (f_max 50 MHz; t_max = 20 ns)
    // Trise[5:0] = (1000 / 20) + 1 = 51 (0x33)
    I2C2->TRISE &= ~(I2C_TRISE_TRISE_Msk); // reset TRISE[5:0]
    I2C2->TRISE |= (0x33UL << I2C_TRISE_TRISE_Pos); // TRISE[5:0] = 51


    // 5. Set up SDA GPIO
    RCC->AHB1ENR |= I2C2_SDA_GPIOEN;
    I2C2_SDA_GPIO->MODER &= ~(0x3 << (I2C2_SDA_PIN * 2)); // reset value (mode)
    I2C2_SDA_GPIO->MODER |= (0x2 << (I2C2_SDA_PIN * 2)); // alternate function
    I2C2_SDA_GPIO->OTYPER |= (0x1 << I2C2_SDA_PIN); // open drain
    I2C2_SDA_GPIO->OSPEEDR &= ~(0x3 << (I2C2_SDA_PIN * 2)); // reset value (speed)
    I2C2_SDA_GPIO->OSPEEDR |= (0x3 << (I2C2_SDA_PIN * 2)); // high speed
    I2C2_SDA_GPIO->PUPDR &= ~(0x3 << (I2C2_SDA_PIN * 2)); // reset value (no pull-up/down)
    // I2C2_SDA_GPIO->PUPDR |= (0x1 << (I2C2_SDA_PIN * 2)); // pull-up
    I2C2_SDA_GPIO->AFR[0] &= ~(0xF << (I2C2_SDA_PIN * 4)); // reset value (alt. func.)
    I2C2_SDA_GPIO->AFR[0] |= (0x9 << (I2C2_SDA_PIN * 4)); // AF9 = I2C2, SDA


    // 6. Set up SCL GPIO
    RCC->AHB1ENR |= I2C2_SCL_GPIOEN;
    I2C2_SCL_GPIO->MODER &= ~(0x3 << (I2C2_SCL_PIN * 2)); // reset value (mode)
    I2C2_SCL_GPIO->MODER |= (0x2 << (I2C2_SCL_PIN * 2)); // alternate function
    I2C2_SCL_GPIO->OTYPER |= (0x1 << I2C2_SCL_PIN); // open drain
    I2C2_SCL_GPIO->OSPEEDR &= ~(0x3 << (I2C2_SCL_PIN * 2)); // reset value (speed)
    I2C2_SCL_GPIO->OSPEEDR |= (0x3 << (I2C2_SCL_PIN * 2)); // high speed
    I2C2_SCL_GPIO->PUPDR &= ~(0x3 << (I2C2_SCL_PIN * 2)); // reset value (no pull-up/down)
    // I2C2_SCL_GPIO->PUPDR |= (0x1 << (I2C2_SCL_PIN * 2)); // pull-up
    I2C2_SCL_GPIO->AFR[1] &= ~(0xF << ((I2C2_SCL_PIN - 8) * 4)); // reset value (alt. func.)
    I2C2_SCL_GPIO->AFR[1] |= (0x4 << ((I2C2_SCL_PIN - 8) * 4)); // AF4 = I2C2, SDA


    // 7. Enable I2C
    I2C2->CR1 |= I2C_CR1_PE;
}


void setUpI2c1(void)
{
    // 1. Disable I2C
    I2C1->CR1 &= ~I2C_CR1_PE;


    // 2. Reset I2C Clock
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    asm("nop");
    asm("nop");
    asm("nop");
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    asm("nop");
    asm("nop");
    asm("nop");
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    asm("nop");
    asm("nop");
    asm("nop");

    // . Disable I2C
    I2C1->CR1 &= ~I2C_CR1_PE;


    // I2C1->CR1 |= I2C_CR1_SWRST;
    // asm("nop");
    // asm("nop");
    // asm("nop");
    // asm("nop");
    // asm("nop");
    // I2C1->CR1 &= ~I2C_CR1_SWRST;
    // asm("nop");
    // asm("nop");
    // asm("nop");
    // asm("nop");
    // asm("nop");


    // 3. Set I2C Clock Source (HSI)
    // there's no neeed for that; all APB1 peripherals get their clock directly
    // from the bus on stm32f411


    // 4. Setup I2C timing
    // 4.1. peripheral clock frequency (must be set to APB1 frequency)
    I2C1->CR2 &= ~(I2C_CR2_FREQ_Msk); // reset FREQ[5:0]
    // I2C1->CR2 |= (0x32UL << I2C_CR2_FREQ_Pos); // 50 MHz
    I2C1->CR2 |= (ABP1_FREQ_MHZ << I2C_CR2_FREQ_Pos);


    // 4.2. Clock control register (CCR)
    I2C1->CCR &= ~I2C_CCR_FS; // F/S = Sm mode
    // for Sm mode (Standard mode, up to 100 kHz)
    // CCR = 0.5 * (f_pclk/f_scl)
    // where f_pclk = APB1 clock frequency
    // f_scl = SCL frequency (100 kHz for Sm mode)
    // CCR = 250 (0xFA)
    I2C1->CCR &= ~(I2C_CCR_CCR_Msk); // reset CCR[11:0]
    // I2C1->CCR |= (0xFAUL << I2C_CCR_CCR_Pos); // CCR[11:0] = 250
    // I2C1->CCR |= (0x10EUL << I2C_CCR_CCR_Pos); // CCR[11:0] = 270
    // I2C1->CCR |= (0x1F4UL << I2C_CCR_CCR_Pos); // CCR[11:0] = 500
    // I2C1->CCR |= (0x15EUL << I2C_CCR_CCR_Pos); // CCR[11:0] = 350
    // I2C1->CCR |= (0x9C4UL << I2C_CCR_CCR_Pos); // CCR[11:0] = 2500
    // I2C1->CCR |= (0x0096UL << I2C_CCR_CCR_Pos); // CCR[11:0] = ?
    // I2C1->CCR |= (0x012CUL << I2C_CCR_CCR_Pos); // CCR[11:0] = ?
    // APB1 = 25 MHz, f_scl = 100 kHz
    // CCR = 0.5 * ((25*10^6)/(100*10^3))
    // CCR = 125
    // I2C1->CCR &= ~(I2C_CCR_CCR_Msk); // reset CCR[11:0]
    // I2C1->CCR |= ((125) << I2C_CCR_CCR_Pos); // CCR[11:0] = 125
    // APB1 = 25 MHz, f_scl = 70 kHz
    // CCR = 0.5 * ((25*10^6)/(70*10^3))
    // CCR = 179
    // I2C1->CCR &= ~(I2C_CCR_CCR_Msk); // reset CCR[11:0]
    // I2C1->CCR |= ((179) << I2C_CCR_CCR_Pos); // CCR[11:0] = 179
    // APB1 = 25 MHz, f_scl = 10 kHz
    // CCR = 0.5 * ((25*10^6)/(10*10^3))
    // CCR = 1250
    I2C1->CCR &= ~(I2C_CCR_CCR_Msk); // reset CCR[11:0]
    I2C1->CCR |= ((1250) << I2C_CCR_CCR_Pos); // CCR[11:0] = 1250

    // 4.3. Rise time register (TRISE)
    // Trise[5:0] = (T_rise_max / T_pclk1) + 1
    // T_rise_max = max rise time for particular mode (1000 ns for Sm mode)
    // T_pclk1 = APB1 period (f_max 50 MHz; t_max = 20 ns)
    // Trise[5:0] = (1000 / 20) + 1 = 51 (0x33)
    I2C1->TRISE &= ~(I2C_TRISE_TRISE_Msk); // reset TRISE[5:0]
    // APB1 = 50 MHz
    // I2C1->TRISE |= (0x33UL << I2C_TRISE_TRISE_Pos); // TRISE[5:0] = 51
    // APB1 = 25 MHz
    // (1/(25*10^6))*10^9 = 40 ns
    // (1000 / 40) + 1 = 26
    I2C1->TRISE |= ((26) << I2C_TRISE_TRISE_Pos); // TRISE[5:0] = 26


    // 5. Set up SDA GPIO
    RCC->AHB1ENR |= I2C1_SDA_GPIOEN;
    I2C1_SDA_GPIO->MODER &= ~(0x3 << (I2C1_SDA_PIN * 2)); // reset value (mode)
    I2C1_SDA_GPIO->MODER |= (0x2 << (I2C1_SDA_PIN * 2)); // alternate function
    I2C1_SDA_GPIO->OTYPER |= (0x1 << I2C1_SDA_PIN); // open drain
    I2C1_SDA_GPIO->OSPEEDR &= ~(0x3 << (I2C1_SDA_PIN * 2)); // reset value (speed)
    I2C1_SDA_GPIO->OSPEEDR |= (0x3 << (I2C1_SDA_PIN * 2)); // high speed
    // I2C1_SDA_GPIO->OSPEEDR |= (0x1 << (I2C1_SDA_PIN * 2)); // medium speed
    I2C1_SDA_GPIO->PUPDR &= ~(0x3 << (I2C1_SDA_PIN * 2)); // reset value (no pull-up/down)
    // I2C1_SDA_GPIO->PUPDR |= (0x1 << (I2C1_SDA_PIN * 2)); // pull-up
    // I2C1_SDA_GPIO->AFR[0] &= ~(0xF << (I2C1_SDA_PIN * 4)); // reset value (alt. func.)
    // I2C1_SDA_GPIO->AFR[0] |= (0x4 << (I2C1_SDA_PIN * 4)); // AF4 = I2C1, SDA
    I2C1_SDA_GPIO->AFR[1] &= ~(0xF << ((I2C1_SDA_PIN - 8) * 4)); // reset value (alt. func.)
    I2C1_SDA_GPIO->AFR[1] |= (0x4 << ((I2C1_SDA_PIN - 8) * 4)); // AF4 = I2C1, SDA


    // 6. Set up SCL GPIO
    RCC->AHB1ENR |= I2C1_SCL_GPIOEN;
    I2C1_SCL_GPIO->MODER &= ~(0x3 << (I2C1_SCL_PIN * 2)); // reset value (mode)
    I2C1_SCL_GPIO->MODER |= (0x2 << (I2C1_SCL_PIN * 2)); // alternate function
    I2C1_SCL_GPIO->OTYPER |= (0x1 << I2C1_SCL_PIN); // open drain
    I2C1_SCL_GPIO->OSPEEDR &= ~(0x3 << (I2C1_SCL_PIN * 2)); // reset value (speed)
    I2C1_SCL_GPIO->OSPEEDR |= (0x3 << (I2C1_SCL_PIN * 2)); // high speed
    // I2C1_SCL_GPIO->OSPEEDR |= (0x1 << (I2C1_SCL_PIN * 2)); // medium speed
    I2C1_SCL_GPIO->PUPDR &= ~(0x3 << (I2C1_SCL_PIN * 2)); // reset value (no pull-up/down)
    // I2C1_SCL_GPIO->PUPDR |= (0x1 << (I2C1_SCL_PIN * 2)); // pull-up
    // I2C1_SCL_GPIO->AFR[0] &= ~(0xF << (I2C1_SCL_PIN * 4)); // reset value (alt. func.)
    // I2C1_SCL_GPIO->AFR[0] |= (0x4 << (I2C1_SCL_PIN * 4)); // AF4 = I2C1, SDA
    I2C1_SCL_GPIO->AFR[1] &= ~(0xF << ((I2C1_SCL_PIN - 8) * 4)); // reset value (alt. func.)
    I2C1_SCL_GPIO->AFR[1] |= (0x4 << ((I2C1_SCL_PIN - 8) * 4)); // AF4 = I2C1, SDA

    // I2C1_SDA_GPIO->ODR |= (0x1 << I2C1_SDA_PIN);
    // I2C1_SCL_GPIO->ODR |= (0x1 << I2C1_SCL_PIN);
    // I2C1_SDA_GPIO->ODR &= ~(0x1 << I2C1_SDA_PIN);
    // I2C1_SCL_GPIO->ODR &= ~(0x1 << I2C1_SCL_PIN);
    // I2C1_SDA_GPIO->ODR |= (0x1 << I2C1_SDA_PIN);
    // I2C1_SCL_GPIO->ODR |= (0x1 << I2C1_SCL_PIN);


    I2C1->OAR1 &= ~(I2C_OAR1_ADDMODE);
    // 0x24 is a completely arbitrary value, btw.
    I2C1->OAR1 |= (0x24 << I2C_OAR1_ADD1_Pos);
    // 0x28 is a completely arbitrary value
    I2C1->OAR2 |= (0x28 << I2C_OAR2_ADD2_Pos);




    I2C1->CR1 |= I2C_CR1_SWRST;
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    I2C1->CR1 &= ~I2C_CR1_SWRST;
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");


    // turn analog filter off
    // I2C1->FLTR |= I2C_FLTR_ANOFF;


    debuggerWriteStr(DBG_USART, "i2c busy check before PE=1\n\r");
    if (I2C1->SR2 & I2C_SR2_BUSY)
    {
        debuggerWriteStr(DBG_USART, "i2c BUSY = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "i2c BUSY = 0\n\r");
    }


    // 7. Enable I2C
    I2C1->CR1 |= I2C_CR1_PE;


    // ACK enable
    I2C1->CR1 |= (I2C_CR1_ACK);


    debuggerWriteStr(DBG_USART, "i2c busy check just after PE=1\n\r");
    if (I2C1->SR2 & I2C_SR2_BUSY)
    {
        debuggerWriteStr(DBG_USART, "i2c BUSY = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "i2c BUSY = 0\n\r");
    }
}


void stm32f411SetupPeripherals(void)
{
    //*************************************************************************
    //                         I2C1 GPIO Demo
    //*************************************************************************
    // // 5. Set up SDA GPIO
    // RCC->AHB1ENR |= I2C1_SDA_GPIOEN;
    // I2C1_SDA_GPIO->MODER &= ~(0x3 << (I2C1_SDA_PIN * 2)); // reset value (mode)
    // // I2C1_SDA_GPIO->MODER |= (0x2 << (I2C1_SDA_PIN * 2)); // alternate function
    // I2C1_SDA_GPIO->MODER |= (0x1 << (I2C1_SDA_PIN * 2)); // GP Output mode
    // I2C1_SDA_GPIO->OTYPER |= (0x1 << I2C1_SDA_PIN); // open drain
    // I2C1_SDA_GPIO->OSPEEDR &= ~(0x3 << (I2C1_SDA_PIN * 2)); // reset value (speed)
    // // I2C1_SDA_GPIO->OSPEEDR |= (0x3 << (I2C1_SDA_PIN * 2)); // high speed
    // I2C1_SDA_GPIO->OSPEEDR |= (0x1 << (I2C1_SDA_PIN * 2)); // medium speed
    // I2C1_SDA_GPIO->PUPDR &= ~(0x3 << (I2C1_SDA_PIN * 2)); // reset value (no pull-up/down)
    // // I2C1_SDA_GPIO->PUPDR |= (0x1 << (I2C1_SDA_PIN * 2)); // pull-up
    // // I2C1_SDA_GPIO->AFR[0] &= ~(0xF << (I2C1_SDA_PIN * 4)); // reset value (alt. func.)
    // // I2C1_SDA_GPIO->AFR[0] |= (0x4 << (I2C1_SDA_PIN * 4)); // AF4 = I2C1, SDA
    // // I2C1_SDA_GPIO->AFR[1] &= ~(0xF << ((I2C1_SDA_PIN - 8) * 4)); // reset value (alt. func.)
    // // I2C1_SDA_GPIO->AFR[1] |= (0x4 << ((I2C1_SDA_PIN - 8) * 4)); // AF4 = I2C1, SDA


    // // 6. Set up SCL GPIO
    // RCC->AHB1ENR |= I2C1_SCL_GPIOEN;
    // I2C1_SCL_GPIO->MODER &= ~(0x3 << (I2C1_SCL_PIN * 2)); // reset value (mode)
    // // I2C1_SCL_GPIO->MODER |= (0x2 << (I2C1_SCL_PIN * 2)); // alternate function
    // I2C1_SCL_GPIO->MODER |= (0x1 << (I2C1_SCL_PIN * 2)); // GP output mode
    // I2C1_SCL_GPIO->OTYPER |= (0x1 << I2C1_SCL_PIN); // open drain
    // I2C1_SCL_GPIO->OSPEEDR &= ~(0x3 << (I2C1_SCL_PIN * 2)); // reset value (speed)
    // // I2C1_SCL_GPIO->OSPEEDR |= (0x3 << (I2C1_SCL_PIN * 2)); // high speed
    // I2C1_SCL_GPIO->OSPEEDR |= (0x1 << (I2C1_SCL_PIN * 2)); // medium speed
    // I2C1_SCL_GPIO->PUPDR &= ~(0x3 << (I2C1_SCL_PIN * 2)); // reset value (no pull-up/down)
    // // I2C1_SCL_GPIO->PUPDR |= (0x1 << (I2C1_SCL_PIN * 2)); // pull-up
    // // I2C1_SCL_GPIO->AFR[0] &= ~(0xF << (I2C1_SCL_PIN * 4)); // reset value (alt. func.)
    // // I2C1_SCL_GPIO->AFR[0] |= (0x4 << (I2C1_SCL_PIN * 4)); // AF4 = I2C1, SDA
    // // I2C1_SCL_GPIO->AFR[1] &= ~(0xF << ((I2C1_SCL_PIN - 8) * 4)); // reset value (alt. func.)
    // // I2C1_SCL_GPIO->AFR[1] |= (0x4 << ((I2C1_SCL_PIN - 8) * 4)); // AF4 = I2C1, SDA
    //*************************************************************************
    //                       end: I2C1 GPIO Demo
    //*************************************************************************
}


void setUpSpi1(void)
{
    // (1) SPI_SCK
    // enable GPIO clock (RCC)
    RCC->AHB1ENR |= SPI1_SCK_GPIOEN;
    // clear mode before assigning it the intended value
    SPI1_SCK_GPIO->MODER &= ~(0x3 << (SPI1_SCK_PIN * 2));
    // set mode to alternate function mode (0b10 = 0x2)
    SPI1_SCK_GPIO->MODER |= (0x2 << (SPI1_SCK_PIN * 2));
    // Port Output Type Register set to 0 (output push-pull)
    SPI1_SCK_GPIO->OTYPER &= ~(1 << SPI1_SCK_PIN);
    // clear Port Output Speed Register (default value = low speed)
    SPI1_SCK_GPIO->OSPEEDR &= ~(0x3 << (SPI1_SCK_PIN * 2));
    // set GPIO speed to medium
    SPI1_SCK_GPIO->OSPEEDR |= (0x1 << (SPI1_SCK_PIN * 2));
    // clear pull-up/pull-down register
    // -> no pull-up, pull-down
    SPI1_SCK_GPIO->PUPDR &= ~(0x3 << (SPI1_SCK_PIN * 2));
    // AFR[0] = AFR Low Register (pins 0..7)
    // AFR[1] = AFR High Register (pins 8..15)
    // clear alternate function (set to zero)
    SPI1_SCK_GPIO->AFR[0] &= ~(0xF << (SPI1_SCK_PIN * 4));
    // select alternate function
    SPI1_SCK_GPIO->AFR[0] |= (SPI1_SCK_AF_NUM << (SPI1_SCK_PIN * 4));


    // (2) SPI_MISO
    // enable GPIO clock (RCC)
    RCC->AHB1ENR |= SPI1_MISO_GPIOEN;
    // clear mode before assigning it the intended value
    SPI1_MISO_GPIO->MODER &= ~(0x3 << (SPI1_MISO_PIN * 2));
    // set mode to alternate function mode (0b10 = 0x2)
    SPI1_MISO_GPIO->MODER |= (0x2 << (SPI1_MISO_PIN * 2));
    // Port Output Type Register set to 0 (output push-pull)
    SPI1_MISO_GPIO->OTYPER &= ~(1 << SPI1_MISO_PIN);
    // clear Port Output Speed Register (default value = low speed)
    SPI1_MISO_GPIO->OSPEEDR &= ~(0x3 << (SPI1_MISO_PIN * 2));
    // set GPIO speed to medium
    // SPI1_MISO_GPIO->OSPEEDR |= (0x1 << (SPI1_MISO_PIN * 2));
    SPI1_MISO_GPIO->OSPEEDR |= (0x3 << (SPI1_MISO_PIN * 2)); // high
    // clear pull-up/pull-down register
    // -> no pull-up, pull-down
    SPI1_MISO_GPIO->PUPDR &= ~(0x3 << (SPI1_MISO_PIN * 2));

    // SPI1_MISO_GPIO->PUPDR |= (0x1 << (SPI1_MISO_PIN * 2)); // PULL-UP
    
    // AFR[0] = AFR Low Register (pins 0..7)
    // AFR[1] = AFR High Register (pins 8..15)
    // clear alternate function (set to zero)
    SPI1_MISO_GPIO->AFR[0] &= ~(0xF << (SPI1_MISO_PIN * 4));
    // select alternate function
    SPI1_MISO_GPIO->AFR[0] |= (SPI1_MISO_AF_NUM << (SPI1_MISO_PIN * 4));


    // (3) SPI_MOSI
    // enable GPIO clock (RCC)
    RCC->AHB1ENR |= SPI1_MOSI_GPIOEN;
    // clear mode before assigning it the intended value
    SPI1_MOSI_GPIO->MODER &= ~(0x3 << (SPI1_MOSI_PIN * 2));
    // set mode to alternate function mode (0b10 = 0x2)
    SPI1_MOSI_GPIO->MODER |= (0x2 << (SPI1_MOSI_PIN * 2));
    // Port Output Type Register set to 0 (output push-pull)
    SPI1_MOSI_GPIO->OTYPER &= ~(1 << SPI1_MOSI_PIN);
    // clear Port Output Speed Register (default value = low speed)
    SPI1_MOSI_GPIO->OSPEEDR &= ~(0x3 << (SPI1_MOSI_PIN * 2));
    // set GPIO speed to medium
    SPI1_MOSI_GPIO->OSPEEDR |= (0x1 << (SPI1_MOSI_PIN * 2));
    // clear pull-up/pull-down register
    // -> no pull-up, pull-down
    SPI1_MOSI_GPIO->PUPDR &= ~(0x3 << (SPI1_MOSI_PIN * 2));
    // AFR[0] = AFR Low Register (pins 0..7)
    // AFR[1] = AFR High Register (pins 8..15)
    // clear alternate function (set to zero)
    SPI1_MOSI_GPIO->AFR[0] &= ~(0xF << (SPI1_MOSI_PIN * 4));
    // select alternate function
    SPI1_MOSI_GPIO->AFR[0] |= (SPI1_MOSI_AF_NUM << (SPI1_MOSI_PIN * 4));


    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    RCC->APB2RSTR &= ~(RCC_APB2RSTR_SPI1RST);
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // 1. configure SCK baud rate
    SPI1->CR1 &= ~(SPI_CR1_BR); // reset BR bits; set to 0b000
    // 0b101 = f_PCLK/64
    // 100 MHz / 64 = 1.5625 MHz
    // SPI1->CR1 |= (SPI_CR1_BR_0 | SPI_CR1_BR_2);
    // 0b010 = f_PCLK/8
    // 100 MHz / 8 = 12.5 MHz
    // SPI1->CR1 |= (SPI_CR1_BR_1);
    
    // 96 MHz / 64 = 1.5 MHz
    // SPI1->CR1 |= (SPI_CR1_BR_2 | SPI_CR1_BR_0);

    // 96 MHz / 8 = 12 MHz
    // SPI1->CR1 |= (SPI_CR1_BR_1);

    // 96 MHz / 16 = 6 MHz
    SPI1->CR1 |= (SPI_CR1_BR_1 | SPI_CR1_BR_0);

    // 96 MHz / 256 = 375 kHz
    // SPI1->CR1 |= (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0);

    // 96 MHz / 128 = 750 kHz
    // SPI1->CR1 |= (SPI_CR1_BR_2 | SPI_CR1_BR_1);

    // 96 MHz / 4 = 24 MHz
    // SPI1->CR1 |= (SPI_CR1_BR_0);


    // 2. configure SCK phase and polarity
    // CPOL = 0; SCK is 0 when idle
    // CPHA = 0; latch data bit on rising edge (when CPOL = 0)
    SPI1->CR1 &= ~(SPI_CR1_CPHA | SPI_CR1_CPOL);
    // CPOL = 1; SCK is 1 when idle
    // CPHA = 1; latch data bit on rising edge (when CPOL = 1)
    SPI1->CR1 |= (SPI_CR1_CPHA | SPI_CR1_CPOL);
    // CPHA = 0; latch data bit on falling edge (when CPOL = 1)
    // SPI1->CR1 |= (SPI_CR1_CPOL);

    // 3. select frame length (8-bit or 16-bit)
    SPI1->CR1 &= ~(SPI_CR1_DFF); // 0: 8-bit

    // 4. configure frame format (MSB first or LSB first)
    SPI1->CR1 &= ~(SPI_CR1_LSBFIRST); // 0: MSB first

    // 2-line unidirectional mode
    SPI1->CR1 &= ~(SPI_CR1_BIDIMODE);

    // 5. NSS pin config
    // I'm not gonna be using NSS
    // software (manual) control of NSS
    SPI1->CR1 |= SPI_CR1_SSM;
    // set NSS to high level
    SPI1->CR1 |= SPI_CR1_SSI;

    // 6. frame format mode (Motorola or TI)
    SPI1->CR2 &= ~(SPI_CR2_FRF); // 0: Motorola


    // enable SPI interrupts
    // SPI1->CR2 |= SPI_CR2_TXEIE;
    SPI1->CR2 |= SPI_CR2_RXNEIE;
    SPI1->CR2 |= SPI_CR2_ERRIE;


    // 7. select master mode and enable SPI
    SPI1->CR1 |= SPI_CR1_MSTR; // master mode
    SPI1->CR1 |= SPI_CR1_SPE; // SPI enable
}


void enableInterruptsSpi1(uint8_t priority)
{
    NVIC_SetPriority(SPI1_IRQn, priority);
    NVIC_EnableIRQ(SPI1_IRQn);
}