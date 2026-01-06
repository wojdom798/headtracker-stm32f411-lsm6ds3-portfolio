#include "stm32f411Blackpill.h"

// Blackpill User Button Debounce - variables
uint32_t bPillButtonDebounceCount = 0;
uint8_t blackPillStatusFlags = 0;



void setUpF411Blackpill(void)
{
    // Blackpill Board LED GPIO setup
	RCC->AHB1ENR |= BLACKPILL_LED_GPIOEN;
	BLACKPILL_LED_GPIO->MODER &= ~(0x3 << (BLACKPILL_LED_PIN * 2));
    // general purpose output mode
	BLACKPILL_LED_GPIO->MODER |= (0x1 << (BLACKPILL_LED_PIN * 2));
	BLACKPILL_LED_GPIO->OTYPER &= ~(1 << BLACKPILL_LED_PIN); // output push-pull
	BLACKPILL_LED_GPIO->OSPEEDR &= ~(0x3 << (BLACKPILL_LED_PIN * 2)); // low speed
	BLACKPILL_LED_GPIO->PUPDR &= ~(0x3 << (BLACKPILL_LED_PIN * 2)); // no pull-up, no pull-down


    // Blackpill Board User Key (Button) GPIO setup
    RCC->AHB1ENR |= BLACKPILL_BUTTON_GPIOEN;
    BLACKPILL_BUTTON_GPIO->MODER &= ~(0x3 << (BLACKPILL_BUTTON_PIN*2)); // input mode
    // reset PUPDR for the button
    BLACKPILL_BUTTON_GPIO->PUPDR &= ~(0x3 << (BLACKPILL_BUTTON_PIN*2));
    // enable pull-up for the button
    BLACKPILL_BUTTON_GPIO->PUPDR |= (0x1 << (BLACKPILL_BUTTON_PIN*2));

    // setup external interrupts for the user button
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // enable system configuration controller
    SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0); // reset
    SYSCFG->EXTICR[0] |= (0x0UL << SYSCFG_EXTICR1_EXTI0_Pos); // PA0

    EXTI->IMR |= (0x1UL << BLACKPILL_BUTTON_PIN); // interrupt request is not masked (=1)
    EXTI->RTSR &= ~(0x1UL << BLACKPILL_BUTTON_PIN); // rising trigger disabled
    EXTI->FTSR |= (0x1UL << BLACKPILL_BUTTON_PIN); // falling trigger enabled
}


void f411BlackpillEnableButtonInterrupts(uint8_t priority)
{
    // enable blackpill user button exti interrupt
    NVIC_SetPriority(EXTI0_IRQn, priority);
    NVIC_EnableIRQ(EXTI0_IRQn);
}