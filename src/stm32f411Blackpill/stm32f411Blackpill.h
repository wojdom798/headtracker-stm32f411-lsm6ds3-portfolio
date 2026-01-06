#ifndef _STM32F411_BLACKPILL_H
#define _STM32F411_BLACKPILL_H

#include <stdint.h>
#include "stm32f411xe.h"

// ----------------------------------------------------------------------------
//             Blackpill Board Defines (User LED and Button)
// ----------------------------------------------------------------------------
#define BLACKPILL_LED_GPIOEN    (RCC_AHB1ENR_GPIOCEN)
#define BLACKPILL_LED_GPIO      (GPIOC)
#define BLACKPILL_LED_PIN       (13)

#define BLACKPILL_BUTTON_GPIOEN (RCC_AHB1ENR_GPIOAEN)
#define BLACKPILL_BUTTON_GPIO   (GPIOA)
#define BLACKPILL_BUTTON_PIN    (0)

// common anode
#define BLACKPILL_LED_ON()      (BLACKPILL_LED_GPIO->ODR &= ~(1 << BLACKPILL_LED_PIN))
#define BLACKPILL_LED_OFF()     (BLACKPILL_LED_GPIO->ODR |= (1 << BLACKPILL_LED_PIN))
#define BLACKPILL_LED_TOGGLE()  (BLACKPILL_LED_GPIO->ODR ^= (1 << BLACKPILL_LED_PIN))



// ----------------------------------------------------------------------------
//             Defines for Blackpill User Button Debounce
// ----------------------------------------------------------------------------
#define B_PILL_BTN_DEBOUNCE_TIME (B_PILL_BTN_DEBOUNCE_150_MS)

// 2 * 500 us = 1000 us = 1 ms
// 200 * 1 ms = 200 ms
#define B_PILL_BTN_DEBOUNCE_200_MS (2 * 200)
// 2 * 500 us = 1000 us = 1 ms
// 120 * 1 ms = 120 ms
#define B_PILL_BTN_DEBOUNCE_120_MS (2 * 120)
// 2 * 500 us = 1000 us = 1 ms
// 150 * 1 ms = 150 ms
#define B_PILL_BTN_DEBOUNCE_150_MS (2 * 150)

#define B_PILL_BTN_STATUS_IS_DEBOUNCE_IN_PROGRESS (1U << 0U)
#define B_PILL_BTN_STATUS_IS_BTN_PRESSED          (1U << 1U)

#define IS_BLACKPILL_BTN_DEBOUNCE_IN_PROGRESS() ((blackPillStatusFlags & \
    B_PILL_BTN_STATUS_IS_DEBOUNCE_IN_PROGRESS) == B_PILL_BTN_STATUS_IS_DEBOUNCE_IN_PROGRESS)
#define SET_BLACKPILL_BTN_DEBOUNCE_IN_PROGRESS() (\
    blackPillStatusFlags |= B_PILL_BTN_STATUS_IS_DEBOUNCE_IN_PROGRESS)
#define CLR_BLACKPILL_DEBOUNCE_IN_PROGRESS() (\
    blackPillStatusFlags &= ~(B_PILL_BTN_STATUS_IS_DEBOUNCE_IN_PROGRESS))

#define IS_BLACKPILL_BTN_PRESSED() ((blackPillStatusFlags & \
    B_PILL_BTN_STATUS_IS_BTN_PRESSED) == B_PILL_BTN_STATUS_IS_BTN_PRESSED)
#define SET_BLACKPILL_BTN_PRESSED() (\
    blackPillStatusFlags |= B_PILL_BTN_STATUS_IS_BTN_PRESSED)
#define CLR_BLACKPILL_BTN_PRESSED() (\
    blackPillStatusFlags &= ~(B_PILL_BTN_STATUS_IS_BTN_PRESSED))
// ----------------------------------------------------------------------------
//            end: Defines for Blackpill User Button Debounce
// ----------------------------------------------------------------------------



void setUpF411Blackpill(void);
void f411BlackpillEnableButtonInterrupts(uint8_t priority);

#endif