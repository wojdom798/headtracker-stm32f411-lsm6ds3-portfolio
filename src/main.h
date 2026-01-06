#ifndef _MAIN_H
#define _MAIN_H

#include <stdint.h>
#include "stm32f411xe.h"


// ----------------------------------------------------------------------------
//             Blackpill Board Defines (User LED and Button)
// ----------------------------------------------------------------------------
#define BLACKPILL_LED_GPIO (GPIOC)
#define BLACKPILL_LED_PIN (13)

#define BLACKPILL_BUTTON_GPIO (GPIOA)
#define BLACKPILL_BUTTON_PIN (0)

// common anode
#define BLACKPILL_LED_ON()     (BLACKPILL_LED_GPIO->ODR &= ~(1 << BLACKPILL_LED_PIN))
#define BLACKPILL_LED_OFF()    (BLACKPILL_LED_GPIO->ODR |= (1 << BLACKPILL_LED_PIN))
#define BLACKPILL_LED_TOGGLE() (BLACKPILL_LED_GPIO->ODR ^= (1 << BLACKPILL_LED_PIN))


#endif