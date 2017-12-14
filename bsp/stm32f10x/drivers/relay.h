#ifndef RELAY_H
#define RELAY_H

#include "board.h"
#include "rtthread.h"


//relay define
#define relay0_rcc      RCC_APB2Periph_GPIOB
#define relay0_gpio     GPIOB
#define relay0_pin      (GPIO_Pin_5)

#define relay1_rcc      RCC_APB2Periph_GPIOB
#define relay1_gpio     GPIOA
#define relay1_pin      (GPIO_Pin_12)

#define relay2_rcc      RCC_APB2Periph_GPIOB
#define relay2_gpio     GPIOB
#define relay2_pin      (GPIO_Pin_3)

void relay_init(void);
void relay_on(rt_int8_t index);
void relay_off(rt_uint8_t index);

#endif // RELAY_H
