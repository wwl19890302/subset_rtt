#include "relay.h"
#include "stm32f10x.h"

//relay define
#define relay0_rcc      RCC_APB2Periph_GPIOB
#define relay0_gpio     GPIOB
#define relay0_pin      (GPIO_Pin_5)

#define relay1_rcc      RCC_APB2Periph_GPIOB
#define relay1_gpio     GPIOB
#define relay1_pin      (GPIO_Pin_4)

#define relay2_rcc      RCC_APB2Periph_GPIOB
#define relay2_gpio     GPIOB
#define relay2_pin      (GPIO_Pin_3)

void relay_init(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(relay0_rcc|relay1_rcc|relay2_rcc, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = relay0_pin;
    GPIO_Init(relay0_gpio, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = relay1_pin;
    GPIO_Init(relay1_gpio, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = relay2_pin;
    GPIO_Init(relay2_gpio, &GPIO_InitStructure);

    GPIO_ResetBits(relay0_gpio, relay0_pin);
    GPIO_ResetBits(relay1_gpio, relay1_pin);
    GPIO_ResetBits(relay2_gpio, relay2_pin);
}

void relay_on(rt_int8_t index)
{
    switch(index)
    {
    case 0:
        GPIO_SetBits(relay0_gpio, relay0_pin);
        break;
    case 1:
        GPIO_SetBits(relay1_gpio, relay1_pin);
        break;
    case 2:
        GPIO_SetBits(relay2_gpio, relay2_pin);
        break;
    default:
        break;
    }
}

void relay_off(rt_uint8_t index)
{
    switch(index)
    {
    case 0:
        GPIO_ResetBits(relay0_gpio, relay0_pin);
        break;
    case 1:
        GPIO_ResetBits(relay1_gpio, relay1_pin);
        break;
    case 2:
        GPIO_ResetBits(relay2_gpio, relay2_pin);
        break;
    default:
        break;
    }
}

