#ifndef KEY_H
#define KEY_H

#include    <stdio.h>
#include    <stm32f10x.h>
#include    "rtthread.h"

#define G_SET_BIT(a, b)     (a |= (1 << b))
#define G_CLEAR_BIT(a, b)   (a &= ~(1 << b))
#define G_IS_BIT_SET(a, b)  (a & (1 << b))

#define KEY_TIME_MS         1
#define KEY_MAX_NUMBER      12
#define DEBOUNCE_TIME       3  //FANG DOU
#define PRESS_LONG_TIME     300

#define NO_KEY      0X0000
#define KEY_DOWN    0X1000
#define KEY_UP      0X2000
#define KEY_LIAN    0X4000
#define KEY_LONG    0X8000

typedef void (*keyfunction)(void);

typedef struct
{
    rt_uint8_t      key_number;
    rt_uint32_t     key_rccperiph;
    GPIO_TypeDef    *key_port;
    rt_uint32_t     key_GPIO;
    keyfunction     short_press;
    keyfunction     long_press;
}key_typedef_t;

typedef struct
{
    rt_uint8_t      key_totol_number;
    key_typedef_t   *single_key;
}keys_typedef_t;

void key_handle(keys_typedef_t *keys);
void key_para_init(keys_typedef_t *keys);
rt_uint16_t get_key(keys_typedef_t *keys);
rt_uint16_t read_key_value(keys_typedef_t *keys);
key_typedef_t key_init_one(rt_uint32_t key_rccperiph, GPIO_TypeDef *key_port, rt_uint32_t key_GPIO, keyfunction short_press, keyfunction long_press);
void key_init(void);

#endif // KEY_H
