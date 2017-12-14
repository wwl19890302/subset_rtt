#include "key.h"
#include "led.h"
// can not differrentiate 2 keys or more

keys_typedef_t  keys;

rt_uint8_t  key_count_time;
static  rt_uint8_t  key_totol_number = 0;

/* 定时器的控制块 */
static rt_timer_t key_timer;

/* 定时器超时函数 */
static void key_timeout(void* parameter)
{
    key_handle((keys_typedef_t *) &keys);
}

void timer_control_init()
{
    /* 创建定时器 */
    key_timer = rt_timer_create("key_timer",  /* 定时器名字是 key_timer */
        key_timeout, /* 超时时回调的处理函数 */
        RT_NULL, /* 超时函数的入口参数 */
        KEY_TIME_MS, /* 定时长度，以OS Tick为单位，即KEY_TIME_MS个OS Tick */
        RT_TIMER_FLAG_PERIODIC); /* 周期性定时器 */
    /* 启动定时器 */
    if (key_timer != RT_NULL)
        rt_timer_start(key_timer);
}

/**
 * @brief  read the GPIO state
 * @param [in] keys :key global structure pointer
 * @return GPIO status
 */
rt_uint16_t get_key(keys_typedef_t *keys)
{
    rt_uint8_t  i = 0;
    rt_uint16_t read_key = 0;

    /*GPIO cyclic scan*/
    for(i=0;i<keys->key_totol_number;i++)
    {
        if(!GPIO_ReadInputDataBit((GPIO_TypeDef*)keys->single_key[i].key_port, keys->single_key[i].key_GPIO))
        {
            G_SET_BIT(read_key, keys->single_key[i].key_number);
        }
    }
    return  read_key;
}

/**
 * @brief read the key value
 * @param [in] keys:key global structure pointer
 * @return GPIO status
 */
rt_uint16_t read_key_value(keys_typedef_t *keys)
{
    static rt_uint8_t key_check = 0;
    static rt_uint8_t key_state = 0;
    static rt_uint16_t key_long_check = 0;
    static rt_uint16_t key_prev = 0;    //last key

    rt_uint16_t key_press = 0;
    rt_uint16_t key_return = 0;

    key_count_time++;

    if(key_count_time >= (DEBOUNCE_TIME / KEY_TIME_MS)) //XIAO DOU
    {
        key_count_time = 0;
        key_check = 1;
    }
    if(1 == key_check)
    {
        key_check = 0;
        key_press = get_key(keys);  //note the down key

        switch (key_state)
        {
        case 0:
            if(key_press != 0)  //key(s) down
            {
                key_prev = key_press;
                key_state = 1;
            }
            break;

        case 1:
            if(key_press == key_prev)   //same key down
            {
                key_state = 2;
                key_return = key_prev | KEY_DOWN;
            }
            else    //button left, fitter
            {
                key_state = 0;
            }
            break;

        case 2:
            if(key_press != key_prev)   //key up
            {
                key_state = 0;
                key_long_check = 0;
                key_return = key_prev | KEY_UP;
                return key_return;
            }
            else    //still down
            {
                key_long_check++;
                if(key_long_check >= (PRESS_LONG_TIME / DEBOUNCE_TIME))//long presss
                {
                    key_long_check = 0;
                    key_state = 3;
                    key_return = key_press | KEY_LONG;
                    return key_return;
                }
            }
            break;

        case 3:
            if(key_press != key_prev)
            {
                key_state = 0;
            }
            break;
        }
    }
    return NO_KEY;
}

/**
* @brief key call-back function
* detects the keys state and call the corresponding callback function
* @param [in] keys: key global structure pointer
* @return none
*/
void key_handle(keys_typedef_t *keys)
{
    rt_uint8_t i = 0;
    rt_uint16_t key_value = 0;

    key_value = read_key_value(keys);

    if(!key_value)  //no key down
        return;

    /*check short press button*/
    if(key_value & KEY_UP)
    {
        //valid key is detected
        for(i=0;i<keys->key_totol_number;i++)
        {
            if(G_IS_BIT_SET(key_value, keys->single_key[i].key_number))//this key is short press
            {
                //key callback function of short press
                if(keys->single_key[i].short_press)
                {
                    keys->single_key[i].short_press();
                }
            }
        }
    }

    //check long press button
    if(key_value & KEY_LONG)
    {
        //valid key is detected
        for(i=0;i<keys->key_totol_number;i++)
        {
            if(G_IS_BIT_SET(key_value, keys->single_key[i].key_number))
            {
                //key callback function of long press
                if(keys->single_key[i].long_press)
                    keys->single_key[i].long_press();
            }
        }
    }
}

/**
* @brief key init function
* @param [in] key_rccperiph APB2_peripheral
* @param [in] key_port peripheral_declaration
* @param [in] key_GPIO GPIO_Pins_define
* @param [in] short_press : short press state callback function address
* @param [in] long_press: long press state callback function address
* @return key structure pointer
*/
key_typedef_t key_init_one(rt_uint32_t key_rccperiph, GPIO_TypeDef *key_port, rt_uint32_t key_GPIO, keyfunction short_press, keyfunction long_press)
{
    static rt_int8_t key_total = -1;

    key_typedef_t single_key;

    //platform-defined GPIO
    single_key.key_rccperiph = key_rccperiph;
    single_key.key_port = key_port;
    single_key.key_GPIO = key_GPIO;
    single_key.key_number = ++key_total;

    //button trigger callback type
    single_key.short_press = short_press;
    single_key.long_press = long_press;

    key_totol_number++;
    return single_key;
}

/**
 * @brief key parameter init function
 * keys gpio init, start timer detect keys state
 * @param [in] keys : key global structure pointer
 * @return none
 */
void key_para_init(keys_typedef_t *keys)
{
    rt_uint8_t i = 0;

    if(NULL == keys)
    {
        return;
    }

    keys->key_totol_number = key_totol_number;

    //limit on the number keys (allowable number:0~12)
    if(KEY_MAX_NUMBER < keys->key_totol_number)
    {
        keys->key_totol_number = KEY_MAX_NUMBER;
    }

    for(i=0;i<keys->key_totol_number;i++)
    {
        GPIO_InitTypeDef    GPIO_InitStructure;
        RCC_APB2PeriphClockCmd(keys->single_key[i].key_rccperiph, ENABLE);

        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;

        GPIO_InitStructure.GPIO_Pin = keys->single_key[i].key_GPIO;
        GPIO_Init(keys->single_key[i].key_port, &GPIO_InitStructure);
    }
    //Init timer
    timer_control_init();
}



/* key define & init */
#include "relay.h"
#define GPIO_KEY_NUM    5
key_typedef_t single_key[GPIO_KEY_NUM];

extern rt_uint8_t rx_tmp_buf[32];       //current rx data
/**
* @brief key1 short press handle
* @param none
* @return none
*/
void key1_short_press(void)
{
    rt_kprintf("key1 short press!\r\n");
}
/**
* @brief key1 long press handle
* @param none
* @return none
*/
void key1_long_press(void)
{
    rt_kprintf("key1 long press!\r\n");
}

/**
* @brief key2 short press handle
* @param none
* @return none
*/
void key2_short_press(void)
{
    rt_kprintf("key2 short press!\r\n");
    rt_hw_led_off(0);
}
/**
* @brief key2 long press handle
* @param none
* @return none
*/
void key2_long_press(void)
{
    rt_kprintf("key2 long press!\r\n");
    rt_hw_led_on(0);
}

/**
* @brief key3 short press handle
* @param none
* @return none
*/
void key3_short_press(void)
{
    rt_kprintf("key3 short press, relay0 change state!\r\n");
    if(GPIO_ReadOutputDataBit(relay0_gpio, relay0_pin))	//当前开
    {
        relay_off(0);
        rx_tmp_buf[5] = 0;
    }
    else
    {
        relay_on(0);
        rx_tmp_buf[5] = 1;
    }
}
/**
* @brief key3 long press handle
* @param none
* @return none
*/
void key3_long_press(void)
{
    rt_kprintf("key3 long press!\r\n");
}

/**
* @brief key4 short press handle
* @param none
* @return none
*/
void key4_short_press(void)
{
    rt_kprintf("key4 short press, relay1 change state!\r\n");
    if(GPIO_ReadOutputDataBit(relay1_gpio, relay1_pin))	//当前开
    {
        relay_off(1);
        rx_tmp_buf[6] = 0;
    }
    else
    {
        relay_on(1);
        rx_tmp_buf[6] = 1;
    }
}
/**
* @brief key4 long press handle
* @param none
* @return none
*/
void key4_long_press(void)
{
    rt_kprintf("key4 long press!\r\n");
}

/**
* @brief key5 short press handle
* @param none
* @return none
*/
void key5_short_press(void)
{
    rt_kprintf("key5 short press, relay2 change state!\r\n");
	if(GPIO_ReadOutputDataBit(relay2_gpio, relay2_pin))	//当前开
    {
        relay_off(2);
    }
    else
    {
        relay_on(2);
    }
}
/**
* @brief key5 long press handle
* @param none
* @return none
*/
void key5_long_press(void)
{
    rt_kprintf("key5 long press!\r\n");
}

//key init
void key_init(void)
{
    single_key[0] = key_init_one(RCC_APB2Periph_GPIOA,GPIOA, GPIO_Pin_0, key1_short_press, key1_long_press);
    single_key[1] = key_init_one(RCC_APB2Periph_GPIOB,GPIOB, GPIO_Pin_10, key2_short_press, key2_long_press);
    single_key[2] = key_init_one(RCC_APB2Periph_GPIOB,GPIOB, GPIO_Pin_11, key3_short_press, key3_long_press);
    single_key[3] = key_init_one(RCC_APB2Periph_GPIOB,GPIOB, GPIO_Pin_1, key4_short_press, key4_long_press);
    single_key[4] = key_init_one(RCC_APB2Periph_GPIOB,GPIOB, GPIO_Pin_0, key5_short_press, key5_long_press);

    keys.single_key = (key_typedef_t *)&single_key;
    key_para_init(&keys);
}

