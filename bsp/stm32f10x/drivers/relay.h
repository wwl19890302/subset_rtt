#ifndef RELAY_H
#define RELAY_H

#include "board.h"
#include "rtthread.h"

#define relay0  PBout(5)
#define relay1  PBout(4)
#define relay2  PBout(3)

void relay_init(void);
void relay_on(rt_int8_t index);
void relay_off(rt_uint8_t index);

#endif // RELAY_H
