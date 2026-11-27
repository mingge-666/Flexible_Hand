#ifndef __LED_H__
#define __LED_H__

#include "driver/gpio.h"

#define LED1_PIN GPIO_NUM_2
#define LED1_MASK (1ULL << LED1_PIN) 

enum LED_State{
    LED_OFF = 0,
    LED_ON
};

void LED_Init(void);
void LED_SetState(enum LED_State state);
void LED_Toggle(void);

#endif
