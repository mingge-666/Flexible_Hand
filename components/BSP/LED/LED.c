#include "LED.h"



void LED_Init(void){
    gpio_config_t GPIO_InitStruct = {0};
    GPIO_InitStruct.intr_type = GPIO_INTR_DISABLE;
    GPIO_InitStruct.mode = GPIO_MODE_INPUT_OUTPUT;
    GPIO_InitStruct.pin_bit_mask = LED1_MASK;
    GPIO_InitStruct.pull_down_en = GPIO_PULLDOWN_ENABLE;
    GPIO_InitStruct.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&GPIO_InitStruct);
}

void LED_SetState(enum LED_State state){
    if(state == LED_ON){
        gpio_set_level(LED1_PIN,LED_ON);
    } else {
        gpio_set_level(LED1_PIN,LED_OFF);
    }
}

void LED_Toggle(void){
    uint8_t level = gpio_get_level(LED1_PIN);
    if(level == LED_ON){
        gpio_set_level(LED1_PIN,LED_OFF);
    } else {
        gpio_set_level(LED1_PIN,LED_ON);
    }
}
