#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "LED.h"
#include "finger_sensor.h"
#include "app_finger.h"
#include "app_print.h"


void app_main(void)
{
    printf("app start\n");
    LED_Init();

    /* 两个压力传感器：手指1 GPIO5，手指2 GPIO7 */
    esp_err_t ret = app_adc_start(5, 7);//启动 ADC 采样任务，GPIO5/7 对应 ADC1_CH4/6，并创建“最新值邮箱”队列和adc数据采集任务
    if (ret != ESP_OK) {
        printf("app_adc_start failed: %s\n", esp_err_to_name(ret));
    }

    /* 两个手指舵机：PCA9685 通道0/1（I2C: SDA=GPIO4，SCL=GPIO8）；串口命令 + 平滑定时器 */
    ret = app_finger_start();
    if (ret != ESP_OK) {
        printf("app_finger_start failed: %s\n", esp_err_to_name(ret));
    }

    /* 统一打印任务：手指状态 + 压力传感器 */
    ret = app_print_start();
    if (ret != ESP_OK) {
        printf("app_print_start failed: %s\n", esp_err_to_name(ret));
    }

    while (1) {
        LED_SetState(LED_ON);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        LED_SetState(LED_OFF);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
