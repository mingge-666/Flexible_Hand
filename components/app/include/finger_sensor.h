#ifndef FINGER_SENSOR_H
#define FINGER_SENSOR_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 压力传感器编号（与手指对应） */
typedef enum {
    SENSOR_1 = 0,   /* 手指1压力：GPIO5 */
    SENSOR_2 = 1,   /* 手指2压力：GPIO7 */
    SENSOR_COUNT = 2
} sensor_id_t;

/* ADC 采样结果（供统一打印任务读取） */
typedef struct {
    int      raw;    /* 原始采样值 */
    float    volt;   /* 电压值（V） */
    uint32_t seq;    /* 采样批次号 */
} app_adc_data_t;

/**
 * @brief 启动双通道 ADC 采样任务（按 FINGER_ADC_SAMPLE_PERIOD_MS 采样，
 *        FINGER_ADC_WINDOW_SIZE 点去极值平均）。
 *
 * @param gpio1 手指1压力传感器 GPIO
 * @param gpio2 手指2压力传感器 GPIO
 * @return ESP_OK 成功；其他为 ADC 初始化或任务创建失败。
 */
esp_err_t app_adc_start(int gpio1, int gpio2);

/**
 * @brief 读取某传感器最近一次滤波后的结果（非阻塞）。
 * @return true 表示有数据；false 表示尚无数据或未初始化。
 */
bool app_adc_get_latest(sensor_id_t sensor, app_adc_data_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_ADC_H */
