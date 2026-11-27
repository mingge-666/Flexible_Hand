/**
 * @file    ADC.h
 * @brief   ADC oneshot 采样驱动（ESP32-S3）
 *          支持多通道：每个通道一个句柄，共享同一个 oneshot 单元。
 */

#ifndef ADC_READER_H
#define ADC_READER_H

#include "esp_err.h"
#include <stdbool.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 默认采集引脚 GPIO5（ADC1_CH4） */
#define ADC_READER_DEFAULT_GPIO 5

/* 一个 ADC 通道的句柄 */
typedef struct {
    int               gpio;
    adc_channel_t     channel;   /* ESP-IDF 枚举，如 ADC_CHANNEL_4 */
    adc_cali_handle_t cali;
    bool              cali_ok;
} adc_reader_channel_t;

/**
 * @brief 初始化一个 ADC 通道（多通道共享同一个 oneshot 单元，要求同一 ADC 单元）。
 *
 * @param ch       通道句柄
 * @param gpio_num GPIO；传 -1 用默认 GPIO5
 * @return ESP_OK 成功；其他为配置失败。
 */
esp_err_t adc_reader_init(adc_reader_channel_t *ch, int gpio_num);

/**
 * @brief 读取某通道电压（单位：V）。内部会做一次 ADC 转换。
 */
esp_err_t adc_reader_read_volt(adc_reader_channel_t *ch, float *volt);

/**
 * @brief 把已经读到的 raw 值换算成电压（单位：V），不再触发 ADC 转换。
 *        便于同一次采样同时得到 raw 与 volt。
 */
esp_err_t adc_reader_raw_to_volt(adc_reader_channel_t *ch, int raw, float *volt);


/**
 * @brief 读取某通道原始采样值
 */
esp_err_t adc_reader_read_raw(adc_reader_channel_t *ch, int *raw);

#ifdef __cplusplus
}
#endif

#endif /* ADC_READER_H */
