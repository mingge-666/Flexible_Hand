/**
 * @file    ADC.c
 * @brief   ADC oneshot 采样驱动（ESP32-S3）
 *          多通道共享一个 oneshot 单元，每个通道独立校准与读取，电压单位 V。
 */

#include "ADC.h"

#include <stdbool.h>
#include <stddef.h>

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "ADC_READER";

#define ADC_ATTEN     ADC_ATTEN_DB_12
#define ADC_BITWIDTH  ADC_BITWIDTH_DEFAULT

static adc_oneshot_unit_handle_t s_adc_unit = NULL;
static adc_unit_t                s_adc_unit_id;

esp_err_t adc_reader_init(adc_reader_channel_t *ch, int gpio_num)
{
    if (ch == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const int gpio = (gpio_num < 0) ? ADC_READER_DEFAULT_GPIO : gpio_num;

    /* GPIO 映射到 ADC 单元与通道，GPIO5 对应 ADC1_CH4，GPIO7 对应 ADC1_CH6 */
    adc_unit_t adc_unit;
    adc_channel_t adc_channel;
    esp_err_t ret = adc_oneshot_io_to_channel(gpio, &adc_unit, &adc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d is not ADC-capable: %s", gpio, esp_err_to_name(ret));
        return ret;
    }

    /* 第一个通道创建单元，后续通道复用（暂不支持跨 ADC 单元） */
    if (s_adc_unit == NULL) {
        adc_oneshot_unit_init_cfg_t unit_cfg = {
            .unit_id  = adc_unit,
            .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ret = adc_oneshot_new_unit(&unit_cfg, &s_adc_unit);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(ret));
            return ret;
        }
        s_adc_unit_id = adc_unit;
    } else if (s_adc_unit_id != adc_unit) {
        ESP_LOGE(TAG, "multi ADC unit not supported (gpio=%d)", gpio);
        return ESP_ERR_NOT_SUPPORTED;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    ret = adc_oneshot_config_channel(s_adc_unit, adc_channel, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ch->gpio    = gpio;
    ch->channel = adc_channel;

    /* 重复初始化（重试）时复用已创建的校准句柄，避免泄漏 */
    if (!(ch->cali_ok && ch->cali != NULL)) {
        ch->cali    = NULL;
        ch->cali_ok = false;

        /* 校准失败不致命：eFuse 未烧录时回退到 raw 粗略换算 */
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id  = adc_unit,
            .chan     = adc_channel,
            .atten    = ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &ch->cali);
        if (ret == ESP_OK) {
            ch->cali_ok = true;
        } else {
            ch->cali = NULL;
            ESP_LOGW(TAG, "gpio%d calibration unavailable (%s), fallback raw",
                     gpio, esp_err_to_name(ret));
        }
    }

    ESP_LOGI(TAG, "init ok: gpio=%d adc%d ch=%d cali=%s",
             gpio, (int)adc_unit + 1, (int)adc_channel, ch->cali_ok ? "yes" : "no");
    return ESP_OK;
}

esp_err_t adc_reader_raw_to_volt(adc_reader_channel_t *ch, int raw, float *volt)
{
    if (ch == NULL || volt == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (ch->cali_ok && ch->cali != NULL) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(ch->cali, raw, &mv) == ESP_OK) {
            *volt = (float)mv / 1000.0f;
            return ESP_OK;
        }
        /* 校准换算失败也保留数据，落到下面的粗略换算 */
    }

    *volt = (float)raw * 3.1f / 4095.0f;   /* 无校准粗略换算 */
    return ESP_OK;
}

esp_err_t adc_reader_read_volt(adc_reader_channel_t *ch, float *volt)
{
    if (ch == NULL || volt == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_adc_unit == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw = 0;
    esp_err_t ret = adc_oneshot_read(s_adc_unit, ch->channel, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    return adc_reader_raw_to_volt(ch, raw, volt);
}


esp_err_t adc_reader_read_raw(adc_reader_channel_t *ch, int *raw)
{
    if (ch == NULL || raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_adc_unit == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw_value = 0;
    esp_err_t ret = adc_oneshot_read(s_adc_unit, ch->channel, &raw_value);
    if (ret != ESP_OK) {
        return ret;
    }

    *raw = raw_value;
    return ESP_OK;
}
