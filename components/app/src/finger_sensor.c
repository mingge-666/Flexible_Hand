/**
 * @file    finger_sensor.c
 * @brief   双通道压力传感器采样任务：按 FINGER_ADC_SAMPLE_PERIOD_MS 各采 1 点，
 *          攒 FINGER_ADC_WINDOW_SIZE 点去极值平均后写入“最新值邮箱”。
 *          周期打印已收敛到 app_print.c。
 */

#include "finger_sensor.h"

#include <stdbool.h>
#include <stdint.h>

#include "sdkconfig.h"
#include "ADC.h"
#include "AverageFilter.h"
#include "finger_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define ADC_SAMPLE_PERIOD_MS  FINGER_ADC_SAMPLE_PERIOD_MS
#define ADC_WINDOW_SIZE       FINGER_ADC_WINDOW_SIZE
#define ADC_QUEUE_DEPTH       1

#define ADC_SAMPLE_TASK_PRIO  6
#define ADC_TASK_STACK_BYTES  4096

/* 采样周期要求 tick 至少 1000Hz，否则 pdMS_TO_TICKS() 会变成 0 */
#if CONFIG_FREERTOS_HZ < 1000
#error "ADC sampling requires CONFIG_FREERTOS_HZ >= 1000"
#endif

static const char *TAG = "APP_ADC";
static adc_reader_channel_t s_channels[SENSOR_COUNT];
static QueueHandle_t s_queue[SENSOR_COUNT];
static bool s_started = false;

/* 启动失败时释放已经创建的队列，允许重试 */
static void adc_cleanup(void)
{
    for (int i = 0; i < SENSOR_COUNT; ++i) {
        if (s_queue[i] != NULL) {
            vQueueDelete(s_queue[i]);
            s_queue[i] = NULL;
        }
    }
}

/**
 * @brief 采样任务：每 FINGER_ADC_SAMPLE_PERIOD_MS 对每个通道各采 1 点，
 *        FINGER_ADC_WINDOW_SIZE 点一批做去极值平均后入队。
 */
static void adc_sample_task(void *arg)
{
    (void)arg;

    float window_raw[SENSOR_COUNT][ADC_WINDOW_SIZE];
    float window_volt[SENSOR_COUNT][ADC_WINDOW_SIZE];
    size_t idx[SENSOR_COUNT] = {0};
    uint32_t seq[SENSOR_COUNT] = {0};
    uint32_t err_count = 0;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(ADC_SAMPLE_PERIOD_MS);

    for (;;) {
        for (int i = 0; i < SENSOR_COUNT; ++i) {
            int raw = 0;
            float volt = 0.0f;

            /* 每次只做一次 ADC 转换，raw 和 volt 来自同一次采样 */
            esp_err_t ret = adc_reader_read_raw(&s_channels[i], &raw);

            if (ret == ESP_OK) {
                adc_reader_raw_to_volt(&s_channels[i], raw, &volt);
                window_raw[i][idx[i]] = raw;
                window_volt[i][idx[i]] = volt;
                idx[i]++;
                if (idx[i] == ADC_WINDOW_SIZE) {
                    app_adc_data_t msg = {
                        .raw = average_filter_trimmed_mean(window_raw[i], ADC_WINDOW_SIZE),
                        .volt = average_filter_trimmed_mean(window_volt[i], ADC_WINDOW_SIZE),
                        .seq  = ++seq[i],
                    };
                    /* 深度 1 邮箱：旧值被覆盖，生产者永不阻塞 */
                    xQueueOverwrite(s_queue[i], &msg);
                    idx[i] = 0;
                }
            } else {
                /* 限频，避免 5ms 一次的日志把串口打满、反过来拖慢节拍 */
                if ((++err_count % 200U) == 1U) {
                    ESP_LOGW(TAG, "sensor %d read failed: %s (total=%lu)",
                             i, esp_err_to_name(ret), (unsigned long)err_count);
                }
            }
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

bool app_adc_get_latest(sensor_id_t sensor, app_adc_data_t *out)//获取指定下标的adc数据，通过out传出
{
    if (out == NULL) {
        return false;
    }

    int idx = (int)sensor;
    if (idx < 0 || idx >= SENSOR_COUNT || s_queue[idx] == NULL) {
        return false;
    }
    return xQueuePeek(s_queue[idx], out, 0) == pdTRUE;
}

esp_err_t app_adc_start(int gpio1, int gpio2)
{
    if (s_started) {
        return ESP_OK;
    }

    int gpios[SENSOR_COUNT] = { gpio1, gpio2 };
    for (int i = 0; i < SENSOR_COUNT; ++i) {//对全部压力传感器进行初始化
        esp_err_t ret = adc_reader_init(&s_channels[i], gpios[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "sensor %d init failed: %s", i, esp_err_to_name(ret));
            adc_cleanup();
            return ret;
        }

        s_queue[i] = xQueueCreate(ADC_QUEUE_DEPTH, sizeof(app_adc_data_t));
        if (s_queue[i] == NULL) {
            ESP_LOGE(TAG, "xQueueCreate failed (sensor %d)", i);
            adc_cleanup();
            return ESP_ERR_NO_MEM;
        }
    }

    TaskHandle_t sample_handle = NULL;
    BaseType_t ok = xTaskCreatePinnedToCore(adc_sample_task, "adc_sample",
                                            ADC_TASK_STACK_BYTES, NULL,
                                            ADC_SAMPLE_TASK_PRIO, &sample_handle, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "create adc_sample task failed");
        adc_cleanup();
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    return ESP_OK;
}
