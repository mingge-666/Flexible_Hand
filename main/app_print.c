/**
 * @file    app_print.c
 * @brief   统一串口打印任务：每 FINGER_PRINT_PERIOD_MS 打印手指状态与压力传感器数据。
 *          所有周期打印信息集中于此任务。
 */

#include "app_print.h"

#include <stdbool.h>
#include <stdio.h>

#include "finger_force.h"
#include "finger_sensor.h"
#include "app_finger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PRINT_PERIOD_MS   FINGER_PRINT_PERIOD_MS
#define PRINT_TASK_PRIO   4
#define PRINT_TASK_STACK  4096

static bool s_started = false;

static const char *finger_state_name(finger_state_t s)
{
    switch (s) {
    case FINGER_STATE_IDLE:       return "IDLE";
    case FINGER_APPROACHING:      return "APPROACHING";
    case FINGER_APPROACHED:       return "APPROACHED";
    case FINGER_CATCHING:         return "CATCHING";
    case FINGER_CATCHED:          return "CATCHED";
    case FINGER_MOVING:           return "MOVING";
    default:                      return "?";
    }
}

static void print_all(void){
    for(int i = 0; i < FINGER_COUNT; ++i){
        app_adc_data_t data = {0};
        bool have = app_adc_get_latest((sensor_id_t)i, &data);

        finger_t f;
        if(!app_finger_get_snapshot((finger_id_t)(i + 1), &f)) {
            printf("finger %d state unknown\n", i + 1);
            continue;
        }

        printf("F%d %-11s tgt %.1f cur %.1f spd %.0f f %.1f/%.1f\n",
               i + 1, finger_state_name(f.state), f.target_deg, f.cur_deg, f.cur_speed,
               f.cur_force, f.target_force);

        if (i < SENSOR_COUNT) {
            if (have) {
                printf("P%d raw:%d  %.3fV force %.1f\n",
                       i + 1, data.raw, data.volt, raw_to_force(data.raw));
            } else {
                printf("P%d no data\n", i + 1);
            }
        }
    }
    fflush(stdout);
}

static void print_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PRINT_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&last_wake, period);
        print_all();
    }
}

esp_err_t app_print_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(print_task, "print",
                                            PRINT_TASK_STACK, NULL,
                                            PRINT_TASK_PRIO, NULL, 0);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    return ESP_OK;
}
