#ifndef FINGER_FORCE_H
#define FINGER_FORCE_H

#include "app_finger.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "finger_config.h"
#include "PID.h"

void finger_pid_init_all(void);
float finger_read_force(int idx);
void finger_pid_reset(pid_t *pid);
pid_t *finger_pid_get(int idx);

/**
 * @brief 该手指下标是否配有力传感器（idx 为 0 基数组下标）。
 */
bool finger_has_sensor(int idx);

/* 压力换算（标定参数见 finger_config.h） */
float voltage_to_force_v(float v);
float raw_to_force(int raw);

#endif
