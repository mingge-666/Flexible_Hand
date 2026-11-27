#ifndef FINGER_STATE_H
#define FINGER_STATE_H

#include "app_finger.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "finger_config.h"

/**
 * @brief 手指平滑运动一步：按状态机推进（接近/抓取）。
 * @param f   手指状态指针
 * @param idx 手指下标（0=FINGER_1, 1=FINGER_2），用于读取对应压力传感器
 * @param dt  本步时间间隔（秒）
 */
void finger_update(finger_t *f, int idx, float dt);



#endif