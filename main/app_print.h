#ifndef APP_PRINT_H
#define APP_PRINT_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动统一打印任务：按 FINGER_PRINT_PERIOD_MS 周期打印 ADC 数据与舵机状态。
 */
esp_err_t app_print_start(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_PRINT_H */
