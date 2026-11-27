/**
 * @file    i2c.h
 * @brief   I2C 主机总线初始化（ESP32-S3，新 i2c_master 驱动）。
 *
 * 引脚分配（原舵机使用的 GPIO4/GPIO8 现已空闲，改作 I2C）：
 *   SDA -> GPIO4
 *   SCL -> GPIO8
 * 如需换线，只改下面的宏即可。
 */

#ifndef I2C_H
#define I2C_H

#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_MASTER_PORT       I2C_NUM_0
#define I2C_MASTER_SDA_GPIO   GPIO_NUM_4
#define I2C_MASTER_SCL_GPIO   GPIO_NUM_8
#define I2C_MASTER_FREQ_HZ    400000

/**
 * @brief 初始化 I2C 主机总线（幂等，重复调用直接返回 ESP_OK）。
 *        总线使用内部上拉；若 PCA9685 模块自带上拉电阻也可正常工作。
 */
esp_err_t i2c_bus_init(void);

/**
 * @brief 获取已初始化的总线句柄，未初始化时返回 NULL。
 */
i2c_master_bus_handle_t i2c_bus_get(void);

#ifdef __cplusplus
}
#endif

#endif /* I2C_H */
