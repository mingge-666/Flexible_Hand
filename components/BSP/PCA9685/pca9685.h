/**
 * @file    pca9685.h
 * @brief   PCA9685 16 通道 12 位 PWM 驱动（I2C 接口，50Hz 舵机输出）。
 *
 * 使用方式：
 *   i2c_bus_init();
 *   pca9685_init(i2c_bus_get(), PCA9685_I2C_ADDR_DEFAULT);
 *   pca9685_set_pulse_us(ch, 1500);   // 通道 0-15，脉宽 500-2500us
 */

#ifndef PCA9685_H
#define PCA9685_H

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PCA9685_I2C_ADDR_DEFAULT   0x40
#define PCA9685_CHANNEL_COUNT      16
#define PCA9685_PWM_FREQ_HZ        50      /* 舵机标准 50Hz */
#define PCA9685_OSC_CLK_HZ         25000000.0f

/* 内部脉宽计数上限（12 位） */
#define PCA9685_COUNTS_MAX         4095

/**
 * @brief 在指定 I2C 总线上挂载 PCA9685，配置为 50Hz 输出并关闭所有通道。
 *        幂等：重复调用直接返回 ESP_OK。
 *
 * @param bus 已初始化的 I2C 主机总线句柄
 * @param addr 7 位从机地址（默认 0x40，A0-A5 全接地）
 */
esp_err_t pca9685_init(i2c_master_bus_handle_t bus, uint8_t addr);

/**
 * @brief 直接设置某通道的 ON/OFF 计数（0-4095，4096 由驱动内部不使用）。
 */
esp_err_t pca9685_set_pwm(uint8_t channel, uint16_t on_count, uint16_t off_count);

/**
 * @brief 设置某通道的高电平脉宽（微秒），按当前 50Hz 换算为 12 位计数。
 */
esp_err_t pca9685_set_pulse_us(uint8_t channel, uint16_t pulse_us);

/**
 * @brief 让某通道停止输出（高电平时间 0）。
 */
esp_err_t pca9685_disable_channel(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* PCA9685_H */
