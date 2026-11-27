/**
 * @file    Servo.h
 * @brief   位置舵机驱动（PCA9685，I2C，50Hz，0.5–2.5ms 对应 0–180°）。
 *
 * 每个舵机接 PCA9685 的一个 PWM 通道（0–15），共用 I2C 总线。
 */

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 脉宽范围（微秒）：0° -> 500us，180° -> 2500us */
#define SERVO_MIN_PULSE_US   500
#define SERVO_MAX_PULSE_US   2500
/* 角度范围 */
#define SERVO_MAX_ANGLE_DEG  180.0f

/**
 * @brief 初始化一个舵机通道。
 *        首次调用会初始化 I2C 总线与 PCA9685（默认地址 0x40），
 *        之后把该通道置于 90°（居中）。
 *
 * @param channel PCA9685 的 PWM 通道号（0–15）
 * @return ESP_OK 成功；其他为 I2C/PCA9685 配置失败。
 */
esp_err_t servo_init(int channel);

/**
 * @brief 设置某通道的绝对角度（0–180°），超范围自动钳制。
 */
esp_err_t servo_set_angle(int channel, float angle_deg);

/**
 * @brief 直接设置某通道的高电平脉宽（微秒），用于标定舵机行程。
 */
esp_err_t servo_set_pulse_us(int channel, uint16_t pulse_us);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */
