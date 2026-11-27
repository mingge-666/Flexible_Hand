/**
 * @file    Servo.c
 * @brief   位置舵机驱动（PCA9685，I2C，50Hz）。
 *
 * 角度换算：
 *   角度 0–180° 映射脉宽 500–2500us，
 *   PCA9685 内部再按 50Hz 把脉宽换算成 12 位 PWM 计数。
 */

#include "Servo.h"

#include <stdbool.h>

#include "esp_log.h"
#include "i2c.h"
#include "pca9685.h"

static const char *TAG = "SERVO";

static bool s_pca_inited = false;

static uint16_t angle_to_pulse_us(float angle_deg)
{
    float pulse_us = SERVO_MIN_PULSE_US +
                     (angle_deg / SERVO_MAX_ANGLE_DEG) *
                     (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    return (uint16_t)(pulse_us + 0.5f);
}
/*通过I2C初始化PCA9685，并将指定通道的舵机居中。而且是使用pca9685_set_pulse_us函数设置脉宽的
，pca9685_set_pulse_us函数会调用pca9685_set_pwm函数，pca9685_set_pwm函数
会调用i2c_master_transmit函数发送数据到PCA9685芯片,让该芯片来输出pwm控制*/
esp_err_t servo_init(int channel)
{
    if (!s_pca_inited) {
        esp_err_t ret = i2c_bus_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "i2c_bus_init failed: %s", esp_err_to_name(ret));
            return ret;
        }

        ret = pca9685_init(i2c_bus_get(), PCA9685_I2C_ADDR_DEFAULT);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "pca9685_init failed: %s", esp_err_to_name(ret));
            return ret;
        }
        s_pca_inited = true;
    }

    esp_err_t ret = servo_set_angle(channel, 90.0f);   /* 上电默认居中 */
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "init ok: pca9685 channel=%d, %dHz", channel, PCA9685_PWM_FREQ_HZ);
    } else {
        ESP_LOGE(TAG, "servo_set_angle failed (ch=%d): %s", channel, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t servo_set_angle(int channel, float angle_deg)//已经在这个函数设置了限幅保护。不用担心写入了一个错误的角度
{
    if (angle_deg < 0.0f)                   angle_deg = 0.0f;
    if (angle_deg > SERVO_MAX_ANGLE_DEG)    angle_deg = SERVO_MAX_ANGLE_DEG;

    return servo_set_pulse_us(channel, angle_to_pulse_us(angle_deg));
}

esp_err_t servo_set_pulse_us(int channel, uint16_t pulse_us)
{
    if (channel < 0 || channel >= PCA9685_CHANNEL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    return pca9685_set_pulse_us((uint8_t)channel, pulse_us);
}
