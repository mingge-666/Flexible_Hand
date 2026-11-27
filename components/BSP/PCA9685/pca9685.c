/**
 * @file    pca9685.c
 * @brief   PCA9685 16 通道 12 位 PWM 驱动（I2C 接口）。
 *
 * 频率计算：
 *   prescale = round(osc_clk / (4096 * freq)) - 1
 *   50Hz -> round(25MHz / (4096 * 50)) - 1 = 121
 *
 * 脉宽换算（50Hz，周期 20ms）：
 *   count = pulse_us / 20000 * 4096 = pulse_us * 4096 / 20000
 *   500us -> 102，1500us -> 307，2500us -> 512
 */

#include "pca9685.h"

#include <stdbool.h>
#include <stddef.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"

static const char *TAG = "PCA9685";

/* ---------- 寄存器与位定义 ---------- */
#define PCA9685_REG_MODE1        0x00
#define PCA9685_REG_MODE2        0x01
#define PCA9685_REG_LED0_ON_L    0x06
#define PCA9685_REG_PRESCALE     0xFE

#define PCA9685_MODE1_RESTART    0x80
#define PCA9685_MODE1_AI         0x20    /* 寄存器地址自动递增 */
#define PCA9685_MODE1_SLEEP      0x10
#define PCA9685_MODE1_ALLCALL    0x01
#define PCA9685_MODE2_OUTDRV     0x04    /* 推挽输出 */

#define PCA9685_I2C_TIMEOUT_MS   50

static i2c_master_dev_handle_t s_pca9685_dev = NULL;

/* ---------- 底层寄存器读写 ---------- */

static esp_err_t write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_pca9685_dev, buf, sizeof(buf), PCA9685_I2C_TIMEOUT_MS);
}

static esp_err_t read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_pca9685_dev, &reg, 1, val, 1,
                                       PCA9685_I2C_TIMEOUT_MS);
}

/* ---------- 公开 API ---------- */

esp_err_t pca9685_init(i2c_master_bus_handle_t bus, uint8_t addr)
{
    if (s_pca9685_dev != NULL) {
        return ESP_OK;
    }
    ESP_RETURN_ON_FALSE(bus != NULL, ESP_ERR_INVALID_ARG, TAG, "bus is NULL");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &s_pca9685_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "add device 0x%02X failed: %s", addr, esp_err_to_name(ret));
        return ret;
    }

    float   prescale_f;
    uint8_t prescale;

    /* 读一次 MODE1，确认芯片在线 */
    uint8_t mode1 = 0;
    ret = read_reg(PCA9685_REG_MODE1, &mode1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 not found at 0x%02X: %s", addr, esp_err_to_name(ret));
        goto fail;
    }

    /* 进入 SLEEP 后才能写预分频 */
    ret = write_reg(PCA9685_REG_MODE1, PCA9685_MODE1_SLEEP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "enter sleep failed: %s", esp_err_to_name(ret));
        goto fail;
    }
    ret = write_reg(PCA9685_REG_MODE2, PCA9685_MODE2_OUTDRV);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "set MODE2 failed: %s", esp_err_to_name(ret));
        goto fail;
    }

    prescale_f = (PCA9685_OSC_CLK_HZ /
                  (4096.0f * (float)PCA9685_PWM_FREQ_HZ)) - 1.0f + 0.5f;
    prescale = (uint8_t)prescale_f;
    ret = write_reg(PCA9685_REG_PRESCALE, prescale);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "write prescale failed: %s", esp_err_to_name(ret));
        goto fail;
    }

    /* 唤醒 + 自动递增 + 响应广播地址 */
    ret = write_reg(PCA9685_REG_MODE1, PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "wake up failed: %s", esp_err_to_name(ret));
        goto fail;
    }
    vTaskDelay(pdMS_TO_TICKS(1));   /* 等内部振荡器稳定（>500us） */

    ret = write_reg(PCA9685_REG_MODE1,
                    PCA9685_MODE1_RESTART |
                    PCA9685_MODE1_AI |
                    PCA9685_MODE1_ALLCALL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "restart failed: %s", esp_err_to_name(ret));
        goto fail;
    }

    /* 所有通道先输出 0（无脉冲），由上层 servo_init 各自居中 */
    for (uint8_t ch = 0; ch < PCA9685_CHANNEL_COUNT; ++ch) {
        ret = pca9685_set_pwm(ch, 0, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "clear channel %d failed: %s", ch, esp_err_to_name(ret));
            goto fail;
        }
    }

    ESP_LOGI(TAG, "init ok: addr=0x%02X prescale=%d (%dHz)",
             addr, prescale, PCA9685_PWM_FREQ_HZ);
    return ESP_OK;

fail:
    /* 初始化失败：释放设备句柄，允许上层重试 */
    i2c_master_bus_rm_device(s_pca9685_dev);
    s_pca9685_dev = NULL;
    return ret;
}

esp_err_t pca9685_set_pwm(uint8_t channel, uint16_t on_count, uint16_t off_count)//4095只有12位
{
    ESP_RETURN_ON_FALSE(s_pca9685_dev != NULL, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(channel < PCA9685_CHANNEL_COUNT, ESP_ERR_INVALID_ARG, TAG,
                        "channel %d out of range", channel);
    ESP_RETURN_ON_FALSE(on_count <= PCA9685_COUNTS_MAX && off_count <= PCA9685_COUNTS_MAX,
                        ESP_ERR_INVALID_ARG, TAG,
                        "count out of range (on=%d off=%d)", on_count, off_count);

    uint8_t buf[5] = {
        (uint8_t)(PCA9685_REG_LED0_ON_L + 4 * channel),
        (uint8_t)(on_count & 0xFF),          //低8位
        (uint8_t)((on_count >> 8) & 0x0F),   //高4位（bit12 是 FULL_ON，必须屏蔽）
        (uint8_t)(off_count & 0xFF),         //低8位
        (uint8_t)((off_count >> 8) & 0x0F),  //高4位（bit12 是 FULL_OFF，必须屏蔽）
    };
    //通过i2c总线发送数据，buf是要发送的数据，sizeof(buf)是数据长度，PCA9685_I2C_TIMEOUT_MS是超时时间
    return i2c_master_transmit(s_pca9685_dev, buf, sizeof(buf), PCA9685_I2C_TIMEOUT_MS);
}

esp_err_t pca9685_set_pulse_us(uint8_t channel, uint16_t pulse_us)
{
    const float period_us = 1000000.0f / (float)PCA9685_PWM_FREQ_HZ;

    if (pulse_us == 0) {
        return pca9685_disable_channel(channel);
    }

    float count_f = (float)pulse_us * 4096.0f / period_us;
    if (count_f > (float)PCA9685_COUNTS_MAX) {
        count_f = (float)PCA9685_COUNTS_MAX;
    }
    uint16_t count = (uint16_t)(count_f + 0.5f);

    /* ON 固定从 0 开始，OFF 决定高电平结束位置 */
    return pca9685_set_pwm(channel, 0, count);
}

esp_err_t pca9685_disable_channel(uint8_t channel)
{
    return pca9685_set_pwm(channel, 0, 0);
}
