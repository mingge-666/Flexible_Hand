/**
 * @file    i2c.c
 * @brief   I2C 主机总线初始化（ESP32-S3，新 i2c_master 驱动）。
 */

#include "i2c.h"

#include "esp_log.h"

static const char *TAG = "I2C_BUS";

static i2c_master_bus_handle_t s_bus = NULL;
static bool s_inited = false;

esp_err_t i2c_bus_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_MASTER_PORT,
        .sda_io_num        = I2C_MASTER_SDA_GPIO,
        .scl_io_num        = I2C_MASTER_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority     = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_inited = true;
    ESP_LOGI(TAG, "init ok: sda=%d scl=%d %dHz",
             I2C_MASTER_SDA_GPIO, I2C_MASTER_SCL_GPIO, I2C_MASTER_FREQ_HZ);
    return ESP_OK;
}

i2c_master_bus_handle_t i2c_bus_get(void)
{
    return s_bus;
}
