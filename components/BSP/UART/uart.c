#include "uart.h"          /* 你的头文件 */
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "driver/gpio.h"

#define TAG "UART_CMD"

static bool s_uart_inited = false;

/**
 * @brief 初始化 UART0（GPIO43 TX / GPIO44 RX），用于串口命令与日志输出。
 */
esp_err_t uart_cmd_init(void)
{
    if (s_uart_inited) {
        return ESP_OK;
    }

    /* 1. UART 参数配置 */
    uart_config_t uart_cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* 2. 安装驱动：RX 缓冲区 1024，TX 缓冲区 0（阻塞发送） */
    esp_err_t ret = uart_driver_install(USART_UX, RX_BUF_SIZE, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(USART_UX, &uart_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        uart_driver_delete(USART_UX);
        return ret;
    }

    /* 3. 绑定引脚（S3 上 43/44 是默认脚，但显式设置更稳） */
    ret = uart_set_pin(USART_UX,
                       USART_TX_GPIO_PIN,
                       USART_RX_GPIO_PIN,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(ret));
        uart_driver_delete(USART_UX);
        return ret;
    }

    /* 让 printf/ESP_LOG 与命令读取走同一个 UART 驱动，避免两套路径混用 */
    uart_vfs_dev_use_driver(USART_UX);

    s_uart_inited = true;
    ESP_LOGI(TAG, "uart0 init ok, tx=%d rx=%d baud=115200",
             USART_TX_GPIO_PIN, USART_RX_GPIO_PIN);
    return ESP_OK;
}

/**
 * @brief 反初始化
 */
esp_err_t uart_cmd_deinit(void)
{
    if (!s_uart_inited) {
        return ESP_OK;
    }
    uart_driver_delete(USART_UX);
    s_uart_inited = false;
    return ESP_OK;
}
