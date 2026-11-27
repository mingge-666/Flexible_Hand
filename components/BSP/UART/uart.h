#ifndef __UART_H_
#define __UART_H_

#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdbool.h>


#define USART_UX UART_NUM_0
#define USART_TX_GPIO_PIN GPIO_NUM_43
#define USART_RX_GPIO_PIN GPIO_NUM_44

#define RX_BUF_SIZE 1024

esp_err_t uart_cmd_init(void);
esp_err_t uart_cmd_deinit(void);


#endif

