#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#define IR_SENSOR_PIN GPIO_NUM_6
#define UART_PORT     UART_NUM_0

void init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void uart_print(const char *str) {
    uart_write_bytes(UART_PORT, str, strlen(str));
}

void init_ir_sensor(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IR_SENSOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void app_main(void) {
    init_uart();
    init_ir_sensor();

    uart_print("\n========================================\n");
    uart_print("  ESP32-S3 IR Line Sensor Test Started  \n");
    uart_print("========================================\n");

    char buffer[128];

    while (1) {
        int sensor_state = gpio_get_level(IR_SENSOR_PIN);

        // Standard IR Sensor Logic:
        // HIGH (1) = Dark / Black Line (IR absorbed, low reflection)
        // LOW (0)  = Bright / White Surface (IR reflected back)
        if (sensor_state == 1) {
            snprintf(buffer, sizeof(buffer), "[STATE: 1] -> BLACK LINE DETECTED\n");
        } else {
            snprintf(buffer, sizeof(buffer), "[STATE: 0] -> WHITE SURFACE (No Line)\n");
        }

        uart_print(buffer);
        vTaskDelay(pdMS_TO_TICKS(400));
    }
}