/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_lcd_touch_cst816s.h"
#include "driver/i2c.h"

esp_lcd_touch_handle_t tp;
static SemaphoreHandle_t touch_mux;

static void touch_callback(esp_lcd_touch_handle_t tp)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(touch_mux, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void i2c_init(void)
{
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 6,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 7,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    i2c_param_config(0, &i2c_conf);
    i2c_driver_install(0, i2c_conf.mode, 0, 0, 0);
}

static void example_touch_task(void *arg)
{
    printf("Starting touch task\n");
    uint32_t task_delay_ms = 500;
    while (1) {
        // Read data from the touch controller and store it in RAM memory. It should be called regularly in poll.
        if (xSemaphoreTake(touch_mux, 0) == pdTRUE) {
            esp_lcd_touch_read_data(tp); // read only when ISR was triggled

            // Get one X and Y coordinates with strength of touch.

            uint16_t touch_x[1];
            uint16_t touch_y[1];
            uint16_t touch_strength[1];
            uint8_t touch_cnt = 0;
            bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp, touch_x, touch_y, touch_strength, &touch_cnt, 1);
            printf("touch pressed = %d , x = %u , y=%u\n", touchpad_pressed, touch_x[0], touch_y[0]);
        }        
        
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
    // start cst init
    touch_mux = xSemaphoreCreateBinary();

    i2c_init();
    
    esp_lcd_panel_io_i2c_config_t io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = 240,
        .y_max = 240,
        .rst_gpio_num = 13,
        .int_gpio_num = 5,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .interrupt_callback = touch_callback,
    };

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)0 , &io_config, &io_handle);
    esp_lcd_touch_new_i2c_cst816s(io_handle, &tp_cfg, &tp);

    xTaskCreate(example_touch_task, "Touch", 4 * 1024, NULL, 3, NULL);
}
