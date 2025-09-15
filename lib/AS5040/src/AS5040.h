/**
 * @file AS5040.h
 * @brief Interface definitions for the ESP_IDF compatible AS5040 driver.
 */

#ifndef AS5040_H
#define AS5040_H

#include "esp_rom_sys.h"
#include "esp_err.h"
#include "driver/gpio.h"

/**
 * @brief Struct representing the current state of device.
 */
typedef struct
{
    gpio_num_t pin_clk;
    gpio_num_t pin_data;
    gpio_num_t pin_cs;
    float offset;
}AS5040_info;

/**
 * @brief Initialize the AS5040 device.
 */
esp_err_t AS5040_init(AS5040_info *info);

/**
 * @brief Read the current angle determined by the AS5040 sensor.
 */
esp_err_t AS5040_read(AS5040_info *info, float *position);

#endif