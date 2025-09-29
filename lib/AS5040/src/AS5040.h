/**
 * Copyright 2025 Konrad Siudziński
 */

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
 * @struct AS5040_info
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
 * @param[in] info Pointer to a structure holding informations about AS5040 sensor.
 * @return Returns ESP_OK if initialization is succesfull.
 */
esp_err_t AS5040_init(AS5040_info *info);

/**
 * @brief Read the current angle determined by the AS5040 sensor.
 * @param[in] info Pointer to a structure holding informations about AS5040 sensor.
 * @param[out] position Pointer to a value that will hold the result of the reading operation.
 * @return Returns ESP_OK if reading is succesfull.
 */
esp_err_t AS5040_read(AS5040_info *info, float *position);

/**
 * @brief Read the current angle determined by the AS5040 sensor.
 * @param[in] info Pointer to a structure holding informations about AS5040 sensor.
 * @param[out] position Pointer to a value that will hold the result of the reading operation.
 * @param[out] devinfo Pointer to an at least 6 long table of bools that will store additional info from the sensor. 
 * @return Returns ESP_OK if reading is succesfull.
 */
esp_err_t AS5040_read_all(AS5040_info *info, float *position, bool devinfo[]);

/**
 * @brief Deinitializes the AS5040 device.
 * @param[in] info Pointer to a structure holding informations about AS5040 sensor.
 * @return Always returns ESP_OK.
 */
esp_err_t AS5040_deinit(AS5040_info *info);

#endif