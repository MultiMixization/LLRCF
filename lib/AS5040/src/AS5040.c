#include "AS5040.h"

esp_err_t AS5040_init(AS5040_info *info)
{
    esp_err_t ret = ESP_OK;
    gpio_reset_pin(info->pin_cs);
    gpio_reset_pin(info->pin_clk);
    gpio_reset_pin(info->pin_data);

    ret = gpio_set_direction(info->pin_cs, GPIO_MODE_OUTPUT);
    ret = gpio_set_direction(info->pin_clk, GPIO_MODE_OUTPUT);
    ret = gpio_set_direction(info->pin_data, GPIO_MODE_INPUT);

    // Idle state
    ret = gpio_set_level(info->pin_cs, 1);
    ret = gpio_set_level(info->pin_clk, 1);  // CLK idle low
    
    return ret;
}

esp_err_t AS5040_read(AS5040_info *info, float *position)
{
    esp_err_t ret = ESP_OK;
    uint16_t value = 0;
    ret = gpio_set_level(info->pin_cs, 0);
    gpio_set_level(info->pin_clk, 1);
    esp_rom_delay_us(1);
    for (int i = 0; i < 16; i++) {
        // rising edge
        gpio_set_level(info->pin_clk, 0);
        esp_rom_delay_us(1);  // min 500 ns -> this is about double

        // falling edge -> data valid
        gpio_set_level(info->pin_clk, 1);
        esp_rom_delay_us(1);

        value <<= 1;
        if (gpio_get_level(info->pin_data)) {
            value |= 1;
        }
    }
    ret = gpio_set_level(info->pin_cs, 1);

    *position = (value >> 6) & 0x03FF;
    uint8_t status = value & 0x3F;
    *position = *position * 360.0 / 1024.0 + info->offset;

    if ((status >> 4) & 0x01) ret = ESP_ERR_INVALID_CRC;
    if ((status >> 3) & 0x01) ret = ESP_FAIL;

    return ret;
}

esp_err_t AS5040_read_all(AS5040_info *info, float *position, bool devinfo[])
{
    esp_err_t ret = ESP_OK;
    uint16_t value = 0;
    for (int i = 0; i < 16; i++) {
        // rising edge
        gpio_set_level(info->pin_clk, 1);
        esp_rom_delay_us(1);  // min 500 ns -> this is about double

        // falling edge -> data valid
        gpio_set_level(info->pin_clk, 0);
        esp_rom_delay_us(1);

        value <<= 1;
        if (gpio_get_level(info->pin_data)) {
            value |= 1;
        }
    }

    *position = (value >> 6) & 0x03FF;
    uint8_t status = value & 0x3F;
    for (int i = 0; i < 6; i++) {
        devinfo[i] = (status >> i) & 0x01;
    }
    *position = *position * 360.0 / 1024.0 + info->offset;
    return ret;
}

esp_err_t AS5040_deinit(AS5040_info *info)
{
    gpio_reset_pin(info->pin_cs);
    gpio_reset_pin(info->pin_clk);
    gpio_reset_pin(info->pin_data);

    return ESP_OK;
}