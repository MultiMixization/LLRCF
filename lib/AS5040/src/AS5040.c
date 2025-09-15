#include "AS5040.h"

esp_err_t AS5040_init(AS5040_info *info)
{
    gpio_reset_pin(info->pin_cs);
    gpio_reset_pin(info->pin_clk);
    gpio_reset_pin(info->pin_data);

    gpio_set_direction(info->pin_cs, GPIO_MODE_OUTPUT);
    gpio_set_direction(info->pin_clk, GPIO_MODE_OUTPUT);
    gpio_set_direction(info->pin_data, GPIO_MODE_INPUT);

    // Idle state
    gpio_set_level(info->pin_cs, 0);   // CSn must be LOW for SSI
    gpio_set_level(info->pin_clk, 0);  // CLK idle low
    
    return ESP_OK;
}

esp_err_t AS5040_read(AS5040_info *info, float *position)
{
    uint16_t value = 0;
    for (int i = 0; i < 16; i++) {
        // rising edge
        gpio_set_level(info->pin_clk, 1);
        esp_rom_delay_us(1);  // min 500 ns

        // falling edge -> data valid
        gpio_set_level(info->pin_clk, 0);
        esp_rom_delay_us(1);

        value <<= 1;
        if (gpio_get_level(info->pin_data)) {
            value |= 1;
        }
    }

    *position = (value >> 6) & 0x03FF;
    *position = *position * 360.0 / 1024.0 + info->offset;
    return ESP_OK;
}