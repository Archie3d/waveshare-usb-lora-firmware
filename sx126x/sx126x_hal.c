#include <sx126x_hal.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "pinout.h"

#include <stddef.h>

static void wait(int n_cycles) {
    for (int i = 0; i < n_cycles; i++) {
        __asm__("nop");
    }
}

void sx126x_hal_wait_on_busy(const void* context)
{
    while (gpio_get(LORA_BUSY_PORT, LORA_BUSY_PIN) != 0) {
        wait(1000);
    }
}

static void spi_rw_buffer(const void* context, const uint8_t* data_out, uint8_t* data_in, uint16_t data_length) {
    for (int i = 0; i < data_length; i++) {
        uint8_t byte = data_out != NULL ? data_out[i] : 0;
        spi_send(LORA_SPI, byte);

        byte = spi_read(LORA_SPI);

        if (data_in != NULL)
            data_in[i] = byte;
    }
}

sx126x_hal_status_t sx126x_hal_write(const void* context, const uint8_t* command, const uint16_t command_length,
                                     const uint8_t* data, const uint16_t data_length)
{
    sx126x_hal_wait_on_busy(context);

    spi_enable(LORA_SPI);

    spi_rw_buffer(context, command, NULL, command_length);
    spi_rw_buffer(context, data, NULL, data_length);

    spi_disable(LORA_SPI);

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read(const void* context, const uint8_t* command, const uint16_t command_length,
                                    uint8_t* data, const uint16_t data_length)
{
    sx126x_hal_wait_on_busy(context);

    spi_enable(LORA_SPI);

    spi_rw_buffer(context, command, NULL, command_length);
    spi_rw_buffer(context, NULL, data, data_length);

    spi_disable(LORA_SPI);

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void* context)
{
    gpio_clear(LORA_RESET_PORT, LORA_RESET_PIN);
    wait(500000);
    gpio_set(LORA_RESET_PORT, LORA_RESET_PIN);

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void* context)
{
    spi_enable_software_slave_management(LORA_SPI);
    spi_enable(LORA_SPI);
    spi_set_nss_low(LORA_SPI);
    wait(500000);
    spi_set_nss_high(LORA_SPI);
    spi_disable(LORA_SPI);
    spi_disable_software_slave_management(LORA_SPI);

    return SX126X_HAL_STATUS_OK;
}
