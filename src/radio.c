#include "radio.h"
#include "pinout.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

void radio_init()
{
    // SX1262 DIO1 interrupt
    exti_select_source(LORA_DIO1_EXTI0, LORA_DIO1_PORT);
    exti_set_trigger(LORA_DIO1_EXTI0, EXTI_TRIGGER_RISING);
    exti_enable_request(LORA_DIO1_EXTI0);
    nvic_enable_irq(LORA_DIO1_IRQ);

    // SPI communication with SX126X
    rcc_periph_clock_enable(LORA_RCC_SPI);
    gpio_set_mode(GPIOB,
                  GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  LORA_SPI_NSS | LORA_SPI_SCK | LORA_SPI_MOSI);

    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, LORA_SPI_MISO);

    spi_init_master(LORA_SPI,
                    SPI_CR1_BAUDRATE_FPCLK_DIV_128,
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_DFF_8BIT,
                    SPI_CR1_MSBFIRST);

    spi_disable_software_slave_management(LORA_SPI);
    spi_enable_ss_output(LORA_SPI);
}
