#include "init.h"
#include "pinout.h"
#include "radio.h"
#include "serial.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include "libopencm3/cm3/scb.h"

void global_init(serial_handler_t* serial_handler, radio_handler_t* radio_handler)
{
    SCB_CCR &= ~SCB_CCR_UNALIGN_TRP; // Disable unaligned access traps

    // Initialize system clock, XTAL 8MHz PLL to 72MHz
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    // Initialize GPIOs
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, BUTTON_PIN);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_RXD_PIN | LED_TXD_PIN | LORA_RESET_PIN);

    gpio_set(BUTTON_PORT, BUTTON_PIN);  // Pull-up the button pin

    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_AFIO);

    // Disable JTAG to use PB4 pin as GPIO
    AFIO_MAPR &= ~AFIO_MAPR_SWJ_MASK;               // Clear SWJ_CFG bits
    AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;  // Disable JTAG but keep SWD on

    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, LORA_DIO1_PIN | LORA_BUSY_PIN);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LORA_RF_SW_PIN);

    // Initialize USART
    serial_init(serial_handler);

    // Initialize SX126X interface
    radio_init(radio_handler);
}