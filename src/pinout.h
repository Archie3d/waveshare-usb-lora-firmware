#ifndef PINOUT_H__
#define PINOUT_H__

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>


#define BUTTON_PORT     GPIOA
#define BUTTON_PIN      GPIO5

#define LED_RXD_PORT    GPIOA
#define LED_RXD_PIN     GPIO6

#define LED_TXD_PORT    GPIOA
#define LED_TXD_PIN     GPIO7

#define LORA_RESET_PORT GPIOA
#define LORA_RESET_PIN  GPIO4

#define LORA_DIO1_PORT  GPIOB
#define LORA_DIO1_PIN   GPIO0
#define LORA_DIO1_EXTI0 EXTI0
#define LORA_DIO1_IRQ   NVIC_EXTI0_IRQ

#define LORA_BUSY_PORT  GPIOB
#define LORA_BUSY_PIN   GPIO1

#define LORA_RF_SW_PORT GPIOB
#define LORA_RF_SW_PIN  GPIO4

#define LORA_SPI        SPI2
#define LORA_RCC_SPI    RCC_SPI2

#define LORA_SPI_NSS    GPIO12
#define LORA_SPI_SCK    GPIO13
#define LORA_SPI_MISO   GPIO14
#define LORA_SPI_MOSI   GPIO15

#endif // PINOUT_H__
