#include <FreeRTOS.h>
#include <task.h>

#include <string.h>

#include "init.h"
#include "pinout.h"
#include "message.h"

void *dbg_memcpy(void *dest, const void *src, size_t n) {
    uint8_t *d = dest;
    const uint8_t *s = src;
    for (size_t i = 0; i < n; i++) {
        d[i] = s[i];
    }
    return dest;
}

static void error_blink()
{
    taskDISABLE_INTERRUPTS();

    gpio_set(LED_RXD_PORT, LED_RXD_PIN);
    gpio_set(LED_TXD_PORT, LED_TXD_PIN);

    for (;;) {
        for (size_t i = 0; i < 1500000; i++) {
            __asm__("nop");
        }

        gpio_toggle(LED_RXD_PORT, LED_RXD_PIN);
        gpio_toggle(LED_TXD_PORT, LED_TXD_PIN);
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    error_blink();
}

void vAssertCalled(const char *file, int line) {
    DBG("Assert failed in file ");
    DBG(file);
    DBG(", line ");
    DBG_I(line);
    DBG("\n");

    error_blink();
}

static void serial_message_received(uint8_t type, const uint8_t* payload, size_t payload_size)
{
    message_process_from_serial(type, payload, payload_size);
}

serial_handler_t serial_handler = {
    .message_received = serial_message_received,
};

static void tx_rx_timeout()
{
    message_timeout();
}

static void lora_packet_received(int8_t rssi, int8_t snr, int8_t signal_rssi, const uint8_t* payload, size_t payload_size)
{
    message_packet_received(rssi, snr, signal_rssi, payload, payload_size);
}

static void lora_packet_transmitted(uint32_t time_on_air)
{
    message_packet_transmitted(time_on_air);
}

static void reported_rssi(int16_t rssi)
{
    message_rssi(rssi);
}

radio_handler_t radio_handler = {
    .timeout = tx_rx_timeout,
    .packet_received = lora_packet_received,
    .packet_transmitted = lora_packet_transmitted,
    .reported_rssi = reported_rssi,
};

/**
 * Main entry point.
 */
int main(void)
{
    global_init(&serial_handler, &radio_handler);

    vTaskStartScheduler();

    for(;;) {
        __asm__("nop");
    }

    return 0;
}
