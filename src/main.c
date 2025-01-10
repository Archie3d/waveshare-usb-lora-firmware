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
    debug_puts("Assert failed in file ");
    debug_puts(file);
    debug_puts(", line ");
    debug_puti(line);
    debug_puts("\n");

    error_blink();
}

static void serial_message_received(uint8_t type, const uint8_t* payload, size_t payload_size)
{
    process_raw_message(type, payload, payload_size);
}

static void serial_message_crc_error()
{
    send_error();
}

serial_handler_t serial_handler = {
    .message_received = serial_message_received,
    .message_crc_error = serial_message_crc_error,
};

static void lora_packet_received(int8_t rssi, int8_t snr, const uint8_t* payload, size_t payload_size)
{
    serial_send_message(MSG_RECEIVED, payload, payload_size);
    /*
    DBG("Packet of ");
    DBG_I((int)payload_size);
    DBG(" bytes received. RSSI: ");
    DBG_I(rssi);
    DBG(" dBm, SNR: ");
    DBG_I(snr);
    DBG(" dB\n");
    */
}

static void lora_packet_transmitted()
{
    serial_send_message(MSG_TRANSMITTED, NULL, 0);
}

radio_handler_t radio_handler = {
    .packet_received = lora_packet_received,
    .packet_transmitted = lora_packet_transmitted
};


int main(void)
{
    global_init(&serial_handler, &radio_handler);

    vTaskStartScheduler();

    for(;;) {
        __asm__("nop");
    }

    return 0;
}
