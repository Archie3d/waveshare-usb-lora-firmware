#include <FreeRTOS.h>
#include <task.h>

#include <string.h>

#include "init.h"
#include "pinout.h"

void *dbg_memcpy(void *dest, const void *src, size_t n) {
    uint8_t *d = dest;
    const uint8_t *s = src;
    for (size_t i = 0; i < n; i++) {
        d[i] = s[i];
    }
    return dest;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

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

void vAssertCalled(const char *file, int line) {
    debug_puts("Assert failed in file ");
    debug_puts(file);
    debug_puts(", line ");
    debug_puti(line);
    debug_puts("\n");

    taskDISABLE_INTERRUPTS();
    for( ;; );
}

static const char* message = "TEST MESSAGE";

static void dummy(void* args __attribute__((unused))) {
    for (;;) {
        gpio_toggle(LED_RXD_PORT, LED_RXD_PIN);
        //serial_putc('C');
        serial_send_message('[', message, 12);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void serial_message_received(uint8_t type, const uint8_t* payload, size_t payload_size)
{
}

serial_handler_t serial_handler = {
    .message_received = serial_message_received
};

//const static char* msg = "TEST MESSAGE";

uint8_t buff1[8];

int main(void)
{
    global_init(&serial_handler);

    xTaskCreate(dummy, "Dummy", 100, NULL, configMAX_PRIORITIES - 2, NULL);

    vTaskStartScheduler();

    for(;;) {
        __asm__("nop");
    }

    return 0;
}
