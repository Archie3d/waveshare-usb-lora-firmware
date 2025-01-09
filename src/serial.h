#ifndef SERIAL_H__
#define SERIAL_H__

#include <stdint.h>
#include <stddef.h>

#define SERIAL_BAUD         115200
#define SERIAL_DATA_BITS    8
#define SERIAL_RX_IRQ_PRIORITY 0xC0 // Must >= max SysCall priority (0xB0 in FreeRTOSConfig.h)

#define SERIAL_TXQ_SIZE     256
#define SERIAL_RXQ_SIZE     256

#define SERIAL_TIMEOUT_MS   200

typedef struct {
    void (*message_received)(uint8_t type, const uint8_t* payload, size_t payload_size);
} serial_handler_t;

void serial_init(serial_handler_t* handler);

void serial_send_message(uint8_t type, const uint8_t* payload, size_t payload_size);

void serial_putc(const uint8_t ch);
void serial_putc_escaped(const uint8_t ch);
void serial_send(const uint8_t* data, size_t size);

void debug_puts(const char* str);
void debug_putc(char ch);
void debug_puti(int i);
void debug_putx(uint32_t x);

#endif // SERIAL_H__
