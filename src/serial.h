#ifndef SERIAL_H__
#define SERIAL_H__

#include <stdint.h>
#include <stddef.h>

#define SERIAL_BAUD         115200
#define SERIAL_DATA_BITS    8

#define SERIAL_TXQ_SIZE     256
#define SERIAL_RXQ_SIZE     256

void serial_init();

void serial_send(const uint8_t* data, size_t size);

#endif // SERIAL_H__
