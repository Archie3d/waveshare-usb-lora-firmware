#ifndef CRC16_H__
#define CRC16_H__

#include <stdint.h>
#include <stddef.h>

uint16_t crc16(uint16_t crc0, uint8_t* data, size_t length);

#endif // CRC16_H__
