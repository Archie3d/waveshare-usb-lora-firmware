#ifndef CRC16_H__
#define CRC16_H__

#include <stdint.h>
#include <stddef.h>

/**
 * Calculate CRC16 on a data block starting from initial CRC value provided.
 */
uint16_t crc16(uint16_t crc0, const uint8_t* data, size_t length);

#endif // CRC16_H__
