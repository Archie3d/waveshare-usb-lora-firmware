#ifndef MESSAGE_H__
#define MESSAGE_H__

#include <stdint.h>
#include <stddef.h>

typedef enum {
    // Controller to device
    MSG_GET_VERSION       = 0x01,
    MSG_SET_LORA_PARAMS   = 0x02,
    MSG_SET_LORA_PACKET   = 0x03,
    MSG_SET_RX_PARAMS     = 0x04,
    MSG_SET_TX_PARAMS     = 0x05,
    MSG_SET_FREQUENCY     = 0x06,
    MSG_SET_FALLBACK_MODE = 0x07,
    MSG_GET_RSSI          = 0x08,
    MSG_SET_RX            = 0x09,
    MSG_SET_TX            = 0x0A,
    MSG_SET_STANDBY       = 0x0B,

    // Device to controller
    MSG_VERSION       = 0x81,
    MSG_LORA_PARAMS   = 0x82,
    MSG_LORA_PACKET   = 0x83,
    MSG_RX_PARAMS     = 0x84,
    MSG_TX_PARAMS     = 0x85,
    MSG_FREQUENCY     = 0x86,
    MSG_FALLBACK_MODE = 0x87,
    MSG_RSSI          = 0x88,
    MSG_RX            = 0x89,
    MSG_TX            = 0x8A,
    MSG_STANDBY       = 0x8B,

    // Unsolicited messages
    MSG_TIMEOUT            = 0x90,
    MSG_PACKET_RECEIVED    = 0x91,
    MSG_PACKET_TRANSMITTED = 0x92,
    MSG_CONTINUOUS_RSSI    = 0x93

} message_type_t;

void message_process_from_serial(uint8_t type, const uint8_t* data, size_t size);

void message_timeout();
void message_packet_received(int8_t rssi, int8_t snr, int8_t signal_rssi, const uint8_t* data, size_t size);
void message_packet_transmitted(uint32_t time_on_air);
void message_rssi(int16_t rssi);

#endif // MESSAGE_H__
