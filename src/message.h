#ifndef MESSAGE_H__
#define MESSAGE_H__

#include <stdint.h>
#include <stddef.h>

typedef enum {
    // Controller to device
    MSG_GET_VERSION = 0x01,
    MSG_SET_LORA_PARAMS = 0x02,
    MSG_SET_TX_PARAMS = 0x03,
    MSG_SET_RX_PARAMS = 0x04,
    MSG_SET_FREQUENCY = 0x05,
    MSG_GET_RSSI = 0x06,
    MSG_SEND = 0x07,

    // Device to controller
    MSG_ACK = 0x80,
    MSG_ERROR = 0x81,
    MSG_VERSION = 0x82,
    MSG_RSSI = 0x083,
    MSG_TRANSMITTED = 0x86,
    MSG_RECEIVED = 0x85,
    MSG_TIMEOUT = 0x86
} message_type_t;

typedef struct __attribute__((packed)) {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
} msg_version_t;

typedef struct __attribute__((packed)) {
    uint8_t spreading_factor;
    uint8_t bandwidth;
    uint8_t coding_rate;
} msg_set_lora_params_t;

void send_ack();
void send_error();

void process_raw_message(uint8_t type, const uint8_t* data, size_t size);

#endif // MESSAGE_H__
