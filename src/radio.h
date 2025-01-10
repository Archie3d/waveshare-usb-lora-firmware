#ifndef RADIO_H__
#define RADIO_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define LORA_SYNCWORD 0x2B  // Meshtastic

#define LORA_IRQ_PRIORITY   0xBF

typedef struct {
    void (*packet_received)(int8_t rssi, int8_t snr, const uint8_t* payload, size_t payload_size);
    void (*packet_transmitted)(void);
} radio_handler_t;

void radio_init(radio_handler_t* handler);

void radio_set_lora_params(uint8_t sf, uint8_t bw, uint8_t cr);
void radio_set_rx_params(uint8_t sf, uint8_t bw, uint8_t cr);

bool radio_is_transmitting();
void radio_transmit(const uint8_t* payload, size_t payload_size);

#endif // RADIO_H__