#ifndef RADIO_H__
#define RADIO_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * By default the LoRa sync word will be initialized to this value.
 * This can be changed via the control interface.
 */
#define MESHTASTIC_SYNCWORD 0x2B

#define LORA_IRQ_PRIORITY   0xBF

typedef struct {
    void (*timeout)();
    void (*packet_received)(int8_t rssi, int8_t snr, int8_t signal_rssi, const uint8_t* payload, size_t payload_size);
    void (*packet_transmitted)(uint32_t time_on_air);
    void (*reported_rssi)(int16_t rssi);
} radio_handler_t;

void radio_init(radio_handler_t* handler);

typedef struct {
    uint8_t spreading_factor;
    uint8_t bandwidth;
    uint8_t coding_rate;
    uint8_t low_data_rate;
} radio_lora_params_t;

void radio_get_lora_params(radio_lora_params_t* params);
void radio_set_lora_params(const radio_lora_params_t* params);

typedef struct {
    uint16_t preamble;
    uint8_t header;
    uint8_t sync_word;
    uint8_t crc_on;
    uint8_t invert_iq;
} radio_lora_packet_t;

void radio_get_lora_packet(radio_lora_packet_t* packet);
void radio_set_lora_packet(const radio_lora_packet_t* packet);

typedef struct {
    uint8_t boost;
} radio_rx_params_t;

void radio_get_rx_params(radio_rx_params_t* params);
void radio_set_rx_params(const radio_rx_params_t* params);

typedef struct {
    uint8_t duty_cycle;
    uint8_t hp_max;
    int8_t power;
    uint8_t ramp_time;
} radio_tx_params_t;

void radio_get_tx_params(radio_tx_params_t* params);
void radio_set_tx_params(const radio_tx_params_t* params);

uint32_t radio_get_frequency();
void radio_set_frequency(uint32_t f);

uint8_t radio_get_fallback_mode();
void radio_set_fallback_mode(uint8_t fbm);

int16_t radio_get_continuous_rssi();

typedef struct {
    uint32_t timeout;
    uint8_t report_rssi;
} radio_rx_t;

void radio_get_rx(radio_rx_t* rx);
void radio_set_rx(const radio_rx_t* rx);

bool radio_is_tx_active();
void radio_set_tx(uint32_t timeout, const uint8_t* payload, size_t payload_size);

uint8_t radio_get_standby();
void radio_set_standby(uint8_t mode);

#endif // RADIO_H__