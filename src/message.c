#include "message.h"
#include "version.h"
#include "serial.h"
#include "radio.h"

//------------------------------------------------------------------------------

static void process_get_version()
{
    struct __attribute__((packed)) {
        uint8_t major;
        uint8_t minor;
        uint8_t patch;
    }  msg = {
        .major = FIRMWARE_VERSION_MAJOR,
        .minor = FIRMWARE_VERSION_MINOR,
        .patch = FIRMWARE_VERSION_PATCH
    };

    serial_send_message(MSG_VERSION, (uint8_t*)&msg, sizeof(msg));
}

//------------------------------------------------------------------------------

typedef struct __attribute__((packed)) {
    uint8_t spreading_factor;
    uint8_t bandwidth;
    uint8_t coding_rate;
    uint8_t low_data_rate;
} msg_lora_params_t;

static void process_set_lora_params(const uint8_t* data, size_t size)
{
    const msg_lora_params_t* req = (const msg_lora_params_t*)data;
    msg_lora_params_t resp;

    radio_lora_params_t lora_params;

    if (size == sizeof(msg_lora_params_t)) {
        lora_params.spreading_factor = req->spreading_factor;
        lora_params.bandwidth = req->bandwidth;
        lora_params.coding_rate = req->coding_rate;
        lora_params.low_data_rate = req->low_data_rate;

        radio_set_lora_params(&lora_params);
    }

    radio_get_lora_params(&lora_params);

    resp.spreading_factor = lora_params.spreading_factor;
    resp.bandwidth = lora_params.bandwidth;
    resp.coding_rate = lora_params.coding_rate;
    resp.low_data_rate = lora_params.low_data_rate;

    serial_send_message(MSG_LORA_PARAMS, (uint8_t*)&resp, sizeof(resp));
}

//------------------------------------------------------------------------------

typedef struct __attribute__((packed)) {
    uint16_t preamble;
    uint8_t header;
    uint8_t sync_word;
    uint8_t crc_on;
    uint8_t invert_iq;
} msg_lora_packet_t;

static void process_set_lora_packet(const uint8_t* data, size_t size)
{
    const msg_lora_packet_t* req = (const msg_lora_packet_t*)data;
    msg_lora_packet_t resp;

    radio_lora_packet_t lora_packet;

    if (size == sizeof(msg_lora_packet_t)) {
        lora_packet.preamble = req->preamble;
        lora_packet.header = req->header;
        lora_packet.sync_word = req->sync_word;
        lora_packet.crc_on = req->crc_on;
        lora_packet.invert_iq = req->invert_iq;

        radio_set_lora_packet(&lora_packet);
    }

    radio_get_lora_packet(&lora_packet);

    resp.preamble = lora_packet.preamble;
    resp.header = lora_packet.header;
    resp.sync_word = lora_packet.sync_word;
    resp.crc_on = lora_packet.crc_on;
    resp.invert_iq = lora_packet.invert_iq;

    serial_send_message(MSG_LORA_PACKET, (uint8_t*)&resp, sizeof(resp));
}

//------------------------------------------------------------------------------

typedef struct __attribute__((packed)) {
    uint8_t boost;
} msg_rx_params_t;

static void process_set_rx_params(const uint8_t* data, size_t size)
{
    const msg_rx_params_t* req = (const msg_rx_params_t*)data;
    msg_rx_params_t resp;

    radio_rx_params_t rx_params;

    if (size == sizeof(msg_rx_params_t)) {
        rx_params.boost = req->boost;

        radio_set_rx_params(&rx_params);
    }

    radio_get_rx_params(&rx_params);

    resp.boost = rx_params.boost;

    serial_send_message(MSG_RX_PARAMS, (uint8_t*)&resp, sizeof(resp));
}

//------------------------------------------------------------------------------

typedef struct __attribute__((packed)) {
    uint8_t duty_cycle;
    uint8_t hp_max;
    int8_t power;
    uint8_t ramp_time;
} msg_tx_params_t;

static void process_set_tx_params(const uint8_t* data, size_t size)
{
    const msg_tx_params_t* req = (const msg_tx_params_t*)data;
    msg_tx_params_t resp;

    radio_tx_params_t tx_params;

    if (size == sizeof(msg_tx_params_t)) {
        tx_params.duty_cycle = req->duty_cycle;
        tx_params.hp_max = req->hp_max;
        tx_params.power = req->power;
        tx_params.ramp_time = req->ramp_time;

        radio_set_tx_params(&tx_params);
    }

    radio_get_tx_params(&tx_params);

    resp.duty_cycle = tx_params.duty_cycle;
    resp.hp_max = tx_params.hp_max;
    resp.power = tx_params.power;
    resp.ramp_time = tx_params.ramp_time;

    serial_send_message(MSG_TX_PARAMS, (uint8_t*)&resp, sizeof(resp));
}

//------------------------------------------------------------------------------

static void process_set_frequency(const uint8_t* data, size_t size)
{
    const uint32_t* req = (const uint32_t*)data;

    if (size == sizeof(uint32_t)) {
        radio_set_frequency(*req);
    }

    const uint32_t resp = radio_get_frequency();

    serial_send_message(MSG_FREQUENCY, (uint8_t*)&resp, sizeof(resp));
}

//------------------------------------------------------------------------------

static void process_set_fallback_mode(const uint8_t* data, size_t size)
{
    const uint8_t* req = (const uint8_t*)data;

    if (size == sizeof(uint8_t)) {
        radio_set_fallback_mode(*req);
    }

    const uint8_t resp = radio_get_fallback_mode();

    serial_send_message(MSG_FALLBACK_MODE, (uint8_t*)&resp, sizeof(resp));
}

//------------------------------------------------------------------------------

static void process_get_rssi(const uint8_t* data, size_t size)
{
    const int16_t resp = radio_get_continuous_rssi();

    serial_send_message(MSG_RSSI, (uint8_t*)&resp, sizeof(resp));
}

//------------------------------------------------------------------------------

typedef struct __attribute__((packed)) {
    uint32_t timeout;
    uint8_t report_rssi;
} msg_rx_t;

static void process_set_rx(const uint8_t* data, size_t size)
{
    const msg_rx_t* req = (const msg_rx_t*)data;

    radio_rx_t rx;

    if (size == sizeof(msg_rx_t)) {
        rx.timeout = req->timeout;
        rx.report_rssi = req->report_rssi;
        radio_set_rx(&rx);
    }

    radio_get_rx(&rx);

    msg_rx_t resp;
    resp.timeout = rx.timeout;
    resp.report_rssi = rx.report_rssi;

    serial_send_message(MSG_RX, (uint8_t*)&resp, sizeof(resp));
}

//------------------------------------------------------------------------------

static void process_set_tx(const uint8_t* data, size_t size)
{
    uint32_t* timeout = (uint32_t*)data;
    uint8_t resp = 0x00;    // OK

    if (size > sizeof(uint32_t) && !radio_is_tx_active()) {
        radio_set_tx(*timeout, data + sizeof(uint32_t), size - sizeof(uint32_t));
    } else {
        resp = 0x01; // Busy
    }

    serial_send_message(MSG_TX, (uint8_t*)&resp, sizeof(resp));
}

//------------------------------------------------------------------------------

static void process_set_standby(const uint8_t* data, size_t size)
{
    uint8_t* mode = (uint8_t*)data;

    if (size == sizeof(uint8_t)) {
        radio_set_standby(*mode);
    }

    uint8_t resp = radio_get_standby();

    serial_send_message(MSG_STANDBY, (uint8_t*)&resp, sizeof(resp));
}


//==============================================================================

void message_process_from_serial(uint8_t type, const uint8_t* data, size_t size)
{
    message_type_t msg = (message_type_t)type;

    switch (msg) {
    case MSG_GET_VERSION:
        process_get_version();
        break;
    case MSG_SET_LORA_PARAMS:
        process_set_lora_params(data, size);
        break;
    case MSG_SET_LORA_PACKET:
        process_set_lora_packet(data, size);
        break;
    case MSG_SET_RX_PARAMS:
        process_set_rx_params(data, size);
        break;
    case MSG_SET_TX_PARAMS:
        process_set_tx_params(data, size);
        break;
    case MSG_SET_FREQUENCY:
        process_set_frequency(data, size);
        break;
    case MSG_SET_FALLBACK_MODE:
        process_set_fallback_mode(data, size);
        break;
    case MSG_GET_RSSI:
        process_get_rssi(data, size);
        break;
    case MSG_SET_RX:
        process_set_rx(data, size);
        break;
    case MSG_SET_TX:
        process_set_tx(data, size);
        break;
    case MSG_SET_STANDBY:
        process_set_standby(data, size);
        break;
    default:
        break;
    }
}

//==============================================================================

void message_timeout()
{
    serial_send_message(MSG_TIMEOUT, NULL, 0);
}

typedef struct __attribute__((packed)) {
    int8_t rssi;
    int8_t snr;
    int8_t signal_rssi;
} msg_packet_received_t;

void message_packet_received(int8_t rssi, int8_t snr, int8_t signal_rssi, const uint8_t* data, size_t size)
{
    msg_packet_received_t head = {
        .rssi = rssi,
        .snr = snr,
        .signal_rssi = signal_rssi
    };

    serial_send_message2(MSG_PACKET_RECEIVED, (const uint8_t*)&head, sizeof(head), data, size);
}

void message_packet_transmitted(uint32_t time_on_air)
{
    serial_send_message(MSG_PACKET_TRANSMITTED, (const uint8_t*)&time_on_air, sizeof(time_on_air));
}

void message_rssi(int16_t rssi)
{
    serial_send_message(MSG_CONTINUOUS_RSSI, (const uint8_t*)&rssi, sizeof(rssi));
}

void message_logging(const char* str)
{
    // strlen does not work for some reason
    size_t len = 0;
    const char* p = str;
    while (*p != '\0') {
        len++;
        p++;
        if (len > 128)
            break;
    }
    serial_send_message(MSG_LOGGING, (const uint8_t*)str, len);
}
