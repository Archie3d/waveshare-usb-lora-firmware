#include "message.h"
#include "serial.h"
#include "radio.h"

void send_ack()
{
    serial_send_message(MSG_ACK, NULL, 0);
}

void send_error()
{
    serial_send_message(MSG_ERROR, NULL, 0);
}

static void process_get_version()
{
    msg_version_t msg = {
        .major = 1,
        .minor = 0,
        .patch = 0
    };

    serial_send_message(MSG_VERSION, (uint8_t*)&msg, sizeof(msg));
}

static void process_set_lora_params(const uint8_t* data, size_t size)
{
    if (size != sizeof(msg_set_lora_params_t)) {
        send_error();
        return;
    }

    msg_set_lora_params_t* msg = (msg_set_lora_params_t*)data;

    radio_set_lora_params(msg->spreading_factor, msg->bandwidth, msg->coding_rate);
    send_ack();
}

static void process_get_rssi()
{

}

static void process_lora_send(const uint8_t* data, size_t size)
{
    if (radio_is_transmitting) {
        send_error();
    } else {
        radio_transmit(data, size);
        send_ack();
    }
}

void process_raw_message(uint8_t type, const uint8_t* data, size_t size)
{
    message_type_t msg = (message_type_t)type;

    switch (msg) {
    case MSG_GET_VERSION:
        process_get_version();
        break;
    case MSG_SET_LORA_PARAMS:
        process_set_lora_params(data, size);
        break;
    case MSG_SEND:
        process_lora_send(data, size);
        break;
    default:
        break;
    }
}