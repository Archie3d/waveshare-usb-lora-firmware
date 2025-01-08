#include "serial.h"
#include "crc16.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#define MESSAGE_START           (0xAA)
#define MESSAGE_ESCAPE          (0x7D)
#define MESSAGE_ESCAPE_START    (0x8A)
#define MESSAGE_ESCAPE_ESCAPE   (0x5D)

#define MAX_MESSAGE_PAYLOAD_LENGTH  256

typedef enum {
    RX_STATE_START = 0,
    RX_STATE_TYPE,
    RX_STATE_LENGTH_LSB,
    RX_STATE_LENGTH_MSB,
    RX_STATE_PAYLOAD,
    RX_STATE_CRC_LSB,
    RX_STATE_CRC_MSB
} rx_state_t;

static QueueHandle_t uart_txq;
static QueueHandle_t uart_rxq;

static void serial_tx_task(void* args __attribute__((unused)))
{
    uint8_t ch;

    for (;;) {
        if (xQueueReceive(uart_txq, &ch, 500) == pdPASS) {

            // Wait until ready
            while (!usart_get_flag(USART1, USART_SR_TXE))
                taskYIELD();

            usart_send(USART1, ch);
        }
    }
}

void usart1_isr(void)
{
    static uint8_t data = 'A';

    if (usart_get_flag(USART1, USART_FLAG_RXNE) != 0) {
        data = usart_recv(USART1);

        xQueueSend(uart_rxq, &data, portMAX_DELAY);
    }

#if 0
    /* Check if we were called because of RXNE. */
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		/* Retrieve the data from the peripheral. */
		data = usart_recv(USART1);

        xQueueSend(uart_rxq, &data, portMAX_DELAY);
	}
#endif
}

static void serial_rx_task(void* args __attribute__((unused)))
{
    static uint8_t msg_type;
    static uint16_t msg_payload_length;
    static uint8_t msg_payload_buffer[MAX_MESSAGE_PAYLOAD_LENGTH];
    static uint16_t msg_crc;
    static uint16_t rolling_crc;

    rx_state_t state = RX_STATE_START;

    bool escape_sequence_detected = false;
    uint16_t payload_bytes_received;
    uint8_t ch;

    for (;;) {
        if (xQueueReceive(uart_rxq, &ch, 500) == pdPASS) {

            // @todo Check for timeout and reset state to RX_STATE_START
            //       if it took too long to receive a complete message.

            if (state != RX_STATE_START && state != RX_STATE_CRC_LSB && state != RX_STATE_CRC_MSB) {
                // Exclude CRC calculation on start byte and final CRC field.
                rolling_crc = crc16(rolling_crc, &ch, 1);
            }

            if (state != RX_STATE_START) {

                /* Handle escape sequences */
                if (ch == MESSAGE_ESCAPE) {
                    escape_sequence_detected = true;
                    continue;
                }

                if (escape_sequence_detected) {
                    if (ch == MESSAGE_ESCAPE_START) {
                        ch = MESSAGE_START;
                    } else if (ch == MESSAGE_ESCAPE_ESCAPE) {
                        ch = MESSAGE_ESCAPE;
                    }
                }

                escape_sequence_detected = false;
            }

            /* Handle message receive state machine */
            switch(state) {
            case RX_STATE_START:
                /* Waiting for the message start byte */
                if (ch == MESSAGE_START) {
                    state = RX_STATE_TYPE;
                    rolling_crc = 0;    // Reset CRC calculation
                }
                break;
            case RX_STATE_TYPE:
                /* Waiting for the message type */
                msg_type = ch;
                state = RX_STATE_LENGTH_LSB;
                break;
            case RX_STATE_LENGTH_LSB:
                msg_payload_length = (uint16_t)ch;
                state = RX_STATE_LENGTH_MSB;
                break;
            case RX_STATE_LENGTH_MSB:
                msg_payload_length = msg_payload_length | (((uint16_t)ch) << 8);
                state = RX_STATE_PAYLOAD;
                payload_bytes_received = 0;
                break;
            case RX_STATE_PAYLOAD:
                if (payload_bytes_received < msg_payload_length) {
                    msg_payload_buffer[payload_bytes_received++] = ch;

                    if (payload_bytes_received == msg_payload_length) {
                        state = RX_STATE_CRC_LSB;
                    }
                }
                break;
            case RX_STATE_CRC_LSB:
                msg_crc = (uint16_t)ch;
                state = RX_STATE_CRC_MSB;
                break;
            case RX_STATE_CRC_MSB:
                msg_crc = msg_crc | (((uint16_t)ch) << 8);

                if (msg_crc == rolling_crc) {
                    // Message is correct
                } else {
                    // Message CRC error
                }
                state = RX_STATE_START;
                break;
            default:
                break;
            }
        }
    }
}

void serial_init()
{
    rcc_periph_clock_enable(RCC_USART1);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
	usart_set_baudrate(USART1, SERIAL_BAUD);
	usart_set_databits(USART1, SERIAL_DATA_BITS);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    // Enable UART RX interrupt
    usart_enable_rx_interrupt(USART1);

    nvic_enable_irq(NVIC_USART1_IRQ);

	usart_enable(USART1);

    uart_txq = xQueueCreate(SERIAL_TXQ_SIZE, sizeof(uint8_t));

    xTaskCreate(serial_tx_task, "UART_TX", 100, NULL, configMAX_PRIORITIES - 1, NULL);
}

void serial_send(const uint8_t* data, size_t size)
{
    for (size_t i = 0; i < size; i++) {
        xQueueSend(uart_txq, &data[i], portMAX_DELAY);
    }
}
