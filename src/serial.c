#include "serial.h"
#include "crc16.h"
#include "pinout.h"

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

static QueueHandle_t uart_txq = NULL;
static QueueHandle_t uart_rxq = NULL;

static serial_handler_t* handler = NULL;

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
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t data;

    if (usart_get_flag(USART1, USART_FLAG_RXNE) != 0) {
        data = usart_recv(USART1);

        xQueueSendFromISR(uart_rxq, &data, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
    uint16_t payload_bytes_received = 0;
    uint8_t ch;

    long start_time = xTaskGetTickCount();

    for (;;) {
        if (xQueueReceive(uart_rxq, &ch, 500) == pdPASS) {
            // Check for timeout and reset state to RX_STATE_START
            // if it took too long to receive a complete message.
            const long current_time = xTaskGetTickCount();

            if (current_time - start_time > pdMS_TO_TICKS(SERIAL_TIMEOUT_MS)) {
                // Message timeout, reset the state
                state = RX_STATE_START;
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

            if (state != RX_STATE_START && state != RX_STATE_CRC_LSB && state != RX_STATE_CRC_MSB) {
                // Exclude CRC calculation on start byte and final CRC field.
                rolling_crc = crc16(rolling_crc, &ch, 1);
            }

            /* Handle message receive state machine */
            switch(state) {
            case RX_STATE_START:
                /* Waiting for the message start byte */
                if (ch == MESSAGE_START) {
                    state = RX_STATE_TYPE;
                    start_time = current_time;  // Reset start time (for timeout calculation)
                    rolling_crc = 0;            // Reset CRC calculation
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
                if (msg_payload_length > 0)
                    state = RX_STATE_PAYLOAD;
                else
                    state = RX_STATE_CRC_LSB;

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
                    if (handler != NULL && handler->message_received != NULL) {
                        handler->message_received(msg_type, msg_payload_buffer, (size_t)msg_payload_length);
                    }
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

void serial_init(serial_handler_t* h)
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
    nvic_set_priority(NVIC_USART1_IRQ, SERIAL_RX_IRQ_PRIORITY);
    nvic_enable_irq(NVIC_USART1_IRQ);

	usart_enable(USART1);

    uart_txq = xQueueCreate(SERIAL_TXQ_SIZE, sizeof(uint8_t));
    uart_rxq = xQueueCreate(SERIAL_RXQ_SIZE, sizeof(uint8_t));

    xTaskCreate(serial_tx_task, "UART_TX", 100, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(serial_rx_task, "UART_RX", 100, NULL, configMAX_PRIORITIES - 1, NULL);

    handler = h;
}

void serial_send_message(uint8_t type, const uint8_t* payload, size_t payload_size)
{
    uint16_t crc = 0;
    uint8_t data;

    serial_putc(MESSAGE_START);
    serial_putc_escaped(type);
    crc = crc16(crc, &type, 1);

    data = (uint8_t)(payload_size & 0xFF);
    serial_putc_escaped(data);
    crc = crc16(crc, &data, 1);

    data = (uint8_t)((payload_size >> 8) & 0xFF);
    serial_putc_escaped(data);
    crc = crc16(crc, &data, 1);

    if (payload != NULL && payload_size > 0) {
        for (size_t i = 0; i < payload_size; i++)
            serial_putc_escaped(payload[i]);

        crc = crc16(crc, payload, payload_size);
    }

    data = (uint8_t)(crc & 0xFF);
    serial_putc_escaped(data);

    data = (uint8_t)((crc >> 8) & 0xFF);
    serial_putc_escaped(data);
}

void serial_send_message2(uint8_t type, const uint8_t* chunk1, size_t chunk1_size,
                                        const uint8_t* chunk2, size_t chunk2_size)
{
    uint16_t crc = 0;
    uint8_t data;

    const uint16_t total_size = (chunk1 != NULL ? chunk1_size : 0) +
                                (chunk2 != NULL ? chunk2_size : 0);

    serial_putc(MESSAGE_START);
    serial_putc_escaped(type);
    crc = crc16(crc, &type, 1);

    data = (uint8_t)(total_size & 0xFF);
    serial_putc_escaped(data);
    crc = crc16(crc, &data, 1);

    data = (uint8_t)((total_size >> 8) & 0xFF);
    serial_putc_escaped(data);
    crc = crc16(crc, &data, 1);

    if ((chunk1 != NULL) && (chunk1_size > 0)) {
        for (size_t i = 0; i < chunk1_size; i++)
            serial_putc_escaped(chunk1[i]);

        crc = crc16(crc, chunk1, chunk1_size);
    }

    if ((chunk2 != NULL) && (chunk2_size > 0)) {
        for (size_t i = 0; i < chunk2_size; i++)
            serial_putc_escaped(chunk2[i]);

        crc = crc16(crc, chunk2, chunk2_size);
    }

    data = (uint8_t)(crc & 0xFF);
    serial_putc_escaped(data);

    data = (uint8_t)((crc >> 8) & 0xFF);
    serial_putc_escaped(data);
}


void serial_putc(const uint8_t ch)
{
    xQueueSend(uart_txq, &ch, portMAX_DELAY);
}

void serial_putc_escaped(const uint8_t ch)
{
    if (ch == MESSAGE_START) {
        serial_putc(MESSAGE_ESCAPE);
        serial_putc(MESSAGE_ESCAPE_START);
    } else if (ch == MESSAGE_ESCAPE) {
        serial_putc(MESSAGE_ESCAPE);
        serial_putc(MESSAGE_ESCAPE_ESCAPE);
    } else {
        serial_putc(ch);
    }
}

void serial_send(const uint8_t* data, size_t size)
{
    for (size_t i = 0; i < size; i++) {
        xQueueSend(uart_txq, &data[i], portMAX_DELAY);
    }
}

#ifdef DEBUG

void debug_puts(const char *str) {
    while (*str) {
        char ch = *str++;
        usart_send_blocking(USART1, ch);

        if (ch == '\n')
            usart_send_blocking(USART1, '\r');
    }
}

void debug_putc(char ch)
{
    usart_send_blocking(USART1, ch);
}

void debug_puti(int i) {
    char buf[16];
    int pos = 0;

    if (i < 0) {
        usart_send_blocking(USART1, '-');
        i = -i;
    }

    do {
        buf[pos++] = '0' + (i % 10);
        i /= 10;
    } while (i);

    while (pos > 0) {
        usart_send_blocking(USART1, buf[--pos]);
    }
}

void debug_putx(uint32_t x) {
    const static char hexChars[] = "0123456789ABCDEF";

    for (int i = 7; i >= 0; i--) {
        const uint8_t nibble = (x >> (i * 4)) & 0x0F;

        usart_send_blocking(USART1, hexChars[nibble]);
    }
}

#endif // DEBUG