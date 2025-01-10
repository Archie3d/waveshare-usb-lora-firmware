#include "radio.h"
#include "pinout.h"

#include "sx126x/sx126x.h"

#include <FreeRTOS.h>
#include <task.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

#define FREQ_HZ 869525000

typedef enum {
    NOTIF_RX_DONE = 0x0001,
    NOTIF_TX_DONE = 0x0002,
    NOTIF_TIMEOUT = 0x0004,
    NOTIF_TRANSMIT = 0x0100,
    NOTIF_SET_LORA_PARAMS = 0x0200,
} isr_notification_t;

// Expected output power = 11dB
static const sx126x_pa_cfg_params_t pa_pwr_cfg = {
    .pa_duty_cycle = 0x1,
    .hp_max = 0x0,
    .device_sel = 0x1,
    .pa_lut = 0x1,
};

static sx126x_mod_params_lora_t lora_mod_params = {
    .sf = SX126X_LORA_SF11,     // LORA_SPREADING_FACTOR
    .bw = SX126X_LORA_BW_250,   // LORA_BANDWIDTH       250 for Long fast
    .cr = SX126X_LORA_CR_4_8,   // LORA_CODING_RATE
    .ldro = 0,                  // Must be OFF for Meshtastic
};

static const sx126x_pkt_params_lora_t lora_pkt_params = {
    .preamble_len_in_symb = 16,                 // LORA_PREAMBLE_LENGTH
    .header_type = SX126X_LORA_PKT_EXPLICIT,    // LORA_PKT_LEN_MODE
    .pld_len_in_bytes = 255,                    // PAYLOAD_LENGTH
    .crc_is_on = true,                          // LORA_CRC
    .invert_iq_is_on = false,                   // LORA_IQ
};


static sx126x_cmd_status_t chip_status(const char* prefix) {
    sx126x_chip_status_t chip_status;
    sx126x_get_status(NULL, &chip_status);

    DBG(prefix);
    DBG("\tChip mode: ");
    DBG_I(chip_status.chip_mode);
    DBG(" Command status: ");
    DBG_I(chip_status.cmd_status);
    DBG("\n");
    return chip_status.cmd_status;
}

static radio_handler_t* handler = NULL;
static TaskHandle_t xRadioIsrTask;

static uint8_t tx_buffer[255];
static uint8_t tx_buffer_size;
static bool transmitting = false;

void exti0_isr(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    exti_reset_request(EXTI0);

    sx126x_irq_mask_t irq_mask;
    sx126x_get_and_clear_irq_status(NULL, &irq_mask);

    DBG("SX126X DIO1 IRQ ");
    DBG_I(irq_mask);
    DBG("\n");

    uint32_t notif = 0;

    if ((irq_mask & SX126X_IRQ_RX_DONE) == SX126X_IRQ_RX_DONE) {
        notif |= NOTIF_RX_DONE;
    }

    if ((irq_mask & SX126X_IRQ_TIMEOUT) == SX126X_IRQ_TIMEOUT) {
        notif |= NOTIF_TIMEOUT;
    }

    if ((irq_mask & SX126X_IRQ_TX_DONE) == SX126X_IRQ_TX_DONE) {
        notif = NOTIF_TX_DONE;
    }

    if (notif) {
        xTaskNotifyFromISR(xRadioIsrTask, notif, eSetBits, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void radio_isr_task(void* args __attribute__((unused)))
{
    static uint8_t rx_buffer[255];
    uint8_t rx_buffer_size = 0;

    uint32_t ulNotifiedValue;

    sx126x_reset(NULL);
    sx126x_set_reg_mode(NULL, SX126X_REG_MODE_LDO);
    sx126x_set_dio2_as_rf_sw_ctrl(NULL, false);
    sx126x_set_standby(NULL, SX126X_STANDBY_CFG_RC);

    sx126x_set_pkt_type(NULL, SX126X_PKT_TYPE_LORA);
    sx126x_set_rf_freq(NULL, FREQ_HZ);

    sx126x_set_pa_cfg(NULL, &pa_pwr_cfg);
    sx126x_set_tx_params(NULL, 27, SX126X_RAMP_3400_US);

    sx126x_set_rx_tx_fallback_mode(NULL, SX126X_FALLBACK_STDBY_RC);
    sx126x_cfg_rx_boosted(NULL, 1); // Enable RX boost mode

    sx126x_set_lora_mod_params(NULL, &lora_mod_params);
    sx126x_set_lora_pkt_params(NULL, &lora_pkt_params);
    sx126x_set_lora_sync_word(NULL, LORA_SYNCWORD);

    sx126x_set_dio_irq_params(
        NULL,
        SX126X_IRQ_ALL,
        //SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_HEADER_ERROR | SX126X_IRQ_CRC_ERROR,
        SX126X_IRQ_ALL,
        SX126X_IRQ_NONE,
        SX126X_IRQ_NONE);

    sx126x_clear_irq_status(NULL, SX126X_IRQ_ALL);
    sx126x_set_dio3_as_tcxo_ctrl(NULL, SX126X_TCXO_CTRL_1_7V, 5000);

    // Switch antenna to RX
    gpio_set(LORA_RF_SW_PORT, LORA_RF_SW_PIN);

    // Continuous RX
    sx126x_set_rx_with_timeout_in_rtc_step(NULL, SX126X_RX_CONTINUOUS);

    for (;;) {
        if (xTaskNotifyWait(0,      // bits to clear on entry
                            0xFFFF, // bits to clear on exit
                            &ulNotifiedValue, // Notified value pass out in
                            1) == pdFALSE)
        {
            // Idle, measure RSSI
            uint16_t rssi;
            sx126x_get_rssi_inst(NULL, &rssi);
            continue;
        }

        if (ulNotifiedValue & NOTIF_RX_DONE) {
            sx126x_rx_buffer_status_t rx_buffer_status;
            sx126x_pkt_status_lora_t pkt_status;

            gpio_clear(LED_RXD_PORT, LED_RXD_PIN);

            sx126x_get_rx_buffer_status(NULL, &rx_buffer_status);

            if (rx_buffer_status.pld_len_in_bytes > 0) {
                sx126x_get_lora_pkt_status(NULL, &pkt_status);

                sx126x_read_buffer(NULL, rx_buffer_status.buffer_start_pointer, rx_buffer, rx_buffer_status.pld_len_in_bytes);
                rx_buffer_size = rx_buffer_status.pld_len_in_bytes;

                // Call handler with the received message
                if (handler != NULL && handler->packet_received != NULL) {
                    handler->packet_received(pkt_status.rssi_pkt_in_dbm, pkt_status.snr_pkt_in_db, rx_buffer, rx_buffer_size);
                }
            }

            gpio_set(LED_RXD_PORT, LED_RXD_PIN);
        }

        if (ulNotifiedValue & NOTIF_TIMEOUT) {
            transmitting = false;
            gpio_set(LED_RXD_PORT, LED_RXD_PIN);
            gpio_set(LED_TXD_PORT, LED_TXD_PIN);
        }

        if (ulNotifiedValue & NOTIF_TX_DONE) {
            transmitting = false;

            gpio_set(LED_TXD_PORT, LED_TXD_PIN);

            // When transmissing is over - switch back to RX mode
            gpio_set(LORA_RF_SW_PORT, LORA_RF_SW_PIN);
            sx126x_set_rx_with_timeout_in_rtc_step(NULL, SX126X_RX_CONTINUOUS);
        }

        if (ulNotifiedValue & NOTIF_TRANSMIT) {
            sx126x_write_buffer(NULL, 0, tx_buffer, tx_buffer_size);

            gpio_clear(LED_TXD_PORT, LED_TXD_PIN);

            gpio_clear(LORA_RF_SW_PORT, LORA_RF_SW_PIN);
            sx126x_set_tx(NULL, 0);

            if (handler != NULL && handler->packet_transmitted != NULL) {
                handler->packet_transmitted();
            }
        }

        if (ulNotifiedValue & NOTIF_SET_LORA_PARAMS) {
            sx126x_set_standby(NULL, SX126X_STANDBY_CFG_RC);
            sx126x_set_lora_mod_params(NULL, &lora_mod_params);
            sx126x_set_rx_with_timeout_in_rtc_step(NULL, SX126X_RX_CONTINUOUS);
        }

    }

}

void radio_init(radio_handler_t* h)
{
    // SX1262 DIO1 interrupt
    exti_select_source(LORA_DIO1_EXTI0, LORA_DIO1_PORT);
    exti_set_trigger(LORA_DIO1_EXTI0, EXTI_TRIGGER_RISING);
    exti_enable_request(LORA_DIO1_EXTI0);
    nvic_set_priority(LORA_DIO1_IRQ, LORA_IRQ_PRIORITY);
    nvic_enable_irq(LORA_DIO1_IRQ);

    // SPI communication with SX126X
    rcc_periph_clock_enable(LORA_RCC_SPI);
    gpio_set_mode(GPIOB,
                  GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  LORA_SPI_NSS | LORA_SPI_SCK | LORA_SPI_MOSI);

    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, LORA_SPI_MISO);

    spi_init_master(LORA_SPI,
                    SPI_CR1_BAUDRATE_FPCLK_DIV_128,
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_DFF_8BIT,
                    SPI_CR1_MSBFIRST);

    spi_disable_software_slave_management(LORA_SPI);
    spi_enable_ss_output(LORA_SPI);

    tx_buffer_size = 0;
    transmitting = false;

    handler = h;

    // Turn LEDs off
    gpio_set(LED_TXD_PORT, LED_TXD_PIN);
    gpio_set(LED_RXD_PORT, LED_RXD_PIN);

    xTaskCreate(radio_isr_task, "RADIO_ISR", 100, NULL, configMAX_PRIORITIES - 1, &xRadioIsrTask);
}

void radio_set_lora_params(uint8_t sf, uint8_t bw, uint8_t cr)
{
    lora_mod_params.sf = sf;
    lora_mod_params.bw = bw;
    lora_mod_params.cr = cr;

    xTaskNotify(xRadioIsrTask, NOTIF_SET_LORA_PARAMS, eSetBits);
}

void radio_set_rx_params(uint8_t sf, uint8_t bw, uint8_t cr)
{

}

bool radio_is_transmitting()
{
    return transmitting;
}

void radio_transmit(const uint8_t* payload, size_t payload_size)
{
    if (transmitting) {
        // Already transmitting
        DBG("Already transmitting\n");
    } else {
        DBG("Transmitting ");
        DBG_I(payload_size);
        DBG(" bytes\n");
        transmitting = true;
        memcpy(tx_buffer, payload, payload_size);
        xTaskNotify(xRadioIsrTask, NOTIF_TRANSMIT, eSetBits);
    }
}
