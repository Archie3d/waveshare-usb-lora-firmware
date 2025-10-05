#include "radio.h"
#include "pinout.h"

#include "sx126x/sx126x.h"

#include <FreeRTOS.h>
#include <task.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

typedef enum {
    // IRQ notifications
    NOTIF_IRQ_RX_DONE = 0x0001,
    NOTIF_IRQ_TX_DONE = 0x0002,
    NOTIF_IRQ_TIMEOUT = 0x0004,

    // Requests processing notifications
    NOTIF_SET_LORA_PARAMS = 0x000100,
    NOTIF_SET_LORA_PACKET = 0x000200,
    NOTIF_SET_RX_PARAMS = 0x000400,
    NOTIF_SET_TX_PARAMS = 0x000800,
    NOTIF_SET_FREQUENCY = 0x001000,
    NOTIF_SET_FALLBACK_MODE = 0x002000,
    NOTIF_SET_RX = 0x004000,
    NOTIF_SET_TX = 0x008000,
    NOTIF_SET_STANDBY = 0x010000,
} isr_notification_t;

// Expected output power = +14dBm
static sx126x_pa_cfg_params_t pa_pwr_cfg = {
    .pa_duty_cycle = 0x2,
    .hp_max = 0x02,
    .device_sel = 0x00, // Always 0 for SX1262
    .pa_lut = 0x01,     // Always 1 for SX1262
};

static sx126x_mod_params_lora_t lora_mod_params = {
    .sf = SX126X_LORA_SF11,     // LORA_SPREADING_FACTOR
    .bw = SX126X_LORA_BW_250,   // LORA_BANDWIDTH       250 for Long fast
    .cr = SX126X_LORA_CR_4_8,   // LORA_CODING_RATE
    .ldro = 0,                  // Must be OFF for Meshtastic
};

static sx126x_pkt_params_lora_t lora_pkt_params = {
    .preamble_len_in_symb = 16,                 // LORA_PREAMBLE_LENGTH
    .header_type = SX126X_LORA_PKT_EXPLICIT,    // LORA_PKT_LEN_MODE
    .pld_len_in_bytes = 255,                    // PAYLOAD_LENGTH
    .crc_is_on = true,                          // LORA_CRC
    .invert_iq_is_on = false,                   // LORA_IQ
};

static uint8_t lora_sync_word = MESHTASTIC_SYNCWORD;
static uint8_t rx_boosted = 1;
static int8_t tx_power = 14;
static sx126x_ramp_time_t tx_ramp_time = SX126X_RAMP_3400_US;

static uint32_t frequency = 869525000;

// Custom fallback mode with continuous RX activation after TX
#define SX126X_FALLBACK_STDBY_XOSC_RX   0x31

static uint8_t fallback_mode = SX126X_FALLBACK_STDBY_RC;

static int16_t continuous_rssi = -180;

static uint32_t rx_timeout = SX126X_RX_CONTINUOUS;
static uint8_t rx_report_rssi = 0;

static uint32_t tx_timeout = 0;

static sx126x_standby_cfg_t standby_mode = SX126X_STANDBY_CFG_RC;

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

static void set_antenna_to_rx()
{
    gpio_set(LORA_RF_SW_PORT, LORA_RF_SW_PIN);
}

static void set_antenna_to_tx()
{
    gpio_clear(LORA_RF_SW_PORT, LORA_RF_SW_PIN);
}

static void update_leds(sx126x_chip_modes_t mode)
{
    if (mode == SX126X_CHIP_MODE_RX) {
        gpio_clear(LED_RXD_PORT, LED_RXD_PIN);
        gpio_set(LED_TXD_PORT, LED_TXD_PIN);
    } else if (mode == SX126X_CHIP_MODE_TX) {
        gpio_set(LED_RXD_PORT, LED_RXD_PIN);
        gpio_clear(LED_TXD_PORT, LED_TXD_PIN);
    } else {
        gpio_set(LED_RXD_PORT, LED_RXD_PIN);
        gpio_set(LED_TXD_PORT, LED_TXD_PIN);
    }
}

static radio_handler_t* handler = NULL;
static TaskHandle_t xRadioIsrTask;

static uint8_t tx_buffer[255];
static uint8_t tx_buffer_size = 0;
static bool transmitting = false;

static void log_message(const char* str) {
    if (handler != NULL && handler->logging != NULL)
        handler->logging(str);
}

void exti0_isr(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    exti_reset_request(EXTI0);

    sx126x_irq_mask_t irq_mask;
    sx126x_get_and_clear_irq_status(NULL, &irq_mask);

    uint32_t notif = 0;

    if ((irq_mask & SX126X_IRQ_RX_DONE) == SX126X_IRQ_RX_DONE) {
        notif |= NOTIF_IRQ_RX_DONE;
    }

    if ((irq_mask & SX126X_IRQ_TX_DONE) == SX126X_IRQ_TX_DONE) {
        notif = NOTIF_IRQ_TX_DONE;
    }

    if ((irq_mask & SX126X_IRQ_TIMEOUT) == SX126X_IRQ_TIMEOUT) {
        notif |= NOTIF_IRQ_TIMEOUT;
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
    sx126x_set_rf_freq(NULL, frequency);

    sx126x_set_pa_cfg(NULL, &pa_pwr_cfg);
    sx126x_set_tx_params(NULL, tx_power, tx_ramp_time);

    sx126x_set_rx_tx_fallback_mode(NULL, fallback_mode & 0xF0);
    sx126x_cfg_rx_boosted(NULL, rx_boosted);

    sx126x_set_lora_mod_params(NULL, &lora_mod_params);
    sx126x_set_lora_pkt_params(NULL, &lora_pkt_params);
    sx126x_set_lora_sync_word(NULL, lora_sync_word);

    sx126x_set_dio_irq_params(
        NULL,
        SX126X_IRQ_ALL,
        SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT,
        SX126X_IRQ_NONE,
        SX126X_IRQ_NONE);

    sx126x_clear_irq_status(NULL, SX126X_IRQ_ALL);
    sx126x_set_dio3_as_tcxo_ctrl(NULL, SX126X_TCXO_CTRL_1_7V, 5000);

    // Switch antenna to RX
    set_antenna_to_rx();

    // Initialize in standby mode
    sx126x_set_standby(NULL, standby_mode);

    for (;;) {
        sx126x_chip_status_t chip_status;
        sx126x_get_status(NULL, &chip_status);

        update_leds(chip_status.chip_mode);

        // Measure RSSI in RX mode
        if (chip_status.chip_mode == SX126X_CHIP_MODE_RX) {
            sx126x_get_rssi_inst(NULL, &continuous_rssi);

            // Report RSSI via callback
            if ((rx_report_rssi != 0 || fallback_mode == SX126X_FALLBACK_STDBY_XOSC_RX) &&
                handler != NULL &&
                handler->reported_rssi != NULL)
            {
                handler->reported_rssi(continuous_rssi);
            }
        }

        // Wait for notifications
        if (xTaskNotifyWait(0,                // bits to clear on entry
                            0xFFFFFFFF,       // bits to clear on exit
                            &ulNotifiedValue, // Notified value pass out in
                            pdMS_TO_TICKS(100)) == pdFALSE)
        {
            continue;
        }

        if (ulNotifiedValue & NOTIF_IRQ_RX_DONE) {
            sx126x_rx_buffer_status_t rx_buffer_status;
            sx126x_pkt_status_lora_t pkt_status;

            sx126x_get_rx_buffer_status(NULL, &rx_buffer_status);

            if (rx_buffer_status.pld_len_in_bytes > 0) {
                sx126x_get_lora_pkt_status(NULL, &pkt_status);

                sx126x_read_buffer(NULL, rx_buffer_status.buffer_start_pointer, rx_buffer, rx_buffer_status.pld_len_in_bytes);
                rx_buffer_size = rx_buffer_status.pld_len_in_bytes;

                // Call handler with the received message
                if (handler != NULL && handler->packet_received != NULL) {
                    handler->packet_received(pkt_status.rssi_pkt_in_dbm,
                                             pkt_status.snr_pkt_in_db,
                                             pkt_status.signal_rssi_pkt_in_dbm,
                                             rx_buffer,
                                             rx_buffer_size);
                }
            }

        }

        if (ulNotifiedValue & NOTIF_IRQ_TX_DONE) {
            transmitting = false;

            // Switch antenna back to RX
            set_antenna_to_rx();

            if (fallback_mode == SX126X_FALLBACK_STDBY_XOSC_RX) {
                // When transmissing is over - switch back to RX mode if enabled in fallback mode
                sx126x_set_rx_with_timeout_in_rtc_step(NULL, SX126X_RX_CONTINUOUS);
            }

            if (handler != NULL && handler->packet_transmitted != NULL) {
                uint32_t time_on_air = sx126x_get_lora_time_on_air_in_ms(&lora_pkt_params, &lora_mod_params);
                handler->packet_transmitted(time_on_air);
            }
        }

        if (ulNotifiedValue & NOTIF_IRQ_TIMEOUT) {
            if (transmitting) {
                set_antenna_to_rx();

                if (fallback_mode == SX126X_FALLBACK_STDBY_XOSC_RX) {
                    // When transmission timed out - switch back to RX mode if enabled in fallback mode
                    sx126x_set_rx_with_timeout_in_rtc_step(NULL, SX126X_RX_CONTINUOUS);
                }

                transmitting = false;
            }

            if (handler != NULL && handler->timeout != NULL)
                handler->timeout();
        }

        if (ulNotifiedValue & NOTIF_SET_LORA_PARAMS) {
            sx126x_set_lora_mod_params(NULL, &lora_mod_params);
        }

        if (ulNotifiedValue & NOTIF_SET_LORA_PACKET) {
            sx126x_set_lora_pkt_params(NULL, &lora_pkt_params);
            sx126x_set_lora_sync_word(NULL, lora_sync_word);
        }

        if (ulNotifiedValue & NOTIF_SET_RX_PARAMS) {
            sx126x_cfg_rx_boosted(NULL, rx_boosted);
        }

        if (ulNotifiedValue & NOTIF_SET_TX_PARAMS) {
            sx126x_set_pa_cfg(NULL, &pa_pwr_cfg);
            sx126x_set_tx_params(NULL, tx_power, tx_ramp_time);
        }

        if (ulNotifiedValue & NOTIF_SET_FREQUENCY) {
            sx126x_set_rf_freq(NULL, frequency);
        }

        if (ulNotifiedValue & NOTIF_SET_FALLBACK_MODE) {
            sx126x_set_rx_tx_fallback_mode(NULL, fallback_mode & 0xF0);
        }

        if (ulNotifiedValue & NOTIF_SET_RX) {
	    lora_pkt_params.pld_len_in_bytes = 255;
            sx126x_set_lora_pkt_params(NULL, &lora_pkt_params);

            if (rx_timeout >= SX126X_RX_CONTINUOUS) {
                sx126x_set_rx_with_timeout_in_rtc_step(NULL, SX126X_RX_CONTINUOUS);
            } else {
                sx126x_set_rx(NULL, rx_timeout);
            }

            set_antenna_to_rx();
        }

        if (ulNotifiedValue & NOTIF_SET_TX) {
            sx126x_write_buffer(NULL, 0, tx_buffer, tx_buffer_size);

	    lora_pkt_params.pld_len_in_bytes = tx_buffer_size;
            sx126x_set_lora_pkt_params(NULL, &lora_pkt_params);

            set_antenna_to_tx();
            sx126x_set_tx(NULL, tx_timeout);
        }

        if (ulNotifiedValue & NOTIF_SET_STANDBY) {
            sx126x_set_standby(NULL, standby_mode);
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

void radio_get_lora_params(radio_lora_params_t* params)
{
    if (params == NULL)
        return;

    params->spreading_factor = lora_mod_params.sf;
    params->bandwidth = lora_mod_params.bw;
    params->coding_rate = lora_mod_params.cr;
    params->low_data_rate = lora_mod_params.ldro;
}

void radio_set_lora_params(const radio_lora_params_t* params)
{
    if (params == NULL)
        return;

    if (params->spreading_factor >= SX126X_LORA_SF5 && params->spreading_factor <= SX126X_LORA_SF12)
        lora_mod_params.sf = params->spreading_factor;

    if (params->bandwidth >= SX126X_LORA_BW_007 &&
        params->bandwidth <= SX126X_LORA_BW_041 &&
        params->bandwidth != 0x07) {
            lora_mod_params.bw = params->bandwidth;
        }

    if (params->coding_rate >= SX126X_LORA_CR_4_5 && params->coding_rate <= SX126X_LORA_CR_4_8)
        lora_mod_params.cr = params->coding_rate;

    lora_mod_params.ldro = params->low_data_rate;

    xTaskNotify(xRadioIsrTask, NOTIF_SET_LORA_PARAMS, eSetBits);
}

void radio_get_lora_packet(radio_lora_packet_t* packet)
{
    if (packet == NULL)
        return;

    packet->preamble = lora_pkt_params.preamble_len_in_symb;
    packet->header = lora_pkt_params.header_type;
    packet->sync_word = lora_sync_word;
    packet->crc_on = lora_pkt_params.crc_is_on;
    packet->invert_iq = lora_pkt_params.invert_iq_is_on;
}

void radio_set_lora_packet(const radio_lora_packet_t* packet)
{
    if (packet == NULL)
        return;

    lora_pkt_params.preamble_len_in_symb = packet->preamble;

    if (packet->header == SX126X_LORA_PKT_EXPLICIT || packet->header == SX126X_LORA_PKT_IMPLICIT)
        lora_pkt_params.header_type = packet->header;

    lora_sync_word = packet->sync_word;
    lora_pkt_params.crc_is_on = packet->crc_on;
    lora_pkt_params.invert_iq_is_on = packet->invert_iq;

    xTaskNotify(xRadioIsrTask, NOTIF_SET_LORA_PACKET, eSetBits);
}

void radio_get_rx_params(radio_rx_params_t* params)
{
    if (params == NULL)
        return;

    params->boost = rx_boosted;
}

void radio_set_rx_params(const radio_rx_params_t* params)
{
    if (params == NULL)
        return;

    rx_boosted = params->boost;

    xTaskNotify(xRadioIsrTask, NOTIF_SET_RX_PARAMS, eSetBits);
}

void radio_get_tx_params(radio_tx_params_t* params)
{
    if (params == NULL)
        return;

    params->duty_cycle = pa_pwr_cfg.pa_duty_cycle;
    params->hp_max = pa_pwr_cfg.hp_max;
    params->power = tx_power;
    params->ramp_time = tx_ramp_time;
}

void radio_set_tx_params(const radio_tx_params_t* params)
{
    if (params == NULL)
        return;

    if (params->duty_cycle <= 0x04)
        pa_pwr_cfg.pa_duty_cycle = params->duty_cycle;

    if (params->hp_max <= 0x07)
        pa_pwr_cfg.hp_max = params->hp_max;

    if (params->power >= -17 && params->power <= 22)
        tx_power = params->power;

    if (params->ramp_time <= 0x07)
        tx_ramp_time = params->ramp_time;

    xTaskNotify(xRadioIsrTask, NOTIF_SET_TX_PARAMS, eSetBits);
}

uint32_t radio_get_frequency()
{
    return frequency;
}

void radio_set_frequency(uint32_t f)
{
    frequency = f;
    xTaskNotify(xRadioIsrTask, NOTIF_SET_FREQUENCY, eSetBits);
}

uint8_t radio_get_fallback_mode()
{
    return fallback_mode;
}

void radio_set_fallback_mode(uint8_t fbm)
{
    if (fbm == SX126X_FALLBACK_STDBY_RC ||
        fbm == SX126X_FALLBACK_STDBY_XOSC ||
        fbm == SX126X_FALLBACK_STDBY_XOSC_RX ||
        fbm == SX126X_FALLBACK_FS)
    {
        fallback_mode = fbm;
        xTaskNotify(xRadioIsrTask, NOTIF_SET_FALLBACK_MODE, eSetBits);
    }
}

int16_t radio_get_continuous_rssi()
{
    return continuous_rssi;
}

void radio_get_rx(radio_rx_t* rx)
{
    if (rx == NULL)
        return;

    rx->timeout = rx_timeout;
    rx->report_rssi = rx_report_rssi;
}

void radio_set_rx(const radio_rx_t* rx)
{
    if (rx == NULL)
        return;

    rx_timeout = rx->timeout & 0x00FFFFFF;
    rx_report_rssi = rx->report_rssi;

    xTaskNotify(xRadioIsrTask, NOTIF_SET_RX, eSetBits);
}

bool radio_is_tx_active()
{
    return transmitting;
}

void radio_set_tx(uint32_t timeout, const uint8_t* payload, size_t payload_size)
{
    if (transmitting) {
        // Already transmitting
        return;
    }

    transmitting = true;
    tx_timeout = timeout;

    tx_buffer_size = payload_size;
    if (tx_buffer_size > 255)
        tx_buffer_size = 255;

    memcpy(tx_buffer, payload, tx_buffer_size);
    xTaskNotify(xRadioIsrTask, NOTIF_SET_TX, eSetBits);
}

uint8_t radio_get_standby()
{
    return standby_mode;
}

void radio_set_standby(uint8_t mode)
{
    if (mode == SX126X_STANDBY_CFG_RC || mode == SX126X_STANDBY_CFG_XOSC)
        standby_mode = mode;

    xTaskNotify(xRadioIsrTask, NOTIF_SET_STANDBY, eSetBits);
}
