"""
Run (to print the captured stdout):
    pytest -rP
"""

import time
import waveshare

api = waveshare.ApiClient('COM4')

#-----------------------------------------------------------

def test_version():
    api.get_version()
    assert api.firmware_version[0] == 1
    assert api.firmware_version[1] == 0
    assert api.firmware_version[2] == 0

#-----------------------------------------------------------

def test_set_lora_parameters():
    api.set_lora_parameters(waveshare.LORA_SF8,
                            waveshare.LORA_BW_125,
                            waveshare.LORA_CR_4_5,
                            True)
    assert api.lora_spreading_factor == waveshare.LORA_SF8
    assert api.lora_bandwidth == waveshare.LORA_BW_125
    assert api.lora_coding_rate == waveshare.LORA_CR_4_5
    assert api.lora_low_data_rate == True

    api.set_lora_parameters(waveshare.LORA_SF12,
                            waveshare.LORA_BW_250,
                            waveshare.LORA_CR_4_8,
                            False)
    assert api.lora_spreading_factor == waveshare.LORA_SF12
    assert api.lora_bandwidth == waveshare.LORA_BW_250
    assert api.lora_coding_rate == waveshare.LORA_CR_4_8
    assert api.lora_low_data_rate == False

#-----------------------------------------------------------

def test_sel_lora_packet_parameters():
    api.set_lora_packet_parameters(preamble_length=8,
                                   implicit_header=True,
                                   sync_word=0x1C,
                                   crc_on=False,
                                   invert_iq=True)
    assert api.lora_preamble_length == 8
    assert api.lora_implicit_header == True
    assert api.lora_sync_word == 0x1C
    assert api.lora_crc_on == False
    assert api.lora_invert_iq == True

    api.set_lora_packet_parameters(preamble_length=16,
                                   implicit_header=False,
                                   sync_word=0x2B,
                                   crc_on=True,
                                   invert_iq=False)
    assert api.lora_preamble_length == 16
    assert api.lora_implicit_header == False
    assert api.lora_sync_word == 0x2B
    assert api.lora_crc_on == True
    assert api.lora_invert_iq == False

#-----------------------------------------------------------

def test_set_rx_parameters():
    api.set_rx_parameters(rx_boost=False)
    assert api.rx_boost == False

    api.set_rx_parameters(rx_boost=True)
    assert api.rx_boost == True

#-----------------------------------------------------------

def test_set_tx_parameters():
    api.set_tx_parameters(duty_cycle=0x03,
                          hp_max=0x05,
                          power=20,
                          ramp_time=waveshare.POWER_RAMP_800)
    assert api.duty_cycle == 0x03
    assert api.hp_max == 0x05
    assert api.power == 20
    assert api.ramp_time == waveshare.POWER_RAMP_800

    api.set_tx_parameters(duty_cycle=0x02,
                          hp_max=0x02,
                          power=14,
                          ramp_time=waveshare.POWER_RAMP_3400)
    assert api.duty_cycle == 0x02
    assert api.hp_max == 0x02
    assert api.power == 14
    assert api.ramp_time == waveshare.POWER_RAMP_3400

#-----------------------------------------------------------

def test_set_frequency():
    api.set_frequency(915000000)
    assert api.frequency == 915000000

    api.set_frequency(869525000)
    assert api.frequency == 869525000

#-----------------------------------------------------------

def test_set_fallback_mode():
    api.set_fallback_mode(waveshare.FALLBACK_FS)
    assert api.fallback_mode == waveshare.FALLBACK_FS

    api.set_fallback_mode(waveshare.FALLBACK_STANDBY_RC)
    assert api.fallback_mode == waveshare.FALLBACK_STANDBY_RC

#-----------------------------------------------------------

def test_get_rssi():
    api.get_rssi()
    # At this point the radio should be off, so the RSSI reported
    # should be low
    assert api.rssi < -80

#-----------------------------------------------------------

def test_rx():
    api.set_rx(timeout=100,
               report_rssi=False)

    assert api.rx_timeout == 100
    assert api.rx_report_rssi == False
    assert api.timeout == False

    # Wait for RX operation to complete
    time.sleep(0.200)

    # We need to perform an I/O operation to update the timeout notification
    api.get_rssi()

    assert api.timeout

#-----------------------------------------------------------

def test_tx():
    ok = api.set_tx(timeout=300,
                    data=b'\x00\x00\x00\x00')

    assert ok
    assert api.timeout == False

    time.sleep(0.500)

    api.get_rssi()

    assert api.timeout

#-----------------------------------------------------------

def test_standby():
    api.set_standby(waveshare.STANDBY_XOSC)
    assert api.standby_mode == waveshare.STANDBY_XOSC

    api.set_standby(waveshare.STANDBY_RC)
    assert api.standby_mode == waveshare.STANDBY_RC
