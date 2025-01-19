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
