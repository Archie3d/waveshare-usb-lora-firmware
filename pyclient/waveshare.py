import serial

def crc16(crc0:int, data:bytes):
    assert type(data) is bytes
    crc = crc0

    for b in data:
        a = (crc >> 8) ^ b
        crc = (a << 2) ^ (a << 1) ^ a ^ (crc << 8)
        crc = crc & 0xFFFF

    return crc

START = 0xAA
ESCAPE = 0x7D
ESCAPE_START = 0x8A
ESCAPE_ESCAPE = 0x5D


def escape(data:bytes):
    output = b''

    for b in data:
        if b == START:
            output += ESCAPE.to_bytes(1, 'little') + ESCAPE_START.to_bytes(1, 'little')
        elif b == ESCAPE:
            output += b + ESCAPE_ESCAPE.to_bytes(1, 'little')
        else:
            output += b.to_bytes(1, 'little')

    return output


def unescape(data:bytes):
    output = b''

    escape_seq_detected = False

    for b in data:
        if escape_seq_detected:
            if b == ESCAPE_START:
                output += START.to_bytes(1, 'little')
            elif b == ESCAPE_ESCAPE:
                output += ESCAPE.to_bytes(1, 'little')
            else:
                raise ValueError(f"Unexpected escaped value {b:02x}")
            escape_seq_detected = False
        else:
            if b == ESCAPE:
                escape_seq_detected = True
            else:
                output += b.to_bytes(1, 'little')

    return output


def print_bytes(data:bytes):
    print(' '.join(f'{x:02x}' for x in data))


class SerialClient:
    """
    Serial interface client to exchange raw messages.
    This class is reponsible for the low-level message formatting,
    including CRC calcualtion and bytes escaping.
    """
    def __init__(self, port):
        self.serial = serial.Serial(port, baudrate=115200, timeout=1)

    def send_message(self, type, data=b''):
        assert type >= 0 and type < 256
        assert len(data) <= 0xFFFF

        buffer = type.to_bytes(1, 'little') + len(data).to_bytes(2, 'little') + data
        crc = crc16(0, buffer)
        buffer = START.to_bytes(1, 'little') + escape(buffer)
        buffer += crc.to_bytes(2, 'little')

        self.serial.write(buffer)

    def recv_message(self):
        start_detected = False

        while not start_detected:
            b = self.serial.read(1)
            if b[0] == START:
                start_detected = True

        # Message type
        type = self._recv_byte()

        # Payload length
        length_lsb = self._recv_byte()[0]
        length_msb = self._recv_byte()[0]
        length = (int(length_msb) << 8) + int(length_lsb)

        # Payload
        received = 0;
        payload = b''

        while received < length:
            b = self._recv_byte()
            payload = payload + b
            received += 1

        # CRC
        crc_lsb = self._recv_byte()[0]
        crc_msb = self._recv_byte()[0]
        crc = (int(crc_msb) << 8) + int(crc_lsb)

        # Check CRC
        buffer = type + length.to_bytes(2, 'little') + payload
        calculated_crc = crc16(0, buffer)

        if (crc != calculated_crc):
            raise ValueError(f"Expected CRC {calculated_crc:04x} but received {crc:04x}")

        return type[0], payload


    def _recv_byte(self):
        b = self.serial.read(1)
        if b[0] == ESCAPE:
            b = self.serial.read(1)
            if b[0] == ESCAPE_START:
                return START.to_bytes(1, 'little')
            elif b[0] == ESCAPE_ESCAPE:
                return ESCAPE.to_bytes(1, 'little')

        return b

# Messages from controller to device
MSG_GET_VERSION       = 0x01
MSG_SET_LORA_PARAMS   = 0x02
MSG_SET_LORA_PACKET   = 0x03
MSG_SET_RX_PARAMS     = 0x04
MSG_SET_TX_PARAMS     = 0x05
MSG_SET_FREQUENCY     = 0x06
MSG_SET_FALLBACK_MODE = 0x07
MSG_GET_RSSI          = 0x08
MSG_SET_RX            = 0x09
MSG_SET_TX            = 0x0A
MSG_SET_STANDBY       = 0x0B

# Response messages from device to controller
MSG_VERSION       = 0x81
MSG_LORA_PARAMS   = 0x82
MSG_LORA_PACKET   = 0x83
MSG_RX_PARAMS     = 0x84
MSG_TX_PARAMS     = 0x85
MSG_FREQUENCY     = 0x86
MSG_FALLBACK_MODE = 0x87
MSG_RSSI          = 0x88
MSG_RX            = 0x89
MSG_TX            = 0x8A
MSG_STANDBY       = 0x8B

# Unsolicited messages (from device)
MSG_TIMEOUT            = 0x90
MSG_PACKET_RECEIVED    = 0x91
MSG_PACKET_TRANSMITTED = 0x92
MSG_CONTINUOUS_RSSI    = 0x93

# LoRa spreading factors
LORA_SF5  = 0x05
LORA_SF6  = 0x06
LORA_SF7  = 0x07
LORA_SF8  = 0x08
LORA_SF9  = 0x09
LORA_SF10 = 0x0A
LORA_SF11 = 0x0B
LORA_SF12 = 0x0C

# LoRa bandwidth
LORA_BW_500 = 6
LORA_BW_250 = 5
LORA_BW_125 = 4
LORA_BW_062 = 3
LORA_BW_041 = 10
LORA_BW_031 = 2
LORA_BW_020 = 9
LORA_BW_015 = 1
LORA_BW_010 = 8
LORA_BW_007 = 0

# Coding rate
LORA_CR_4_5 = 0x01
LORA_CR_4_6 = 0x02
LORA_CR_4_7 = 0x03
LORA_CR_4_8 = 0x04


class ApiClient:

    def __init__(self, port):
        self.serial = SerialClient(port)
        self.timeout = False
        self.received_packets = []
        self.transmitting = False
        self.rssi = 0

        self.firmware_version = [0, 0, 0]
        self.lora_spreading_factor = LORA_SF12
        self.lora_bandwidth = LORA_BW_250
        self.lora_coding_rate = LORA_CR_4_8
        self.lora_low_data_rate = False
        self.lora_preamble_length = 16
        self.lora_implicit_header = False
        self.lora_sync_word = 0x2B # For Meshtastic
        self.lora_crc_on = True
        self.lora_invert_iq = False


    def wait_for_response(self, msg_type):
        type = 0x00

        while type != msg_type:
            type, resp = self.serial.recv_message()

            if type == MSG_TIMEOUT:
                self.timeout = True
            elif type == MSG_PACKET_RECEIVED:
                self.received_packets.append(resp)
            elif type == MSG_PACKET_TRANSMITTED:
                self.transmitting = False
            elif type == MSG_CONTINUOUS_RSSI:
                self.rssi = int.from_bytes(resp, byteorder='little', signed=True)

        return resp


    def get_version(self):
        self.serial.send_message(MSG_GET_VERSION)
        resp = self.wait_for_response(MSG_VERSION)
        self.firmware_version = [int(resp[0]), int(resp[1]), int(resp[2])]


    def set_lora_parameters(self,
                            spreading_factor,
                            bandwidth,
                            coding_rate,
                            low_data_rate=False):
        payload = b''
        payload += spreading_factor.to_bytes(1, 'little')
        payload += bandwidth.to_bytes(1, 'little')
        payload += coding_rate.to_bytes(1, 'little')
        payload += b'\x01' if low_data_rate else b'\x00'

        assert len(payload) == 4
        self.serial.send_message(MSG_SET_LORA_PARAMS, payload)
        resp = self.wait_for_response(MSG_LORA_PARAMS)
        assert len(resp) == 4
        self.lora_spreading_factor = int(resp[0])
        self.lora_bandwidth = int(resp[1])
        self.lora_coding_rate = int(resp[2])
        self.lora_low_data_rate = (int(resp[3]) != 0)


    def set_lora_packet_parameters(self,
                                   preamble_length,
                                   implicit_header,
                                   sync_word,
                                   crc_on,
                                   invert_iq):
        payload = b''
        payload += preamble_length.to_bytes(2, 'little')
        payload += b'\x01' if implicit_header else b'\x00'
        payload += sync_word.to_bytes(1, 'little')
        payload += b'\x01' if crc_on else b'\x00'
        payload += b'\x01' if invert_iq else b'\x00'
        assert len(payload) == 6
        self.serial.send_message(MSG_SET_LORA_PACKET, payload)
        resp = self.wait_for_response(MSG_LORA_PACKET)
        assert(len(resp) == 6)
        self.lora_preamble_length = int.from_bytes(resp[0:2], byteorder='little', signed=False)
        self.lora_implicit_header = (int(resp[2]) != 0)
        self.lora_sync_word = int(resp[3])
        self.lora_crc_on = (int(resp[4]) != 0)
        self.lora_invert_iq = (int(resp[5]) != 0)


if __name__ == "__main__":

    api = ApiClient('COM4')
    api.get_version()
    print(f"Firmware version: {api.firmware_version}")

