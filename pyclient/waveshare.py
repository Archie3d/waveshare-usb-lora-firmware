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


class Client:

    def __init__(self, port):
        self.serial = serial.Serial(port, baudrate=115200, timeout=1)

    def send_message(self, type, data):
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


if __name__ == "__main__":

    client = Client('COM4')

    client.send_message(0x01, b'')
    type, resp = client.recv_message()

    print(f"Received response of type {type:02X}, payload: {resp}")
