# Waveshare usb-lora custom firmware
Custom firmware to send and receive LoRa messages (particularly Meshtastic) using [Waveshare USB-to-LoRa module](https://github.com/Archie3d/waveshare-usb-lora).

## Programming
To flash the device with this firmware install [the bootloader](https://github.com/Archie3d/waveshare-usb-lora-bootloader) first. Then use [the CLI programmer](https://github.com/Archie3d/waveshare-usb-lora-programmer) to write the firmware.

## Communication interface
Once programmed, the module can be accessed via serial interface via the API described below.
The serial port must be configured as following:
| Parameter       | Value       |
|:----------------|:------------|
| Transfer rate   | 115200 baud |
| Data bits       | 8           |
| Stop bits       | 1           |
| Parity          | OFF         |
| HW flow control | OFF         |

## API

Binary communication protocol is used to talk to the device. Format of the messages is the same for both device to host and host to device communication.

All numbers are encoded as **little endian**, meaning that the least significant byte (LSB) is transmitted firts, and the most significant byte (MSB) is transmitted last.

### Binary message format
A message has the following format:
| # | Field   | Size | Value                                           |
|:--|:--------|:-----|:------------------------------------------------|
| 0 | 0xAA    | 1    | Message start byte                              |
| 1 | CMD     | 1    | Command/type                                    |
| 2 | SIZE    | 2    | Payload size in bytes as uint16                 |
| 3 | PAYLOAD | SIZE | Payload, message dependent                      |
| 4 | CRC16   | 2    | CRC as uint16 calculated over the fields [1..3] |

### Escaping

`0xAA` value is used to detect a message start in a data stream. This means that this value cannot be used anywhere else. For this reason all the fields of the message except the start byte are escaped.

The escape substitution is the following:
| Original byte | Escape sequence |
|:--------------|:----------------|
| `0xAA`        | `0x7D` `0x8A`   |
| `0x7D`        | `0x7D` `0x5D`   |

### CRC calculation

> :warning: CRC is calculated on **unescaped** data.

CRC is calculated on the entire unescaped message excluding the start byte (and the CRC itself, obviously).

The CRC-16 algorithm is the following:

#### CRC-16 in C
```c
uint16_t crc16(const uint8_t* data, size_t length)
{
    uint16_t crc = 0;

    for (size_t i = 0; i < length; i++) {
        uint16_t a = (crc >> 8) ^ data[i];
        crc = (a << 2) ^ (a << 1) ^ a ^ (crc << 8);
    }

    return crc;
}
```

#### CRC-16 in Python
```python
def crc16(data:bytes):
    crc = 0

    for b in data:
        a = (crc >> 8) ^ b
        crc = (a << 2) ^ (a << 1) ^ a ^ (crc << 8)
        crc = crc & 0xFFFF

    return crc
```

### Messages
----
#### :envelope: Get firmware version

Firmware version is encoded with three values: `major.minor.patch`, for example: `1.0.0`.
##### Request from host to device

| Field | Size | Value |
|:------|:-----|:------|
| CMD   | 1    | 0x01  |
| SIZE  | 2    | 0     |

##### Response from device to host

| Field | Size | Value                |
|:------|:-----|:---------------------|
| CMD   | 1    | 0x81                 |
| SIZE  | 2    | 3                    |
| major | 1    | major version number |
| minor | 1    | minor version number |
| patch | 1    | patch level          |

----
#### :envelope: Set LoRa parameters

##### Request from host to device

| Field            | Size | Value                                                   |
|:-----------------|:-----|:--------------------------------------------------------|
| CMD              | 1    | 0x02                                                    |
| SIZE             | 2    | 4                                                       |
| Spreading factor | 1    |                                                         |
| Bandwidth        | 1    |                                                         |
| Coding rate      | 1    |                                                         |
| Low data rate    | 1    | `0x01` for low data rate optimization, `0x00` otherwise |

The values in these fields are encoded accordingly to the SX1262 chip specs.
If values passed are outside of the ranges below, the change will be ignored.

###### Spreading factor
```
SX126X_LORA_SF5  = 0x05
SX126X_LORA_SF6  = 0x06
SX126X_LORA_SF7  = 0x07
SX126X_LORA_SF8  = 0x08
SX126X_LORA_SF9  = 0x09
SX126X_LORA_SF10 = 0x0A
SX126X_LORA_SF11 = 0x0B
SX126X_LORA_SF12 = 0x0C
```

###### Bandwidth
```
SX126X_LORA_BW_500 = 6
SX126X_LORA_BW_250 = 5
SX126X_LORA_BW_125 = 4
SX126X_LORA_BW_062 = 3
SX126X_LORA_BW_041 = 10
SX126X_LORA_BW_031 = 2
SX126X_LORA_BW_020 = 9
SX126X_LORA_BW_015 = 1
SX126X_LORA_BW_010 = 8
SX126X_LORA_BW_007 = 0
```

###### Coding rate
```
SX126X_LORA_CR_4_5 = 0x01
SX126X_LORA_CR_4_6 = 0x02
SX126X_LORA_CR_4_7 = 0x03
SX126X_LORA_CR_4_8 = 0x04
```
##### Response from device to host
The response includes the values as they've been set in the PX1262 chip.

| Field            | Size | Value |
|:-----------------|:-----|:------|
| CMD              | 1    | 0x02  |
| SIZE             | 2    | 3     |
| Spreading factor | 1    |       |
| Bandwidth        | 1    |       |
| Coding rate      | 1    |       |

----
#### :envelope: Set LoRa packet paremeters

##### Request from host to device

| Field           | Size | Value                                    |
|:----------------|:-----|:-----------------------------------------|
| CMD             | 1    | 0x03                                     |
| SIZE            | 1    | 6                                        |
| Preamble length | 2    | Preamble length as `uint16`              |
| Header type     | 1    | `0x00` for explicit, `0x01` for implicit |
| Sync word       | 1    |                                          |
| CRC is on       | 1    | `0x00` for OFF, `0x01` for CR ON         |
| Invert IQ       | 1    | `0x00` for OFF, `0x0` for ON             |

For Meshtastic messaging the parameters must be as following:

| Preamble | Header type | Sync word | CRC  | Invert IQ |
|:---------|:------------|:----------|:---- |:----------|
| 16       | 0x00        | 0x2B      | 0x01 | 0x00      |

##### Response from device to host

| Field           | Size | Value                                    |
|:----------------|:-----|:-----------------------------------------|
| CMD             | 1    | 0x83                                     |
| SIZE            | 1    | 6                                        |
| Preamble length | 2    | Preamble length as `uint16`              |
| Header type     | 1    | `0x00` for explicit, `0x01` for implicit |
| Sync word       | 1    |                                          |
| CRC is on       | 1    | `0x00` for OFF, `0x01` for CR ON         |
| Invert IQ       | 1    | `0x00` for OFF, `0x01` for ON            |

----
#### :envelope: Set reception parameters

##### Request from host to device

| Field      | Size | Value            |
|:-----------|:-----|:-----------------|
| CMD        | 1    | 0x04             |
| SIZE       | 1    | 1                |
| RX boosted | 1    | `0x00` or `0x01` |

Set `RX boosted` to `0x00` to disable RX boot, or to non-zero value to enable it.

##### Response from device to host

| Field      | Size | Value            |
|:-----------|:-----|:-----------------|
| CMD        | 1    | 0x84             |
| SIZE       | 1    | 1                |
| RX boosted | 1    | `0x00` or `0x01` |

----
#### :envelope: Set transmission parameters

##### Request from host to device

| Field        | Size | Value                                    |
|:-------------|:-----|:-----------------------------------------|
| CMD          | 1    | 0x05                                     |
| SIZE         | 1    | 4                                        |
| paDutyCycle  | 1    |                                          |
| hpMax        | 1    | Between `0x00` and `0x07`                |
| power        | 1    | Output power in dBm as `int8`            |
| ramp time    | 1    |                                          |

The values must follow the SX1262 chip specs.
> `paDutyCycle` must not be higher than `0x04`.

> `hpMax` must be set to `0x07` to achieve the maximum +22dBm output power.

Example values for SX1262 chip:
| paDutyCycle | hpMax | Output power |
|:------------|:------|:-------------|
| 0x04        | 0x07  | +22 dBm      |
| 0x03        | 0x05  | +20 dBm      |
| 0x02        | 0x03  | +17 dBm      |
| 0x02        | 0x02  | +14 dBm      |

The output power is defined as power in dBm in a range of
- -17 (`0xEF`) to +14 (`0x0E`) dBm by step of 1 dB if low power PA is selected
- -9 (`0xF7`) to +22 (`0x16`) dBm by step of 1 dB if high power PA is selected

| Ramp Time | RampTime (μs) |
|:----------|:--------------|
| 0x00      | 10            |
| 0x01      | 20            |
| 0x02      | 40            |
| 0x03      | 80            |
| 0x04      | 200           |
| 0x05      | 800           |
| 0x06      | 1700          |
| 0x07      | 3400          |

##### Response from device to host

| Field               | Size | Value            |
|:--------------------|:-----|:-----------------|
| CMD                 | 1    | 0x85             |
| SIZE                | 1    | 4                |
| paDutyCycle         | 1    |                  |
| hpMax               | 1    |                  |
| power               | 1    |                  |
| ramp time           | 1    |                  |
| Switch to RX        | 1    | `0x00` or `0x01` |

----
#### :envelope: Set radio frequency

##### Request from host to device

| Field      | Size | Value                        |
|:-----------|:-----|:-----------------------------|
| CMD        | 1    | 0x06                         |
| SIZE       | 1    | 4                            |
| Frequency  | 4    | Frequency in Mhz as `uint32` |

##### Response from device to host

| Field      | Size | Value                        |
|:-----------|:-----|:-----------------------------|
| CMD        | 1    | 0x86                         |
| SIZE       | 1    | 4                            |
| Frequency  | 4    | Frequency in Mhz as `uint32` |

----
#### :envelope: Set RX/TX fallback mode

##### Request from host to device

| Field         | Size | Value                |
|:--------------|:-----|:---------------------|
| CMD           | 1    | 0x07                 |
| SIZE          | 1    | 1                    |
| Fallback mode | 1    | SX1262 fallback mode |

##### Response from device to host

| Field         | Size | Value                |
|:--------------|:-----|:---------------------|
| CMD           | 1    | 0x87                 |
| SIZE          | 1    | 1                    |
| Fallback mode | 1    | SX1262 fallback mode |

Fallback mode is defined in SX1262 specs as following:

| Fallback Mode | Value | Description                                        |
|:--------------|:------|:---------------------------------------------------|
| FS            | 0x40  | The radio goes into FS mode after Tx or Rx         |
| STDBY_XOSC    | 0x30  | The radio goes into STDBY_XOSC mode after Tx or Rx |
| STDBY_RC      | 0x20  | The radio goes into STDBY_RC mode after Tx or Rx   |
| STDBY_XOSC_RX | 0x31  | The same as STDBY_XOSC with continuous RX after TX |

> `STDBY_XOSC_RX` Is a custom mode that gets translated to `STDBT_XOSC` for the SX1262 chip. This mode will switch the radio into conrinuous RX mode after the TX is done (time-out or packet received).

----
#### :envelope: Get instantaneous RSSI

Received Signal Strength Indicator is estimated when the device is in RX mode.

##### Request from host to device
| Field      | Size | Value |
|:-----------|:-----|:------|
| CMD        | 1    | 0x08  |
| SIZE       | 1    | 0    |

##### Response from device to host
| Field      | Size | Value                  |
|:-----------|:-----|:-----------------------|
| CMD        | 1    | 0x88                   |
| SIZE       | 1    | 2                      |
| RSSI       | 2    | RSSI in dBm as `int16` |

----
#### Receive
Put device in RX mode.

##### Request from host to device
| Field      | Size | Value                                      |
|:-----------|:-----|:-------------------------------------------|
| CMD        | 1    | 0x09                                       |
| SIZE       | 1    | 5                                          |
| Timeout    | 4    | Timeout in ms as `uint32`                  |
| RSSI       | 1    | `0x01` to enable continuous RSSI reporting |

Setting timeout value as `0x00FFFFFF` (or larger) will put the device in continuous receive mode.

> Enabling the RSSI will make the device to continuously report the RSSI while it is in RX mode.

##### Response from device to host
| Field      | Size | Value                               |
|:-----------|:-----|:------------------------------------|
| CMD        | 1    | 0x89                                |
| SIZE       | 1    | 5                                   |
| Timeout    | 4    | Timeout in ms as `uint32`           |
| RSSI       | 1    | `0x01` if RSSI reporting is enabled |

The response message is sent immediately. The received data will follow as an unsolicited message (packet received or timeout).

When the device is in RX mode, it will send unsolicited messages to the host on packet received or timeout.

> When a finite timeout is set (device is not in continuous RX mode), the device will switch to standby mode immediately after the timeout or a packet being received.

----
#### :envelope: Transmit
Put device in TX mode and send the data.

##### Request from host to device
| Field        | Size          | Value                                    |
|:-------------|:--------------|:-----------------------------------------|
| CMD          | 1             | 0x0A                                     |
| SIZE         | data size + 4 | Data size + 1                            |
| Timeout      | 4             | Timeout in ms as `uint32`                |
| Data         | SIZE - 4      | Data to be transmitted                   |

Using `0` value for the timeout will disable the timeout.

##### Response from device to host
| Field      | Size | Value                                            |
|:-----------|:-----|:-------------------------------------------------|
| CMD        | 1    | 0x8A                                             |
| SIZE       | 1    | 1                                                |
| TX status  | 1    | `0x00` if transmission schedules, `0x01` is busy |

The response message will be sent immediately. Unsolisited message (packet transmitted or timeout) will follow.

> If device has not finished a previous transmission a busy status `0x01` will be returned, and no data will not be transmitted.

----
#### :envelope: Standby
Put device in a standby mode

##### Request from host to device
| Field        | Size | Value                                      |
|:-------------|:-----|:-------------------------------------------|
| CMD          | 1    | 0x0B                                       |
| SIZE         | 1    | 1                                          |
| Standby mode | 1    | `0x00` for STDBY_RC, `0x01` for STDBY_XOSC |

##### Response from device to host
| Field        | Size | Value                                      |
|:-------------|:-----|:-------------------------------------------|
| CMD          | 1    | 0x8B                                       |
| SIZE         | 1    | 1                                          |
| Standby mode | 1    | `0x00` for STDBY_RC, `0x01` for STDBY_XOSC |

----
#### Unsolicited messages from device to host

##### :envelope: Timeout
Send by the device on receive or transmit timeout.

| Field      | Size | Value |
|:-----------|:-----|:------|
| CMD        | 1    | 0x90  |
| SIZE       | 1    | 0     |

##### :envelope: Packet received

| Field       | Size     | Value                                          |
|:------------|:---------|:-----------------------------------------------|
| CMD         | 1        | 0x91                                           |
| SIZE        | 1        | 3 + received data length                       |
| Packet RSSI | 1        | RSSI of the last packet in dBm as `int8`       |
| Packet SNR  | 1        | SNR if the last packet in dB as `int8`         |
| Signal RSSI | 1        | Signal RSSI in dBm after despreading as `int8` |
| Data        | SIZE - 3 | Received packer raw data                       |

##### :envelope: Packet transmitted

| Field       | Size | Value                               |
|:------------|:-----|:------------------------------------|
| CMD         | 1    | 0x92                                |
| SIZE        | 1    | 4                                   |
| Time on air | 4    | Transmission time in ms as `uint32` |

Depending on the parameters set in the transmit request, the device will switch to standby or continuous RX mode after the transmission.

##### :envelope: Continuous RSSI

When device is in continuous RX mode, it can continuously report the RSSI.

| Field      | Size | Value                  |
|:-----------|:-----|:-----------------------|
| CMD        | 1    | 0x93                   |
| SIZE       | 1    | 2                      |
| RSSI       | 2    | RSSI in dBm as `int16` |
