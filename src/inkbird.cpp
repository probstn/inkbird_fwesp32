#include "inkbird.h"

/**
Decoder for Inkbird ITH-20R.

https://www.ink-bird.com/products-data-logger-ith20r.html

Also: Inkbird IBS-P01R Pool Thermometer.

The compact 3-in-1 multifunction outdoor sensor transmits the data on 433.92 MHz.
The device uses FSK-PCM encoding,
The device sends a transmission every ~80 sec.

Decoding borrowed from https://groups.google.com/forum/#!topic/rtl_433/oeExmwoBI0w

- Total packet length 14563 bits:
- Preamble: aa aa aa ... aa aa (14400 on-off sync bits)
- Sync Word (16 bits): 2DD4
- Data (147 bits):
- Byte    Sample      Comment
- 0-2     D3910F      Always the same across devices, a device type?
- 3       00          00 - normal work , 40 - unlink sensor (button pressed 5s), 80 - battery replaced
- 4       01          Changes from 1 to 2 if external sensor present
- 5-6     0301        Unknown (also seen 0201), sw version? Seen 0x0001 on IBS-P01R.
- 7       58          Battery % 0-100
- 8-9     A221        Device id, always the same for a sensor but each sensor is different
- 10-11   D600        Temperature in C * 10, little endian, so 0xD200 is 210, 21.0C or 69.8F
- 12-13   F400        Temperature C * 10 for the external sensor,  0x1405 if not connected
- 14-15   D301        Relative humidity %  * 10, little endian, so 0xC501 is 453 or 45.3%
- 16-17   38FB        CRC16
- 18      0           Unknown 3 bits (seen 0 and 2)

CRC16 (bytes 0-15), without sync word):
poly=0x8005  init=0x2f61  refin=true  refout=true  xorout=0x0000  check=0x3583  residue=0x0000

To look at unknown data fields run with -vv key.

Decoder written by Dmitriy Kozyrev, 2020
*/

uint16_t crc16lsb(const uint8_t message[], unsigned nBytes, uint16_t polynomial, uint16_t init)
{
    uint16_t remainder = init;

    for (unsigned byte = 0; byte < nBytes; ++byte)
    {
        remainder ^= message[byte];
        for (int bit = 0; bit < 8; ++bit)
        {
            if (remainder & 1)
                remainder = (remainder >> 1) ^ polynomial;
            else
                remainder >>= 1;
        }
    }
    return remainder;
}

void printPacket(const uint8_t *packet)
{
    if (packet[0] == 0xD3 && packet[1] == 0x91)
    {

        Serial.print("Device Type: ");
        for (int i = 0; i < 3; i++)
            Serial.printf("%02X ", packet[i]);
        Serial.println();

        Serial.print("Status: ");
        switch (packet[3])
        {
        case 0x00:
            Serial.println("Normal");
            break;
        case 0x40:
            Serial.println("Unlink");
            break;
        case 0x80:
            Serial.println("Battery Replaced");
            break;
        default:
            Serial.printf("Unknown (%02X)\n", packet[3]);
            break;
        }

        Serial.print("Sensor Mode: ");
        Serial.println(packet[4] == 2 ? "External Present" : "Internal Only");

        Serial.printf("FW/Unknown: %02X %02X\n", packet[5], packet[6]);
        Serial.printf("Battery: %u%%\n", packet[7]);
        Serial.printf("Device ID: %02X%02X\n", packet[8], packet[9]);

        float temp = (packet[10] | (packet[11] << 8)) / 10.0;
        Serial.printf("Internal Temp: %.1f°C\n", temp);

        float ext_temp = (packet[12] | (packet[13] << 8)) / 10.0;
        Serial.printf("External Temp: %.1f°C\n", ext_temp);

        float humidity = (packet[14] | (packet[15] << 8)) / 10.0;
        Serial.printf("Humidity: %.1f%%\n", humidity);

        uint16_t crc_received = (packet[17] << 8) | packet[16];
        uint16_t crc_calc = crc16lsb(packet, 16, INKBIRD_ITH20R_CRC_POLY, INKBIRD_ITH20R_CRC_INIT);

        Serial.printf("CRC16: %04X (calculated), %04X (received) - %s\n",
                      crc_calc, crc_received,
                      crc_calc == crc_received ? "OK" : "BAD");

        Serial.printf("Final byte (unknown 3 bits): %02X\n", packet[18]);
    }
}

InkbirdData decodePacket(const uint8_t *packet)
{
    InkbirdData data = {0.0f, 0, false};

    uint16_t crc_received = (packet[17] << 8) | packet[16];
    uint16_t crc_calc = crc16lsb(packet, 16, INKBIRD_ITH20R_CRC_POLY, INKBIRD_ITH20R_CRC_INIT);

    if(crc_calc == crc_received)
    {
        data.valid = true;
        data.temperature = (packet[10] | (packet[11] << 8)) / 10.0f;
        data.battery = packet[7];
    }
    return data;
}