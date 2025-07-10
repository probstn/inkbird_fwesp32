#include "inkbird.h"

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