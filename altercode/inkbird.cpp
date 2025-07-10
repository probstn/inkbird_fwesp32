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

void decodePacket(const uint8_t *packet)
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

        float ext_temp = (packet[12] | (packet[13] << 8))
