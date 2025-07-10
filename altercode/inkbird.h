#ifndef INKBIRD_H
#define INKBIRD_H

#include <Arduino.h>

#define INKBIRD_ITH20R_CRC_POLY 0xA001
#define INKBIRD_ITH20R_CRC_INIT 0x86F4

void decodePacket(const uint8_t *packet);
uint16_t crc16lsb(const uint8_t message[], unsigned nBytes, uint16_t polynomial, uint16_t init);

#endif
