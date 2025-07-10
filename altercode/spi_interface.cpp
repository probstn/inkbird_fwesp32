#include "spi_interface.h"

SPIClass hspi1(HSPI);

void beginSPI()
{
    hspi1.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);
}

void strobe(uint8_t cmd)
{
    digitalWrite(CC1101_CS, LOW);
    while (digitalRead(CC1101_MISO));
    hspi1.transfer(cmd);
    digitalWrite(CC1101_CS, HIGH);
}

void writeReg(uint8_t addr, uint8_t value)
{
    digitalWrite(CC1101_CS, LOW);
    while (digitalRead(CC1101_MISO));
    hspi1.transfer(addr);
    hspi1.transfer(value);
    digitalWrite(CC1101_CS, HIGH);
}

byte readReg(uint8_t addr)
{
    byte value;
    digitalWrite(CC1101_CS, LOW);
    while (digitalRead(CC1101_MISO));
    hspi1.transfer(addr | 0x80);
    value = hspi1.transfer(0x00);
    digitalWrite(CC1101_CS, HIGH);
    return value;
}

byte readStatus(uint8_t addr)
{
    byte value;
    digitalWrite(CC1101_CS, LOW);
    while (digitalRead(CC1101_MISO));
    hspi1.transfer(addr | 0xC0);
    value = hspi1.transfer(0x00);
    digitalWrite(CC1101_CS, HIGH);
    return value;
}

void burstWrite(uint8_t addr, uint8_t *buffer, uint8_t count)
{
    addr |= 0x40;
    digitalWrite(CC1101_CS, LOW);
    while (digitalRead(CC1101_MISO));
    hspi1.transfer(addr);
    hspi1.writeBytes(buffer, count);
    digitalWrite(CC1101_CS, HIGH);
}

void burstRead(uint8_t addr, uint8_t *buffer, uint8_t count)
{
    addr |= 0xC0;
    digitalWrite(CC1101_CS, LOW);
    while (digitalRead(CC1101_MISO));
    hspi1.transfer(addr);
    for (int i = 0; i < count; i++)
        buffer[i] = hspi1.transfer(0x00);
    digitalWrite(CC1101_CS, HIGH);
}

byte getStatusByte()
{
    byte status;
    digitalWrite(CC1101_CS, LOW);
    while (digitalRead(CC1101_MISO));
    status = hspi1.transfer(0x3D);
    digitalWrite(CC1101_CS, HIGH);
    return status;
}
