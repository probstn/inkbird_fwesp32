#include "spi.h"

/*
 * spi pins
 */
#define CC1101_CS 27
#define CC1101_MOSI 13
#define CC1101_MISO 12
#define CC1101_SCK 14

SPIClass hspi1(HSPI);

void beginSPI() {
    hspi1.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);
}
void strobe(uint8_t cmd)
{
    digitalWrite(CC1101_CS, LOW);
    while (digitalRead(CC1101_MISO))
        ;
    hspi1.transfer(cmd); // Send command strobe
    digitalWrite(CC1101_CS, HIGH);
}
void writeReg(uint8_t addr, uint8_t value)
{
    digitalWrite(CC1101_CS, LOW);
    while (digitalRead(CC1101_MISO))
        ;
    hspi1.transfer(addr);  // Write command
    hspi1.transfer(value); // Write value
    digitalWrite(CC1101_CS, HIGH);
}
byte readReg(uint8_t addr)
{
    byte value;
    digitalWrite(CC1101_CS, LOW);
    while (digitalRead(CC1101_MISO))
        ;
    hspi1.transfer(addr | READ);  // Read command
    value = hspi1.transfer(0x00); // Dummy byte to read the value
    digitalWrite(CC1101_CS, HIGH);
    return value;
}
byte readStatus(uint8_t addr)
{
    byte value;
    digitalWrite(CC1101_CS, LOW);
    while (digitalRead(CC1101_MISO))
        ;
    hspi1.transfer(addr | READ | BURST); // Read status command
    value = hspi1.transfer(0x00);        // Dummy byte to read the value
    digitalWrite(CC1101_CS, HIGH);
    return value;
}
void burstWrite(uint8_t addr, uint8_t *buffer, uint8_t count)
{
    addr |= BURST; // Set burst mode
    digitalWrite(CC1101_CS, LOW);
    while (digitalRead(CC1101_MISO))
        ;
    hspi1.transfer(addr);            // Write command
    hspi1.writeBytes(buffer, count); // Write buffer
    digitalWrite(CC1101_CS, HIGH);
}
void burstRead(uint8_t addr, uint8_t *buffer, uint8_t count)
{
    addr |= READ | BURST; // Set read and burst mode
    digitalWrite(CC1101_CS, LOW);
    while (digitalRead(CC1101_MISO))
        ;
    hspi1.transfer(addr); // Read command
    for (int i = 0; i < count; i++)
    {
        buffer[i] = hspi1.transfer(0x00); // Read bytes
    }
    digitalWrite(CC1101_CS, HIGH);
}
byte getStatusByte()
{
    byte status;
    digitalWrite(CC1101_CS, LOW);
    while (digitalRead(CC1101_MISO))
        ;
    status = hspi1.transfer(SNOP); // Send no operation command to read status
    digitalWrite(CC1101_CS, HIGH);
    return status;
}