#include "cc1101.h"

CC1101::CC1101(SPIClass& spi_bus) : spi(spi_bus) 
{
}

void CC1101::begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss)
{
    csPin = ss;

    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);

    spi.begin(sck, miso, mosi, ss);

    delay(500);

    strobe(SRES); // Reset CC1101 chip

    // Configure radio
    writeReg(PKTLEN, PACKETSIZE);      // Packet length
    writeReg(PKTCTRL0, 0x00);          // No CRC
    writeReg(PKTCTRL1, 0x01);          // Address check
    writeReg(ADDR, 0xD3);              // Device address
    writeReg(FSCTRL1, 0x06);
    writeReg(FREQ2, 0x10);
    writeReg(FREQ1, 0xB0);
    writeReg(FREQ0, 0x71);
    writeReg(MDMCFG4, 0xC8);
    writeReg(MDMCFG3, 0x93);
    writeReg(DEVIATN, 0x34);
    writeReg(MCSM0, 0x18);
    writeReg(FOCCFG, 0x16);
    writeReg(AGCTRL2, 0x43);
    writeReg(FSCAL3, 0xE9);
    writeReg(FSCAL2, 0x2A);
    writeReg(FSCAL1, 0x00);
    writeReg(FSCAL0, 0x1F);
    writeReg(SYNC1, 0x2D);
    writeReg(SYNC0, 0xD4);

    strobe(SFRX); // Flush RX FIFO
    strobe(SRX);  // Enter RX mode

    // test functionality
    byte partnum = readReg(PARTNUM);
    Serial.print("CC1101 Part Number: ");
    Serial.println(partnum, HEX);
    byte version = readReg(VERSION);
    Serial.print("CC1101 Version: ");
    Serial.println(version, HEX);
}

byte* CC1101::checkFIFO()
{
    byte rxBytes = readReg(RXBYTES) & 0x7F; // mask off overflow bit
    byte marcstate = readReg(MARCSTATE); // state of the radio

    //idle after packet received
    if(marcstate == 0x01) {
        if(rxBytes > 0) {
            burstRead(RX_FIFO, buffer, rxBytes); // Read received bytes
            strobe(SFRX); // Flush RX FIFO
            strobe(SRX); // Re-enter RX mode
            return buffer; // Return the received data
        }
    }
    return nullptr;
}

void CC1101::strobe(byte cmd)
{
    digitalWrite(csPin, LOW);
    while (digitalRead(CC1101_MISO)) {} // Wait for MISO to go low (chip ready)
    spi.transfer(cmd);
    digitalWrite(csPin, HIGH);
}

void CC1101::writeReg(byte addr, byte value)
{
    digitalWrite(csPin, LOW);
    while (digitalRead(CC1101_MISO)) {}
    spi.transfer(addr);
    spi.transfer(value);
    digitalWrite(csPin, HIGH);
}

void CC1101::burstWrite(byte addr, const byte *buffer, byte count)
{
    addr |= BURST;
    digitalWrite(csPin, LOW);
    while (digitalRead(CC1101_MISO)) {}
    spi.transfer(addr);
    spi.writeBytes(buffer, count);
    digitalWrite(csPin, HIGH);
}

byte CC1101::readReg(byte addr)
{
    byte value;
    digitalWrite(csPin, LOW);
    while (digitalRead(CC1101_MISO)) {}
    spi.transfer(addr | 0xC0); //READ command, response status byte
    value = spi.transfer(0x00); //dummy byte to read value byte
    digitalWrite(csPin, HIGH);
    return value;
}

void CC1101::burstRead(byte addr, byte *buffer, byte count)
{
    addr |= READ | BURST;
    digitalWrite(csPin, LOW);
    while (digitalRead(CC1101_MISO)) {}
    spi.transfer(addr);
    for (byte i = 0; i < count; i++) {
        buffer[i] = spi.transfer(0x00);
    }
    digitalWrite(csPin, HIGH);
}
