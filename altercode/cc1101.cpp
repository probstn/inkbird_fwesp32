#include "cc1101.h"
#include "spi_interface.h"

void cc1101Init()
{
    strobe(SRES);
    delay(100);

    uint8_t partNum = readReg(PARTNUM);
    uint8_t version = readReg(VERSION);

    Serial.print("STATUS: ");
    Serial.println(partNum, HEX);
    Serial.print("VERSION: ");
    Serial.println(version, HEX);

    writeReg(PKTLEN, 0x12);
    writeReg(PKTCTRL0, 0x00);
    writeReg(PKTCTRL1, 0x01);
    writeReg(ADDR, 0xD3);
    writeReg(FSCTRL1, 0x06);
    writeReg(FREQ2, 0x10);
    writeReg(FREQ1, 0xB0);
    writeReg(FREQ0, 0x71);
    writeReg(MDMCFG4, 0xC8);
    writeReg(MDMCFG3, 0x93);
    writeReg(DEVIATN, 0x34);
    writeReg(MCSM0, 0x18);
    writeReg(FOCCFG, 0x16);
    writeReg(AGCCTRL2, 0x43);
    writeReg(FSCAL3, 0xE9);
    writeReg(FSCAL2, 0x2A);
    writeReg(FSCAL1, 0x00);
    writeReg(FSCAL0, 0x1F);
    writeReg(SYNC1, 0x2D);
    writeReg(SYNC0, 0xD4);

    strobe(SFRX);
    strobe(SRX);
}
