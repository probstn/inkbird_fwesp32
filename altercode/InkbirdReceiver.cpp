#include <Arduino.h>
#include "spi.h"


#define MAX_MESSAGE_LENGTH 1024
uint8_t ringBuffer[MAX_MESSAGE_LENGTH];
uint16_t idx = 0;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("CC1101 SPI Test");

    pinMode(CC1101_CS, OUTPUT);
    digitalWrite(CC1101_CS, HIGH);
    
    beginSPI();

    // Reset CC1101
    digitalWrite(CC1101_CS, LOW);
    strobe(SRES); // Send reset command
    digitalWrite(CC1101_CS, HIGH);

    Serial.println("CC1101 reset command sent");
    delay(100); // Wait for the reset to complete

    // Read part number
    uint8_t partNum = readReg(PARTNUM); // Read part number

    uint8_t version = readReg(VERSION); // Read version number

    Serial.print("STATUS: ");
    Serial.println(partNum, HEX);
    Serial.print("VERSION: ");
    Serial.println(version, HEX);

    //
    // Rf settings for CC1100
    //
    writeReg(PKTLEN, 0x12);   // Packet Length
    writeReg(PKTCTRL0, 0x00); //no crc
    writeReg(PKTCTRL1, 0x01); // address check, No CRC, No append status
    writeReg(ADDR, 0xD3);    // Device Address
    writeReg(FSCTRL1, 0x06);  // Frequency Synthesizer Control
    writeReg(FREQ2, 0x10);    // Frequency Control Word, High Byte
    writeReg(FREQ1, 0xB0);    // Frequency Control Word, Middle Byte
    writeReg(FREQ0, 0x71);    // Frequency Control Word, Low Byte
    writeReg(MDMCFG4, 0xC8);  // Modem Configuration
    writeReg(MDMCFG3, 0x93);  // Modem Configuration
    // writeReg(MDMCFG2, 0x00);  // Modem Configuration
    writeReg(DEVIATN, 0x34);  // Modem Deviation Setting
    writeReg(MCSM0, 0x18);    // Main Radio Control State Machine Configuration
    writeReg(FOCCFG, 0x16);   // Frequency Offset Compensation Configuration
    writeReg(AGCCTRL2, 0x43); // AGC Control
    writeReg(FSCAL3, 0xE9);   // Frequency Synthesizer Calibration
    writeReg(FSCAL2, 0x2A);   // Frequency Synthesizer Calibration
    writeReg(FSCAL1, 0x00);   // Frequency Synthesizer Calibration
    writeReg(FSCAL0, 0x1F);   // Frequency Synthesizer Calibration

    // Sync Word
    writeReg(SYNC1, 0x2D); // Sync Word, High Byte
    writeReg(SYNC0, 0xD4); // Sync Word, Low Byte

    // flush rx fifo
    strobe(SFRX);
    // strobe rx
    strobe(SRX);
}

void loop()
{
    
    /*
    Serial.print("Marcstate: ");
    Serial.print(marcstate, HEX);
    Serial.print(" RxBytes: ");
    Serial.println(rxBytes);
    */
}
