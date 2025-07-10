#ifndef SPI_INTERFACE_H
#define SPI_INTERFACE_H

#include <Arduino.h>
#include <SPI.h>

void beginSPI();
void strobe(uint8_t cmd);
void writeReg(uint8_t addr, uint8_t value);
byte readReg(uint8_t addr);
byte readStatus(uint8_t addr);
void burstWrite(uint8_t addr, uint8_t *buffer, uint8_t count);
void burstRead(uint8_t addr, uint8_t *buffer, uint8_t count);
byte getStatusByte();

#endif
