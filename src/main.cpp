#include <Arduino.h>
#include <SPI.h>

#define F_XOSC 26e6 // Crystal frequency
#define READ 0x80
#define WRITE 0x00
#define BURST 0x40
#define TX_FIFO 0x3F // Single byte access to TX FIFO
#define RX_FIFO 0xBF // Single byte access to RX FIFO
/*
 * command strobes
 */
#define SRES 0x30    // Reset chip.
#define SFSTXON 0x31 // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). If in RX (with CCA): Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround).
#define SXOFF 0x32   // Turn off crystal oscillator.
#define SCAL 0x33    // Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
#define SRX 0x34     // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.
#define STX 0x35     // In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled: Only go to TX if channel is clear.
#define SIDLE 0x36   // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable.
#define SWOR 0x38    // Start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if WORCTRL.RC_PD=0.
#define SPWD 0x39    // Enter power down mode when CSn goes high.
#define SFRX 0x3A    // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states.
#define SFTX 0x3B    // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states.
#define SWORRST 0x3C // Reset real time clock to Event1 value.
#define SNOP 0x3D    // No operation. May be used to get access to the chip status byte.

/*
 * configuration registers
 */
#define IOCFG2 0x00   // GDO2 output pin configuration
#define IOCFG1 0x01   // GDO1 output pin configuration
#define IOCFG0 0x02   // GDO0 output pin configuration
#define FIFOTHR 0x03  // RX FIFO and TX FIFO thresholds
#define SYNC1 0x04    // Sync word, high byte
#define SYNC0 0x05    // Sync word, low byte
#define PKTLEN 0x06   // Packet length
#define PKTCTRL1 0x07 // Packet automation control
#define PKTCTRL0 0x08 // Packet automation control
#define ADDR 0x09     // Device address
#define CHANNR 0x0A   // Channel number
#define FSCTRL1 0x0B  // Frequency synthesizer control
#define FSCTRL0 0x0C  // Frequency synthesizer control
#define FREQ2 0x0D    // Frequency control word, high byte
#define FREQ1 0x0E    // Frequency control word, middle byte
#define FREQ0 0x0F    // Frequency control word, low byte
#define MDMCFG4 0x10  // Modem configuration
#define MDMCFG3 0x11  // Modem configuration
#define MDMCFG2 0x12  // Modem configuration
#define MDMCFG1 0x13  // Modem configuration
#define MDMCFG0 0x14  // Modem configuration
#define DEVIATN 0x15  // Modem deviation setting
#define MCSM2 0x16    // Main Radio Control State Machine configuration
#define MCSM1 0x17    // Main Radio Control State Machine configuration
#define MCSM0 0x18    // Main Radio Control State Machine configuration
#define FOCCFG 0x19   // Frequency Offset Compensation configuration
#define BSCFG 0x1A    // Bit Synchronization configuration
#define AGCTRL2 0x1B  // AGC control
#define AGCTRL1 0x1C  // AGC control
#define AGCTRL0 0x1D  // AGC control
#define WOREVT1 0x1E  // High byte Event 0 timeout
#define WOREVT0 0x1F  // Low byte Event 0 timeout
#define WORCTRL 0x20  // Wake On Radio control
#define FREND1 0x21   // Front end RX configuration
#define FREND0 0x22   // Front end TX configuration
#define FSCAL3 0x23   // Frequency synthesizer calibration
#define FSCAL2 0x24   // Frequency synthesizer calibration
#define FSCAL1 0x25   // Frequency synthesizer calibration
#define FSCAL0 0x26   // Frequency synthesizer calibration
#define RCCTRL1 0x27  // RC oscillator configuration
#define RCCTRL0 0x28  // RC oscillator configuration
#define FSTEST 0x29   // Frequency synthesizer calibration control
#define PTEST 0x2A    // Production test
#define AGCTEST 0x2B  // AGC test
#define TEST2 0x2C    // Various test settings
#define TEST1 0x2D    // Various test settings
#define TEST0 0x2E    // Various test settings

/*
 * status registers
 */
#define PARTNUM 0x30        // (0xF0) Part number for CC1101 92
#define VERSION 0x31        // (0xF1) Current version number 92
#define FREQEST 0x32        // (0xF2) Frequency Offset Estimate 92
#define LQI 0x33            // (0xF3) Demodulator estimate for Link Quality 92
#define RSSI 0x34           // (0xF4) Received signal strength indication 92
#define MARCSTATE 0x35      // (0xF5) Control state machine state 93
#define WORTIME1 0x36       // (0xF6) High byte of WOR timer 93
#define WORTIME0 0x37       // (0xF7) Low byte of WOR timer 93
#define PKTSTATUS 0x38      // (0xF8) Current GDOx status and packet status 94
#define VCO_VC_DAC 0x39     // (0xF9) Current setting from PLL calibration module 94
#define TXBYTES 0x3A        // (0xFA) Underflow and number of bytes in the TX FIFO 94
#define RXBYTES 0x3B        // (0xFB) Overflow and number of bytes in the RX FIFO 94
#define RCCTRL1_STATUS 0x3C // (0xFC) Last RC oscillator calibration result 94
#define RCCTRL0_STATUS 0x3D // (0xFD) Last RC oscillator calibration result 95

/*
 * spi pins
 */
#define CC1101_CS 27
#define CC1101_MOSI 13
#define CC1101_MISO 12
#define CC1101_SCK 14

SPIClass hspi1(HSPI);

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

void writeConfig(uint8_t addr, uint8_t value)
{
    //
    // Rf settings for CC1100
    //
    writeReg(IOCFG0, 0x06);   // GDO0 Output Pin Configuration
    writeReg(PKTCTRL0, 0x05); // Packet Automation Control
    writeReg(FSCTRL1, 0x06);  // Frequency Synthesizer Control
    writeReg(FREQ2, 0x10);    // Frequency Control Word, High Byte
    writeReg(FREQ1, 0xB0);    // Frequency Control Word, Middle Byte
    writeReg(FREQ0, 0x71);    // Frequency Control Word, Low Byte
    writeReg(MDMCFG4, 0xF5);  // Modem Configuration
    writeReg(MDMCFG3, 0x83);  // Modem Configuration
    writeReg(MDMCFG2, 0x03);  // Modem Configuration
    writeReg(DEVIATN, 0x15);  // Modem Deviation Setting
    writeReg(MCSM0, 0x18);    // Main Radio Control State Machine Configuration
    writeReg(FOCCFG, 0x16);   // Frequency Offset Compensation Configuration
    writeReg(FSCAL3, 0xE9);   // Frequency Synthesizer Calibration
    writeReg(FSCAL2, 0x2A);   // Frequency Synthesizer Calibration
    writeReg(FSCAL1, 0x00);   // Frequency Synthesizer Calibration
    writeReg(FSCAL0, 0x1F);   // Frequency Synthesizer Calibration
    writeReg(TEST2, 0x81);    // Various Test Settings
    writeReg(TEST1, 0x35);    // Various Test Settings
    writeReg(TEST0, 0x09);    // Various Test Settings
}

void printStatus(uint8_t status)
{
    uint8_t ready = (status & 0b10000000) >> 7; // Check if the chip is ready
    uint8_t state = (status & 0b01110000) >> 4; // Get the current state
    uint8_t fifo = (status & 0b00001111);       // Get the FIFO status

    Serial.print("Chip State: ");
    Serial.print(ready);
    Serial.print(", State: ");
    switch (state)
    {
    case 0x00:
        Serial.print("IDLE");
        break;
    case 0x01:
        Serial.print("RX");
        break;
    case 0x02:
        Serial.print("TX");
        break;
    case 0x03:
        Serial.print("FSTXON");
        break;
    case 0x04:
        Serial.print("CALIBRATE");
        break;
    case 0x05:
        Serial.print("SETTLING");
        break;
    case 0x06:
        Serial.print("RXFIFO_OVERFLOW");
        break;
    case 0x07:
        Serial.print("TXFIFO_UNDERFLOW");
        break;
    }
    Serial.print(", FIFO: ");
    Serial.println(fifo);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("CC1101 SPI Test");

    pinMode(CC1101_CS, OUTPUT);
    digitalWrite(CC1101_CS, HIGH);
    hspi1.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);

    // Reset CC1101
    digitalWrite(CC1101_CS, LOW);
    uint8_t byte = hspi1.transfer(SRES); // Send reset command
    digitalWrite(CC1101_CS, HIGH);

    Serial.println("CC1101 reset command sent");
    delay(100); // Wait for the reset to complete

    // Read part number
    digitalWrite(CC1101_CS, LOW);
    uint8_t status = hspi1.transfer(VERSION | READ | BURST); // Read part
    uint8_t version = hspi1.transfer(0x00);                  // Dummy byte to read the value
    digitalWrite(CC1101_CS, HIGH);

    Serial.print("STATUS: ");
    Serial.println(status, HEX);
    Serial.print("VERSION: ");
    Serial.println(version, HEX);

    //
    // Rf settings for CC1100
    //
    //
    // Rf settings for CC1100
    //
    writeReg(IOCFG0, 0x06);   // GDO0 Output Pin Configuration
    writeReg(PKTCTRL0, 0x05); // Packet Automation Control
    writeReg(FSCTRL1, 0x06);  // Frequency Synthesizer Control
    writeReg(FREQ2, 0x10);    // Frequency Control Word, High Byte
    writeReg(FREQ1, 0xB0);    // Frequency Control Word, Middle Byte
    writeReg(FREQ0, 0x71);    // Frequency Control Word, Low Byte
    writeReg(MDMCFG4, 0xC8);  // Modem Configuration
    writeReg(MDMCFG3, 0x93);  // Modem Configuration
    writeReg(MDMCFG2, 0x03);  // Modem Configuration
    writeReg(DEVIATN, 0x34);  // Modem Deviation Setting
    writeReg(MCSM0, 0x18);    // Main Radio Control State Machine Configuration
    writeReg(FOCCFG, 0x16);   // Frequency Offset Compensation Configuration
    writeReg(FSCAL3, 0xE9);   // Frequency Synthesizer Calibration
    writeReg(FSCAL2, 0x2A);   // Frequency Synthesizer Calibration
    writeReg(FSCAL1, 0x00);   // Frequency Synthesizer Calibration
    writeReg(FSCAL0, 0x1F);   // Frequency Synthesizer Calibration
    writeReg(TEST2, 0x81);    // Various Test Settings
    writeReg(TEST1, 0x35);    // Various Test Settings
    writeReg(TEST0, 0x09);    // Various Test Settings

    writeReg(SYNC0, 0xAA);
    writeReg(SYNC1, 0xAA);


    // flush rx fifo
    strobe(SFRX);
    // strobe rx
    strobe(SRX);
}

void loop()
{
    uint8_t rxBytes = readStatus(RXBYTES) & 0x7F;
    uint8_t numbytes = rxBytes & 0x7F;
    uint8_t marcstate = readStatus(MARCSTATE) & 0x07; // Get the current state

    if(numbytes) {
        Serial.println("Received: ");
        uint8_t buff[numbytes];
        burstRead(RX_FIFO, buff, numbytes);
        for(int i = 0; i < numbytes; i++) {
            Serial.print(buff[i], HEX);
        }
        Serial.println();
    }
    if(marcstate != 0x0D) {
        strobe(SFRX);
        strobe(SRX);
    }
    Serial.print("Marcstate: ");
    Serial.print(marcstate);
    Serial.print(" RxBytes: ");
    Serial.println(rxBytes);
    delay(500);
}
