#include "cc1101.h"
#include "gpio.h"
#include "spi.h"
#include "stdio.h"

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

void writeReg(uint8_t addr, uint8_t value)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &addr, 1, 100);
    HAL_SPI_Transmit(&hspi1, &value, 1, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void burstWrite(uint8_t addr, uint8_t *buffer, uint8_t count)
{
    addr |= BURST;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &addr, 1, 100);
    HAL_SPI_Transmit(&hspi1, buffer, count, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

/*
    For register addresses in the range 0x30-
    0x3D, the burst bit is used to select between
    status registers when burst bit is one, and
    between command strobes when burst bit is zero.
*/
uint8_t readReg(uint8_t addr)
{
    uint8_t value;
    addr |= READ;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &addr, 1, 100);
    HAL_SPI_Receive(&hspi1, &value, 1, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    return value;
}

void burstRead(uint8_t addr, uint8_t *buffer, uint8_t count)
{
    addr |= READ | BURST;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &addr, 1, 100);
    HAL_SPI_Receive(&hspi1, buffer, count, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void strobe(uint8_t addr)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &addr, 1, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void init()
{
    // Reset CC1101
    strobe(SRES);

    // read partnum
    uint8_t partnum = readReg(PARTNUM | BURST); // BURST bit differentiates between status registers and command strobes
    printf("PARTNUM: %d\n", partnum);

    //
    // Rf settings for CC1100
    //
    //
    // Rf settings for CC1100
    //
    writeReg(IOCFG0, 0x06);   // GDO0 Output Pin Configuration
    writeReg(PKTCTRL0, 0x05); // Packet Automation Control
    writeReg(FSCTRL1, 0x06);  // Frequency Synthesizer Control
    writeReg(FREQ2, 0x21);    // Frequency Control Word, High Byte
    writeReg(FREQ1, 0x66);    // Frequency Control Word, Middle Byte
    writeReg(FREQ0, 0x66);    // Frequency Control Word, Low Byte
    writeReg(MDMCFG4, 0xF8);  // Modem Configuration
    writeReg(MDMCFG3, 0xB6);  // Modem Configuration
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

    while (1)
    {
        // write to the FIFO
        uint8_t data[] = {
            0xaa,
            0xaa,
            0xab,
            0xfe,
            0x05,
            0x2d,
            0x2c,
            0xaa,
            0xcc,
            0xd4,
            0xca,
            0xd3,
            0x54,
            0xd2,
            0xcc,
            0xcb,
            0x2b,
            0x2d,
            0x4d,
            0x33,
            0x32,
            0xb2,
            0xcc,
            0xb2,
            0xaf,
            0xfc,
            0x0a,
            0x5a,
            0x59,
            0x55,
            0x99,
            0xa9,
            0x95,
            0xa6,
            0xa9,
            0xa5,
            0x99,
            0x96,
            0x56,
            0x5a,
            0x9a,
            0x66,
            0x65,
            0x65,
            0x99,
            0x65,
            0x5f,
            0xf8,
            0x14,
            0xb4,
            0xb2,
            0xab,
            0x33,
            0x53,
            0x2b,
            0x4d,
            0x53,
            0x4b,
            0x33,
            0x2c,
            0xac,
            0xb5,
            0x34,
            0xcc,
            0xca,
            0xcb,
            0x32,
            0xca,
            0xbf,
            0xf0,
            0x29,
            0x69,
            0x65,
            0x56,
            0x66,
            0xa6,
            0x56,
            0x9a,
            0xa6,
            0x96,
            0x66,
            0x59,
            0x59,
            0x6a,
            0x69,
            0x99,
            0x95,
            0x96,
            0x65,
            0x95,
            0x7};

        data[0] = sizeof(data) - 1;
        burstWrite(TX_FIFO, data, sizeof(data));

        // print txbytes
        uint8_t txbytes = readReg(TXBYTES | BURST);
        printf("TXBYTES: %d\n", txbytes);

        // sftx
        strobe(STX);
        HAL_Delay(100);
        txbytes = readReg(TXBYTES | BURST);
        printf("TXBYTES: %d\n", txbytes);

        // read state machine state
        uint8_t marcstate = readReg(MARCSTATE | BURST);
        printf("MARCSTATE: %d\n", marcstate);
        while (1)
            ;
    }
}
