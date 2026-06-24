// CC1101.h
// Minimal register-level CC1101 driver for ESP32 (Cardputer-Adv), tailored
// for receiving Davis Instruments ISS (Vantage Pro2 / Vue) transmissions.
//
// Implemented directly against the TI CC1101 SPI interface (no third-party
// CC1101 library dependency) so every register written for the Davis-specific
// radio configuration is visible and easy to tune.
#pragma once
#include <Arduino.h>
#include <SPI.h>

class CC1101 {
public:
  // sck/miso/mosi/cs are the SPI pins; gdo0/gdo2 are optional GPIO pins
  // wired to the module's GDO0/GDO2 lines (not required for the polling
  // approach used in this sketch, but wired up for future use).
  void begin(int sck, int miso, int mosi, int cs, int gdo0 = -1, int gdo2 = -1);

  // Resets the chip and loads the Davis ISS GFSK/19.2kbps radio + packet
  // configuration. xtalHz is the crystal frequency on your CC1101 module
  // (almost always 26,000,000; a few boards use 27,000,000 -- check the
  // crystal can or the seller's listing if reception never locks).
  void configureForDavis(uint32_t xtalHz = 26000000UL);

  // Tunes to one of the 51 Davis North-America/Australia/NZ channels
  // (0 = 902.5 MHz ... 50 = 927.5 MHz, 0.5 MHz spacing) and (re)starts RX.
  void setDavisChannel(uint8_t channel);

  // Number of bytes currently sitting in the RX FIFO (0-127), or -1 if the
  // FIFO has overflowed (caller should flush and restart RX).
  int rxBytesAvailable();

  // Burst-reads `len` bytes out of the RX FIFO into buf.
  void readFifo(uint8_t *buf, uint8_t len);

  void idle();      // SIDLE strobe, waits for the chip to settle
  void startRx();   // SFRX (flush) + SRX strobes
  void flushRx();

  int8_t  lastRssiDbm(uint8_t rawRssi);  // converts the raw status byte
  uint8_t lastLqi(uint8_t rawLqi) { return rawLqi & 0x7F; }

  uint8_t readReg(uint8_t addr);
  void    writeReg(uint8_t addr, uint8_t val);
  uint8_t readStatusReg(uint8_t addr); // status regs need the burst bit set
  void    strobe(uint8_t cmd);

private:
  SPIClass *_spi = nullptr;
  int _cs = -1, _gdo0 = -1, _gdo2 = -1;
  uint32_t _xtalHz = 26000000UL;

  void csLow()  { digitalWrite(_cs, LOW); }
  void csHigh() { digitalWrite(_cs, HIGH); }
  void waitOnMiso(); // CC1101 pulls MISO low once it's ready after CS asserted

  void writeBurst(uint8_t addr, const uint8_t *data, uint8_t len);
  void setFrequencyHz(uint32_t freqHz);
};

// --- CC1101 register addresses -------------------------------------------
#define CC1101_IOCFG2     0x00
#define CC1101_IOCFG0     0x02
#define CC1101_FIFOTHR    0x03
#define CC1101_SYNC1      0x04
#define CC1101_SYNC0      0x05
#define CC1101_PKTLEN     0x06
#define CC1101_PKTCTRL1   0x07
#define CC1101_PKTCTRL0   0x08
#define CC1101_ADDR       0x09
#define CC1101_CHANNR     0x0A
#define CC1101_FSCTRL1    0x0B
#define CC1101_FSCTRL0    0x0C
#define CC1101_FREQ2      0x0D
#define CC1101_FREQ1      0x0E
#define CC1101_FREQ0      0x0F
#define CC1101_MDMCFG4    0x10
#define CC1101_MDMCFG3    0x11
#define CC1101_MDMCFG2    0x12
#define CC1101_MDMCFG1    0x13
#define CC1101_MDMCFG0    0x14
#define CC1101_DEVIATN    0x15
#define CC1101_MCSM2      0x16
#define CC1101_MCSM1      0x17
#define CC1101_MCSM0      0x18
#define CC1101_FOCCFG     0x19
#define CC1101_BSCFG      0x1A
#define CC1101_AGCCTRL2   0x1B
#define CC1101_AGCCTRL1   0x1C
#define CC1101_AGCCTRL0   0x1D
#define CC1101_FREND1     0x21
#define CC1101_FREND0     0x22
#define CC1101_FSCAL3     0x23
#define CC1101_FSCAL2     0x24
#define CC1101_FSCAL1     0x25
#define CC1101_FSCAL0     0x26
#define CC1101_TEST2      0x2C
#define CC1101_TEST1      0x2D
#define CC1101_TEST0      0x2E
#define CC1101_FIFO       0x3F

// Status registers share addresses with the strobes; reading them requires
// the read+burst bits (0xC0) set, which is handled in readStatusReg().
#define CC1101_RSSI       0x34
#define CC1101_LQI        0x33
#define CC1101_RXBYTES    0x3B

// Command strobes
#define CC1101_SRES       0x30
#define CC1101_SCAL       0x33
#define CC1101_SRX        0x34
#define CC1101_SIDLE      0x36
#define CC1101_SFRX       0x3A
#define CC1101_SNOP       0x3D
