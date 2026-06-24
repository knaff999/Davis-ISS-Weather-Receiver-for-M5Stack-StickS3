// CC1101.cpp
#include "CC1101.h"

void CC1101::begin(int sck, int miso, int mosi, int cs, int gdo0, int gdo2) {
  _cs = cs; _gdo0 = gdo0; _gdo2 = gdo2;
  pinMode(_cs, OUTPUT);
  csHigh();
  if (_gdo0 >= 0) pinMode(_gdo0, INPUT);
  if (_gdo2 >= 0) pinMode(_gdo2, INPUT);

  _spi = new SPIClass(HSPI);
  _spi->begin(sck, miso, mosi, -1 /* CS handled manually */);
}

void CC1101::waitOnMiso() {
  // After CS goes low, CC1101 holds MISO high while it wakes its crystal
  // oscillator, then pulls it low when ready for the SPI transaction.
  // SPIClass doesn't expose the MISO pin directly while a transaction is
  // open on all cores, so we just give it a short fixed settle time, which
  // is enough once the chip is already running (only matters after SRES /
  // waking from sleep).
  delayMicroseconds(100);
}

uint8_t CC1101::readReg(uint8_t addr) {
  uint8_t val;
  csLow();
  waitOnMiso();
  _spi->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  _spi->transfer(addr | 0x80); // read bit
  val = _spi->transfer(0x00);
  _spi->endTransaction();
  csHigh();
  return val;
}

uint8_t CC1101::readStatusReg(uint8_t addr) {
  uint8_t val;
  csLow();
  waitOnMiso();
  _spi->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  _spi->transfer(addr | 0xC0); // read + burst bit required for status regs
  val = _spi->transfer(0x00);
  _spi->endTransaction();
  csHigh();
  return val;
}

void CC1101::writeReg(uint8_t addr, uint8_t val) {
  csLow();
  waitOnMiso();
  _spi->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  _spi->transfer(addr);
  _spi->transfer(val);
  _spi->endTransaction();
  csHigh();
}

void CC1101::writeBurst(uint8_t addr, const uint8_t *data, uint8_t len) {
  csLow();
  waitOnMiso();
  _spi->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  _spi->transfer(addr | 0x40); // burst bit
  for (uint8_t i = 0; i < len; i++) _spi->transfer(data[i]);
  _spi->endTransaction();
  csHigh();
}

void CC1101::strobe(uint8_t cmd) {
  csLow();
  waitOnMiso();
  _spi->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  _spi->transfer(cmd);
  _spi->endTransaction();
  csHigh();
}

void CC1101::idle() {
  strobe(CC1101_SIDLE);
  // Wait for MARCSTATE-free confirmation isn't strictly required for our
  // polling use case; a short settle time is enough.
  delayMicroseconds(200);
}

void CC1101::flushRx() {
  strobe(CC1101_SFRX);
}

void CC1101::startRx() {
  idle();
  flushRx();
  strobe(CC1101_SRX);
}

int CC1101::rxBytesAvailable() {
  uint8_t v = readStatusReg(CC1101_RXBYTES);
  if (v & 0x80) return -1; // RXFIFO_OVERFLOW
  return v & 0x7F;
}

void CC1101::readFifo(uint8_t *buf, uint8_t len) {
  csLow();
  waitOnMiso();
  _spi->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  _spi->transfer(CC1101_FIFO | 0xC0); // read + burst
  for (uint8_t i = 0; i < len; i++) buf[i] = _spi->transfer(0x00);
  _spi->endTransaction();
  csHigh();
}

int8_t CC1101::lastRssiDbm(uint8_t rawRssi) {
  // Per CC1101 datasheet section 17.3
  if (rawRssi >= 128) return ((int16_t)(rawRssi - 256)) / 2 - 74;
  return ((int16_t)rawRssi) / 2 - 74;
}

void CC1101::setFrequencyHz(uint32_t freqHz) {
  // FREQ[23:0] = freqHz * 2^16 / xtalHz   (CC1101 datasheet 21)
  uint64_t freqWord = ((uint64_t)freqHz << 16) / _xtalHz;
  writeReg(CC1101_FREQ2, (freqWord >> 16) & 0xFF);
  writeReg(CC1101_FREQ1, (freqWord >> 8) & 0xFF);
  writeReg(CC1101_FREQ0, freqWord & 0xFF);
}

void CC1101::setDavisChannel(uint8_t channel) {
  if (channel > 50) channel = 50;
  // North America / Australia / New Zealand table: 902.5 MHz + 0.5 MHz * ch
  uint32_t freqHz = 902500000UL + (uint32_t)channel * 500000UL;
  idle();
  setFrequencyHz(freqHz);
  startRx();
}

// Finds the DRATE_E/DRATE_M pair (CC1101 datasheet eq. 12) that best matches
// the requested baud rate, and returns the resulting MDMCFG3 (mantissa) byte
// while writing DRATE_E into the low nibble of MDMCFG4 (the caller supplies
// the channel-bandwidth bits for the high nibble separately).
static void computeDataRateRegs(uint32_t xtalHz, uint32_t baud,
                                 uint8_t &drateE, uint8_t &drateM) {
  uint32_t bestErr = 0xFFFFFFFF;
  drateE = 0; drateM = 0;
  for (uint8_t e = 0; e <= 15; e++) {
    // Rdata = (256+M) * 2^e * xtal / 2^28  =>  M = Rdata*2^28/(2^e*xtal) - 256
    double m = ((double)baud * 268435456.0) / ((double)(1UL << e) * xtalHz) - 256.0;
    long mi = lround(m);
    if (mi < 0 || mi > 255) continue;
    double actual = (256.0 + mi) * (double)(1UL << e) * xtalHz / 268435456.0;
    uint32_t err = (uint32_t)fabs(actual - (double)baud);
    if (err < bestErr) { bestErr = err; drateE = e; drateM = (uint8_t)mi; }
  }
}

// Finds DEVIATN_E/DEVIATN_M (CC1101 datasheet eq. 14) for the requested
// frequency deviation in Hz.
static uint8_t computeDeviationReg(uint32_t xtalHz, uint32_t devHz) {
  uint32_t bestErr = 0xFFFFFFFF;
  uint8_t bestE = 0, bestM = 0;
  for (uint8_t e = 0; e <= 7; e++) {
    for (uint8_t m = 0; m <= 7; m++) {
      // Fdev = (8+M) * 2^E * xtal / 2^17
      double actual = (8.0 + m) * (double)(1UL << e) * xtalHz / 131072.0;
      uint32_t err = (uint32_t)fabs(actual - (double)devHz);
      if (err < bestErr) { bestErr = err; bestE = e; bestM = m; }
    }
  }
  return (bestE << 4) | bestM;
}

void CC1101::configureForDavis(uint32_t xtalHz) {
  _xtalHz = xtalHz;

  // Reset
  csHigh();
  delayMicroseconds(5);
  csLow();
  delayMicroseconds(10);
  csHigh();
  delayMicroseconds(45);
  strobe(CC1101_SRES);
  delay(5);

  // --- Davis ISS protocol parameters -------------------------------------
  // 19.2 kBaud 2-GFSK, ~4.8 kHz deviation, 4-byte preamble (0xAA), 2-byte
  // sync word 0xCB 0x89, 10-byte fixed-length packet, no hardware CRC (the
  // Davis CRC-16/CCITT is verified in software), data bytes are LSB-first
  // over the air so they're bit-reversed after reading the FIFO.
  // Sources: dekay/DavisRFM69 wiki (RF & Message Protocol pages) and
  // cmatteri/CC1101-Weather-Receiver (a CC1101-based implementation of the
  // same reverse-engineered protocol).
  uint8_t drateE, drateM;
  computeDataRateRegs(_xtalHz, 19200, drateE, drateM);
  uint8_t deviatn = computeDeviationReg(_xtalHz, 4800);

  // ~200 kHz channel bandwidth: CHANBW_E=1, CHANBW_M=0 -> xtal/(8*4*2^1)
  uint8_t chanbwBits = (1 << 6) | (0 << 4);

  writeReg(CC1101_IOCFG2, 0x0D);     // GDO2: unused here, set to serial clock (harmless)
  writeReg(CC1101_IOCFG0, 0x06);     // GDO0: asserts on sync found, deasserts at packet end
  writeReg(CC1101_FIFOTHR, 0x47);    // default-ish FIFO threshold
  writeReg(CC1101_SYNC1, 0xCB);
  writeReg(CC1101_SYNC0, 0x89);
  writeReg(CC1101_PKTLEN, 10);       // Davis packets are 10 bytes on air
  writeReg(CC1101_PKTCTRL1, 0x04);   // append RSSI+LQI status bytes, no addr filtering
  writeReg(CC1101_PKTCTRL0, 0x00);   // fixed length, hw CRC off, no whitening
  writeReg(CC1101_ADDR, 0x00);
  writeReg(CC1101_CHANNR, 0x00);     // we set FREQx directly per hop, not via CHANNR
  writeReg(CC1101_FSCTRL1, 0x06);
  writeReg(CC1101_FSCTRL0, 0x00);
  writeReg(CC1101_MDMCFG4, chanbwBits | drateE);
  writeReg(CC1101_MDMCFG3, drateM);
  writeReg(CC1101_MDMCFG2, 0x22);    // 2-GFSK, 16/16 sync bits, no Manchester
  writeReg(CC1101_MDMCFG1, 0x20);    // 4-byte preamble, no FEC
  writeReg(CC1101_MDMCFG0, 0xF8);
  writeReg(CC1101_DEVIATN, deviatn);
  writeReg(CC1101_MCSM2, 0x07);
  writeReg(CC1101_MCSM1, 0x0C);      // stay in RX after packet; idle after TX
  writeReg(CC1101_MCSM0, 0x18);      // auto-calibrate IDLE->RX/TX
  writeReg(CC1101_FOCCFG, 0x16);
  writeReg(CC1101_BSCFG, 0x6C);
  writeReg(CC1101_AGCCTRL2, 0x03);
  writeReg(CC1101_AGCCTRL1, 0x40);
  writeReg(CC1101_AGCCTRL0, 0x91);
  writeReg(CC1101_FREND1, 0x56);
  writeReg(CC1101_FREND0, 0x10);
  writeReg(CC1101_FSCAL3, 0xE9);
  writeReg(CC1101_FSCAL2, 0x2A);
  writeReg(CC1101_FSCAL1, 0x00);
  writeReg(CC1101_FSCAL0, 0x1F);
  writeReg(CC1101_TEST2, 0x81);
  writeReg(CC1101_TEST1, 0x35);
  writeReg(CC1101_TEST0, 0x09);

  setDavisChannel(0);
}
