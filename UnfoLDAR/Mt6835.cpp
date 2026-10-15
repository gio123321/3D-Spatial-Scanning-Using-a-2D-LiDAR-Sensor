#include "Mt6835.h"
#include <esp_timer.h>

// Command nibbles, shifted into the top 4 bits of the 16-bit cmd+addr word.
static constexpr uint16_t CMD_READ  = 0x3000;   // 0011
static constexpr uint16_t CMD_BURST = 0xA000;   // 1010
// 0110 (write), 1100 (program EEPROM) and 0101 (auto-zero) are intentionally
// absent. 1100 and 0101 write NVM.

static constexpr uint16_t REG_ANGLE_HI = 0x003;   // ANGLE[20:13]
static constexpr uint16_t REG_ANGLE_MD = 0x004;   // ANGLE[12:5]
static constexpr uint16_t REG_ANGLE_LO = 0x005;   // ANGLE[4:0]<<3 | STATUS[2:0]
static constexpr uint16_t REG_CRC      = 0x006;   // CRC[7:0]

void Mt6835::begin(SPIClass* spi, uint8_t csPin) {
  _spi = spi;
  _cs  = csPin;
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);        // idle high
}

uint8_t Mt6835::crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

void Mt6835::decode(Mt6835Reading& r) {
  r.raw     = ((uint32_t)r.b[0] << 13) | ((uint32_t)r.b[1] << 5) | (uint32_t)(r.b[2] >> 3);
  r.status  = r.b[2] & 0x07;
  r.crcRecv = r.b[3];
  r.crcCalc = crc8(r.b, 3);       // CRC covers regs 0x003..0x005 = ANGLE+STATUS
  r.crcOk   = (r.crcRecv == r.crcCalc);

  // Reject dead-bus frames even if their CRC matches (all-zero CRC is zero).
  bool allZero = (r.b[0] == 0x00 && r.b[1] == 0x00 && r.b[2] == 0x00 && r.b[3] == 0x00);
  bool allOnes = (r.b[0] == 0xFF && r.b[1] == 0xFF && r.b[2] == 0xFF && r.b[3] == 0xFF);
  r.degenerate = allZero || allOnes;

  r.ok  = r.crcOk && !r.degenerate;
  r.deg = (float)((double)r.raw / (double)COUNTS * 360.0);
}

Mt6835Reading Mt6835::readBurst(uint8_t spiMode, uint32_t hz) {
  Mt6835Reading r;
  if (!_spi) return r;

  _spi->beginTransaction(SPISettings(hz, MSBFIRST, spiMode));

  // Timestamp the sample instant: the CSN falling edge latches 0x003..0x006.
  r.t_us = esp_timer_get_time();
  digitalWrite(_cs, LOW);

  _spi->transfer16(CMD_BURST | REG_ANGLE_HI);   // 4 cmd bits + 12 addr bits
  for (uint8_t i = 0; i < 4; i++) r.b[i] = _spi->transfer(0x00);

  digitalWrite(_cs, HIGH);
  _spi->endTransaction();

  decode(r);
  return r;
}

uint8_t Mt6835::readReg(uint16_t addr) {
  digitalWrite(_cs, LOW);
  _spi->transfer16(CMD_READ | (addr & 0x0FFF));
  uint8_t v = _spi->transfer(0x00);
  digitalWrite(_cs, HIGH);
  return v;
}

Mt6835Reading Mt6835::readSequential() {
  Mt6835Reading r;
  if (!_spi) return r;

  static const uint16_t regs[4] = { REG_ANGLE_HI, REG_ANGLE_MD, REG_ANGLE_LO, REG_CRC };

  _spi->beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE3));
  r.t_us = esp_timer_get_time();                // first CSN edge is the sample instant
  for (uint8_t i = 0; i < 4; i++) r.b[i] = readReg(regs[i]);
  _spi->endTransaction();

  decode(r);
  return r;
}
