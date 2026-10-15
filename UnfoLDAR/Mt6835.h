#pragma once
#include <Arduino.h>
#include <SPI.h>

// MT6835 read-only encoder driver: SPI mode 3, MSB first, 21-bit angle.
// Uses register read and burst read; EEPROM programming and auto-zero are excluded.
// CSN falling edge latches registers 0x003..0x006 and defines the timestamp.
// Reject all-zero and all-one frames even if their CRC matches; check ok before use.

struct Mt6835Reading {
  uint32_t raw       = 0;      // ANGLE[20:0]
  float    deg       = 0.0f;   // raw / 2^21 * 360
  uint8_t  status    = 0;      // bit0 overspeed, bit1 weak field, bit2 undervoltage
  uint8_t  crcRecv   = 0;
  uint8_t  crcCalc   = 0;
  bool     crcOk     = false;  // CRC byte matched the computed CRC
  bool     degenerate = false; // all-0x00 or all-0xFF frame
  bool     ok        = false;  // crcOk && !degenerate — the only flag callers should trust
  int64_t  t_us      = 0;      // esp_timer_get_time() at the CSN falling edge
  uint8_t  b[4]      = {0,0,0,0};   // regs 0x003..0x006 as received
};

class Mt6835 {
public:
  static constexpr uint32_t COUNTS   = 2097152UL;   // 2^21
  static constexpr uint32_t SPI_HZ   = 1000000UL;   // 1 MHz for bring-up. Do not raise.

  static constexpr uint8_t STATUS_OVERSPEED    = 0x01;
  static constexpr uint8_t STATUS_WEAK_FIELD   = 0x02;
  static constexpr uint8_t STATUS_UNDERVOLTAGE = 0x04;

  // spi must already have begun on the shared VSPI bus.
  void begin(SPIClass* spi, uint8_t csPin);

  // Burst command 1010 at 0x003. Mode/rate overrides are for bus diagnostics.
  Mt6835Reading readBurst(uint8_t spiMode = SPI_MODE3, uint32_t hz = SPI_HZ);
  Mt6835Reading readSequential();     // four separate 0011 reads - fallback/diagnostic

  // CRC-8 over the 24 bits ANGLE[20:0]+STATUS[2:0], poly X^8+X^2+X+1,
  // init 0x00, ANGLE[20] first. Verified against the CRC-8/SMBUS check
  // vector: crc8("123456789") == 0xF4.
  static uint8_t crc8(const uint8_t* data, size_t len);

  uint8_t csPin() const { return _cs; }

private:
  SPIClass* _spi = nullptr;
  uint8_t   _cs  = 255;

  void      decode(Mt6835Reading& r);
  uint8_t   readReg(uint16_t addr);
};
