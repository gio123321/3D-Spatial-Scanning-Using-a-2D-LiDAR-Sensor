#pragma once
#include <Arduino.h>

// INA226 monitor on the shared I2C bus. Probe die/manufacturer IDs across
// 0x40..0x4F. Calculate current from measured shunt voltage and the fitted
// resistor rather than relying on the CURRENT/POWER calibration registers.
// Missing hardware does not block scanning.

struct PowerSample {
  float shunt_mv = 0.0f;    // voltage across the shunt, mV (signed)
  float bus_v    = 0.0f;    // bus voltage at the VBUS pin, V, after busScale
  float current_a = 0.0f;   // shunt_mv / shunt_ohms, A (signed: sign = direction)
  float power_w  = 0.0f;    // bus_v * current_a, W
  // Raw ADC counts before scaling; bus_raw * 1.25 mV gives bus voltage.
  uint16_t bus_raw   = 0;
  int16_t  shunt_raw = 0;
  bool  ok       = false;
};

class Ina226 {
public:
  // Use the fitted shunt resistance. busScale corrects measured bus ADC gain;
  // it defaults to 1.0.
  bool begin(float shuntOhms, uint8_t addrHint = 0x40, float busScale = 1.0f,
             uint8_t excludedAddr = 0);

  bool    present() const { return _present; }
  uint8_t addr()    const { return _addr; }
  uint16_t dieId()  const { return _dieId; }
  uint16_t mfgId()  const { return _mfgId; }

  bool read(PowerSample& out);

  // Raw registers for comparison with decoded readings.
  struct RawRegs { uint16_t config, shunt, bus, mfg, die; bool ok; };
  RawRegs readRaw();

private:
  bool     _present    = false;
  uint8_t  _addr       = 0;
  uint16_t _mfgId      = 0;
  uint16_t _dieId      = 0;
  float    _shuntOhms  = 0.1f;
  float    _busScale   = 1.0f;

  bool probe(uint8_t addr);
  bool readReg(uint8_t reg, uint16_t& out);
  bool writeReg(uint8_t reg, uint16_t val);
};
