#include "Ina226.h"
#include <Wire.h>

static constexpr uint8_t REG_CONFIG  = 0x00;
static constexpr uint8_t REG_SHUNT_V = 0x01;
static constexpr uint8_t REG_BUS_V   = 0x02;
static constexpr uint8_t REG_MFG_ID  = 0xFE;
static constexpr uint8_t REG_DIE_ID  = 0xFF;
// 0x03 POWER, 0x04 CURRENT and 0x05 CALIBRATION are intentionally unused -
// Current is calculated from the shunt voltage and fitted resistance.

static constexpr uint16_t MFG_ID_TI    = 0x5449;   // "TI"
static constexpr uint16_t DIE_ID_INA226 = 0x2260;

// CONFIG = 0x4527:
//   [14:12] 100  fixed
//   [11:9]  010  average 16 samples   - kills stepper switching noise
//   [8:6]   100  bus conversion 1.1 ms
//   [5:3]   100  shunt conversion 1.1 ms
//   [2:0]   111  continuous shunt + bus
// 16 averages x (1.1 + 1.1) ms gives about 35 ms per conversion.
static constexpr uint16_t CONFIG_VALUE = 0x4527;

static constexpr float SHUNT_LSB_MV = 0.0025f;   // 2.5 uV per LSB
static constexpr float BUS_LSB_V    = 0.00125f;  // 1.25 mV per LSB

bool Ina226::readReg(uint8_t reg, uint16_t& out) {
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)_addr, 2) != 2) return false;
  // Read the high byte before the low byte.
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  out = ((uint16_t)hi << 8) | lo;                 // INA226 is big-endian
  return true;
}

Ina226::RawRegs Ina226::readRaw() {
  RawRegs r{};
  if (!_present) return r;
  r.ok = readReg(REG_CONFIG,  r.config)
      && readReg(REG_SHUNT_V, r.shunt)
      && readReg(REG_BUS_V,   r.bus)
      && readReg(REG_MFG_ID,  r.mfg)
      && readReg(REG_DIE_ID,  r.die);
  return r;
}

bool Ina226::writeReg(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  return Wire.endTransmission() == 0;
}

bool Ina226::probe(uint8_t addr) {
  _addr = addr;
  Wire.beginTransmission(addr);
  if (Wire.endTransmission() != 0) return false;  // nothing acknowledges here

  uint16_t mfg = 0, die = 0;
  if (!readReg(REG_MFG_ID, mfg) || !readReg(REG_DIE_ID, die)) return false;
  if (mfg != MFG_ID_TI || die != DIE_ID_INA226) return false;

  _mfgId = mfg;
  _dieId = die;
  return true;
}

bool Ina226::begin(float shuntOhms, uint8_t addrHint, float busScale,
                   uint8_t excludedAddr) {
  _present   = false;
  _shuntOhms = (shuntOhms > 1e-6f) ? shuntOhms : 0.1f;
  _busScale  = (busScale  > 1e-6f) ? busScale  : 1.0f;

  // The hint first, then the rest of the block it can be strapped into.
  if (addrHint == excludedAddr || !probe(addrHint)) {
    for (uint8_t a = 0x40; a <= 0x4F; a++) {
      if (a == addrHint || a == excludedAddr) continue;
      if (probe(a)) { _present = true; break; }
    }
  } else {
    _present = true;
  }
  if (!_present) { _addr = 0; return false; }

  if (!writeReg(REG_CONFIG, CONFIG_VALUE)) { _present = false; _addr = 0; return false; }
  return true;
}

bool Ina226::read(PowerSample& out) {
  out = PowerSample();
  if (!_present) return false;

  uint16_t sh = 0, bv = 0;
  if (!readReg(REG_SHUNT_V, sh)) return false;
  if (!readReg(REG_BUS_V,   bv)) return false;

  out.shunt_raw = (int16_t)sh;
  out.bus_raw   = bv;
  out.shunt_mv  = (float)(int16_t)sh * SHUNT_LSB_MV;   // shunt reg is signed
  // Apply gain calibration only here; preserve the raw ADC counts.
  out.bus_v     = (float)bv * BUS_LSB_V * _busScale;   // bus reg is unsigned
  out.current_a = (out.shunt_mv / 1000.0f) / _shuntOhms;
  out.power_w   = out.bus_v * out.current_a;
  out.ok        = true;
  return true;
}
