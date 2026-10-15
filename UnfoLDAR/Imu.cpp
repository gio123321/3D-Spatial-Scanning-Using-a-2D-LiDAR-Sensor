#include "Imu.h"
#include "Config.h"
#include <Wire.h>
#include <math.h>

static const float G = 9.80665f;

// ---- bounded I2C helpers ---------------------------------------------------

static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)n) != (int)n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// BMI323 has 16-bit registers and prepends 2 dummy bytes to every I2C read.
static bool bmi323Read(uint8_t addr, uint8_t reg, uint8_t* buf, size_t n) {
  uint8_t tmp[32];
  if (n + 2 > sizeof(tmp)) return false;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)(n + 2)) != (int)(n + 2)) return false;
  for (size_t i = 0; i < n + 2; i++) tmp[i] = Wire.read();
  memcpy(buf, tmp + 2, n);
  return true;
}

static bool bmi323Write16(uint8_t addr, uint8_t reg, uint16_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write((uint8_t)(val & 0xFF));
  Wire.write((uint8_t)(val >> 8));
  return Wire.endTransmission() == 0;
}

// ---------------------------------------------------------------------------

const char* Imu::partName() const {
  switch (_kind) {
    case IMU_ICM42688: return "ICM-42688-P";
    case IMU_ICM45686: return "ICM-45686";
    case IMU_BMI323:   return "BMI323";
    default:           return "none";
  }
}

bool Imu::identify(uint8_t addr) {
  uint8_t v = 0;
  if (i2cRead(addr, 0x75, &v, 1) && v == 0x47) {
    _kind = IMU_ICM42688; _whoReg = 0x75; _whoVal = v; return true;
  }
  if (i2cRead(addr, 0x72, &v, 1) && v == 0xE9) {
    _kind = IMU_ICM45686; _whoReg = 0x72; _whoVal = v; return true;
  }
  uint8_t b[2];
  if (bmi323Read(addr, 0x00, b, 2) && b[0] == 0x43) {
    _kind = IMU_BMI323; _whoReg = 0x00; _whoVal = b[0]; return true;
  }
  return false;
}

bool Imu::configure() {
  switch (_kind) {
    case IMU_ICM42688:
      if (!i2cWrite8(_addr, 0x4E, 0x0F)) return false;   // PWR_MGMT0: accel+gyro low-noise
      delay(50);
      _dataReg      = 0x1F;        // ACCEL_DATA_X1 .. GYRO_DATA_Z0
      _accLsbPerG   = 2048.0f;     // default FS +-16 g
      _gyrLsbPerDps = 16.4f;       // default FS +-2000 dps
      _bigEndian    = true;
      return true;

    case IMU_ICM45686:
      if (!i2cWrite8(_addr, 0x10, 0x0F)) return false;   // PWR_MGMT0
      delay(50);
      _dataReg      = 0x00;        // ACCEL_DATA_X..GYRO_DATA_Z
      _accLsbPerG   = 1024.0f;     // default FS +-32 g
      _gyrLsbPerDps = 8.2f;        // default FS +-4000 dps
      // Little-endian, confirmed on hardware: stationary board returned
      // A1 FF CE FF 09 04 00 00 FE FF 00 00, which decodes LE as
      // accel (-95,-50,1033) = 1.014 g. Big-endian gave ~27 g.
      _bigEndian    = false;
      return true;

    case IMU_BMI323:
      if (!bmi323Write16(_addr, 0x20, 0x7028)) return false;  // ACC_CONF 100Hz +-8g
      if (!bmi323Write16(_addr, 0x21, 0x7048)) return false;  // GYR_CONF 100Hz +-2000dps
      delay(50);
      _dataReg      = 0x03;
      _accLsbPerG   = 4096.0f;
      _gyrLsbPerDps = 16.384f;
      _bigEndian    = false;
      return true;

    default: return false;
  }
}

bool Imu::begin() {
  _kind = IMU_NONE;

  for (uint8_t a : { (uint8_t)0x68, (uint8_t)0x69 }) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() != 0) continue;    // nothing at this address
    if (identify(a)) { _addr = a; break; }
  }
  if (_kind == IMU_NONE) return false;

  if (!configure()) { _kind = IMU_NONE; return false; }
  return true;
}

bool Imu::read(ImuSample& out) {
  out = ImuSample();
  if (_kind == IMU_NONE) return false;

  uint8_t d[12];
  bool ok = (_kind == IMU_BMI323) ? bmi323Read(_addr, _dataReg, d, 12)
                                  : i2cRead(_addr, _dataReg, d, 12);
  if (!ok) return false;

  int16_t r[6];
  for (uint8_t i = 0; i < 6; i++) {
    r[i] = _bigEndian ? (int16_t)(((uint16_t)d[2 * i] << 8) | d[2 * i + 1])
                      : (int16_t)(((uint16_t)d[2 * i + 1] << 8) | d[2 * i]);
  }

  out.ax = r[0] / _accLsbPerG * G;
  out.ay = r[1] / _accLsbPerG * G;
  out.az = r[2] / _accLsbPerG * G;
  out.gx = r[3] / _gyrLsbPerDps;
  out.gy = r[4] / _gyrLsbPerDps;
  out.gz = r[5] / _gyrLsbPerDps;

  out.magnitude = sqrtf(out.ax * out.ax + out.ay * out.ay + out.az * out.az);

  // Tilt from gravity. Meaningful only while the rig is still.
  out.pitch = degrees(atan2f(-out.ax, sqrtf(out.ay * out.ay + out.az * out.az)));
  out.roll  = degrees(atan2f(out.ay, out.az));

  out.ok = true;
  return true;
}
