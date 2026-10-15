#pragma once
#include <Arduino.h>

// GY-601N1 variants are identified by WHO_AM_I:
// 0x47 at 0x75: ICM-42688-P; 0xE9 at 0x72: ICM-45686;
// 0x43 at 0x00: BMI323 (16-bit registers, two dummy read bytes).
// Used for leveling telemetry; scanning also works without an IMU.

enum ImuKind { IMU_NONE, IMU_ICM42688, IMU_ICM45686, IMU_BMI323 };

struct ImuSample {
  float ax = 0, ay = 0, az = 0;      // m/s^2
  float gx = 0, gy = 0, gz = 0;      // deg/s
  float pitch = 0, roll = 0;         // deg, from gravity
  float magnitude = 0;               // |accel|, ~9.81 when stationary
  bool  ok = false;
};

class Imu {
public:
  bool        begin();                    // probes 0x68 then 0x69
  bool        present() const { return _kind != IMU_NONE; }
  ImuKind     kind()    const { return _kind; }
  uint8_t     addr()    const { return _addr; }
  uint8_t     whoVal()  const { return _whoVal; }
  const char* partName() const;

  bool read(ImuSample& out);

private:
  ImuKind _kind   = IMU_NONE;
  uint8_t _addr   = 0;
  uint8_t _whoReg = 0;
  uint8_t _whoVal = 0;

  float   _accLsbPerG   = 1.0f;
  float   _gyrLsbPerDps = 1.0f;
  uint8_t _dataReg      = 0;
  bool    _bigEndian    = true;

  bool identify(uint8_t addr);
  bool configure();
};
