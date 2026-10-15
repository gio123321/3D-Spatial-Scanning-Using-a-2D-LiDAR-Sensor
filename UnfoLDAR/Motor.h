#pragma once
#include <Arduino.h>

class AccelStepper;

// Per-axis hardware and motion settings; azimuth and tilt use different microsteps.
struct AxisConfig {
  uint8_t stepPin;
  uint8_t dirPin;
  long    stepsPerRev;
  float   maxSpeedSps;
  float   accelSps2;
  bool    invertDir;
};

extern const AxisConfig AXIS_AZ;
extern const AxisConfig AXIS_TILT;

class Motor {
public:
  void  begin(const AxisConfig& cfg);
  void  moveToAngle(float deg);   // absolute, deg from the current zero
  void  zero();                   // call the present position 0 deg
  void  stopSmooth();             // decelerate to a halt
  void  update();                 // MUST be called every loop - runs the step engine
  bool  isMoving();
  float currentAngle();           // degrees using this axis' steps/rev

  long  degToSteps(float deg) const;      
  float stepsToDeg(long steps) const;     

  // Override the speed/acceleration from AxisConfig (used for the fast rewind).
  void  setMotionProfile(float maxSps, float accelSps2);
  void  restoreMotionProfile();           // back to this axis' configured values

  // DRV_EN is a single net shared by both drivers, so power is reference
  // counted: the coils stay energized until every axis has released.
  void  enable();
  void  disable();

  static void powerOffAll();      // force EN high regardless of refcount

private:
  AccelStepper* _s = nullptr;
  AxisConfig    _cfg{};
  bool          _holdsPower = false;
};
