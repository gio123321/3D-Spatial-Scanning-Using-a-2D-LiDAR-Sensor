
#include "Motor.h"
#include "Config.h"
#include <AccelStepper.h>

const AxisConfig AXIS_AZ = {
  PIN_STEP_AZ, PIN_DIR_AZ, STEPS_PER_REV_AZ,
  MAX_SPEED_SPS_AZ, ACCEL_SPS2_AZ, INVERT_DIR_AZ
};

const AxisConfig AXIS_TILT = {
  PIN_STEP_TILT, PIN_DIR_TILT, STEPS_PER_REV_TILT,
  MAX_SPEED_SPS_TILT, ACCEL_SPS2_TILT, INVERT_DIR_TILT
};

// ---- shared DRV_EN net (active low, external 10k pull-up = disabled) -------
static int  s_powerRefs = 0;
static bool s_enInit    = false;

static void enPinInit() {
  if (s_enInit) return;
  pinMode(PIN_DRV_EN, OUTPUT);
  digitalWrite(PIN_DRV_EN, HIGH);   // disabled
  s_enInit = true;
}

// ---------------------------------------------------------------------------

void Motor::begin(const AxisConfig& cfg) {
  _cfg = cfg;
  enPinInit();

  if (!_s) _s = new AccelStepper(AccelStepper::DRIVER, _cfg.stepPin, _cfg.dirPin);

  _s->setPinsInverted(_cfg.invertDir, false, false);
  _s->setMaxSpeed(_cfg.maxSpeedSps);
  _s->setAcceleration(_cfg.accelSps2);
  _s->setCurrentPosition(0);
}

long  Motor::degToSteps(float deg)  const { return lround((double)deg / 360.0 * _cfg.stepsPerRev); }
float Motor::stepsToDeg(long steps) const { return (float)steps * 360.0f / (float)_cfg.stepsPerRev; }

void Motor::enable() {
  enPinInit();
  if (_holdsPower) return;
  _holdsPower = true;
  if (s_powerRefs++ == 0) digitalWrite(PIN_DRV_EN, LOW);
}

void Motor::disable() {
  if (!_holdsPower) return;
  _holdsPower = false;
  if (--s_powerRefs <= 0) {
    s_powerRefs = 0;
    digitalWrite(PIN_DRV_EN, HIGH);
  }
}

void Motor::powerOffAll() {
  enPinInit();
  s_powerRefs = 0;
  digitalWrite(PIN_DRV_EN, HIGH);
}

void Motor::setMotionProfile(float maxSps, float accelSps2) {
  if (!_s) return;
  _s->setMaxSpeed(maxSps);
  _s->setAcceleration(accelSps2);
}

void Motor::restoreMotionProfile() {
  setMotionProfile(_cfg.maxSpeedSps, _cfg.accelSps2);
}

void  Motor::zero()                 { if (_s) _s->setCurrentPosition(0); }
void  Motor::moveToAngle(float deg) { if (_s) _s->moveTo(degToSteps(deg)); }
void  Motor::stopSmooth()           { if (_s) _s->stop(); }
void  Motor::update()               { if (_s) _s->run(); }
bool  Motor::isMoving()             { return _s && _s->distanceToGo() != 0; }
float Motor::currentAngle()         { return _s ? stepsToDeg(_s->currentPosition()) : 0.0f; }
