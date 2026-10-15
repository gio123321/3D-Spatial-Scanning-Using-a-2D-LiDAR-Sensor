#include "ScanController.h"
#include "Config.h"

void ScanController::begin(Motor* motor, LidarBridge* lidar, Motor* tilt) {
  _m = motor; _l = lidar; _t = tilt; _state = ScanState::IDLE;
}

void ScanController::armTiltSweep(float from_deg, float to_deg, int cycles) {
  _sweep_from = constrain(from_deg, (float)TILT_MIN_DEG, (float)TILT_MAX_DEG);
  _sweep_to   = constrain(to_deg,   (float)TILT_MIN_DEG, (float)TILT_MAX_DEG);
  _sweep_cycles = constrain(cycles, 1, 60);
  _sweep_armed = true;
}

void ScanController::disarmTiltSweep() {
  _sweep_armed = false;
  _sweep_running = false;
  _sweep_paused_for_shot = false;
}

bool ScanController::startScan() {
  if (!_m || !_l || _state != ScanState::IDLE) return false;
  _m->restoreMotionProfile();    // undo the fast profile a previous rewind set
  _done_event = false;
  _returning_event = false;

  // Position camera tilt during LiDAR warmup; it may still be moving when
  // the independent azimuth capture begins.
  if (_sweep_armed && _t) {
    _t->setMotionProfile(TILT_JOG_SPS, TILT_JOG_ACCEL_SPS2);
    _t->enable();
    _t->moveToAngle(_sweep_from);
  }
  _sweep_running = false;

  // Set the current position to zero so every scan travels a full revolution
  // and returns to the same starting position.
  _m->zero();
  _l->start();                   // lidar on — warm-up begins
  _m->enable();
  _warmup_start = millis();
  _state = ScanState::WARMING_UP;
  return true;
}

// Oscillate between the configured tilt endpoints during the azimuth pass.
// Requested round trips determine the speed, bounded by TILT_JOG_SPS. Shot
// pauses and reversals affect the actual traversal count. Periodic CAMTILT_AT
// readings monitor tilt; IMAGE_META carries the commanded photograph pose.
void ScanController::startTiltSweep() {
  if (!_sweep_armed || !_t) return;

  float span = fabsf(_sweep_to - _sweep_from);
  if (span < 0.01f) { _sweep_armed = false; return; }

  // Seconds the azimuth pass will take.
  float az_secs = (SCAN_ANGLE_DEG / 360.0f * (float)STEPS_PER_REV_AZ)
                  / MAX_SPEED_SPS_AZ;

  // Each cycle is two traverses of the span.
  float traverses = 2.0f * (float)_sweep_cycles;
  float tilt_steps = span / 360.0f * (float)STEPS_PER_REV_TILT * traverses;
  float sps = tilt_steps / az_secs;

  if (sps < 1.0f) sps = 1.0f;
  if (sps > TILT_JOG_SPS) sps = TILT_JOG_SPS;   // do not outrun the axis

  // Keep reversal ramps short relative to each traverse.
  _t->setMotionProfile(sps, sps * 20.0f);
  _t->enable();
  _t->moveToAngle(_sweep_to);
  _sweep_target_is_to = true;
  _sweep_running = true;
  _sweep_paused_for_shot = false;
}

// Reverse at each end for as long as the azimuth pass lasts. Called from
// update() while FORWARD.
void ScanController::pumpTiltSweep() {
  if (!_sweep_running || !_t || _sweep_paused_for_shot) return;
  if (_t->isMoving()) return;
  _sweep_target_is_to = !_sweep_target_is_to;
  _t->moveToAngle(_sweep_target_is_to ? _sweep_to : _sweep_from);
}

// Stop camera tilt for exposure to avoid rolling-shutter pose errors.
// The independent azimuth axis continues collecting LiDAR.
void ScanController::pauseTiltForShot() {
  if (!_sweep_running || !_t || _sweep_paused_for_shot) return;
  _sweep_paused_for_shot = true;
  _t->stopSmooth();
}

bool ScanController::tiltReadyForShot() const {
  return !_sweep_running || !_t || (_sweep_paused_for_shot && !_t->isMoving());
}

void ScanController::resumeTiltAfterShot() {
  if (!_sweep_running || !_t || !_sweep_paused_for_shot) return;
  _sweep_paused_for_shot = false;
  _t->moveToAngle(_sweep_target_is_to ? _sweep_to : _sweep_from);
}

// Park the camera at the horizon after a sweep. Without a home switch,
// a different power-on position changes the commanded tilt reference.
void ScanController::parkTilt() {
  if (!_t) return;
  _t->setMotionProfile(TILT_JOG_SPS, TILT_JOG_ACCEL_SPS2);
  _t->enable();
  _t->moveToAngle(TILT_PARK_DEG);
}

void ScanController::abort() {
  if (_sweep_running && _t) {
    _t->stopSmooth();
    _t->restoreMotionProfile();
    _sweep_running = false;
  }
  _sweep_paused_for_shot = false;
  // Park tilt on abort as well as completion.
  parkTilt();
  if (_l) _l->stop();
  if (!_m) return;
  _m->enable();
  _m->stopSmooth();
  _state = ScanState::STOPPING;
}

void ScanController::update() {
  if (!_m) return;
  switch (_state) {
    case ScanState::WARMING_UP:
      if (millis() - _warmup_start >= LIDAR_WARMUP_MS) {
        _m->moveToAngle(SCAN_ANGLE_DEG);  // motor starts only after warm-up
        _state = ScanState::FORWARD;
        startTiltSweep();
      }
      break;
    case ScanState::FORWARD:
      pumpTiltSweep();   // reverse the camera at each end of its travel
      if (!_m->isMoving()) {
        if (_sweep_running && _t) {
          _t->restoreMotionProfile();
          _sweep_running = false;
        }
        _sweep_paused_for_shot = false;
        parkTilt();
        if (_l) _l->stop();       // capture is over; nothing to record on the way back
        // Rewind to the starting position so the lidar cable does not stay
        // wound up a full turn. Faster than the capture pass - see Config.h.
        _m->setMotionProfile(RETURN_SPEED_SPS_AZ, RETURN_ACCEL_SPS2_AZ);
        _m->moveToAngle(0.0f);
        _state = ScanState::RETURNING;
        _returning_event = true;
      }
      break;

    case ScanState::STOPPING:
      if (!_m->isMoving()) {
        _m->setMotionProfile(RETURN_SPEED_SPS_AZ, RETURN_ACCEL_SPS2_AZ);
        _m->moveToAngle(0.0f);
        _state = ScanState::RETURNING;
        _returning_event = true;
      }
      break;
    case ScanState::RETURNING:
      // Wait for both axes to settle before reporting completion.
      if (!_m->isMoving() && !(_t && _t->isMoving())) {
        if (_l) _l->stop();
        _m->restoreMotionProfile();
        _m->disable();            // release azimuth power
        if (_t) {
          _t->restoreMotionProfile();
          // Hold tilt coils energized so camera weight cannot back-drive the parked axis.
        }
        _state = ScanState::IDLE;
        _done_event = true;
      }
      break;
    default: break;
  }
}

bool ScanController::isActive()    { return _state != ScanState::IDLE; }
bool ScanController::isWarming()   { return _state == ScanState::WARMING_UP; }
bool ScanController::isCapturing() { return _state == ScanState::FORWARD; }
bool ScanController::isReturning() { return _state == ScanState::RETURNING; }

bool ScanController::consumeDoneEvent() {
  if (_done_event) { _done_event = false; return true; }
  return false;
}

bool ScanController::consumeReturningEvent() {
  if (_returning_event) { _returning_event = false; return true; }
  return false;
}
