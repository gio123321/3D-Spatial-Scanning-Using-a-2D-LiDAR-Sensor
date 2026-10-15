#pragma once
#include <Arduino.h>
#include "Motor.h"
#include "LidarBridge.h"

enum class ScanState { IDLE, WARMING_UP, FORWARD, RETURNING, STOPPING };

class ScanController {
public:
  void begin(Motor* motor, LidarBridge* lidar, Motor* tilt = nullptr);
  bool startScan();
  void abort();
  void update();

  // Arm repeated camera-only traverses between from/to for the next scan.
  // Requested cycles and azimuth duration determine the sweep speed. Tilt
  // pauses for photographs; the LiDAR remains on the azimuth platform, so
  // camera tilt does not change captured point geometry.
  void armTiltSweep(float from_deg, float to_deg, int cycles);
  void disarmTiltSweep();
  void startTiltSweep();
  void pumpTiltSweep();
  void pauseTiltForShot();
  bool tiltReadyForShot() const;
  void resumeTiltAfterShot();
  void parkTilt();              // send the camera back to the horizon
  bool tiltSweepArmed() const { return _sweep_armed; }
  float tiltSweepFrom()  const { return _sweep_from; }
  float tiltSweepTo()    const { return _sweep_to; }
  int   tiltSweepCycles() const { return _sweep_cycles; }
  bool isActive();              // true while a scan is in progress
  bool isWarming();             // true during lidar warm-up (before motor moves)
  bool isCapturing();           // true only during the forward motor pass
  bool isReturning();           // true while rewinding to the start position
  bool consumeDoneEvent();
  bool consumeReturningEvent();
private:
  Motor*  _m = nullptr;
  Motor*  _t = nullptr;         // tilt - carries the camera only
  LidarBridge*   _l = nullptr;
  ScanState      _state = ScanState::IDLE;
  bool           _done_event = false;
  bool           _returning_event = false;
  unsigned long  _warmup_start = 0;

  bool  _sweep_armed = false;
  float _sweep_from  = 0.0f;
  float _sweep_to    = 0.0f;
  int   _sweep_cycles = 1;
  bool  _sweep_target_is_to = true;
  bool  _sweep_running = false;
  bool  _sweep_paused_for_shot = false;
};
