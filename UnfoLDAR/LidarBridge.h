#pragma once
#include <Arduino.h>
#include <BluetoothSerial.h>
#include "Config.h"

// Phone-facing Bluetooth SPP link, owned by the sketch.
extern BluetoothSerial HOST;

class LidarBridge {
public:
  void begin();
  void start();                 // A5 60 -> enter scan mode
  void stop();                  // A5 65 -> standby
  void pump(float theta_deg);   // forward one bounded lidar frame tagged with theta
  void drain();                 // read and discard all pending lidar bytes (warm-up)
  bool isRunning();
  uint32_t droppedFrames() const { return _droppedFrames; }
private:
  bool _running = false;
  uint32_t _droppedFrames = 0;
  uint8_t _chunk[LIDAR_CHUNK_MAX];
};
