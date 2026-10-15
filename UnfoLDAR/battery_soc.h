#pragma once
#include <Arduino.h>
#include "Config.h"   // BATTERY_SOC_UPDATE_MS / _REST_CURRENT_A / _CURRENT_SIGN
#include "Ina226.h"

// 2S Li-ion state-of-charge estimate using a dedicated pack-side INA226.
// Integrates current against PACK_CAPACITY_MAH and corrects voltage drift
// only while pack current is below BATTERY_SOC_REST_CURRENT_A.
// A missing sensor leaves present() false and does not block scanning.
class BatterySoc {
public:
  // Use the dedicated pack INA226 and the rated pack capacity.
  bool begin(float capacityMah, float shuntOhms, uint8_t addrHint = 0x40,
             uint8_t excludedAddr = 0) {
    _capacityMah = (capacityMah > 1.0f) ? capacityMah : 2000.0f;
    _socPercent  = -1.0f;      // unknown until the first good read
    _lastUpdateMs = 0;
    return _ina.begin(shuntOhms, addrHint, 1.0f, excludedAddr);
  }

  bool present() const { return _ina.present(); }
  uint8_t addr() const { return _ina.addr(); }

  // Rate-limited by BATTERY_SOC_UPDATE_MS. Reads the latest continuously
  // converted INA226 values; no conversion delay is required in the loop.
  void update() {
    uint32_t now = millis();
    if (_lastUpdateMs != 0 && (uint32_t)(now - _lastUpdateMs) < BATTERY_SOC_UPDATE_MS)
      return;
    float dtSec = (_lastUpdateMs == 0) ? 0.0f : (now - _lastUpdateMs) / 1000.0f;
    _lastUpdateMs = now;

    if (!_ina.present()) return;
    PowerSample p;
    if (!_ina.read(p)) return;   // leave the last known SOC in place
    _lastSample = p;
    _lastSample.current_a *= (float)BATTERY_SOC_CURRENT_SIGN;

    float ocvEstimate = voltageToSoc(p.bus_v);

    if (_socPercent < 0.0f) {
      // Initialize from voltage; assumes a near-rest reading at startup.
      _socPercent = ocvEstimate;
      return;
    }

    if (dtSec > 0.0f) {
      float mA = _lastSample.current_a * 1000.0f;   // positive = discharging
      _socPercent -= (mA * (dtSec / 3600.0f)) / _capacityMah * 100.0f;
    }

    // Correct voltage drift only at rest to limit load-sag error.
    if (fabsf(_lastSample.current_a) < BATTERY_SOC_REST_CURRENT_A) {
      const float kRestCorrection = 0.02f;  // 2% correction per rest sample
      _socPercent += (ocvEstimate - _socPercent) * kRestCorrection;
    }

    _socPercent = constrain(_socPercent, 0.0f, 100.0f);
  }

  float socPercent()    const { return _socPercent < 0.0f ? 0.0f : _socPercent; }
  float packVoltage()   const { return _lastSample.bus_v; }
  float currentMilliA() const { return _lastSample.current_a * 1000.0f; }

private:
  Ina226      _ina;
  float       _capacityMah  = 2000.0f;
  float       _socPercent   = -1.0f;   // -1 = no good reading yet
  uint32_t    _lastUpdateMs = 0;
  PowerSample _lastSample;

  // Generic piecewise-linear 2S open-circuit voltage curve, descending by voltage.
  // Use the installed cell specifications for a more accurate rest estimate.
  static float voltageToSoc(float packV) {
    struct Point { float volts, percent; };
    static const Point kCurve[] = {
      {8.40f, 100.0f}, {8.12f, 90.0f}, {7.96f, 80.0f}, {7.84f, 70.0f},
      {7.74f, 60.0f},  {7.64f, 50.0f}, {7.58f, 40.0f}, {7.54f, 30.0f},
      {7.46f, 20.0f},  {7.10f, 10.0f}, {6.80f, 5.0f},  {6.00f, 0.0f},
    };
    constexpr int n = sizeof(kCurve) / sizeof(kCurve[0]);

    if (packV >= kCurve[0].volts) return 100.0f;
    if (packV <= kCurve[n - 1].volts) return 0.0f;

    for (int i = 0; i < n - 1; i++) {
      if (packV <= kCurve[i].volts && packV >= kCurve[i + 1].volts) {
        float span = kCurve[i].volts - kCurve[i + 1].volts;
        float t = (span > 1e-6f) ? (packV - kCurve[i + 1].volts) / span : 0.0f;
        return kCurve[i + 1].percent + t * (kCurve[i].percent - kCurve[i + 1].percent);
      }
    }
    return 0.0f;  // unreachable given the two bounds checks above
  }
};
