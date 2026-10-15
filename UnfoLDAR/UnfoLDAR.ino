#include "Config.h"
#include "Motor.h"
#include "LidarBridge.h"
#include "ScanController.h"
#include "Imu.h"
#include "Mt6835.h"
#include "Ina226.h"
#include "battery_soc.h"

#include <Wire.h>
#include <SPI.h>
#include <BluetoothSerial.h>
#include <string.h>

// Phone-facing Bluetooth SPP link for EVT/OK/ERR lines and binary frames.
BluetoothSerial HOST;

// Types used by Arduino-generated prototypes must precede the first function.
enum class Command { NONE, SCAN, LIDAR_ON, STOP, PING, STATUS, MOTOR_TEST,
                     I2CSCAN, ENCTEST, CAMTEST, POWER, SENSORS,
                     TILT, TILT_ZERO, PAN, POSE, HOLD, CAMTILT, DRAINCAM, CAMPING,
                     CAMCMD, TELEM_ON, TELEM_OFF, UNKNOWN };

struct ShotMeta {
  bool used;
  uint16_t idx;
  float cmdAz, cmdTilt, measAz, measTilt;
  bool encAz, encTilt;
};

static String cmdBuf;
static String g_arg;         // argument of the command just parsed, "" if none

// Verb plus optional argument: "TILT -12.5" sets g_arg to "-12.5".
Command readCommand() {
  while (HOST.available()) {
    char c = (char)HOST.read();
    if (c != '\n' && c != '\r') {
      if (cmdBuf.length() < 31) cmdBuf += c;
      continue;
    }
    if (cmdBuf.length() == 0) return Command::NONE;

    cmdBuf.trim(); cmdBuf.toUpperCase();
    String line = cmdBuf; cmdBuf = "";

    int sp = line.indexOf(' ');
    String verb = (sp < 0) ? line : line.substring(0, sp);
    g_arg = (sp < 0) ? String("") : line.substring(sp + 1);
    g_arg.trim();

    if (verb == "SCAN")       return Command::SCAN;
    if (verb == "LIDAR_ON")   return Command::LIDAR_ON;
    if (verb == "STOP")       return Command::STOP;
    if (verb == "PING")       return Command::PING;
    if (verb == "STATUS")     return Command::STATUS;
    if (verb == "MOTOR_TEST") return Command::MOTOR_TEST;
    if (verb == "I2CSCAN")    return Command::I2CSCAN;
    if (verb == "ENCTEST")    return Command::ENCTEST;
    if (verb == "CAMTEST")    return Command::CAMTEST;
    if (verb == "POWER")      return Command::POWER;
    if (verb == "SENSORS")    return Command::SENSORS;
    if (verb == "TILT")       return Command::TILT;
    if (verb == "TILT_ZERO")  return Command::TILT_ZERO;
    if (verb == "PAN")        return Command::PAN;
    if (verb == "POSE")       return Command::POSE;
    if (verb == "HOLD")       return Command::HOLD;
    if (verb == "CAMTILT")    return Command::CAMTILT;
    if (verb == "DRAINCAM")   return Command::DRAINCAM;
    if (verb == "CAMPING")    return Command::CAMPING;
    if (verb == "CAMCMD")     return Command::CAMCMD;
    if (verb == "TELEM")
      return (g_arg == "OFF") ? Command::TELEM_OFF : Command::TELEM_ON;
    return Command::UNKNOWN;
  }
  return Command::NONE;
}

Motor          motor;        // azimuth - the scan axis
Motor          motorTilt;    // tilt - jogged by hand, not part of the scan pass
LidarBridge    lidar;
ScanController scan;
Imu            imu;
Mt6835         encAz;
Mt6835         encTilt;
Ina226         ina;
BatterySoc     gauge;        // pack SOC - a SEPARATE INA226 from `ina` above
bool lidarOnly = false;

static bool     g_encAzPresent   = false;
static bool     g_encTiltPresent = false;
static bool     g_inaPresent     = false;
static bool     g_batteryPresent = false;
static uint32_t g_lastTelemMs    = 0;
static uint32_t g_lastBatteryPktMs = 0;
static bool     g_telemOn        = true;

// Start each disconnected LED cycle with a full one-second ON interval.
static void pumpBluetoothStatusLed() {
  static uint32_t lastToggleMs = 0;
  static bool ledOn = false;
  static bool waitingStarted = false;

  if (HOST.hasClient()) {
    if (ledOn) digitalWrite(PIN_BT_STATUS_LED, LOW);
    ledOn = false;
    waitingStarted = false;
    return;
  }

  if (!waitingStarted) {
    waitingStarted = true;
    ledOn = true;
    lastToggleMs = millis();
    digitalWrite(PIN_BT_STATUS_LED, HIGH);
    return;
  }

  if (millis() - lastToggleMs >= BT_LED_TOGGLE_MS) {
    lastToggleMs = millis();
    ledOn = !ledOn;
    digitalWrite(PIN_BT_STATUS_LED, ledOn ? HIGH : LOW);
  }
}

// HOLD keeps both drivers energized between camera positioning commands.
static bool     g_hold           = false;

// PAN/TILT acknowledge immediately; emit ARRIVED when asynchronous motion stops.
static bool     g_azWasMoving    = false;
static bool     g_tiltWasMoving  = false;
static uint32_t g_lastCamTiltMs  = 0;

// g_shotIdx also counts the frames expected by DRAINCAM.
static uint16_t g_shotIdx        = 0;
static float    g_lastShotAz     = 0.0f;
enum class ShotCaptureState { IDLE, STOPPING_TILT, WAITING_ACK };
static ShotCaptureState g_shotCaptureState = ShotCaptureState::IDLE;
static uint32_t g_shotCaptureStateMs = 0;
static bool     g_camSerialStarted = false;
static bool     g_camResetReleased = false;
static uint32_t g_camResetReleaseMs = 0;

static ShotMeta g_shotMeta[CAM_MAX_TRACKED_SHOTS];

// Probe I2C devices and candidate IMU identity registers.
static void i2cScanReport() {
#if !ENABLE_IMU && !ENABLE_INA226
  HOST.println("OK I2CSCAN skipped (ENABLE_IMU and ENABLE_INA226 are both 0)");
  return;
#else
  HOST.println("OK I2CSCAN");
  uint8_t found = 0;

  for (uint8_t a = 0x08; a <= 0x77; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() != 0) continue;
    found++;
    HOST.printf("  device 0x%02X%s\n", a,
                  (a >= 0x40 && a <= 0x4F) ? "  (INA226 address range)" : "");

    // Candidate identity registers across the parts this board is sold with.
    //   0x75 ICM-4xxxx WHO_AM_I     0x72 ICM-456xx WHO_AM_I
    //   0x00 BMI323 CHIP_ID          0x0F BMI160/BMA CHIP_ID
    //   0x01 QMI8658 WHO_AM_I
    static const uint8_t regs[] = { 0x75, 0x72, 0x00, 0x0F, 0x01 };
    for (uint8_t i = 0; i < sizeof(regs); i++) {
      uint8_t reg = regs[i], v = 0xFF;
      Wire.beginTransmission(a);
      Wire.write(reg);
      if (Wire.endTransmission(false) != 0) continue;
      if (Wire.requestFrom((int)a, 1) != 1) continue;
      v = Wire.read();
      HOST.printf("    reg 0x%02X = 0x%02X\n", reg, v);
    }
  }

  HOST.printf("  %u device(s) on SDA=%d SCL=%d\n", found, PIN_I2C_SDA, PIN_I2C_SCL);
  if (found == 0)
    HOST.println("  No I2C devices: check SDA/SCL, 3V3 and IMU CS strapping.");
  HOST.println("  known ids: 0x47=ICM-42688-P(0x75) 0xE9=ICM-45686(0x72) 0x43=BMI323(0x00)");
  HOST.println("  INA226 identifies by reg 0xFE=0x5449 and 0xFF=0x2260, not by address.");
#endif
}

// Decode MT6835 STATUS[2:0] for diagnostics.
static const char* mt6835StatusText(uint8_t st) {
  static char buf[40];
  if (st == 0) return "ok";
  buf[0] = 0;
  if (st & Mt6835::STATUS_OVERSPEED)    strcat(buf, "overspeed,");
  if (st & Mt6835::STATUS_WEAK_FIELD)   strcat(buf, "WEAK_FIELD,");
  if (st & Mt6835::STATUS_UNDERVOLTAGE) strcat(buf, "undervoltage,");
  size_t n = strlen(buf);
  if (n && buf[n - 1] == ',') buf[n - 1] = 0;
  return buf;
}

// Compare internal pull-up/down readings to detect a floating line.
static const char* probeLine(uint8_t pin) {
  pinMode(pin, INPUT_PULLUP);
  delayMicroseconds(500);
  int up = digitalRead(pin);
  pinMode(pin, INPUT_PULLDOWN);
  delayMicroseconds(500);
  int down = digitalRead(pin);
  pinMode(pin, INPUT);

  if (up == HIGH && down == LOW)  return "floating";      // nothing connected
  if (up == LOW  && down == LOW)  return "held LOW";      // tied low / unpowered chip
  if (up == HIGH && down == HIGH) return "held HIGH";
  return "driven";
}

// Check the external DRV_EN pull-up before configuring the pin as an output.
// The pull-up keeps both drivers disabled during startup.
static bool drvEnPullupFitted() {
  pinMode(PIN_DRV_EN, INPUT_PULLDOWN);
  delayMicroseconds(500);
  bool high = (digitalRead(PIN_DRV_EN) == HIGH);
  pinMode(PIN_DRV_EN, INPUT);
  return high;
}

static void dumpRead(const char* label, Mt6835Reading r) {
  HOST.printf("  %-22s %02X %02X %02X %02X  crc_recv=%02X crc_calc=%02X  %s\n",
                label, r.b[0], r.b[1], r.b[2], r.b[3], r.crcRecv, r.crcCalc,
                r.ok ? "VALID" : (r.degenerate ? "no data" : "CRC FAIL"));
}

// Compare encoder reads on the shared SPI bus.
static void encTestOne(const char* name, Mt6835& enc) {
  HOST.printf("  --- %s (CSN=GPIO%d) ---\n", name, enc.csPin());

  // Compare SPI modes and clock rates.
  dumpRead("mode 3 @ 1 MHz:",   enc.readBurst(SPI_MODE3, 1000000));
  dumpRead("mode 0 @ 1 MHz:",   enc.readBurst(SPI_MODE0, 1000000));
  dumpRead("mode 3 @ 100 kHz:", enc.readBurst(SPI_MODE3, 100000));

  // Check whether repeated reads change.
  uint32_t first = 0;
  bool     varies = false, anyValid = false;
  for (uint8_t i = 0; i < 8; i++) {
    Mt6835Reading r = enc.readBurst();
    uint32_t packed = ((uint32_t)r.b[0] << 24) | ((uint32_t)r.b[1] << 16) |
                      ((uint32_t)r.b[2] << 8)  |  (uint32_t)r.b[3];
    if (i == 0) first = packed;
    else if (packed != first) varies = true;
    if (r.ok) anyValid = true;
    delay(5);
  }
  HOST.printf("  8 reads              : %s, %s\n",
                varies ? "values change" : "all identical",
                anyValid ? "at least one valid" : "none valid");
}

// Work out why an encoder is silent, without reflashing the test firmware.
static void encTestReport() {
#if !ENABLE_ENCODER
  HOST.println("OK ENCTEST skipped (ENABLE_ENCODER is 0)");
  return;
#else
  HOST.println("OK ENCTEST");
  HOST.printf("  bus: SCK=GPIO%d MISO=GPIO%d MOSI=GPIO%d  CS_AZ=GPIO%d CS_TILT=GPIO%d\n",
                PIN_ENC_SCK, PIN_ENC_MISO, PIN_ENC_MOSI,
                PIN_ENC_CS_AZ, PIN_ENC_CS_TILT);

  // Probe MISO before SPI touches it again, then restore the bus. MISO is
  // shared, so this one verdict covers both encoders.
  const char* miso = probeLine(PIN_ENC_MISO);
  SPI.begin(PIN_ENC_SCK, PIN_ENC_MISO, PIN_ENC_MOSI, -1);
  encAz.begin(&SPI, PIN_ENC_CS_AZ);
  encTilt.begin(&SPI, PIN_ENC_CS_TILT);
  HOST.printf("  MISO (GPIO%d) line   : %s\n", PIN_ENC_MISO, miso);

  encTestOne("azimuth", encAz);
  encTestOne("tilt",    encTilt);

  HOST.print("  VERDICT: ");
  if (!strcmp(miso, "floating")) {
    HOST.printf("MISO is floating. GPIO%d is not connected to either encoder's "
                  "DO/SDO pin.\n", PIN_ENC_MISO);
  } else if (!strcmp(miso, "held LOW")) {
    HOST.println("MISO is held LOW - the encoders have no 3V3, or DO is shorted to GND.");
  } else {
    HOST.println("Check each encoder's CRC and status above; verify CSN, SCK and MOSI.");
  }
#endif
}

// On-demand power readings and raw registers.
static void powerReport() {
#if !ENABLE_INA226
  HOST.println("OK POWER skipped (ENABLE_INA226 is 0)");
  return;
#else
  HOST.println("OK POWER");
  if (!ina.present()) {
    HOST.println("  INA226 not present.");
    HOST.println("  Run I2CSCAN; check power, wiring and address straps.");
    return;
  }
  HOST.printf("  addr=0x%02X mfg=0x%04X die=0x%04X shunt=%.4f ohm bus_scale=%.5f\n",
                ina.addr(), ina.mfgId(), ina.dieId(), (double)INA226_SHUNT_OHMS,
                (double)INA226_BUS_SCALE);

  PowerSample p;
  if (!ina.read(p)) { HOST.println("  read FAILED - I2C error"); return; }

  HOST.printf("  bus   %8.3f V\n",  (double)p.bus_v);
  HOST.printf("  shunt %8.3f mV\n", (double)p.shunt_mv);
  HOST.printf("  curr  %8.3f A\n",  (double)p.current_a);
  HOST.printf("  power %8.3f W\n",  (double)p.power_w);

  // Report raw counts alongside the decode for comparison with a meter.
  Ina226::RawRegs r = ina.readRaw();
  if (!r.ok) { HOST.println("  raw register read FAILED"); return; }

  HOST.println("  --- raw registers ---");
  HOST.printf("  0x00 CONFIG = 0x%04X  (expect 0x4527: avg 16, 1.1 ms, continuous)\n", r.config);
  HOST.printf("  0x01 SHUNT  = 0x%04X  = %6d counts x 2.5 uV  = %8.3f mV\n",
                r.shunt, (int)(int16_t)r.shunt, (double)((int16_t)r.shunt * 0.0025f));
  HOST.printf("  0x02 BUS    = 0x%04X  = %6u counts x 1.25 mV = %8.3f V\n",
                r.bus, r.bus, (double)(r.bus * 0.00125f));
  HOST.printf("  0xFE MFG    = 0x%04X  (expect 0x5449)\n", r.mfg);
  HOST.printf("  0xFF DIE    = 0x%04X  (expect 0x2260)\n", r.die);

  // The +-81.92 mV differential input is the hard limit, not the shunt's power
  // rating: past it the current reading clips instead of reading high.
  if (fabsf(p.shunt_mv) > 78.0f)
    HOST.println("  WARN shunt voltage is near the +-81.92 mV full scale - the current"
                   " reading is about to clip. Fit a smaller shunt.");
  if (p.bus_v < 0.2f)
    HOST.println("  NOTE bus is at ~0 V: the bus pin senses the load side, so this means"
                   " the load rail is off or the shunt is not in circuit.");
#endif
}

// Live key=value telemetry; omit numeric fields for absent sensors.
// Send one buffer after LiDAR traffic. Gravity-derived tilt is only valid
// while stationary; motion flags let the host reject moving readings.
static void pumpTelemetry() {
  if (!g_telemOn) return;

  uint32_t period = (scan.isActive() || lidarOnly) ? TELEM_SCAN_MS : TELEM_IDLE_MS;
  if (millis() - g_lastTelemMs < period) return;
  g_lastTelemMs = millis();

  const char* state = scan.isCapturing()  ? "SCANNING"   :
                      (scan.isWarming()   ? "WARMING"    :
                      (scan.isReturning() ? "RETURNING"  :
                      (scan.isActive()    ? "STOPPING"   :
                      (lidarOnly          ? "LIDAR_ONLY" : "IDLE"))));

  char line[400];
  int  n = 0;
  n += snprintf(line + n, sizeof(line) - n,
                "EVT TELEM ms=%lu state=%s cmd_az=%.3f cmd_tilt=%.3f "
                "mov_az=%d mov_tilt=%d drv_en=%d",
                (unsigned long)millis(), state,
                motor.currentAngle(), motorTilt.currentAngle(),
                motor.isMoving() ? 1 : 0, motorTilt.isMoving() ? 1 : 0,
                digitalRead(PIN_DRV_EN));

  ImuSample s;
  if (imu.present() && imu.read(s)) {
    n += snprintf(line + n, sizeof(line) - n,
                  " imu=1 ax=%.3f ay=%.3f az=%.3f gx=%.2f gy=%.2f gz=%.2f"
                  " pitch=%.2f roll=%.2f amag=%.3f",
                  s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.pitch, s.roll, s.magnitude);
  } else {
    n += snprintf(line + n, sizeof(line) - n, " imu=0");
  }

  // Include status bits so weak-field readings can be identified.
  if (g_encAzPresent) {
    Mt6835Reading r = encAz.readBurst();
    if (r.ok) n += snprintf(line + n, sizeof(line) - n,
                            " encaz=1 encaz_deg=%.3f encaz_st=%u", r.deg, r.status);
    else      n += snprintf(line + n, sizeof(line) - n,
                            " encaz=0 encaz_err=%s", r.degenerate ? "no_data" : "crc");
  } else {
    n += snprintf(line + n, sizeof(line) - n, " encaz=0");
  }

  if (g_encTiltPresent) {
    Mt6835Reading r = encTilt.readBurst();
    if (r.ok) n += snprintf(line + n, sizeof(line) - n,
                            " enctl=1 enctl_deg=%.3f enctl_st=%u", r.deg, r.status);
    else      n += snprintf(line + n, sizeof(line) - n,
                            " enctl=0 enctl_err=%s", r.degenerate ? "no_data" : "crc");
  } else {
    n += snprintf(line + n, sizeof(line) - n, " enctl=0");
  }

  PowerSample p;
  if (g_inaPresent && ina.read(p)) {
    n += snprintf(line + n, sizeof(line) - n,
                  " ina=1 vbus=%.3f vsh=%.3f cur=%.4f pwr=%.3f"
                  " vbus_raw=%u vsh_raw=%d",
                  p.bus_v, p.shunt_mv, p.current_a, p.power_w,
                  (unsigned)p.bus_raw, (int)p.shunt_raw);
  } else {
    n += snprintf(line + n, sizeof(line) - n, " ina=0");
  }

  if (n < 0) return;                       // encoding error - nothing to send
  if (n >= (int)sizeof(line)) n = sizeof(line) - 1;   // clamp, snprintf never overran
  line[n++] = '\n';
  HOST.write((const uint8_t*)line, n);     // best-effort telemetry
}

// Send the binary battery/SOC frame defined in Config.h.
// BatterySoc::update() limits sensor reads internally.
static void pumpBatterySoc() {
  gauge.update();

  if (millis() - g_lastBatteryPktMs < BATTERY_PKT_MS) return;
  g_lastBatteryPktMs = millis();
  if (!gauge.present()) return;   // nothing to report without the pack sensor

  uint16_t socX10    = (uint16_t)constrain(gauge.socPercent() * 10.0f, 0.0f, 1000.0f);
  uint16_t voltageMv = (uint16_t)constrain(gauge.packVoltage() * 1000.0f, 0.0f, 65535.0f);
  int16_t  currentMa = (int16_t)constrain(gauge.currentMilliA(), -32768.0f, 32767.0f);

  uint8_t frame[12];
  frame[0] = BATTERY_PKT_SYNC0;
  frame[1] = BATTERY_PKT_SYNC1;
  frame[2] = (uint8_t)socX10;         frame[3] = (uint8_t)(socX10 >> 8);
  frame[4] = (uint8_t)voltageMv;      frame[5] = (uint8_t)(voltageMv >> 8);
  frame[6] = (uint8_t)(uint16_t)currentMa;
  frame[7] = (uint8_t)(((uint16_t)currentMa) >> 8);
  uint32_t crc = crc32Update(0, frame + 2, 6);
  writeU32LE(frame + 8, crc);
  HOST.write(frame, sizeof(frame));    // best-effort telemetry
}

// Shortest signed difference from a to b, in [-180, 180) for wrapped angles.
static float angleDelta(float a, float b) {
  return fmodf(b - a + 540.0f, 360.0f) - 180.0f;
}

// Absolute tilt positioning within software travel limits.
static void doTilt(const String& arg) {
  // Move the camera tilt independently of the LiDAR azimuth axis.
  if (arg.length() == 0) {
    HOST.println("ERR TILT needs an angle, e.g. TILT -12.5");
    return;
  }
  float target  = arg.toFloat();
  float clamped = constrain(target, (float)TILT_MIN_DEG, (float)TILT_MAX_DEG);
  if (clamped != target)
    HOST.printf("EVT WARN tilt %.2f clamped to %.2f (limits %.1f..%.1f)\n",
                  target, clamped, (double)TILT_MIN_DEG, (double)TILT_MAX_DEG);

  motorTilt.setMotionProfile(TILT_JOG_SPS, TILT_JOG_ACCEL_SPS2);
  motorTilt.enable();
  motorTilt.moveToAngle(clamped);
  HOST.printf("OK TILT %.3f\n", clamped);

  // Emit ARRIVED for zero-length moves, which have no moving-to-stopped edge.
  if (!motorTilt.isMoving())
    HOST.printf("EVT ARRIVED tilt %.3f\n", motorTilt.currentAngle());
}

// Move azimuth in commanded scan coordinates, within the cable travel limits.
// Reject positioning commands while the scan controller owns the axis.
static void doPan(const String& arg) {
  if (scan.isActive() || lidarOnly) {
    HOST.println("ERR BUSY azimuth belongs to the scan controller");
    return;
  }
  if (arg.length() == 0) {
    HOST.println("ERR PAN needs an angle, e.g. PAN -12.5");
    return;
  }
  float target  = arg.toFloat();
  float clamped = constrain(target, (float)PAN_MIN_DEG, (float)PAN_MAX_DEG);
  if (clamped != target)
    HOST.printf("EVT WARN pan %.2f clamped to %.2f (limits %.1f..%.1f)\n",
                  target, clamped, (double)PAN_MIN_DEG, (double)PAN_MAX_DEG);

  motor.setMotionProfile(PAN_JOG_SPS, PAN_JOG_ACCEL_SPS2);
  motor.enable();
  motor.moveToAngle(clamped);
  HOST.printf("OK PAN %.3f\n", clamped);

  // A zero-length PAN still needs an ARRIVED reply.
  if (!motor.isMoving())
    HOST.printf("EVT ARRIVED az %.3f\n", motor.currentAngle());
}

// CAMTILT <from> <to> arms a sweep; CAMTILT OFF cancels it.
static void doCamTilt(const String& arg) {
  if (arg.length() == 0 || arg == "OFF") {
    scan.disarmTiltSweep();
    HOST.println("OK CAMTILT OFF");
    return;
  }

  int sp = arg.indexOf(' ');
  if (sp < 0) {
    HOST.println("ERR CAMTILT needs two angles, e.g. CAMTILT 40 -40 8");
    return;
  }
  float from = arg.substring(0, sp).toFloat();
  String rest = arg.substring(sp + 1); rest.trim();

  int sp2 = rest.indexOf(' ');
  float to  = (sp2 < 0) ? rest.toFloat() : rest.substring(0, sp2).toFloat();
  int cycles = (sp2 < 0) ? 1 : rest.substring(sp2 + 1).toInt();
  if (cycles < 1) cycles = 1;

  scan.armTiltSweep(from, to, cycles);

  // Report the clamped sweep settings used by the firmware.
  HOST.printf("OK CAMTILT %.3f %.3f %d\n",
                scan.tiltSweepFrom(), scan.tiltSweepTo(), scan.tiltSweepCycles());
}

// Read commanded and measured angles synchronously for both axes.
// Field names and absent-sensor values match pumpTelemetry().
static void poseReport() {
  HOST.printf("OK POSE ms=%lu cmd_az=%.3f cmd_tilt=%.3f mov_az=%d mov_tilt=%d",
                (unsigned long)millis(),
                motor.currentAngle(), motorTilt.currentAngle(),
                motor.isMoving() ? 1 : 0, motorTilt.isMoving() ? 1 : 0);

  if (g_encAzPresent) {
    Mt6835Reading r = encAz.readBurst();
    if (r.ok) HOST.printf(" encaz=1 encaz_deg=%.4f encaz_st=%u", r.deg, r.status);
    else      HOST.printf(" encaz=0 encaz_err=%s", r.degenerate ? "no_data" : "crc");
  } else {
    HOST.print(" encaz=0");
  }

  if (g_encTiltPresent) {
    Mt6835Reading r = encTilt.readBurst();
    if (r.ok) HOST.printf(" enctl=1 enctl_deg=%.4f enctl_st=%u", r.deg, r.status);
    else      HOST.printf(" enctl=0 enctl_err=%s", r.degenerate ? "no_data" : "crc");
  } else {
    HOST.print(" enctl=0");
  }

  HOST.println();
}

// Trigger camera shots during capture and relay buffered JPEGs afterward.
// Wire formats are defined in Config.h.

static bool ensureCamSerialStarted() {
  if (g_camSerialStarted) return true;
  if (!g_camResetReleased) {
    if (millis() < CAM_POWER_SETTLE_MS) return false;
    digitalWrite(PIN_CAM_RESET, HIGH);  // open-drain release
    g_camResetReleased = true;
    g_camResetReleaseMs = millis();
    return false;
  }
  if (millis() - g_camResetReleaseMs < CAM_BOOT_WAIT_MS) return false;

  CAM_SERIAL.setRxBufferSize(1024);
  CAM_SERIAL.begin(CAM_BAUD, SERIAL_8N1, PIN_CAM_RX, PIN_CAM_TX);
  while (CAM_SERIAL.available()) CAM_SERIAL.read();
  g_camSerialStarted = true;
  return true;
}

// Starts a non-blocking capture roughly every SHOT_PAN_STEP_DEG. Tilt first
// decelerates to a complete stop, then TRIGGER is sent. The sweep resumes only
// after the camera ACK, ensuring every JPEG has one rigid cmd_tilt pose.
static void pumpShotTrigger() {
  if (!(scan.isCapturing() && scan.tiltSweepArmed())) {
    g_shotCaptureState = ShotCaptureState::IDLE;
    return;
  }
  if (!ensureCamSerialStarted()) return;
  if (g_shotCaptureState == ShotCaptureState::WAITING_ACK) {
    if (millis() - g_shotCaptureStateMs > CAM_SHOT_ACK_TIMEOUT_MS) {
      HOST.println("EVT WARN camera shot ACK timeout; resuming tilt sweep");
      scan.resumeTiltAfterShot();
      g_shotCaptureState = ShotCaptureState::IDLE;
    }
    return;
  }
  if (g_shotCaptureState == ShotCaptureState::STOPPING_TILT) {
    if (!scan.tiltReadyForShot()) return;
  } else {
    if (g_shotIdx > 0 && fabsf(angleDelta(g_lastShotAz, motor.currentAngle())) < SHOT_PAN_STEP_DEG)
      return;
    if (g_shotIdx >= CAM_MAX_TRACKED_SHOTS) return;
    scan.pauseTiltForShot();
    g_shotCaptureState = ShotCaptureState::STOPPING_TILT;
    g_shotCaptureStateMs = millis();
    return;
  }
  g_lastShotAz = motor.currentAngle();

  uint16_t idx = g_shotIdx++;
  ShotMeta& meta = g_shotMeta[idx];
  meta.used = true;
  meta.idx = idx;
  meta.cmdAz = meta.measAz = motor.currentAngle();
  meta.cmdTilt = meta.measTilt = motorTilt.currentAngle();
  meta.encAz = meta.encTilt = false;
  CAM_SERIAL.printf("TRIGGER %u\n", idx);
  g_shotCaptureState = ShotCaptureState::WAITING_ACK;
  g_shotCaptureStateMs = millis();

  char line[160];
  int  n = 0;
  n += snprintf(line + n, sizeof(line) - n,
                "EVT SHOT idx=%u ms=%lu cmd_az=%.3f cmd_tilt=%.3f",
                idx, (unsigned long)millis(), meta.cmdAz, meta.cmdTilt);
  if (g_encAzPresent) {
    Mt6835Reading r = encAz.readBurst();
    if (r.ok) {
      meta.encAz = true; meta.measAz = r.deg;
      n += snprintf(line + n, sizeof(line) - n,
                    " encaz=1 encaz_deg=%.4f encaz_st=%u", r.deg, r.status);
    }
    else      n += snprintf(line + n, sizeof(line) - n,
                            " encaz=0 encaz_err=%s", r.degenerate ? "no_data" : "crc");
  } else {
    n += snprintf(line + n, sizeof(line) - n, " encaz=0");
  }
  if (g_encTiltPresent) {
    Mt6835Reading r = encTilt.readBurst();
    if (r.ok) {
      meta.encTilt = true; meta.measTilt = r.deg;
      n += snprintf(line + n, sizeof(line) - n,
                    " enctl=1 enctl_deg=%.4f enctl_st=%u", r.deg, r.status);
    }
    else      n += snprintf(line + n, sizeof(line) - n,
                            " enctl=0 enctl_err=%s", r.degenerate ? "no_data" : "crc");
  } else {
    n += snprintf(line + n, sizeof(line) - n, " enctl=0");
  }
  if (n < 0) return;
  if (n >= (int)sizeof(line)) n = sizeof(line) - 1;
  line[n++] = '\n';
  HOST.write((const uint8_t*)line, n);   // best-effort telemetry
}

// Forward asynchronous capture acknowledgements as EVT CAM_ACK so the
// phone can distinguish triggered shots from successful captures.
static void pumpCamAcks() {
  static char line[64];
  static uint8_t n = 0;
  while (CAM_SERIAL.available()) {
    char c = (char)CAM_SERIAL.read();
    if (c == '\n') {
      if (n > 0) {
        line[n] = 0;
        // Tilt remains stationary during exposure. Azimuth keeps moving; use the
        // commanded midpoint between trigger and ACK for its estimated exposure pose.
        unsigned ackIdx = 0;
        if (sscanf(line, "OK SHOT idx=%u", &ackIdx) == 1) {
          for (uint16_t i = 0; i < g_shotIdx && i < CAM_MAX_TRACKED_SHOTS; i++) {
            ShotMeta& meta = g_shotMeta[i];
            if (!meta.used || meta.idx != (uint16_t)ackIdx) continue;
            float endAz = motor.currentAngle();
            float endTilt = motorTilt.currentAngle();
            meta.cmdAz += 0.5f * angleDelta(meta.cmdAz, endAz);
            meta.cmdTilt = 0.5f * (meta.cmdTilt + endTilt);
            break;
          }
        }
        HOST.printf("EVT CAM_ACK %s\n", line);
        if (g_shotCaptureState == ShotCaptureState::WAITING_ACK) {
          scan.resumeTiltAfterShot();
          g_shotCaptureState = ShotCaptureState::IDLE;
        }
      }
      n = 0;
    } else if (c != '\r' && n < sizeof(line) - 1) {
      line[n++] = c;
    }
  }
}

static uint32_t readU32LE(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void writeU32LE(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t len) {
  crc = ~crc;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++)
      crc = (crc >> 1) ^ (0xEDB88320UL & (uint32_t)-(int32_t)(crc & 1));
  }
  return ~crc;
}

static bool parseDecField(const String& line, const char* key, uint32_t& out) {
  String needle = String(key) + "=";
  int start = line.indexOf(needle);
  if (start < 0) return false;
  start += needle.length();
  int end = line.indexOf(' ', start);
  String value = end < 0 ? line.substring(start) : line.substring(start, end);
  char* tail = nullptr;
  unsigned long parsed = strtoul(value.c_str(), &tail, 10);
  if (!tail || *tail) return false;
  out = (uint32_t)parsed;
  return true;
}

static bool parseHexField(const String& line, const char* key, uint32_t& out) {
  String needle = String(key) + "=";
  int start = line.indexOf(needle);
  if (start < 0) return false;
  start += needle.length();
  int end = line.indexOf(' ', start);
  String value = end < 0 ? line.substring(start) : line.substring(start, end);
  char* tail = nullptr;
  unsigned long parsed = strtoul(value.c_str(), &tail, 16);
  if (!tail || *tail) return false;
  out = (uint32_t)parsed;
  return true;
}

static ShotMeta* findShotMeta(uint16_t idx) {
  for (uint16_t i = 0; i < g_shotIdx && i < CAM_MAX_TRACKED_SHOTS; i++)
    if (g_shotMeta[i].used && g_shotMeta[i].idx == idx) return &g_shotMeta[i];
  return nullptr;
}

static bool sendImageMeta(const ShotMeta& m, uint32_t len, uint32_t crc) {
  char line[192];
  int n = snprintf(line, sizeof(line),
      "EVT IMAGE_META idx=%u len=%lu crc32=%08lX cmd_az=%.3f cmd_tilt=%.3f "
      "meas_az=%.4f meas_tilt=%.4f encaz=%d enctl=%d\n",
      m.idx, (unsigned long)len, (unsigned long)crc,
      m.cmdAz, m.cmdTilt, m.measAz, m.measTilt, m.encAz ? 1 : 0, m.encTilt ? 1 : 0);
  return n > 0 && n < (int)sizeof(line) &&
         HOST.write((const uint8_t*)line, (size_t)n) == (size_t)n;
}

// Query captured frames and relay one requested chunk at a time.
// Handle STOP, link loss and whole-packet deadlines between chunks.
enum class DrainState { IDLE, COUNT, INFO, CHUNK, WRITE };
static DrainState g_drainState = DrainState::IDLE;
static uint32_t g_drainCount=0, g_drainOrdinal=0, g_drainRelayed=0, g_drainFailed=0;
static uint32_t g_drainTotal=0, g_drainOffset=0, g_drainExpectedCrc=0, g_drainCrc=0;
static uint16_t g_drainIdx=0, g_drainWant=0;
static uint8_t g_drainAttempt=0;
static uint32_t g_drainStarted=0;
static uint8_t g_drainPacket[14+CAM_TRANSFER_CHUNK+4];
static size_t g_drainPacketSize=0;
static String g_drainReply;
static bool g_scanAborted=false;
static const char* g_abortReason="none";
static const char* g_transferOutcome="none";

static bool drainActive() { return g_drainState != DrainState::IDLE; }

static void finishDrain(bool complete, const char* reason) {
  g_transferOutcome=complete ? "complete" : reason;
  g_drainState=DrainState::IDLE;
  HOST.printf("%s DRAINCAM expected=%lu relayed=%lu failed=%lu triggered=%u outcome=%s\n",
      complete ? "OK" : "ERR", (unsigned long)g_drainCount,
      (unsigned long)g_drainRelayed, (unsigned long)g_drainFailed, g_shotIdx, g_transferOutcome);
}

static void requestDrainInfo() {
  if (g_drainOrdinal>=g_drainCount) { finishDrain(true,"complete"); return; }
  CAM_SERIAL.printf("INFO %lu\n",(unsigned long)g_drainOrdinal);
  g_drainReply=""; g_drainStarted=millis(); g_drainState=DrainState::INFO;
}

static void requestDrainChunk() {
  g_drainWant=(uint16_t)min((uint32_t)CAM_TRANSFER_CHUNK,g_drainTotal-g_drainOffset);
  // Discard a timed-out packet before retrying the identical offset.
  while (CAM_SERIAL.available()) CAM_SERIAL.read();
  CAM_SERIAL.printf("READ %u %lu %u\n",g_drainIdx,(unsigned long)g_drainOffset,g_drainWant);
  g_drainPacketSize=0; g_drainStarted=millis(); g_drainState=DrainState::CHUNK;
}

static void endDrainImage(bool ok, const char* reason) {
  HOST.printf("EVT IMAGE_END idx=%u ok=%d reason=%s\n",g_drainIdx,ok ? 1 : 0,reason);
  if (ok) ++g_drainRelayed; else ++g_drainFailed;
  ++g_drainOrdinal; requestDrainInfo();
}

static void drainCamToHost() {
  if (drainActive()) { HOST.println("ERR BUSY"); return; }
  if (!ensureCamSerialStarted()) { HOST.println("ERR DRAINCAM camera_not_ready"); return; }
  g_drainCount=g_drainOrdinal=g_drainRelayed=g_drainFailed=0;
  g_transferOutcome="running"; g_drainReply="";
  CAM_SERIAL.println("COUNT"); g_drainStarted=millis(); g_drainState=DrainState::COUNT;
}

static void pumpDrain() {
  if (!drainActive()) return;
  if (!HOST.hasClient()) { finishDrain(false,"disconnected"); return; }
  if (g_drainState==DrainState::COUNT || g_drainState==DrainState::INFO) {
    while (CAM_SERIAL.available()) {
      char c=(char)CAM_SERIAL.read();
      if (c=='\r') continue;
      if (c!='\n') {
        if (g_drainReply.length()<192) g_drainReply+=c;
        else { finishDrain(false,"reply_overflow"); return; }
        continue;
      }
      String reply=g_drainReply; g_drainReply=""; reply.trim();
      // A final capture ACK may arrive before the COUNT reply.
      if (g_drainState==DrainState::COUNT) {
        if (!reply.startsWith("OK COUNT")) continue;
        if (!parseDecField(reply,"count",g_drainCount) || g_drainCount>CAM_MAX_TRACKED_SHOTS) {
          finishDrain(false,"bad_count"); return;
        }
        HOST.printf("EVT TRANSFER expected=%lu\n",(unsigned long)g_drainCount);
        requestDrainInfo(); return;
      }
      if (!reply.startsWith("OK INFO")) continue;
      uint32_t idx=0;
      if (!parseDecField(reply,"idx",idx) || idx>65535 ||
          !parseDecField(reply,"len",g_drainTotal) || g_drainTotal==0 || g_drainTotal>CAM_MAX_IMG_LEN ||
          !parseHexField(reply,"crc32",g_drainExpectedCrc)) {
        ++g_drainFailed; ++g_drainOrdinal; requestDrainInfo(); return;
      }
      g_drainIdx=(uint16_t)idx;
      ShotMeta* meta=findShotMeta(g_drainIdx);
      if (!meta || !sendImageMeta(*meta,g_drainTotal,g_drainExpectedCrc)) {
        endDrainImage(false,"missing_pose_or_host_write"); return;
      }
      g_drainOffset=g_drainCrc=0; g_drainAttempt=0; requestDrainChunk(); return;
    }
    if (millis()-g_drainStarted>CAM_PING_TIMEOUT_MS) finishDrain(false,"camera_reply_timeout");
    return;
  }
  if (g_drainState==DrainState::CHUNK) {
    while (CAM_SERIAL.available() && g_drainPacketSize<sizeof(g_drainPacket)) {
      uint8_t byte=(uint8_t)CAM_SERIAL.read();
      if (g_drainPacketSize==0 && byte!=CAM_CHUNK_SYNC0) continue;
      if (g_drainPacketSize==1 && byte!=CAM_CHUNK_SYNC1) { g_drainPacketSize=0; continue; }
      g_drainPacket[g_drainPacketSize++]=byte;
      if (g_drainPacketSize==14+g_drainWant+4) break;
    }
    if (g_drainPacketSize==14+g_drainWant+4) {
      const uint8_t* hdr=g_drainPacket+2;
      bool valid=((uint16_t)hdr[0]|((uint16_t)hdr[1]<<8))==g_drainIdx &&
        readU32LE(hdr+2)==g_drainOffset && readU32LE(hdr+6)==g_drainTotal &&
        ((uint16_t)hdr[10]|((uint16_t)hdr[11]<<8))==g_drainWant &&
        crc32Update(0,hdr,12+g_drainWant)==readU32LE(g_drainPacket+14+g_drainWant);
      if (valid) {
        g_drainPacket[0]=IMG_CHUNK_SYNC0; g_drainPacket[1]=IMG_CHUNK_SYNC1;
        g_drainStarted=millis(); g_drainState=DrainState::WRITE;
      } else if (++g_drainAttempt<=CAM_CHUNK_RETRIES) requestDrainChunk();
      else endDrainImage(false,"chunk_crc_or_header");
      return;
    }
    if (millis()-g_drainStarted>CAM_CHUNK_TIMEOUT_MS) {
      if (++g_drainAttempt<=CAM_CHUNK_RETRIES) requestDrainChunk();
      else endDrainImage(false,"chunk_timeout");
    }
    return;
  }
  if (g_drainState==DrainState::WRITE) {
    if (HOST.write(g_drainPacket,g_drainPacketSize)==g_drainPacketSize) {
      g_drainCrc=crc32Update(g_drainCrc,g_drainPacket+14,g_drainWant);
      g_drainOffset+=g_drainWant; g_drainAttempt=0;
      if (g_drainOffset==g_drainTotal) endDrainImage(g_drainCrc==g_drainExpectedCrc,"image_crc");
      else requestDrainChunk();
    } else if (millis()-g_drainStarted>IMG_DRAIN_WRITE_TIMEOUT_MS) endDrainImage(false,"host_timeout");
  }
}

static void stopAndPark(const char* reason) {
  if (drainActive()) finishDrain(false,reason);
  g_scanAborted=true; g_abortReason=reason;
  g_shotCaptureState=ShotCaptureState::IDLE;
  if (lidarOnly) { lidar.stop(); lidarOnly=false; }
  scan.abort(); g_hold=false;
}

// Forward a camera command and return its first trimmed reply line,
// or an empty string after the timeout. Used by CAMPING and CAMCMD.
static String camForward(const String& cmd, uint32_t timeoutMs) {
  if (!ensureCamSerialStarted()) return String();
  pumpCamAcks();                 // clear any stray bytes first
  CAM_SERIAL.println(cmd);

  uint32_t t0 = millis();
  String line;
  while (millis() - t0 < timeoutMs) {
    while (CAM_SERIAL.available()) {
      char c = (char)CAM_SERIAL.read();
      if (c == '\n') {
        line.trim();
        if (line.length()) return line;
        line = "";                 // blank line - keep listening
      } else if (c != '\r') {
        line += c;
      }
    }
  }
  return String();
}

static String camPingWithRetries() {
  String reply;
  for (uint8_t attempt = 1; attempt <= CAM_PING_RETRIES; ++attempt) {
    reply = camForward("PING", CAM_PING_TIMEOUT_MS);
    if (reply.startsWith("OK PONG") && reply.indexOf("protocol=2") >= 0)
      return reply;
    // CAMERA_NOT_READY means its CPU and UART are alive while the sensor-side
    // background retry is still running, so give that retry time as well.
    if (attempt < CAM_PING_RETRIES) delay(CAM_PING_RETRY_MS);
  }
  return reply;
}

static void camTestReport() {
  HOST.printf("OK CAMTEST UART1 baud=%lu main_tx=GPIO%d main_rx=GPIO%d\n",
              (unsigned long)CAM_BAUD, PIN_CAM_TX, PIN_CAM_RX);
  HOST.printf("  wiring: camera TX GPIO14 -> main RX GPIO%d; "
              "camera RX GPIO13 <- main TX GPIO%d; shared GND\n",
              PIN_CAM_RX, PIN_CAM_TX);
  String ping = camPingWithRetries();
  HOST.printf("  PING: %s\n", ping.length() ? ping.c_str() : "NO REPLY");
  String status = camForward("STATUS", CAM_PING_TIMEOUT_MS);
  HOST.printf("  STATUS: %s\n", status.length() ? status.c_str() : "NO REPLY");
  if (ping.indexOf("protocol=2") < 0)
    HOST.println("  VERDICT: camera missing, wrong firmware/protocol, or UART link unstable");
  else
    HOST.println("  VERDICT: camera protocol v2 link OK");
}

static void camPing() {
  String reply = camPingWithRetries();
  if (reply.startsWith("OK PONG") && reply.indexOf("protocol=2") >= 0)
    HOST.printf("OK CAMPING %s\n", reply.c_str());
  else if (reply.length())
    HOST.printf("ERR CAMPING camera replied '%s'\n", reply.c_str());
  else
    HOST.println("ERR CAMPING no reply from cam module (check wiring/baud/flash)");
}

// Forward CAMCMD arguments to the camera and return its first reply.
static void camForwardCmd(const String& arg) {
  if (arg.length() == 0) {
    HOST.println("ERR CAMCMD needs a command, e.g. CAMCMD STATUS");
    return;
  }
  String reply = camForward(arg, CAM_PING_TIMEOUT_MS);
  if (reply.length()) HOST.printf("OK CAMCMD %s\n", reply.c_str());
  else HOST.println("ERR CAMCMD no reply from cam module");
}

void setup() {
  // Hold camera reset until the switched supply settles.
  pinMode(PIN_CAM_RESET, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_CAM_RESET, LOW);

  // Keep camera UART pins high-impedance to avoid back-powering during startup.
  pinMode(PIN_CAM_TX, INPUT);
  pinMode(PIN_CAM_RX, INPUT);

  // Start the status LED low after the boot-strap level has been sampled.
  pinMode(PIN_BT_STATUS_LED, OUTPUT);
  digitalWrite(PIN_BT_STATUS_LED, LOW);

  // Drive STEP/DIR low before probing shared enable to prevent spurious motion.
  pinMode(PIN_STEP_AZ,   OUTPUT); digitalWrite(PIN_STEP_AZ,   LOW);
  pinMode(PIN_DIR_AZ,    OUTPUT); digitalWrite(PIN_DIR_AZ,    LOW);
  pinMode(PIN_STEP_TILT, OUTPUT); digitalWrite(PIN_STEP_TILT, LOW);
  pinMode(PIN_DIR_TILT,  OUTPUT); digitalWrite(PIN_DIR_TILT,  LOW);

  // Check the external pull-up before configuring DRV_EN as an output.
  bool enPullup = drvEnPullupFitted();

  Motor::powerOffAll();          // drivers de-energized

  HOST.begin(HOST_BT_NAME);
  delay(300);

  // Report firmware identity before initializing sensors.
  HOST.println("EVT BOOT UnfoLDAR 1.0.0-rc.1 protocol=2 board=revB");
  if (!enPullup) {
    HOST.printf("EVT WARN DRV_EN GPIO%d needs a 10k pull-up to 3V3 to prevent motion during reset.\n",
                PIN_DRV_EN);
  }
  HOST.flush();

  // Initialize both axes: shared DRV_EN can energize tilt during azimuth motion.
  motor.begin(AXIS_AZ);
  motorTilt.begin(AXIS_TILT);
  HOST.flush();

  lidar.begin();
  scan.begin(&motor, &lidar, &motorTilt);

  cmdBuf.reserve(32);
  g_arg.reserve(24);
  HOST.flush();

  // Optional sensors.
#if ENABLE_IMU || ENABLE_INA226 || ENABLE_BATTERY_SOC
  HOST.flush();
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000);
  Wire.setTimeOut(50);                    // bounded; a stuck bus must not hang us
  HOST.flush();
#endif

#if ENABLE_IMU
  HOST.flush();
  bool imuOk = imu.begin();
  HOST.printf("EVT IMU_PART %s present=%d\n", imu.partName(), imuOk ? 1 : 0);
  if (!imuOk)
    HOST.println("EVT WARN IMU unavailable; run I2CSCAN");
#else
  HOST.println("EVT IMU_PART disabled present=0");
#endif
  HOST.flush();

#if ENABLE_INA226
  HOST.flush();
  g_inaPresent = ina.begin(INA226_SHUNT_OHMS, INA226_ADDR_HINT, INA226_BUS_SCALE);
  HOST.printf("EVT INA_PART present=%d\n", g_inaPresent ? 1 : 0);
  if (!g_inaPresent)
    HOST.println("EVT WARN INA226 unavailable; run I2CSCAN");
#else
  g_inaPresent = false;
  HOST.println("EVT INA_PART disabled present=0");
#endif
  HOST.flush();

#if ENABLE_BATTERY_SOC
  HOST.flush();
  g_batteryPresent = gauge.begin(PACK_CAPACITY_MAH, INA226_PACK_SHUNT_OHMS,
                                  INA226_PACK_ADDR_HINT,
                                  g_inaPresent ? ina.addr() : 0);
  HOST.printf("EVT BATTERY_PART present=%d\n", g_batteryPresent ? 1 : 0);
  if (!g_batteryPresent)
    HOST.println("EVT WARN battery sensor unavailable; check wiring and address");
#else
  g_batteryPresent = false;
  HOST.println("EVT BATTERY_PART disabled present=0");
#endif
  HOST.flush();

#if ENABLE_ENCODER
  HOST.flush();
  SPI.begin(PIN_ENC_SCK, PIN_ENC_MISO, PIN_ENC_MOSI, -1);
  encAz.begin(&SPI, PIN_ENC_CS_AZ);
  encTilt.begin(&SPI, PIN_ENC_CS_TILT);
  {
    Mt6835Reading a = encAz.readBurst();
    g_encAzPresent = a.ok;
    HOST.printf("EVT ENC_AZ present=%d status=%s\n",
                  g_encAzPresent ? 1 : 0,
                  a.ok ? mt6835StatusText(a.status) : "n/a");

    Mt6835Reading t = encTilt.readBurst();
    g_encTiltPresent = t.ok;
    HOST.printf("EVT ENC_TILT present=%d status=%s\n",
                  g_encTiltPresent ? 1 : 0,
                  t.ok ? mt6835StatusText(t.status) : "n/a");

    if ((a.ok && (a.status & Mt6835::STATUS_WEAK_FIELD)) ||
        (t.ok && (t.status & Mt6835::STATUS_WEAK_FIELD))) {
      HOST.println("EVT WARN encoder weak field; check magnet");
    }
  }
#else
  g_encAzPresent = g_encTiltPresent = false;
  HOST.println("EVT ENC_AZ disabled present=0");
  HOST.println("EVT ENC_TILT disabled present=0");
#endif
  HOST.flush();

  HOST.println("EVT READY");
}

void loop() {
  static bool hadClient=false;
  bool client=HOST.hasClient();
  if (hadClient && !client) { stopAndPark("bluetooth_loss"); cmdBuf=""; }
  hadClient=client;
  pumpBluetoothStatusLed();
  ensureCamSerialStarted();
  motor.update();       // step engines — always first
  motorTilt.update();
  scan.update();        // check scan endpoints

  if (scan.isCapturing() || lidarOnly) {
    motor.update();                         // keep steps ahead of serial writes
    lidar.pump(motor.currentAngle());       // forward one lidar chunk tagged with angle
  } else if (scan.isWarming()) {
    lidar.drain();                          // discard unstable pre-scan readings
  }

  // Queue rate-limited telemetry after LiDAR frames.
  pumpTelemetry();
  pumpBatterySoc();       // rate-limited internally, see that function's comment
  if (!drainActive()) pumpCamAcks(); // keep CAM_SERIAL's RX ring from filling with acks

  // One trigger roughly every SHOT_PAN_STEP_DEG of azimuth, while armed.
  pumpShotTrigger();

  // Report camera tilt throughout capture for host pose interpolation.
  if (scan.isCapturing() && scan.tiltSweepArmed()
      && millis() - g_lastCamTiltMs >= CAMTILT_SAMPLE_MS) {
    g_lastCamTiltMs = millis();
    HOST.printf("EVT CAMTILT_AT %.3f\n", motorTilt.currentAngle());
  }

  if (scan.consumeReturningEvent())
    HOST.println("EVT RETURNING rewinding to start position");
  if (scan.consumeDoneEvent()) HOST.printf("EVT DONE outcome=%s reason=%s dropped_lidar_frames=%lu firmware=1.0.0-rc.1\n",
      g_scanAborted ? "partial" : "complete",g_abortReason,(unsigned long)lidar.droppedFrames());

  // Report positioning completion when an axis stops. Suppress these events
  // while the scan controller owns azimuth and reports its own lifecycle.
  {
    bool azMoving = motor.isMoving();
    if (g_azWasMoving && !azMoving && !scan.isActive())
      HOST.printf("EVT ARRIVED az %.3f\n", motor.currentAngle());
    g_azWasMoving = azMoving;

    bool tiltMoving = motorTilt.isMoving();
    if (g_tiltWasMoving && !tiltMoving)
      HOST.printf("EVT ARRIVED tilt %.3f\n", motorTilt.currentAngle());
    g_tiltWasMoving = tiltMoving;
  }

  pumpDrain();
  Command command=readCommand();
  // Camera UART and motion belong to the transfer until it ends.
  if (drainActive() && command!=Command::NONE && command!=Command::STOP &&
      command!=Command::STATUS && command!=Command::PING) {
    HOST.println("ERR BUSY transfer_active"); command=Command::NONE;
  }
  switch (command) {
    case Command::SCAN:
      if (scan.isActive() || lidarOnly || lidar.isRunning()) {
        HOST.println("ERR BUSY");
      } else {
        // Camera tilt is independent of LiDAR geometry.
        g_scanAborted=false; g_abortReason="none"; g_transferOutcome="none";
        g_shotIdx = 0;                   // restart the shot index/count for DRAINCAM
        g_lastShotAz = 0.0f;
        g_shotCaptureState = ShotCaptureState::IDLE;
        memset(g_shotMeta, 0, sizeof(g_shotMeta));
        CAM_SERIAL.println("CLEAR");     // drop any frames left over from a previous run
        if (scan.startScan()) {
          g_lastCamTiltMs = millis() - CAMTILT_SAMPLE_MS;   // sample immediately
          HOST.println("OK SCAN_STARTED");
          if (scan.tiltSweepArmed())
            // Report sweep settings and expected pass duration to the phone.
            HOST.printf("EVT CAMTILT %.3f %.3f %d %.3f\n",
                          scan.tiltSweepFrom(), scan.tiltSweepTo(),
                          scan.tiltSweepCycles(),
                          (SCAN_ANGLE_DEG / 360.0f * (float)STEPS_PER_REV_AZ)
                              / MAX_SPEED_SPS_AZ);
        } else {
          HOST.println("ERR BUSY");
        }
      }
      break;

    case Command::LIDAR_ON:
      if (scan.isActive()) {
        HOST.println("ERR BUSY");
      } else {
        lidar.start();
        lidarOnly = true;
        HOST.println("OK LIDAR_STARTED");
      }
      break;

    // STOP and Bluetooth loss share deceleration, rewind and tilt parking.
    case Command::STOP: {
      stopAndPark("user_stop");
      HOST.println("OK STOPPING");
      break;
    }

    case Command::PING:
      HOST.println("OK PONG UnfoLDAR 1.0.0-rc.1 protocol=2");
      break;

    case Command::I2CSCAN:
      i2cScanReport();
      break;

    case Command::ENCTEST:
      if (scan.isActive()) HOST.println("ERR BUSY");
      else                 encTestReport();
      break;

    case Command::CAMTEST:
      if (scan.isActive()) HOST.println("ERR BUSY");
      else                 camTestReport();
      break;

    case Command::POWER:
      powerReport();
      break;

    // Force one telemetry line right now, whatever the rate limit says.
    case Command::SENSORS:
      g_lastTelemMs = millis() - TELEM_SCAN_MS - 1;
      pumpTelemetry();
      break;

    case Command::TELEM_ON:
      g_telemOn = true;
      HOST.println("OK TELEM ON");
      break;

    case Command::TELEM_OFF:
      g_telemOn = false;
      HOST.println("OK TELEM OFF");
      break;

    case Command::TILT:
      doTilt(g_arg);
      break;

    case Command::TILT_ZERO:
      motorTilt.zero();
      HOST.println("OK TILT_ZERO");
      break;

    case Command::PAN:
      doPan(g_arg);
      break;

    case Command::POSE:
      poseReport();
      break;

    case Command::HOLD:
      g_hold = (g_arg != "0");
      HOST.printf("OK HOLD %d\n", g_hold ? 1 : 0);
      break;

    // CAMTILT <from> <to>  - arm a camera tilt sweep for the next SCAN
    // CAMTILT OFF          - cancel it
    case Command::CAMTILT:
      doCamTilt(g_arg);
      break;

    // Drain buffered JPEGs cooperatively after both axes finish moving.
    case Command::DRAINCAM:
      if (scan.isActive()) HOST.println("ERR BUSY");
      else                 drainCamToHost();
      break;

    case Command::CAMPING:
      if (scan.isActive()) HOST.println("ERR BUSY");
      else                 camPing();
      break;

    case Command::CAMCMD:
      if (scan.isActive()) HOST.println("ERR BUSY");
      else                 camForwardCmd(g_arg);
      break;

    case Command::STATUS:
      HOST.printf("OK STATUS state=%s lidar_only=%d moving=%d angle=%.3f "
                    "tilt=%.3f tilt_moving=%d drv_en=%d hold=%d "
                    "az_step=%d az_dir=%d tilt_step=%d tilt_dir=%d en=%d "
                    "spr_az=%ld spr_tilt=%ld telem=%d link=BT bt_client=%d dropped_lidar_frames=%lu transfer=%s firmware=1.0.0-rc.1\n",
                    scan.isCapturing() ? "SCANNING" :
                      (scan.isWarming() ? "WARMING" :
                        (scan.isReturning() ? "RETURNING" :
                          (scan.isActive() ? "STOPPING" : "IDLE"))),
                    lidarOnly ? 1 : 0,
                    motor.isMoving() ? 1 : 0,
                    motor.currentAngle(),
                    motorTilt.currentAngle(),
                    motorTilt.isMoving() ? 1 : 0,
                    digitalRead(PIN_DRV_EN), g_hold ? 1 : 0,
                    PIN_STEP_AZ, PIN_DIR_AZ, PIN_STEP_TILT, PIN_DIR_TILT, PIN_DRV_EN,
                    STEPS_PER_REV_AZ, STEPS_PER_REV_TILT,
                    g_telemOn ? 1 : 0, HOST.hasClient() ? 1 : 0, (unsigned long)lidar.droppedFrames(),g_transferOutcome);
      break;

    // Test motor motion without starting LiDAR.
    case Command::MOTOR_TEST:
      if (scan.isActive() || lidarOnly) {
        HOST.println("ERR BUSY");
      } else {
        HOST.println("OK MOTOR_TEST 90 deg");
        motor.enable();
        motor.moveToAngle(motor.currentAngle() + 90.0f);
      }
      break;

    case Command::UNKNOWN: HOST.println("ERR UNKNOWN"); break;
    default: break;
  }

  // Release each axis after manual positioning unless HOLD is active.
  // The drivers share a reference-counted enable line.
  if (!g_hold) {
    if (!scan.isActive() && !motor.isMoving()) motor.disable();
    if (!motorTilt.isMoving()) motorTilt.disable();
  }
}
