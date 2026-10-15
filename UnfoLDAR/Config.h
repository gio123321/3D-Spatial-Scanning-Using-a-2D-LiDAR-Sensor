#pragma once
#include <Arduino.h>

// Board revision B pin map. GPIO2 must stay low during reset; wire its LED active-high.
// GPIO12 is unused. GPIO4 controls camera reset.

// LiDAR - YDLIDAR T-mini Plus on UART2
#define PIN_LIDAR_RX      16      // <- lidar Tx
#define PIN_LIDAR_TX      17      // -> lidar Rx

// Camera UART1 relay; see UnfoLDAR_Cam for the command protocol.
#define PIN_CAM_RX        25      // <- camera Tx
#define PIN_CAM_TX        26      // -> camera Rx
// Through a 1 kOhm series resistor to the ESP32-CAM pin labelled RST.
// Driven open-drain so the camera board's own pull-up releases reset safely.
#define PIN_CAM_RESET      4

// Encoders - two MT6835 on the shared VSPI bus
#define PIN_ENC_SCK       18
#define PIN_ENC_MISO      19
#define PIN_ENC_MOSI      23
#define PIN_ENC_CS_AZ     15      
#define PIN_ENC_CS_TILT    5      

// I2C - IMU + INA226 share one bus
#define PIN_I2C_SDA       21
#define PIN_I2C_SCL       22

// Motors - two TMC2209, one shared enable
#define PIN_STEP_AZ       33      
#define PIN_DIR_AZ        32      
#define PIN_STEP_TILT     14
#define PIN_DIR_TILT      27      
#define PIN_DRV_EN        13      // ACTIVE LOW, shared by BOTH drivers,
                                  // 10k pull-up to 3V3 (floating == disabled)

// Per-axis motion settings; microsteps must match the MS1/MS2 straps.
#define MOTOR_FULL_STEPS       200

// Motor revolutions per output revolution. Match the installed gearing:
// all commanded angles and saved theta values use output-shaft degrees.
#define GEAR_RATIO_AZ          1.0f
#define GEAR_RATIO_TILT        1.0f

// Round steps per revolution to the nearest integer.
// Microsteps must match the physical TMC2209 straps; there is no driver UART.
// MS2/MS1: 0/0 = 1/8, 0/1 = 1/2, 1/0 = 1/4, 1/1 = 1/16.
// AZ: both high (1/16). TILT: both low (1/8).
#define MICROSTEPS_AZ           16                                  // MS1 high, MS2 high
#define STEPS_PER_REV_AZ       ((long)(MOTOR_FULL_STEPS * MICROSTEPS_AZ * GEAR_RATIO_AZ + 0.5f))

#define MICROSTEPS_TILT          8                                  // MS1 low, MS2 low
#define STEPS_PER_REV_TILT     ((long)(MOTOR_FULL_STEPS * MICROSTEPS_TILT * GEAR_RATIO_TILT + 0.5f))

#define MAX_SPEED_SPS_AZ       50.0f    // ~64 s per 360 degrees - slow for dense capture
#define ACCEL_SPS2_AZ          50.0f
#define INVERT_DIR_AZ          false    // set true if it spins the wrong way

// Match azimuth angular speed: 1600/3200 * 50 = 25 steps/s.
#define MAX_SPEED_SPS_TILT     25.0f
#define ACCEL_SPS2_TILT        25.0f
#define INVERT_DIR_TILT        false

// Separate jog profile for manual tilt positioning.
#define TILT_JOG_SPS          400.0f    // 90 deg/s at 1600 steps/rev
#define TILT_JOG_ACCEL_SPS2  1600.0f    // ramps to full speed in 0.25 s

// Software tilt limits; this axis has no home switch or hard stop.
#define TILT_MIN_DEG          -45.0f
#define TILT_MAX_DEG           45.0f

// Camera park angle relative to the zero established at startup.
#define TILT_PARK_DEG           0.0f

// PAN positioning profile: 400 steps/s at 3200 steps/rev = 45 deg/s.
#define PAN_JOG_SPS           400.0f
#define PAN_JOG_ACCEL_SPS2   1600.0f

// Camera tilt telemetry interval during capture.
#define CAMTILT_SAMPLE_MS       100

// PAN travel limits protect the camera cable from accumulated twist.
// SCAN performs one revolution and rewinds to its starting position.
#define PAN_MIN_DEG          -200.0f
#define PAN_MAX_DEG           200.0f

// Host link: Classic Bluetooth SPP carrying text replies and binary frames.
#define HOST_BT_NAME      "UnfoLDAR"

// GPIO2 LED: one-second blink while disconnected, off while connected.
// Wire active-high so GPIO2 is not pulled high during reset.
#define PIN_BT_STATUS_LED       2
#define BT_LED_TOGGLE_MS     1000

// BluetoothSerial has no usable availableForWrite() headroom report.
// Queue each binary packet in one write and check the returned byte count.

// YDLIDAR T-mini Plus (UART2)
#define LIDAR_SERIAL      Serial2
#define LIDAR_BAUD        230400
#define LIDAR_CMD_START   0x60      // A5 60
#define LIDAR_CMD_STOP    0x65      // A5 65
#define LIDAR_RX_BUFFER   2048
#define LIDAR_CHUNK_MAX   96        // raw lidar bytes per host frame

// Camera UART1 baud rate; must match RIG_LINK_BAUD in CamConfig.h.
#define CAM_SERIAL        Serial1
#define CAM_BAUD          460800

// Camera commands: TRIGGER <idx>, COUNT, INFO <ordinal>, READ <idx> <offset> <len>, CLEAR.
// The rig requests each READ chunk after forwarding the previous one to Bluetooth.
// CLEAR discards the previous scan before capture begins.
#define CAM_CHUNK_SYNC0        0x43   // 'C'
#define CAM_CHUNK_SYNC1        0x48   // 'H'
#define CAM_TRANSFER_CHUNK     256
#define CAM_MAX_IMG_LEN        262144UL   // sanity cap on a single JPEG (256 KB)
#define CAM_CHUNK_TIMEOUT_MS   1000
#define CAM_CHUNK_RETRIES      2
#define CAM_PING_TIMEOUT_MS     1000      // CAMPING: cam module's PONG should be
#define CAM_POWER_SETTLE_MS     2000      // leave UART high-Z while power rises
#define CAM_BOOT_WAIT_MS        2000      // after releasing hardware reset
#define CAM_PING_RETRIES           5
#define CAM_PING_RETRY_MS         500

// Wire protocol, rig->phone, independently checksummed image chunks:
// [IMG_SYNC0][IMG_SYNC1][idx u16][offset u32][total u32][len u16]
// [payload <= 256][crc32 u32].
#define IMG_CHUNK_SYNC0        0x49   // 'I'
#define IMG_CHUNK_SYNC1        0x43   // 'C'
#define IMG_DRAIN_WRITE_TIMEOUT_MS 5000   // per-write retry budget while relaying
#define CAM_MAX_TRACKED_SHOTS  64
#define CAM_SHOT_ACK_TIMEOUT_MS 3000

// Azimuth spacing between camera triggers; 10 degrees gives about 36 shots per scan.
#define SHOT_PAN_STEP_DEG      10.0f

// Rig-to-phone LiDAR frame sync bytes.
#define FRAME_SYNC0       0x5A
#define FRAME_SYNC1       0xA5

// Scan motion
#define SCAN_ANGLE_DEG    360.0f
#define LIDAR_WARMUP_MS   800     // ms to let lidar stabilize before motor moves

// Rewind after capture to unwind the LiDAR cable. Keep the open-loop return
// slow enough to avoid lost steps and a shifted zero.
#define RETURN_SPEED_SPS_AZ   800.0f   // 0.25 rev/s -> ~4.5 s for 360 deg
#define RETURN_ACCEL_SPS2_AZ  800.0f

// Optional sensors; disabling them does not disable scanning.
#define ENABLE_IMU        1
#define ENABLE_ENCODER    1
#define ENABLE_INA226     1
#define ENABLE_BATTERY_SOC 1

// INA226 address hint; the driver checks die IDs across 0x40..0x4F.
#define INA226_ADDR_HINT      0x40

// Fitted shunt resistance. A 0.1-ohm shunt reaches the +/-81.92 mV ADC
// limit at 0.819 A; update this value if the shunt changes.
#define INA226_SHUNT_OHMS     0.1f

// Bus ADC gain correction. Leave at 1.0 unless a reference meter confirms
// a gain error; shunt calibration does not correct bus voltage.
#define INA226_BUS_SCALE      1.0f

// Dedicated pack-side INA226 for the 2S battery gauge. This is separate
// from the bus-side monitor used for telemetry.
#define INA226_PACK_ADDR_HINT      0x41
#define INA226_PACK_SHUNT_OHMS     0.1f     // shunt fitted on the PACK-side breakout

// Set to the installed pack capacity; scales coulomb-counted SOC.
#define PACK_CAPACITY_MAH          2200.0f

// BatterySoc::update() limits I2C reads to this interval internally.
#define BATTERY_SOC_UPDATE_MS      250

// Only correct SOC from voltage below this current, to limit load-sag error.
#define BATTERY_SOC_REST_CURRENT_A 0.05f

// Set polarity so discharge current is positive; verify against the shunt wiring.
#define BATTERY_SOC_CURRENT_SIGN   1

// Battery frame, separate from ASCII telemetry (all fields little-endian):
// [sync 0x42 0x53][SOC x10 u16][mV u16][mA i16][CRC32 u32].
// CRC covers bytes 2..7; RigFrameDecoder reads this layout.
#define BATTERY_PKT_SYNC0          0x42
#define BATTERY_PKT_SYNC1          0x53
#define BATTERY_PKT_MS             500

// Sensor telemetry interval in active and idle states.
#define TELEM_IDLE_MS         200      // 5 Hz while idle
#define TELEM_SCAN_MS        1000      // 1 Hz during a scan
