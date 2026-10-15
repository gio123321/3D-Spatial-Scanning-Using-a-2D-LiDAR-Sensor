#include "LidarBridge.h"
#include <string.h>   // memcpy

static void sendLidarCommand(uint8_t cmd) {
  uint8_t pkt[2] = {0xA5, cmd};
  LIDAR_SERIAL.write(pkt, 2);
  LIDAR_SERIAL.flush();
}

// Frame: [SYNC0][SYNC1][length u16 LE][theta i32 LE + raw bytes][XOR].
// Use one HOST.write() per packet to limit Bluetooth queue usage.
static bool sendFrame(int32_t theta_milli, const uint8_t* raw, uint16_t rawLen) {
  uint16_t len   = 4 + rawLen;
  uint16_t total = 4 + len + 1;              // header(4) + theta+raw(len) + crc(1)

  uint8_t buf[4 + 4 + LIDAR_CHUNK_MAX + 1];
  buf[0] = FRAME_SYNC0;
  buf[1] = FRAME_SYNC1;
  buf[2] = (uint8_t)(len & 0xFF);
  buf[3] = (uint8_t)(len >> 8);

  buf[4] = (uint8_t)(theta_milli);
  buf[5] = (uint8_t)(theta_milli >> 8);
  buf[6] = (uint8_t)(theta_milli >> 16);
  buf[7] = (uint8_t)(theta_milli >> 24);

  memcpy(buf + 8, raw, rawLen);

  uint8_t crc = 0;
  for (uint16_t i = 4; i < 8 + rawLen; i++) crc ^= buf[i];
  buf[8 + rawLen] = crc;

  return HOST.write(buf, total) == total;
}

void LidarBridge::begin() {
  LIDAR_SERIAL.setRxBufferSize(LIDAR_RX_BUFFER);          // before begin()
  LIDAR_SERIAL.begin(LIDAR_BAUD, SERIAL_8N1, PIN_LIDAR_RX, PIN_LIDAR_TX);
  LIDAR_SERIAL.setTimeout(2);
}

void LidarBridge::start()   { sendLidarCommand(LIDAR_CMD_START); _running = true; _droppedFrames = 0; }
void LidarBridge::stop()    { sendLidarCommand(LIDAR_CMD_STOP);  _running = false; }
void LidarBridge::drain()   { while (LIDAR_SERIAL.available()) LIDAR_SERIAL.read(); }
bool LidarBridge::isRunning() { return _running; }

void LidarBridge::pump(float theta_deg) {
  if (!_running) return;
  int avail = LIDAR_SERIAL.available();
  if (avail <= 0) return;

  int n = (avail > LIDAR_CHUNK_MAX) ? LIDAR_CHUNK_MAX : avail;

  // Drain the UART even when Bluetooth is congested to avoid receive-buffer
  // overflow. Failed outgoing frames are counted and cannot be retransmitted.
  int got = 0;
  while (got < n && LIDAR_SERIAL.available() > 0) {
    int b = LIDAR_SERIAL.read();
    if (b < 0) break;
    _chunk[got++] = (uint8_t)b;
  }
  if (got <= 0) return;

  int32_t theta_milli = (int32_t)lroundf(theta_deg * 1000.0f);
  if (!sendFrame(theta_milli, _chunk, (uint16_t)got)) ++_droppedFrames;
}
