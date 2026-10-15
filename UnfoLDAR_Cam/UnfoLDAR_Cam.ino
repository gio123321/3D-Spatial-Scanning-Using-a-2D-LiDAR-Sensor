// UnfoLDAR camera firmware for the AI-Thinker ESP32-CAM (OV2640).
// UART commands: PING, TRIGGER <idx>, COUNT, INFO <ordinal>,
// READ <idx> <offset> <length>, CLEAR, STATUS, RESET.
// The rig controls capture timing and requests each buffered JPEG chunk.
#include "CamConfig.h"
#include <esp_camera.h>
#include <string.h>   // memcpy

struct Shot {
  uint16_t idx;
  uint8_t* buf;
  uint32_t len;
  uint32_t crc32;
};

static Shot    g_shots[MAX_SHOTS];
static uint8_t g_shotCount = 0;
static bool    g_camReady = false;
static bool    g_psramReady = false;
static uint32_t g_lastCamInitAttempt = 0;

static String cmdBuf;
static bool   cmdOverflow = false;

enum class Command { NONE, PING, TRIGGER, COUNT, INFO, READ, CLEAR, STATUS, RESET, UNKNOWN };

static uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t len) {
  crc = ~crc;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++)
      crc = (crc >> 1) ^ (0xEDB88320UL & (uint32_t)-(int32_t)(crc & 1));
  }
  return ~crc;
}

// Parse a newline-terminated verb and optional arguments.
static Command readCommand(String& arg) {
  while (RIG_SERIAL.available()) {
    char c = (char)RIG_SERIAL.read();
    if (c != '\n' && c != '\r') {
      if (cmdBuf.length() < 31) cmdBuf += c;
      else cmdOverflow = true;
      continue;
    }
    if (cmdOverflow) {
      cmdBuf = "";
      cmdOverflow = false;
      arg = "";
      return Command::UNKNOWN;
    }
    if (cmdBuf.length() == 0) return Command::NONE;

    cmdBuf.trim(); cmdBuf.toUpperCase();
    String line = cmdBuf; cmdBuf = "";

    int sp = line.indexOf(' ');
    String verb = (sp < 0) ? line : line.substring(0, sp);
    arg = (sp < 0) ? String("") : line.substring(sp + 1);
    arg.trim();

    if (verb == "PING")    return Command::PING;
    if (verb == "TRIGGER") return Command::TRIGGER;
    if (verb == "COUNT")   return Command::COUNT;
    if (verb == "INFO")    return Command::INFO;
    if (verb == "READ")    return Command::READ;
    if (verb == "CLEAR")   return Command::CLEAR;
    if (verb == "STATUS")  return Command::STATUS;
    if (verb == "RESET")   return Command::RESET;
    return Command::UNKNOWN;
  }
  return Command::NONE;
}

static bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = CAM_FRAME_SIZE;
  config.jpeg_quality = CAM_JPEG_QUALITY;
  config.fb_count     = CAM_FB_COUNT;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("EVT CAM_INIT_FAILED err=0x%x\n", err);
    return false;
  }

  // Apply capture settings explicitly rather than relying on driver defaults.
  sensor_t* sensor = esp_camera_sensor_get();
  if (!sensor ||
      sensor->set_framesize(sensor, CAM_FRAME_SIZE) != 0 ||
      sensor->set_quality(sensor, CAM_JPEG_QUALITY) != 0 ||
      sensor->set_brightness(sensor, CAM_BRIGHTNESS) != 0 ||
      sensor->set_contrast(sensor, CAM_CONTRAST) != 0 ||
      sensor->set_saturation(sensor, CAM_SATURATION) != 0) {
    Serial.println("EVT CAM_SENSOR_CONFIG_FAILED");
    return false;
  }
  return true;
}

static bool initCameraWithRetries(uint8_t attempts) {
  for (uint8_t attempt = 1; attempt <= attempts; ++attempt) {
    g_lastCamInitAttempt = millis();
    if (initCamera()) return true;

    // Discard any partial DMA/frame-buffer state before the next attempt.
    esp_camera_deinit();
    delay(CAM_INIT_RETRY_MS);
  }
  return false;
}

// READ sends a bounded chunk and CRC32 without copying the whole JPEG.
static void writeU32LE(uint8_t* dst, uint32_t v) {
  dst[0] = (uint8_t)v; dst[1] = (uint8_t)(v >> 8);
  dst[2] = (uint8_t)(v >> 16); dst[3] = (uint8_t)(v >> 24);
}

static Shot* findShot(uint16_t idx) {
  for (uint8_t i = 0; i < g_shotCount; i++)
    if (g_shots[i].idx == idx) return &g_shots[i];
  return nullptr;
}

static bool parseU16(const String& text, uint16_t& out) {
  if (text.length() == 0 || text[0] == '-') return false;
  char* tail = nullptr;
  unsigned long value = strtoul(text.c_str(), &tail, 10);
  if (!tail || tail == text.c_str() || *tail != '\0' || value > 0xFFFFUL)
    return false;
  out = (uint16_t)value;
  return true;
}

static void sendChunk(const Shot& shot, uint32_t offset, uint16_t len) {
  uint8_t hdr[14] = {CAM_CHUNK_SYNC0, CAM_CHUNK_SYNC1};
  hdr[2] = (uint8_t)shot.idx; hdr[3] = (uint8_t)(shot.idx >> 8);
  writeU32LE(hdr + 4, offset);
  writeU32LE(hdr + 8, shot.len);
  hdr[12] = (uint8_t)len; hdr[13] = (uint8_t)(len >> 8);
  uint32_t crc = crc32Update(0, hdr + 2, 12);
  crc = crc32Update(crc, shot.buf + offset, len);
  uint8_t crcBytes[4]; writeU32LE(crcBytes, crc);

  RIG_SERIAL.write(hdr, sizeof(hdr));
  RIG_SERIAL.write(shot.buf + offset, len);
  RIG_SERIAL.write(crcBytes, sizeof(crcBytes));
}

static void doTrigger(const String& arg) {
  if (!g_camReady) {
    RIG_SERIAL.println("ERR CAMERA_NOT_READY");
    return;
  }
  uint16_t idx = 0;
  if (!parseU16(arg, idx)) {
    RIG_SERIAL.println("ERR TRIGGER needs an index, e.g. TRIGGER 7");
    return;
  }
  if (g_shotCount >= MAX_SHOTS) {
    RIG_SERIAL.println("ERR BUFFER_FULL");
    return;
  }
  // Discard the frame held since the previous trigger, then wait for a fresh
  // exposure. Required with one framebuffer and CAMERA_GRAB_WHEN_EMPTY.
  camera_fb_t* stale = esp_camera_fb_get();
  if (!stale) {
    RIG_SERIAL.println("ERR CAPTURE_FAILED");
    return;
  }
  esp_camera_fb_return(stale);

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    RIG_SERIAL.println("ERR CAPTURE_FAILED");
    return;
  }

  if (fb->len == 0 || fb->len > CAM_MAX_JPEG_LEN) {
    esp_camera_fb_return(fb);
    RIG_SERIAL.println("ERR IMAGE_TOO_LARGE");
    return;
  }

  // Copy into PSRAM before returning the driver framebuffer.
  uint8_t* copy = (uint8_t*)ps_malloc(fb->len);
  if (!copy) {
    esp_camera_fb_return(fb);
    RIG_SERIAL.println("ERR NOMEM");
    return;
  }
  memcpy(copy, fb->buf, fb->len);
  uint32_t len = fb->len;
  esp_camera_fb_return(fb);

  g_shots[g_shotCount].idx = idx;
  g_shots[g_shotCount].buf = copy;
  g_shots[g_shotCount].len = len;
  g_shots[g_shotCount].crc32 = crc32Update(0, copy, len);
  g_shotCount++;

  RIG_SERIAL.printf("OK SHOT idx=%u size=%lu\n", idx, (unsigned long)len);
}

static void doInfo(const String& arg) {
  uint16_t ordinal = 0;
  if (!parseU16(arg, ordinal) || ordinal >= g_shotCount) {
    RIG_SERIAL.println("ERR INFO_RANGE");
    return;
  }
  const Shot& s = g_shots[ordinal];
  RIG_SERIAL.printf("OK INFO ordinal=%u idx=%u len=%lu crc32=%08lX\n",
                    ordinal, s.idx, (unsigned long)s.len, (unsigned long)s.crc32);
}

static void doRead(const String& arg) {
  unsigned idxArg = 0, lenArg = 0;
  unsigned long offsetArg = 0;
  char extra = 0;
  if (sscanf(arg.c_str(), "%u %lu %u %c", &idxArg, &offsetArg, &lenArg, &extra) != 3 ||
      idxArg > 0xFFFFU) {
    RIG_SERIAL.println("ERR READ_ARGS");
    return;
  }
  Shot* shot = findShot((uint16_t)idxArg);
  if (!shot) { RIG_SERIAL.println("ERR READ_NO_SHOT"); return; }
  if (offsetArg >= shot->len || lenArg == 0) {
    RIG_SERIAL.println("ERR READ_RANGE");
    return;
  }
  uint32_t remaining = shot->len - (uint32_t)offsetArg;
  uint16_t len = (uint16_t)min((uint32_t)min(lenArg, (unsigned)CAM_READ_CHUNK_MAX), remaining);
  sendChunk(*shot, (uint32_t)offsetArg, len);
}

static void doClear() {
  for (uint8_t i = 0; i < g_shotCount; i++) {
    free(g_shots[i].buf);
    g_shots[i].buf = nullptr;
  }
  g_shotCount = 0;
  RIG_SERIAL.println("OK CLEAR");
}

static void doStatus() {
  RIG_SERIAL.printf("OK STATUS ready=%d psram=%d shots=%u free_psram=%u free_heap=%u "
                    "quality=%d brightness=%d contrast=%d saturation=%d\n",
                     g_camReady ? 1 : 0, g_psramReady ? 1 : 0, g_shotCount,
                     (unsigned)ESP.getFreePsram(), (unsigned)ESP.getFreeHeap(),
                     CAM_JPEG_QUALITY, CAM_BRIGHTNESS, CAM_CONTRAST, CAM_SATURATION);
}

void setup() {
  Serial.begin(115200);   // local USB/FTDI debug only - never the rig link
  RIG_SERIAL.begin(RIG_LINK_BAUD, SERIAL_8N1, RIG_RX_PIN, RIG_TX_PIN);
  cmdBuf.reserve(32);

  delay(CAM_STARTUP_SETTLE_MS);
  Serial.println("EVT BOOT UnfoLDAR_Cam 1.0.0-rc.1");

  pinMode(PIN_FLASH_LED, OUTPUT);
  digitalWrite(PIN_FLASH_LED, LOW);   // off - this is a lens, not a flash unit

  g_psramReady = psramFound();
  if (!g_psramReady)
    Serial.println("EVT FATAL no PSRAM");

  g_camReady = g_psramReady && initCameraWithRetries(CAM_INIT_ATTEMPTS);
  Serial.printf("EVT CAM_INIT present=%d\n", g_camReady ? 1 : 0);

  Serial.println("EVT READY");
}

void loop() {
  // If only the sensor missed cold start, recover without another power cycle.
  if (g_psramReady && !g_camReady &&
      millis() - g_lastCamInitAttempt >= CAM_BACKGROUND_RETRY_MS) {
    g_camReady = initCameraWithRetries(1);
  }

  String arg;
  switch (readCommand(arg)) {
    case Command::PING:
      if (g_camReady) RIG_SERIAL.println("OK PONG UnfoLDAR_CAM 1.0.0-rc.1 protocol=2 ready=1");
      else            RIG_SERIAL.printf("ERR CAMERA_NOT_READY protocol=2 ready=0 psram=%d\n",
                                        g_psramReady ? 1 : 0);
      break;
    case Command::TRIGGER:
      doTrigger(arg);
      break;
    case Command::COUNT:
      RIG_SERIAL.printf("OK COUNT count=%u\n", g_shotCount);
      break;
    case Command::INFO:
      doInfo(arg);
      break;
    case Command::READ:
      doRead(arg);
      break;
    case Command::CLEAR:
      doClear();
      break;
    case Command::STATUS:
      doStatus();
      break;
    case Command::RESET:
      // Acknowledge and flush before the software reset.
      RIG_SERIAL.println("OK RESETTING");
      RIG_SERIAL.flush();
      Serial.flush();
      delay(50);
      ESP.restart();
      break;
    case Command::UNKNOWN:
      RIG_SERIAL.println("ERR UNKNOWN");
      break;
    default:
      break;
  }
}
