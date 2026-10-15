#pragma once
#include <Arduino.h>

// ESP32-CAM OV2640 configuration. UART0 remains available for programming
// and diagnostics; UART2 connects to the main rig.

// AI-Thinker ESP32-CAM OV2640 pin map.
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// UART2 uses the unused SD-card pins. TX uses GPIO14 because GPIO12
// is a boot strap whose level selects the flash voltage at reset.
#define RIG_SERIAL        Serial2
#define RIG_RX_PIN        13
#define RIG_TX_PIN        14
#define RIG_LINK_BAUD     460800   // must match CAM_BAUD in the rig's Config.h

// Onboard flash LED.
#define PIN_FLASH_LED      4

#define CAM_FRAME_SIZE     FRAMESIZE_VGA   // 640x480 - see UnfoLDAR_Cam.ino
#define CAM_JPEG_QUALITY   12              // 0-63; lower values give higher quality
#define CAM_BRIGHTNESS      0              // neutral brightness
#define CAM_CONTRAST        0
#define CAM_SATURATION      0
#define CAM_FB_COUNT       1               // one shot at a time; copy into PSRAM

// Must match the maximum JPEG length accepted by the rig and phone.
#define CAM_MAX_JPEG_LEN   262144UL

// Cold-start recovery for a slowly rising switched battery supply.
#define CAM_STARTUP_SETTLE_MS   500
#define CAM_INIT_ATTEMPTS         5
#define CAM_INIT_RETRY_MS       500
#define CAM_BACKGROUND_RETRY_MS 5000

// Keep buffered JPEGs in PSRAM until the rig drains them.
#define MAX_SHOTS          64

// Paced READ chunks must fit the rig UART receive buffer.
// The rig forwards each chunk to Bluetooth before requesting another.
#define CAM_READ_CHUNK_MAX 256

// Protocol-v2 paced chunk returned by READ
// [CHUNK_SYNC0][CHUNK_SYNC1][idx u16][offset u32][total u32][len u16]
// [payload][crc32 u32].  CRC32 covers the header after sync plus payload.
// Must match CAM_CHUNK_SYNC0/1 in the rig's Config.h exactly.
#define CAM_CHUNK_SYNC0    0x43   // 'C'
#define CAM_CHUNK_SYNC1    0x48   // 'H'
