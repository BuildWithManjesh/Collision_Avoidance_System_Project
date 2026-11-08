/*
  CAM NODE  -  ESP32-CAM (AI-Thinker)
  --------------------------------------------------------------
  Listens to the sensor node over ESP-NOW.
  On alert: lights the on-board red LED and saves a JPEG to the SD card.
  Sends a 1 Hz heartbeat back so the sensor node's OLED can show "Cam OK".

  Arduino IDE: Board = "AI Thinker ESP32-CAM", PSRAM = Enabled
  Upload: hold GPIO0 to GND while uploading via FTDI, then remove it and reset.
  Board pkg: esp32 by Espressif (core 3.x). Core 2.x: change onRecv() signature
  and rename pin_sccb_sda/scl to pin_sscb_sda/scl.
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

#define RED_LED 33          // on-board red LED, active LOW

const uint32_t SENSOR_TIMEOUT_MS = 1000;  // no packets -> treat as no alert (fail quiet, shown on sensor OLED)
const uint32_t SNAP_COOLDOWN_MS  = 3000;
const uint32_t HEARTBEAT_MS      = 1000;

#define MAGIC 0xA5
enum : uint8_t { PKT_SENSOR = 1, PKT_CAM = 2 };

typedef struct __attribute__((packed)) {
  uint8_t  magic;
  uint8_t  type;
  uint8_t  alert;
  uint8_t  flags;   // sensor: bit0 human, bit1 obstacle | cam: bit0 camOk, bit1 sdOk
  uint16_t value;   // sensor: nearest obstacle mm | cam: snapshot count
  uint32_t seq;
} Packet;

const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

volatile bool     alertFlag = false;
volatile uint32_t lastSensorRx = 0;
volatile uint16_t lastDist = 0xFFFF;
volatile uint8_t  lastFlags = 0;

bool camOk = false, sdOk = false;
uint16_t snapCount = 0;
uint32_t lastSnap = 0, lastBeat = 0, beatSeq = 0;

// Core 3.x signature. Core 2.x: void onRecv(const uint8_t *mac, const uint8_t *data, int len)
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(Packet)) return;
  Packet p;
  memcpy(&p, data, sizeof(p));
  if (p.magic != MAGIC || p.type != PKT_SENSOR) return;
  alertFlag = p.alert;
  lastDist  = p.value;
  lastFlags = p.flags;
  lastSensorRx = millis();
}

bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = 5;  c.pin_d1 = 18; c.pin_d2 = 19; c.pin_d3 = 21;
  c.pin_d4 = 36; c.pin_d5 = 39; c.pin_d6 = 34; c.pin_d7 = 35;
  c.pin_xclk = 0;  c.pin_pclk = 22; c.pin_vsync = 25; c.pin_href = 23;
  c.pin_sccb_sda = 26; c.pin_sccb_scl = 27;
  c.pin_pwdn = 32; c.pin_reset = -1;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = FRAMESIZE_VGA;
  c.jpeg_quality = 12;
  c.fb_count     = 2;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;
  return esp_camera_init(&c) == ESP_OK;
}

void saveSnapshot() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;
  if (sdOk) {
    char path[32];
    do {                                   // pick the first unused number, so a reboot never overwrites old photos
      snapCount++;
      snprintf(path, sizeof(path), "/alert_%u.jpg", (unsigned)snapCount);
    } while (SD_MMC.exists(path));
    File f = SD_MMC.open(path, FILE_WRITE);
    if (f) {
      f.write(fb->buf, fb->len);
      f.close();
    }
  }
  esp_camera_fb_return(fb);
}

void sendHeartbeat() {
  Packet p;
  p.magic = MAGIC;
  p.type  = PKT_CAM;
  p.alert = 0;
  p.flags = (camOk ? 1 : 0) | (sdOk ? 2 : 0);
  p.value = snapCount;
  p.seq   = beatSeq++;
  esp_now_send(BCAST, (uint8_t *)&p, sizeof(p));
  lastBeat = millis();
}

void setup() {
  Serial.begin(115200);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);              // off

  camOk = initCamera();
  if (!camOk) Serial.println("Camera init failed");

  // 1-bit SD mode keeps GPIO4 (flash LED) free
  sdOk = SD_MMC.begin("/sdcard", true);
  if (!sdOk) Serial.println("SD card not found (snapshots disabled)");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);   // must match the sensor node
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

void loop() {
  uint32_t now = millis();

  bool linkOk = (lastSensorRx != 0) && (now - lastSensorRx < SENSOR_TIMEOUT_MS);
  bool alert  = linkOk && alertFlag;

  digitalWrite(RED_LED, alert ? LOW : HIGH);   // active LOW

  if (alert && camOk && (now - lastSnap > SNAP_COOLDOWN_MS)) {
    lastSnap = now;
    saveSnapshot();
  }

  if (now - lastBeat >= HEARTBEAT_MS) sendHeartbeat();
  delay(2);
}
