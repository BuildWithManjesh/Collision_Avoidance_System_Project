/*!
  * @file  machine_node.ino
  * @brief MACHINE NODE (ESP32 DevKit V1 / WROVER)
  *        = your working sensor_node_v2.ino, with the radar, ToF, OLED, buzzer, LED
  *          and CAM-link code kept exactly as it was, plus a two-way ESP-NOW link
  *          to the worker node (the UWB anchor on the worker's arm).
  *
  *  New lines are marked "// [WORKER]". Nothing else was changed.
  *  The UWB tag on this machine only needs 3.3 V + GND (no wiring to this ESP32).
  *
  *  Buzzer + LED are ON when:  your own alert is active (radar human / ToF obstacle,
  *                             with your 2 s hold)  OR  the worker is too close.
  *  The packet sent to your CAM node is unchanged and still reports only THIS node's
  *  own alert, so snapshots behave exactly as before.
  *
  *  Board pkg: esp32 by Espressif (works with core 3.x and 2.x)
  */

#include "DFRobot_C4001.h"
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/* ---------------------------- Pin definitions ---------------------------- */
#define RADAR_RX_PIN   16   // ESP32 RX2  <- C4001 TX
#define RADAR_TX_PIN   17   // ESP32 TX2  -> C4001 RX
//#define MOTOR_PIN      18   // vibration motor module IN
#define BUZZER_PIN     14  // active buzzer module I/O
#define LED_PIN        33

#define OLED_CLK       18
#define OLED_MOSI      19
#define OLED_RES       25
#define OLED_DC        26
#define OLED_CS        27

#define TOF_SDA        21
#define TOF_SCL        22
#define TOF_XSHUT      23   // optional hardware reset line

#define ACTIVE_LEVEL   HIGH
#define IDLE_LEVEL     ((ACTIVE_LEVEL == HIGH) ? LOW : HIGH)

const unsigned long ALERT_HOLD_MS   = 2000;
const unsigned long UI_PERIOD_MS    = 200;
const unsigned long TOF_PERIOD_MS   = 100;   // VL53L0X ranging cadence
const uint16_t       PROXIMITY_MM   = 1500;  // 1.5 m alert distance
const uint16_t       ALERT_DIST_CM  = 150;   // radar alert distance, same 1.5 m

/* ------------------------- Radar tuning (C4001) --------------------------- */
//   1 = SPEED mode    (mRangeVelocity example): reports the real target range in metres,
//                      alert when range <= ALERT_DIST_CM. Fretting detection also catches tiny movements.
//   0 = PRESENCE mode (motionDetection example): the sensor itself says "someone is inside the trigger range".
#define RADAR_USE_SPEED_MODE  1

// Library limit: the sensor's max range must be 240-2500 cm. The 1.5 m alert gate is ALERT_DIST_CM, not this.
const uint16_t RADAR_MAX_CM     = 300;

// Speed mode
const uint16_t RADAR_THRES      = 10;   // target detection threshold (0.1 units); lower = more sensitive

// Presence mode (the library example uses 1 / 2 / 100 / 4)
const uint8_t  RADAR_TRIG_SENS  = 6;    // 0-9, higher = more sensitive
const uint8_t  RADAR_KEEP_SENS  = 6;    // 0-9, sensitivity once triggered
const uint16_t RADAR_TRIG_DELAY = 10;   // unit 0.01 s, range 0-200; lower = reports sooner (example: 100 = 1 s)
const uint16_t RADAR_KEEP_DELAY = 4;    // unit 0.5 s; how long presence is held after the person stops

/* ------------------------------ ESP-NOW setup ----------------------------- */
// Both nodes must use the same channel (no router involved).
const uint8_t  WIFI_CHANNEL    = 1;
const unsigned long TX_PERIOD_MS    = 100;   // status packet rate (also sent instantly on change)
const unsigned long CAM_TIMEOUT_MS  = 3000;  // no CAM heartbeat for this long -> "--"

#define MAGIC 0xA5
enum : uint8_t { PKT_SENSOR = 1, PKT_CAM = 2 };

// Same struct in both sketches (machine <-> CAM, and machine -> worker).
typedef struct __attribute__((packed)) {
  uint8_t  magic;
  uint8_t  type;    // PKT_SENSOR / PKT_CAM
  uint8_t  alert;   // 1 = alert active
  uint8_t  flags;   // sensor: bit0 human(radar), bit1 obstacle(ToF) | cam: bit0 camera OK, bit1 SD OK
  uint16_t value;   // sensor: ToF distance mm (0xFFFF = none)       | cam: snapshots saved
  uint32_t seq;
} Packet;

const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* ------------------------- [WORKER] worker link --------------------------- */
// Packet from the worker node. Different magic AND different length from Packet,
// so your CAM node ignores it.
#define WORKER_MAGIC  0xA7
#define WORKER_TYPE   2
typedef struct __attribute__((packed)) {
  uint8_t  magic;   // WORKER_MAGIC
  uint8_t  type;    // WORKER_TYPE
  uint8_t  seq;
  uint8_t  uwbOk;   // 1 = worker has a fresh UWB distance
  uint16_t distCm;  // UWB distance to this machine, 0xFFFF = none
  uint8_t  hazard;  // 1 = worker is inside the UWB alarm distance
} WorkerPkt;

const unsigned long WORKER_TIMEOUT_MS = 2000;   // no worker packet this long = link lost
volatile uint32_t lastWorkerRx    = 0;
volatile uint8_t  rxWorkerHazard  = 0;
volatile uint8_t  rxUwbOk         = 0;
volatile uint16_t rxDistCm        = 0xFFFF;
bool outputsOn = false;

bool workerLinkOk() { return lastWorkerRx != 0 && (millis() - lastWorkerRx) < WORKER_TIMEOUT_MS; }
bool workerAlarm()  { return workerLinkOk() && rxWorkerHazard; }
/* -------------------------------------------------------------------------- */

DFRobot_C4001_UART radar(&Serial2, 9600, /*rx*/RADAR_RX_PIN, /*tx*/RADAR_TX_PIN);

U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0,
    /*cs*/OLED_CS, /*dc*/OLED_DC, /*reset*/OLED_RES);
// U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, OLED_CS, OLED_DC, OLED_RES);

Adafruit_VL53L0X tof = Adafruit_VL53L0X();

bool          alertActive   = false;
bool          radarMotion   = false;
float         radarRangeM   = 0;       // speed mode: measured target range in metres (0 = none)
unsigned long lastRadarDbgMs = 0;
bool          tofClose      = false;
bool          srcHuman      = false;   // what caused the current alert (for the OLED text)
unsigned long lastMotionMs  = 0;
unsigned long lastUiMs      = 0;
unsigned long lastTofMs     = 0;
unsigned long lastTxMs      = 0;
uint32_t      seqNo         = 0;
uint32_t      eventCount    = 0;
uint16_t      tofDistanceMm = 9999;
bool          tofOk         = false;

volatile uint32_t lastCamRx = 0;       // time of last CAM heartbeat

uint16_t cfgMin, cfgMax, cfgTrigSens, cfgKeepSens;

void setAlert(bool on)
{
  //digitalWrite(MOTOR_PIN,  on ? ACTIVE_LEVEL : IDLE_LEVEL);
  digitalWrite(BUZZER_PIN, on ? ACTIVE_LEVEL : IDLE_LEVEL);
  digitalWrite(LED_PIN,    on ? ACTIVE_LEVEL : IDLE_LEVEL);
}

// [WORKER] buzzer + LED follow (own alert OR worker too close)
void updateOutputs()
{
  bool want = alertActive || workerAlarm();
  if(want != outputsOn){
    outputsOn = want;
    setAlert(want);
  }
}

void splash(const char *line1, const char *line2)
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 12, line1);
  if(line2) u8g2.drawStr(0, 26, line2);
  u8g2.sendBuffer();
}

/* ------------------------------ ESP-NOW ----------------------------------- */
// [WORKER] one handler for both CAM heartbeats (your original logic) and worker packets
void handleRx(const uint8_t *data, int len)
{
  if(len == (int)sizeof(Packet)){                       // CAM heartbeat (unchanged)
    Packet p;
    memcpy(&p, data, sizeof(p));
    if(p.magic != MAGIC || p.type != PKT_CAM) return;
    if(p.flags & 0x01) lastCamRx = millis();            // only count heartbeats where the camera itself is OK
  }
  else if(len == (int)sizeof(WorkerPkt)){               // [WORKER] worker status
    WorkerPkt w;
    memcpy(&w, data, sizeof(w));
    if(w.magic != WORKER_MAGIC || w.type != WORKER_TYPE) return;
    rxDistCm       = w.distCm;
    rxUwbOk        = w.uwbOk;
    rxWorkerHazard = w.hazard;
    lastWorkerRx   = millis();
  }
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) { handleRx(data, len); }
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) { handleRx(data, len); }
#endif

// Unchanged: this one broadcast reaches both the CAM node and the worker node.
void sendStatus()
{
  Packet p;
  memset(&p, 0, sizeof(p));
  p.magic = MAGIC;
  p.type  = PKT_SENSOR;
  p.alert = alertActive ? 1 : 0;
  p.flags = (radarMotion ? 1 : 0) | (tofClose ? 2 : 0);
  p.value = tofOk ? tofDistanceMm : 0xFFFF;
  p.seq   = seqNo++;
  esp_now_send(BCAST, (uint8_t *)&p, sizeof(p));
  lastTxMs = millis();
}

/* ------------------------------ VL53L0X read ------------------------------ */
void pollToF()
{
  VL53L0X_RangingMeasurementData_t m;
  tof.rangingTest(&m, false);

  // ignore the 8190 / 65535 "nothing there" values
  if(m.RangeStatus != 4 && m.RangeMilliMeter < 2000){
    tofDistanceMm = m.RangeMilliMeter;
    tofOk = true;
  } else {
    tofDistanceMm = 9999;
    tofOk = false;
  }
  tofClose = tofOk && (tofDistanceMm < PROXIMITY_MM);
}

/* -------------------------------- OLED UI --------------------------------- */
void drawUI()
{
  char buf[24];
  unsigned long held = alertActive ? (millis() - lastMotionMs) : 0;
  bool camAlive = (lastCamRx != 0) && (millis() - lastCamRx < CAM_TIMEOUT_MS);

  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 10, "C4001 + ToF");
  u8g2.drawStr(80, 10, camAlive ? "CAM:OK" : "CAM:--");
  u8g2.drawHLine(0, 13, 128);

  if(alertActive){
    u8g2.drawBox(0, 17, 128, 20);
    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_helvB12_tr);
    if(srcHuman) u8g2.drawStr(30, 32, "HUMAN!");
    else         u8g2.drawStr(16, 32, "OBSTACLE!");
    u8g2.setDrawColor(1);
  } else if(workerAlarm()){                                  // [WORKER]
    u8g2.drawBox(0, 17, 128, 20);
    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_helvB12_tr);
    u8g2.drawStr(8, 32, "WORKER NEAR!");
    u8g2.setDrawColor(1);
  } else {
    u8g2.setFont(u8g2_font_helvB12_tr);
    u8g2.drawStr(24, 32, "CLEAR");
  }

  u8g2.setFont(u8g2_font_5x8_tr);

  if(tofOk) snprintf(buf, sizeof(buf), "ToF: %u mm", tofDistanceMm);
  else      snprintf(buf, sizeof(buf), "ToF: ---");
  u8g2.drawStr(0, 47, buf);

  // [WORKER] worker distance (from the UWB anchor, via ESP-NOW)
  if(workerLinkOk() && rxUwbOk) snprintf(buf, sizeof(buf), "W:%ucm", (unsigned)rxDistCm);
  else if(workerLinkOk())       snprintf(buf, sizeof(buf), "W:noUWB");
  else                          snprintf(buf, sizeof(buf), "W:--");
  u8g2.drawStr(70, 47, buf);

#if RADAR_USE_SPEED_MODE
  if(radarRangeM > 0) snprintf(buf, sizeof(buf), "Radar: %.2f m", radarRangeM);   // live range
  else                snprintf(buf, sizeof(buf), "Radar: ---");
#else
  snprintf(buf, sizeof(buf), "Radar %u-%u cm", cfgMin, cfgMax);
#endif
  u8g2.drawStr(0, 56, buf);
  snprintf(buf, sizeof(buf), "N=%lu", (unsigned long)eventCount);
  u8g2.drawStr(90, 56, buf);

  if(alertActive){
    int w = 128 - (int)((held * 128) / ALERT_HOLD_MS);
    if(w < 0) w = 0;
    u8g2.drawFrame(0, 60, 128, 4);
    u8g2.drawBox(0, 60, w, 4);
  }

  u8g2.sendBuffer();
}

void setup()
{
  Serial.begin(115200);

  //pinMode(MOTOR_PIN,  OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN,    OUTPUT);
  setAlert(false);

  pinMode(TOF_XSHUT, OUTPUT);
  digitalWrite(TOF_XSHUT, LOW);   // hold sensor in reset briefly
  delay(10);
  digitalWrite(TOF_XSHUT, HIGH);  // release -> sensor boots
  delay(10);

  SPI.begin(OLED_CLK, /*miso*/-1, OLED_MOSI, OLED_CS);
  u8g2.begin();
  splash("Booting...", "Radar: waiting");

  while(!radar.begin()){
    Serial.println("NO Devices !");
    splash("RADAR NOT FOUND", "Check TX/RX + GND");
    delay(1000);
  }
  Serial.println("Device connected!");

#if RADAR_USE_SPEED_MODE
  // speed mode: real range + speed + energy (from the mRangeVelocity example)
  radar.setSensorMode(eSpeedMode);
  if(radar.setDetectThres(/*min*/30, /*max*/RADAR_MAX_CM, /*thres*/RADAR_THRES)) Serial.println("Radar: threshold set");
  else                                                                          Serial.println("Radar: setDetectThres FAILED");
  radar.setFrettingDetection(eON);        // also picks up tiny movements (breathing, standing still)
  cfgMin = radar.getTMinRange();
  cfgMax = radar.getTMaxRange();
  Serial.printf("Radar SPEED mode: range %u-%u cm, threshold %u, fretting %u\n",
                (unsigned)cfgMin, (unsigned)cfgMax, (unsigned)radar.getThresRange(), (unsigned)radar.getFrettingDetection());
#else
  // presence mode (from the motionDetection example). Max range must be >= 240 cm;
  // the 150 cm alert distance is given as the trigger range.
  radar.setSensorMode(eExitMode);
  if(radar.setDetectionRange(/*min*/30, /*max*/RADAR_MAX_CM, /*trig*/ALERT_DIST_CM)) Serial.println("Radar: range set");
  else                                                                               Serial.println("Radar: setDetectionRange FAILED");
  radar.setTrigSensitivity(RADAR_TRIG_SENS);
  radar.setKeepSensitivity(RADAR_KEEP_SENS);
  radar.setDelay(/*trig*/RADAR_TRIG_DELAY, /*keep*/RADAR_KEEP_DELAY);

  cfgMin      = radar.getMinRange();
  cfgMax      = radar.getMaxRange();
  cfgTrigSens = radar.getTrigSensitivity();
  cfgKeepSens = radar.getKeepSensitivity();
  Serial.printf("Radar PRESENCE mode: range %u-%u cm, trig range %u cm, trigSens %u, keepSens %u, trig delay %u\n",
                (unsigned)cfgMin, (unsigned)cfgMax, (unsigned)radar.getTrigRange(),
                (unsigned)cfgTrigSens, (unsigned)cfgKeepSens, (unsigned)radar.getTrigDelay());
#endif

  splash("Radar OK", "Starting ToF...");

  Wire.begin(TOF_SDA, TOF_SCL);
  if(!tof.begin()){
    Serial.println("VL53L0X not found!");
    splash("ToF NOT FOUND", "Check SDA/SCL");
    delay(1500);
  } else {
    tof.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);   // needed to reach ~1.5 m
    Serial.println("VL53L0X ready.");
  }

  // ESP-NOW link (CAM node + worker node), no router, fixed channel
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if(esp_now_init() != ESP_OK){
    Serial.println("ESP-NOW init failed");
  } else {
    esp_now_register_recv_cb(onRecv);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BCAST, 6);
    peer.channel = 0;        // 0 = current channel
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    Serial.println("ESP-NOW ready.");
  }

  setAlert(true);  delay(200);
  setAlert(false);

  drawUI();
}

void loop()
{
#if RADAR_USE_SPEED_MODE
  // gate on the real measured range
  radarRangeM = 0;
  if(radar.getTargetNumber() > 0) radarRangeM = radar.getTargetRange();
  radarMotion = (radarRangeM > 0) && (radarRangeM * 100.0f <= ALERT_DIST_CM);

  // Serial tuning aid: shows what the radar sees twice a second
  if(millis() - lastRadarDbgMs >= 500){
    lastRadarDbgMs = millis();
    if(radarRangeM > 0){
      Serial.printf("radar: range %.2f m, speed %.2f m/s, energy %lu\n",
                    radarRangeM, radar.getTargetSpeed(), (unsigned long)radar.getTargetEnergy());
    }
  }
#else
  radarMotion = radar.motionDetection();
#endif

  if(millis() - lastTofMs >= TOF_PERIOD_MS){
    lastTofMs = millis();
    pollToF();
  }

  bool prevAlert = alertActive;
  bool trigger = radarMotion || tofClose;

  if(trigger){
    lastMotionMs = millis();
    srcHuman = radarMotion;
    if(!alertActive){
      alertActive = true;
      eventCount++;
      Serial.print("ALERT ON  (radar=");
      Serial.print(radarMotion);
      Serial.print(", tof=");
      Serial.print(tofDistanceMm);
      Serial.println("mm)");
    }
  }
  else if(alertActive && (millis() - lastMotionMs >= ALERT_HOLD_MS)){
    alertActive = false;
    Serial.println("ALERT OFF");
  }

  updateOutputs();     // [WORKER] local alert first - never waits on the radio

  // tell the CAM + worker nodes: instantly on alert change, otherwise every TX_PERIOD_MS
  if(alertActive != prevAlert || millis() - lastTxMs >= TX_PERIOD_MS){
    sendStatus();
  }

  if(millis() - lastUiMs >= UI_PERIOD_MS){
    lastUiMs = millis();
    drawUI();
  }

  delay(10);
}
