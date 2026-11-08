/*
  ============================================================
  WORKER NODE  -  ESP32-C3 Super Mini  (worn on the arm)
  ------------------------------------------------------------
  UWB      : RYUW122 in ANCHOR mode, measures distance to the
             tag that sits on the machine.
  Outputs  : buzzer, red LED, vibration motor, SH1106 SPI OLED
  Radio    : ESP-NOW broadcast, two-way with the machine node.

  Arduino IDE : Board "ESP32C3 Dev Module", USB CDC On Boot = Enabled
  Library     : U8g2

  Alarm rule  : ALARM = (UWB distance < threshold)
                        OR (machine node reports human/obstacle in front)
  Each node sends only its OWN detection flag, so the two
  nodes never latch each other's alarm.
  ============================================================
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <U8g2lib.h>

// ---------------- Pins ----------------
#define UWB_RX      20     // ESP32 RX <- module TXD
#define UWB_TX      21     // ESP32 TX -> module RXD
#define UWB_NRST    7
#define UWB_BAUD    115200

#define OLED_CLK    2      // SCK
#define OLED_MOSI   4      // SDA
#define OLED_RST    0
#define OLED_DC     1
#define OLED_CS     5

#define BUZZER_PIN  6
#define LED_PIN     3
#define MOTOR_PIN   10     // motor module IN (parked, harmless if unused)

// ---------------- Settings ----------------
#define ALARM_ON_CM       150   // worker alarm starts below this distance
#define ALARM_OFF_CM      170   // ...and stops above this (hysteresis)
#define CONFIRM_COUNT     2     // consecutive low readings needed
#define LOST_LIMIT        5     // failed UWB polls before "no signal"
#define UWB_LOST_ALARM    0     // 1 = alarm while UWB signal is lost
#define UWB_TIMEOUT_MS    1000  // wait for +ANCHOR_RCV

#define HEARTBEAT_MS      200
#define LINK_TIMEOUT_MS   2000
#define LINK_LOST_WARN    0     // 1 = short chirp while machine link is lost
#define BLINK_MS          250
#define ESPNOW_CHANNEL    1     // must be the same on ALL nodes

#define OLED_IS_SH1106    1     // 0 = SSD1306

const char* TAG_ADDR = "TAG00001";
const char* PAYLOAD  = "PING";

// ---------------- Packets ----------------
// Machine -> worker: the SAME Packet your sensor_node already broadcasts to the CAM node.
#define SENSOR_MAGIC 0xA5
#define PKT_SENSOR   1

struct __attribute__((packed)) Packet {
  uint8_t  magic;
  uint8_t  type;    // PKT_SENSOR
  uint8_t  alert;   // 1 = machine's own alert (radar human / ToF obstacle)
  uint8_t  flags;   // bit0 human (radar), bit1 obstacle (ToF)
  uint16_t value;   // ToF distance in mm, 0xFFFF = none
  uint32_t seq;
};

// Worker -> machine (different magic + length, so the CAM node ignores it).
#define PKT_MAGIC    0xA7
#define PKT_WORKER   2

struct __attribute__((packed)) WorkerPkt {    // worker -> machine
  uint8_t  magic;
  uint8_t  type;
  uint8_t  seq;
  uint8_t  uwbOk;
  uint16_t distCm;
  uint8_t  hazard;
};

const uint8_t BROADCAST_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---------------- OLED ----------------
#if OLED_IS_SH1106
U8G2_SH1106_128X64_NONAME_F_4W_SW_SPI u8g2(U8G2_R0, OLED_CLK, OLED_MOSI, OLED_CS, OLED_DC, OLED_RST);
#else
U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI u8g2(U8G2_R0, OLED_CLK, OLED_MOSI, OLED_CS, OLED_DC, OLED_RST);
#endif

HardwareSerial UWB(1);

// ---------------- State ----------------
// from machine (written in the ESP-NOW callback)
volatile bool     haveMachine    = false;
volatile uint32_t lastMachineRx  = 0;
volatile uint8_t  rxHuman        = 0;
volatile uint8_t  rxObstacle     = 0;
volatile uint16_t rxTofMm        = 0xFFFF;
volatile uint8_t  rxMachineHazard = 0;

// local UWB
const int WIN = 5;
int  hist[WIN];
int  histN = 0, histIdx = 0;
int  uwbDist = -1;            // filtered cm, -1 = no signal
bool uwbAlarm = false;
int  belowCount = 0, failCount = 0;

uint8_t  seqNo = 0;
uint32_t lastTx = 0;

// ---------------- Logic ----------------
bool machineLinkOk() { return haveMachine && (millis() - lastMachineRx) < LINK_TIMEOUT_MS; }
bool machineHazard() { return machineLinkOk() && rxMachineHazard; }
bool alarmNow()      { return uwbAlarm || machineHazard(); }

void driveOutputs() {
  bool alarm = alarmNow();
  bool phase = (millis() / BLINK_MS) & 1;
  bool warn  = false;
#if LINK_LOST_WARN
  warn = !alarm && !machineLinkOk() && ((millis() % 2000) < 100);
#endif
  bool on = alarm ? phase : warn;
  digitalWrite(MOTOR_PIN, alarm ? HIGH : LOW);
  digitalWrite(BUZZER_PIN, on);
  digitalWrite(LED_PIN, on);
}

// ---------------- ESP-NOW ----------------
void handleRx(const uint8_t *data, int len) {
  if (len != (int)sizeof(Packet)) return;
  Packet p;
  memcpy(&p, data, sizeof(p));
  if (p.magic != SENSOR_MAGIC || p.type != PKT_SENSOR) return;
  rxHuman         = (p.flags & 0x01) ? 1 : 0;
  rxObstacle      = (p.flags & 0x02) ? 1 : 0;
  rxTofMm         = p.value;
  rxMachineHazard = p.alert;
  lastMachineRx   = millis();
  haveMachine     = true;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) { handleRx(data, len); }
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) { handleRx(data, len); }
#endif

bool espnowInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onRecv);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_ADDR, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

void sendHeartbeat() {
  WorkerPkt p;
  p.magic  = PKT_MAGIC;
  p.type   = PKT_WORKER;
  p.seq    = seqNo++;
  p.uwbOk  = (uwbDist >= 0) ? 1 : 0;
  p.distCm = (uwbDist >= 0) ? (uint16_t)uwbDist : 0xFFFF;
  p.hazard = uwbAlarm ? 1 : 0;
  esp_now_send(BROADCAST_ADDR, (uint8_t *)&p, sizeof(p));
}

// Called from inside the UWB wait loop so heartbeats and blinking
// keep running even while waiting for a UWB reply.
void background() {
  uint32_t now = millis();
  if (now - lastTx >= HEARTBEAT_MS) { lastTx = now; sendHeartbeat(); }
  driveOutputs();
}

// ---------------- UWB ----------------
void sendCmd(const String &cmd) {
  UWB.print(cmd);
  UWB.print("\r\n");
}

bool readLine(String &out, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  out = "";
  while (millis() - t0 < timeoutMs) {
    background();
    while (UWB.available()) {
      char c = UWB.read();
      if (c == '\n') {
        out.trim();
        if (out.length()) return true;
        out = "";
      } else {
        out += c;
      }
    }
    delay(1);
  }
  return false;
}

void resetModule() {
  pinMode(UWB_NRST, OUTPUT);
  digitalWrite(UWB_NRST, LOW);  delay(100);
  digitalWrite(UWB_NRST, HIGH); delay(1000);
}

// Parses: +ANCHOR_RCV=TAG00001,4,PING,62 cm[,-70 dBm]
int parseDistance(const String &line) {
  int eq = line.indexOf('=');
  if (eq < 0) return -1;
  String f[6];
  int n = 0, start = eq + 1;
  for (int i = start; i <= (int)line.length() && n < 6; i++) {
    if (i == (int)line.length() || line[i] == ',') {
      f[n++] = line.substring(start, i);
      start = i + 1;
    }
  }
  if (n < 4) return -1;
  return f[3].toInt();
}

int measureDistance() {
  while (UWB.available()) UWB.read();
  sendCmd(String("AT+ANCHOR_SEND=") + TAG_ADDR + "," + strlen(PAYLOAD) + "," + PAYLOAD);

  String line;
  uint32_t t0 = millis();
  while (millis() - t0 < UWB_TIMEOUT_MS) {
    if (!readLine(line, UWB_TIMEOUT_MS - (millis() - t0))) break;
    if (line.startsWith("+ANCHOR_RCV")) return parseDistance(line);
    if (line.startsWith("+ERR")) { Serial.println(line); return -1; }
  }
  return -1;
}

int addAndMedian(int cm) {
  hist[histIdx] = cm;
  histIdx = (histIdx + 1) % WIN;
  if (histN < WIN) histN++;
  int tmp[WIN];
  memcpy(tmp, hist, sizeof(int) * histN);
  for (int i = 1; i < histN; i++) {
    int k = tmp[i], j = i - 1;
    while (j >= 0 && tmp[j] > k) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = k;
  }
  return tmp[histN / 2];
}

void updateUwbAlarm(int cm) {
  if (!uwbAlarm) {
    if (cm < ALARM_ON_CM) {
      if (++belowCount >= CONFIRM_COUNT) uwbAlarm = true;
    } else {
      belowCount = 0;
    }
  } else if (cm > ALARM_OFF_CM) {
    uwbAlarm = false;
    belowCount = 0;
  }
}

// ---------------- Display ----------------
void showOled() {
  char b[28];
  bool link = machineLinkOk();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 10, "WORKER");
  if (alarmNow()) u8g2.drawStr(86, 10, "ALARM!");

  if (uwbDist >= 0) {
    u8g2.setFont(u8g2_font_logisoso22_tn);
    snprintf(b, sizeof(b), "%d", uwbDist);
    u8g2.drawStr(0, 40, b);
    int w = u8g2.getStrWidth(b);
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(w + 6, 40, "cm");
  } else {
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(0, 36, "UWB: no signal");
  }

  u8g2.setFont(u8g2_font_6x12_tr);
  if (!link)                snprintf(b, sizeof(b), "Front: no link");
  else if (rxHuman)         snprintf(b, sizeof(b), "Front: HUMAN");
  else if (rxObstacle)      snprintf(b, sizeof(b), "Front: OBJ %u cm", (unsigned)(rxTofMm / 10));
  else                      snprintf(b, sizeof(b), "Front: clear");
  u8g2.drawStr(0, 52, b);

  snprintf(b, sizeof(b), "Link : %s", link ? "OK" : "LOST");
  u8g2.drawStr(0, 63, b);
  u8g2.sendBuffer();
}

// ---------------- Main ----------------
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(MOTOR_PIN, LOW);

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 12, "Worker node");
  u8g2.drawStr(0, 26, "Starting...");
  u8g2.sendBuffer();

  UWB.begin(UWB_BAUD, SERIAL_8N1, UWB_RX, UWB_TX);
  resetModule();
  bool nowOk = espnowInit();
  Serial.printf("ESP-NOW:%s\n", nowOk ? "OK" : "ERR");
}

void loop() {
  int d = measureDistance();
  if (d >= 0) {
    failCount = 0;
    uwbDist = addAndMedian(d);
    updateUwbAlarm(uwbDist);
  } else {
    if (failCount < 100) failCount++;
    if (failCount >= LOST_LIMIT) {
      histN = 0; histIdx = 0; belowCount = 0;
      uwbDist = -1;
      uwbAlarm = UWB_LOST_ALARM;
    }
  }

  Serial.printf("uwb raw:%d filt:%d | machine link:%d human:%d tof:%u mHaz:%d | ALARM:%d\n",
                d, uwbDist, machineLinkOk(), rxHuman, (unsigned)rxTofMm,
                rxMachineHazard, alarmNow());

  showOled();
  background();
  delay(50);
}
