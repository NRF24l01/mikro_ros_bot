#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <AsyncWebSocket.h>
#include "driver/pcnt.h"
#include <math.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "driver/adc.h"
#include "driver/uart.h"
#include "esp_task_wdt.h"
#include "config.h"

/*
КАК НАСТРОИТЬ?

Отправить в сериал порт команду, типа
ssid|password|main_port|lidar_port|client_ip|client_port
*/

/* ───── Аппаратные параметры ────────────────────────────── */
#define L_A 32
#define L_B 33
#define R_A 26
#define R_B 25
#define ENC_R_A 18
#define ENC_R_B 19
#define ENC_L_A 35
#define ENC_L_B 34
#define BATT_PIN 39

constexpr float WHEEL_RADIUS = 0.0223f;
constexpr float BASE         = 0.097f;
constexpr int   TICKS_PER_TURN = 2936;

/* ───── Serial1 TX/RX pins for outgoing config ──────────── */
#define RXD1 22  // RX для Serial1
#define TXD1 23  // TX для Serial1

/* ───── Глобальные значения ───────────────────────────── */
volatile uint8_t dutyLA = 0, dutyLB = 0, dutyRA = 0, dutyRB = 0;
volatile int32_t encTotalL = 0, encTotalR = 0;
volatile float currentBatteryVoltage = 0.0f;

AsyncWebServer* server = nullptr;
AsyncWebSocket* ws = nullptr;

/* ───── LIDAR SECTION ───── */
#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 17
#define LIDAR_BAUD   115200
#define LIDAR_BODY_LEN 32
#define LIDAR_POINTS_PER_FRAME 80
#define LIDAR_FRAME_SZ (4 + 2 * LIDAR_POINTS_PER_FRAME) // 164 байта

AsyncWebServer* lidarServer = nullptr;
AsyncWebSocket* lidarWs = nullptr;

static QueueHandle_t lidarQueue;
volatile uint32_t lidar_rx_fps = 0, lidar_tx_fps = 0;
AsyncWebSocketClient* lidarSole = nullptr;

static const uint8_t LIDAR_HDR[4] = {0x55,0xAA,0x03,0x08};
static const uint8_t LIDAR_INTENSITY_MIN = 2;
static const float   LIDAR_MAX_SPREAD_DEG = 20.0;

static inline float decodeAngle(uint16_t r){
  float a = (r - 0xA000) / 64.0f;
  if (a < 0)      a += 360.0f;
  else if(a>=360) a -= 360.0f;
  return a;
}
static bool readBytesTO(HardwareSerial& s, uint8_t* b, size_t len, uint32_t to=500){
  uint32_t t0=millis(); size_t got=0;
  while(got<len){
    int c=s.read();
    if(c>=0) b[got++]=uint8_t(c);
    else     vTaskDelay(1);
    if(millis()-t0>to) return false;
    esp_task_wdt_reset();
  }
  return true;
}
static bool waitHeader(HardwareSerial& s){
  uint8_t pos=0; uint32_t t0=millis();
  while(true){
    int c=s.read();
    if(c>=0){
      uint8_t uc=uint8_t(c);
      if(uc==LIDAR_HDR[pos]){ if(++pos==4) return true; }
      else pos=0;
    }else vTaskDelay(1);
    if(millis()-t0>200) return false;
    esp_task_wdt_reset();
  }
}

void lidarTask(void*) {
  esp_task_wdt_add(nullptr);
  Serial1.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  uint16_t cloud[LIDAR_POINTS_PER_FRAME];
  uint8_t nPts = 0;
  float startDeg = 0, endDeg = 0;
  while(true){
    if(!waitHeader(Serial1)) continue;
    uint8_t body[LIDAR_BODY_LEN];
    if(!readBytesTO(Serial1, body, LIDAR_BODY_LEN)) continue;

    float sDeg = decodeAngle(body[2] | (body[3]<<8));
    uint8_t off = 4;
    uint16_t dist[8]; uint8_t inten[8];
    for(int i=0;i<8;i++){ dist[i]=body[off]|(body[off+1]<<8); inten[i]=body[off+2]; off+=3; }
    float eDeg = decodeAngle(body[off] | (body[off+1]<<8));
    if(eDeg < sDeg) eDeg += 360.0f;
    if(eDeg - sDeg > LIDAR_MAX_SPREAD_DEG) continue;

    for(int i=0; i<8; i++){
      if (inten[i] >= LIDAR_INTENSITY_MIN) {
        if (nPts == 0) startDeg = sDeg;
        if (nPts < LIDAR_POINTS_PER_FRAME) {
          cloud[nPts] = dist[i];
          nPts++;
        }
      }
      if (nPts >= LIDAR_POINTS_PER_FRAME) break;
    }
    endDeg = eDeg;

    if (nPts >= LIDAR_POINTS_PER_FRAME) {
      uint8_t pkt[LIDAR_FRAME_SZ];
      uint8_t* p = pkt;
      uint16_t s = (uint16_t)(startDeg*100+0.5f), e = (uint16_t)(endDeg*100+0.5f);
      *p++ = s; *p++ = s >> 8; *p++ = e; *p++ = e >> 8;
      for(int i=0; i<LIDAR_POINTS_PER_FRAME; i++) {
        uint16_t d = cloud[i];
        *p++ = d; *p++ = d >> 8;
      }
      xQueueOverwrite(lidarQueue, pkt);
      ++lidar_rx_fps;
      nPts = 0;
    }
    esp_task_wdt_reset();
  }
}

void lidarWsTask(void*) {
  esp_task_wdt_add(nullptr);
  const TickType_t slot = pdMS_TO_TICKS(5);
  TickType_t prev = xTaskGetTickCount();
  uint8_t pkt[LIDAR_FRAME_SZ];
  for(;;){
    vTaskDelayUntil(&prev, slot);
    if (lidarSole && lidarSole->canSend() && xQueueReceive(lidarQueue, pkt, 0) == pdTRUE) {
      lidarSole->binary(pkt, LIDAR_FRAME_SZ);
      ++lidar_tx_fps;
    }
    esp_task_wdt_reset();
  }
}

void onLidarWsEvent(AsyncWebSocket*, AsyncWebSocketClient* c, AwsEventType t, void*, uint8_t*, size_t){
  if(t==WS_EVT_CONNECT){
    if(lidarSole && lidarSole->status()==WS_CONNECTED){
      Serial.printf("[LIDAR WS] kick old client %u\n", lidarSole->id());
      lidarSole->close();
    }
    lidarSole=c;
    Serial.printf("[LIDAR WS] + client %u\n", c->id());
  }
  if(t==WS_EVT_DISCONNECT && c==lidarSole){
    lidarSole = nullptr;
    Serial.printf("[LIDAR WS] - client %u\n", c->id());
  }
}

void setupLidarServer() {
  lidarWs = new AsyncWebSocket("/ws");
  lidarWs->onEvent(onLidarWsEvent);
  lidarServer = new AsyncWebServer(camConfig.lidar_port);
  lidarServer->addHandler(lidarWs);

  lidarServer->on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "ESP32 Lidar WebSocket Server is running. Connect to /ws");
  });

  lidarServer->begin();
  Serial.printf("LIDAR HTTP/WebSocket server started (port %d).\n", camConfig.lidar_port);
}

/* ───── ОДОМЕТРИЯ и PID ─────────────────────────────────── */
struct Odom {
  float x = 0, y = 0, theta = 0;
  float v = 0, w = 0;
} odom;
struct WheelSpeed {
  float left = 0, right = 0;
} wheelSpeed;
struct VWTarget {
  float v = 0;
  float w = 0;
} vwTarget;
struct PID {
  float kp = 1.8, ki = 3.8, kd = 0.0, kff = 0.6;
  float sumL = 0, sumR = 0, prevErrL = 0, prevErrR = 0;
  float outL = 0, outR = 0;
  float targetL = 0, targetR = 0;
} pid;

const char* PID_CONFIG_FILE = "/pid.json";

/* ───── ADDED FOR STRAIGHT DRIVING ───── */
bool straightMode = false;
int32_t leftBase = 0, rightBase = 0;
float KencStraight = 3.0f;

float readBatteryVoltage() {
  int adc_raw = adc1_get_raw(ADC1_CHANNEL_3);
  float v_adc_pin = adc_raw * (3.3f / 4095.0f);
  float v_battery = v_adc_pin * 3.0f;
  static float last_voltage = v_battery;
  v_battery = (v_battery + last_voltage) / 2.0f;
  last_voltage = v_battery;
  return v_battery;
}

void savePIDToSPIFFS() {
  DynamicJsonDocument doc(256);
  doc["kp"] = pid.kp; doc["ki"] = pid.ki; doc["kd"] = pid.kd; doc["kff"] = pid.kff;
  File file = SPIFFS.open(PID_CONFIG_FILE, FILE_WRITE);
  if (file) { serializeJson(doc, file); file.close(); }
}
void loadPIDFromSPIFFS() {
  if (!SPIFFS.exists(PID_CONFIG_FILE)) return;
  File file = SPIFFS.open(PID_CONFIG_FILE, FILE_READ);
  if (file) {
    DynamicJsonDocument doc(256);
    if (deserializeJson(doc, file) == DeserializationError::Ok) {
      pid.kp = doc["kp"] | pid.kp;
      pid.ki = doc["ki"] | pid.ki;
      pid.kd = doc["kd"] | pid.kd;
      pid.kff = doc["kff"] | pid.kff;
    }
    file.close();
  }
  Serial.printf("PID: %.3f %.3f %.3f %.3f\n", pid.kp, pid.ki, pid.kd, pid.kff);
}

void analogWriteTrack(uint8_t pin, uint8_t duty) {
  analogWrite(pin, duty);
  switch (pin) {
    case L_A: dutyLA = duty; break;  case L_B: dutyLB = duty; break;
    case R_A: dutyRA = duty; break;  case R_B: dutyRB = duty; break;
  }
}
void stopMotors() {
  analogWriteTrack(L_A,0); analogWriteTrack(L_B,0);
  analogWriteTrack(R_A,0); analogWriteTrack(R_B,0);
}

void setupEncoder(pcnt_unit_t unit, gpio_num_t a, gpio_num_t b) {
  pcnt_config_t c{};
  c.pulse_gpio_num=a; c.ctrl_gpio_num=b; c.unit=unit; c.channel=PCNT_CHANNEL_0;
  c.pos_mode=PCNT_COUNT_INC; c.neg_mode=PCNT_COUNT_DEC;
  c.lctrl_mode=PCNT_MODE_REVERSE; c.hctrl_mode=PCNT_MODE_KEEP;
  c.counter_h_lim=32767; c.counter_l_lim=-32768;
  pcnt_unit_config(&c);
  pcnt_set_filter_value(unit,100); pcnt_filter_enable(unit);
  pcnt_counter_clear(unit); pcnt_counter_resume(unit);
}
inline int16_t snapPCNT(pcnt_unit_t u){
  int16_t v; pcnt_get_counter_value(u,&v); pcnt_counter_clear(u); return v;
}

float constrainf(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }

void pidUpdate(float dt, float vL, float vR) {
  float errL = pid.targetL - vL;
  pid.sumL += errL * dt;
  pid.sumL = constrainf(pid.sumL, -500, 500);
  float dErrL = (dt > 0.0001f) ? (errL - pid.prevErrL) / dt : 0.0f;
  float ffL = pid.kff * pid.targetL;
  pid.outL = pid.kp*errL + pid.ki*pid.sumL + pid.kd*dErrL + ffL;
  pid.prevErrL = errL;
  float errR = pid.targetR - vR;
  pid.sumR += errR * dt;
  pid.sumR = constrainf(pid.sumR, -500, 500);
  float dErrR = (dt > 0.0001f) ? (errR - pid.prevErrR) / dt : 0.0f;
  float ffR = pid.kff * pid.targetR;
  pid.outR = pid.kp*errR + pid.ki*pid.sumR + pid.kd*dErrR + ffR;
  pid.prevErrR = errR;
  pid.outL = constrainf(pid.outL, -255, 255); pid.outR = constrainf(pid.outR, -255, 255);
  if (straightMode) {
    long diff = (encTotalL - leftBase) - (encTotalR - rightBase);
    float corr = KencStraight * (float)diff;
    pid.outL -= corr * 0.5f;
    pid.outR += corr * 0.5f;
    pid.outL = constrainf(pid.outL, -255, 255);
    pid.outR = constrainf(pid.outR, -255, 255);
  }
  if (pid.outL > 0) { analogWriteTrack(L_A, (uint8_t)pid.outL); analogWriteTrack(L_B, 0); }
  else { analogWriteTrack(L_A, 0); analogWriteTrack(L_B, (uint8_t)(-pid.outL)); }
  if (pid.outR > 0) { analogWriteTrack(R_A, (uint8_t)pid.outR); analogWriteTrack(R_B, 0); }
  else { analogWriteTrack(R_A, 0); analogWriteTrack(R_B, (uint8_t)(-pid.outR)); }
}

void updateOdometryAndSpeed(int32_t dTicksL, int32_t dTicksR, float dt) {
  float dL = (float)dTicksL * 2.0f * M_PI * WHEEL_RADIUS / TICKS_PER_TURN;
  float dR = (float)dTicksR * 2.0f * M_PI * WHEEL_RADIUS / TICKS_PER_TURN;
  wheelSpeed.left  = (dt > 0.0001f) ? (dL * 1000.0f / dt) : 0.0f;
  wheelSpeed.right = (dt > 0.0001f) ? (dR * 1000.0f / dt) : 0.0f;
  float dCenter = (dL + dR) * 0.5f;
  float dTheta = (dR - dL) / BASE;
  odom.x += dCenter * cosf(odom.theta + dTheta*0.5f);
  odom.y += dCenter * sinf(odom.theta + dTheta*0.5f);
  odom.theta += dTheta;
  if (dt > 0.0001f) {
    odom.v = dCenter / dt;
    odom.w = dTheta / dt;
  } else {
    odom.v = 0; odom.w = 0;
  }
}

void set_targets_from_vw(float v_mm_s, float w_rad_s) {
  pid.targetL = v_mm_s - (BASE * 500.0f) * w_rad_s;
  pid.targetR = v_mm_s + (BASE * 500.0f) * w_rad_s;
  vwTarget.v = v_mm_s;
  vwTarget.w = w_rad_s;
  if (fabs(w_rad_s) < 0.0001f && fabs(v_mm_s) > 0.0001f) {
    if (!straightMode) {
      straightMode = true;
      leftBase = encTotalL;
      rightBase = encTotalR;
    }
  } else {
    straightMode = false;
  }
}
void update_vw_from_targets() {
  vwTarget.v = (pid.targetL + pid.targetR) * 0.5f;
  vwTarget.w = (pid.targetR - pid.targetL) / (BASE * 1000.0f);
}

void sendStateOverWebSocket() {
  update_vw_from_targets();
  currentBatteryVoltage = readBatteryVoltage();
  char js_buffer[900];
  snprintf(js_buffer, sizeof(js_buffer),
    "{\"type\":\"state\","
    "\"duty\":{\"L_A\":%u,\"L_B\":%u,\"R_A\":%u,\"R_B\":%u},"
    "\"enc\":{\"left\":%ld,\"right\":%ld},"
    "\"odom\":{\"x\":%.4f,\"y\":%.4f,\"theta\":%.4f,\"v\":%.4f,\"w\":%.4f},"
    "\"speed\":{\"left\":%.1f,\"right\":%.1f},"
    "\"pid_coeffs\":{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"kff\":%.3f},"
    "\"vw_target\":{\"v\":%.1f,\"w\":%.3f},"
    "\"pwm_out\":{\"L\":%.1f,\"R\":%.1f},"
    "\"battery_v\":%.2f,"
    "\"config\":%s"
    "}",
    dutyLA, dutyLB, dutyRA, dutyRB, encTotalL, encTotalR,
    odom.x, odom.y, odom.theta, odom.v, odom.w,
    wheelSpeed.left, wheelSpeed.right,
    pid.kp, pid.ki, pid.kd, pid.kff,
    vwTarget.v, vwTarget.w,
    pid.outL, pid.outR,
    currentBatteryVoltage,
    camConfig.toJSON().c_str()
  );
  ws->textAll(js_buffer);
}

/* ───── Обработка WebSocket команд ───── */
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            data[len] = 0;
            char* msg_payload = (char*)data;
            Serial.printf("WS Rcv from #%u: %s\n", client->id(), msg_payload);

            DynamicJsonDocument doc(512);
            DeserializationError error = deserializeJson(doc, msg_payload);

            if (error) {
                client->text("{\"type\":\"status\",\"status\":\"error\",\"message\":\"Invalid JSON\"}");
                return;
            }
            const char* action = doc["action"];
            if (!action) {
                client->text("{\"type\":\"status\",\"status\":\"error\",\"message\":\"Action missing\"}");
                return;
            }

            if (strcmp(action, "setVW") == 0) {
                float v = doc["v"] | 0.0f;
                float w = doc["w"] | 0.0f;
                set_targets_from_vw(v, w);
                 if (v == 0.0f && w == 0.0f) {
                    stopMotors(); pid.targetL = 0; pid.targetR = 0;
                    pid.sumL = 0; pid.sumR = 0; pid.outL = 0; pid.outR = 0;
                }
                client->text("{\"type\":\"status\",\"action_acked\":\"setVW\",\"status\":\"success\"}");
            } else if (strcmp(action, "setPID") == 0) {
                bool changed = false;
                if (doc.containsKey("kp")) { pid.kp = doc["kp"]; changed = true; }
                if (doc.containsKey("ki")) { pid.ki = doc["ki"]; changed = true; }
                if (doc.containsKey("kd")) { pid.kd = doc["kd"]; changed = true; }
                if (doc.containsKey("kff")) { pid.kff = doc["kff"]; changed = true; }
                if (changed) savePIDToSPIFFS();
                client->text("{\"type\":\"status\",\"action_acked\":\"setPID\",\"status\":\"success\"}");
            } else if (strcmp(action, "setWheelsSpeed") == 0) {
                if (doc.containsKey("left"))  pid.targetL = doc["left"];
                if (doc.containsKey("right")) pid.targetR = doc["right"];
                update_vw_from_targets();
                client->text("{\"type\":\"status\",\"action_acked\":\"setWheelsSpeed\",\"status\":\"success\"}");
            } else if (strcmp(action, "resetAll") == 0) {
                encTotalL=encTotalR=0; odom = Odom(); wheelSpeed = WheelSpeed();
                pid.sumL = 0; pid.sumR = 0; pid.prevErrL = 0; pid.prevErrR = 0; pid.outL = 0; pid.outR = 0;
                pid.targetL = 0; pid.targetR = 0; vwTarget.v = 0; vwTarget.w = 0;
                stopMotors(); pcnt_counter_clear(PCNT_UNIT_0); pcnt_counter_clear(PCNT_UNIT_1);
                client->text("{\"type\":\"status\",\"action_acked\":\"resetAll\",\"status\":\"success\"}");
            } else if (strcmp(action, "getPID") == 0) {
                char js[128];
                snprintf(js, sizeof(js), "{\"type\":\"pid_coeffs\",\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"kff\":%.3f}", pid.kp, pid.ki, pid.kd, pid.kff);
                client->text(js);
            } else if (strcmp(action, "getBattery") == 0) {
                currentBatteryVoltage = readBatteryVoltage();
                char js[64];
                snprintf(js, sizeof(js), "{\"type\":\"battery\",\"voltage\":%.2f}", currentBatteryVoltage);
                client->text(js);
            } else if (strcmp(action, "setConfig") == 0) {
                // { "action": "setConfig", ... }
                String err;
                bool changed = false;
                if (doc.containsKey("ssid")) { camConfig.ssid = String((const char*)doc["ssid"]); changed = true; }
                if (doc.containsKey("password")) { camConfig.password = String((const char*)doc["password"]); changed = true; }
                if (doc.containsKey("main_port")) { camConfig.main_port = doc["main_port"]; changed = true; }
                if (doc.containsKey("lidar_port")) { camConfig.lidar_port = doc["lidar_port"]; changed = true; }
                if (doc.containsKey("client_ip")) { camConfig.client_ip = String((const char*)doc["client_ip"]); changed = true; }
                if (doc.containsKey("client_port")) { camConfig.client_port = doc["client_port"]; changed = true; }
                if (changed) {
                  camConfig.save();
                  client->text("{\"type\":\"status\",\"action_acked\":\"setConfig\",\"status\":\"success\",\"message\":\"Reboot required for network/port changes\"}");
                } else {
                  client->text("{\"type\":\"status\",\"action_acked\":\"setConfig\",\"status\":\"nochange\"}");
                }
            } else if (strcmp(action, "getConfig") == 0) {
                client->text(camConfig.toJSON());
            } else {
                client->text("{\"type\":\"status\",\"status\":\"error\",\"message\":\"Unknown action\"}");
            }
        }
      }
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      Serial.printf("WebSocket client #%u error #%u: %s\n", client->id(), *(uint16_t*)arg, (char*)data);
      break;
  }
}

void setupWiFiAndServer() {
  WiFi.mode(WIFI_STA); WiFi.begin(camConfig.ssid.c_str(),camConfig.password.c_str());
  Serial.print("Connecting to WiFi");
  while(WiFi.status()!=WL_CONNECTED){ Serial.print('.'); delay(400); checkSerialConfig();}
  Serial.printf("\nIP: %s\n",WiFi.localIP().toString().c_str());

  ws = new AsyncWebSocket("/ws");
  ws->onEvent(onWsEvent);
  server = new AsyncWebServer(camConfig.main_port);
  server->addHandler(ws);

  server->on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "ESP32 Robot WebSocket Server is running. Connect to /ws");
  });

  server->begin();
  Serial.printf("HTTP server and WebSocket server started (port %d).\n", camConfig.main_port);
}

/* ───── Функция для отправки строки с конфигом в Serial1 ───── */
void sendConfigToSerial1() {
  // Формат: ssid|password|client_ip|client_port
  String cfg = camConfig.ssid + "|" + camConfig.password + "|" + camConfig.client_ip + "|" + String(camConfig.client_port);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1); // RX - 22, TX - 23
  delay(100);
  Serial1.println(cfg);
  Serial.printf("[Config->Serial1] %s\n", cfg.c_str());
  delay(100); // Дать время на отправку
  Serial1.end();
}

/* ───── MAIN SETUP ────────────────────────────────── */
void setup() {
  Serial.begin(115200);
  delay(1000);

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);

  camConfig.load();

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed!");
  } else {
    loadPIDFromSPIFFS();
  }

  pinMode(L_A,OUTPUT); pinMode(L_B,OUTPUT);
  pinMode(R_A,OUTPUT); pinMode(R_B,OUTPUT); stopMotors();

  pinMode(ENC_R_A,INPUT); pinMode(ENC_R_B,INPUT);
  pinMode(ENC_L_A,INPUT); pinMode(ENC_L_B,INPUT);
  setupEncoder(PCNT_UNIT_0,(gpio_num_t)ENC_R_A,(gpio_num_t)ENC_R_B);
  setupEncoder(PCNT_UNIT_1,(gpio_num_t)ENC_L_A,(gpio_num_t)ENC_L_B);
  
  checkSerialConfig();
  setupWiFiAndServer();

  lidarQueue = xQueueCreate(1, LIDAR_FRAME_SZ);
  setupLidarServer();
  xTaskCreatePinnedToCore(lidarTask, "LIDAR", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(lidarWsTask, "LIDAR_WS_TX", 4096, nullptr, 2, nullptr, 0);

  currentBatteryVoltage = readBatteryVoltage();
  Serial.printf("Initial Battery Voltage: %.2f V\n", currentBatteryVoltage);

  // Отправить данные на Serial1 при запуске
  sendConfigToSerial1();

  esp_task_wdt_config_t wdt_config = { .timeout_ms = 5000, .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, .trigger_panic = true };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
}

/* ───── LIDAR MONITOR PRINT ───────────────────────── */
void printLidarStats() {
  static uint32_t t0 = 0;
  if (millis() - t0 >= 1000) {
    t0 = millis();
    uint32_t rx = lidar_rx_fps; lidar_rx_fps = 0;
    uint32_t tx = lidar_tx_fps; lidar_tx_fps = 0;
    if (rx > 0 || tx > 0)
      Serial.printf("[LIDAR] rx:%3u  tx:%3u  client:%s\n", rx, tx, lidarSole ? "yes" : "no");
  }
}

/* ───── Чтение конфигурации с Serial ───────────────────────── */
void checkSerialConfig() {
  static String line = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length() > 5) {
        String err;
        if (camConfig.parseAndSet(line, err)) {
          camConfig.save();
          Serial.println("Config updated, please reboot for changes to take effect.");
          ESP.restart();
        }
        else
        {
          Serial.print("Config error: "); Serial.println(err);
        }
      }
      line = "";
    } else {
      line += c;
    }
  }
}

void loop() {
  static uint32_t tEnc=0;
  static uint32_t lastUpdate = 0;
  uint32_t now_ms = millis();

  checkSerialConfig();

  if(now_ms - tEnc >= 10){
    tEnc = now_ms;
    int16_t dR = snapPCNT(PCNT_UNIT_0);
    int16_t dL = snapPCNT(PCNT_UNIT_1);
    encTotalR += dR;
    encTotalL += dL;
    float dt = (lastUpdate > 0) ? (now_ms - lastUpdate) / 1000.0f : 0.01f;
    if(dt > 0.0001f) {
      updateOdometryAndSpeed(dL, dR, dt);
      if (vwTarget.v != 0.0f || vwTarget.w != 0.0f || pid.outL !=0 || pid.outR !=0 ) {
         pidUpdate(dt, wheelSpeed.left, wheelSpeed.right);
      } else {
         stopMotors();
      }
    }
    lastUpdate = now_ms;
  }

  static uint32_t tStateWs = 0;
  if(now_ms - tStateWs >= 500) {
    tStateWs = now_ms;
    if (ws && ws->count() > 0) {
        sendStateOverWebSocket();
    }
    Serial.printf("Clients: %d, V:%.1f W:%.3f PWM L:%.1f R:%.1f Batt:%.2fV\n", ws ? ws->count() : 0, vwTarget.v, vwTarget.w, pid.outL, pid.outR, currentBatteryVoltage);
  }

  if (lidarWs) lidarWs->cleanupClients();
  printLidarStats();
  esp_task_wdt_reset();
}