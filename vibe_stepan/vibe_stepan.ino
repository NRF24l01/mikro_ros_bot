#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <AsyncWebSocket.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "driver/pcnt.h"
#include "esp_task_wdt.h"

/* ======== Конфигурация ======== */
struct RobotConfig {
  String ssid = "robotx";
  String password = "78914040";
  uint16_t main_port = 80;
  uint16_t lidar_port = 81;
  String client_ip = "192.168.1.42";
  uint16_t client_port = 9001;

  void load() {
    if (!SPIFFS.exists("/config.json")) return;
    File f = SPIFFS.open("/config.json", FILE_READ);
    if (!f) return;
    DynamicJsonDocument doc(1024);
    if (deserializeJson(doc, f) == DeserializationError::Ok) {
      ssid = doc["ssid"] | ssid;
      password = doc["password"] | password;
      main_port = doc["main_port"] | main_port;
      lidar_port = doc["lidar_port"] | lidar_port;
      client_ip = doc["client_ip"] | client_ip;
      client_port = doc["client_port"] | client_port;
    }
    f.close();
  }
  
  void save() const {
    File f = SPIFFS.open("/config.json", FILE_WRITE);
    if (!f) return;
    DynamicJsonDocument doc(1024);
    doc["ssid"] = ssid;
    doc["password"] = password;
    doc["main_port"] = main_port;
    doc["lidar_port"] = lidar_port;
    doc["client_ip"] = client_ip;
    doc["client_port"] = client_port;
    serializeJson(doc, f);
    f.close();
  }
  
  String toString() const {
    char buf[256];
    snprintf(buf, sizeof(buf), "%s|%s|%d|%d|%s|%d", 
             ssid.c_str(), password.c_str(), main_port, lidar_port, 
             client_ip.c_str(), client_port);
    return String(buf);
  }
  
  String toSecondBoardString() const {
    char buf[256];
    snprintf(buf, sizeof(buf), "%s|%s", 
             ssid.c_str(), password.c_str());
    return String(buf);
  }
};

RobotConfig robotConfig;

/* ======== Аппаратные параметры ======== */
#define L_A 32
#define L_B 33
#define R_A 26
#define R_B 25
#define ENC_R_A 18
#define ENC_R_B 19
#define ENC_L_A 35
#define ENC_L_B 34
#define RXD1 22
#define TXD1 23
#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 17
#define LIDAR_BAUD   115200
constexpr float WHEEL_RADIUS = 0.0223f;
constexpr float BASE         = 0.097f;
constexpr int   TICKS_PER_TURN = 2936;

/* ======== Глобальные переменные ======== */
volatile uint8_t dutyLA = 0, dutyLB = 0, dutyRA = 0, dutyRB = 0;
volatile int32_t encTotL = 0, encTotR = 0;
volatile int32_t prevEncL = 0, prevEncR = 0;
volatile float speedL = 0.0f, speedR = 0.0f;
volatile float tgtL = 0.0f, tgtR = 0.0f;
volatile float odomX = 0.0f, odomY = 0.0f, odomTh = 0.0f;
volatile float kp = 1.0f, ki = 0.8f, kd = 0.02f, kff = 0.25f;
volatile uint32_t lastCmdMs = 0;
bool alignMode = false;
float alignSign = 1.0f;
int32_t alignRefL = 0, alignRefR = 0;
constexpr float kAlign = 1.0f;
constexpr float MM_PER_TICK = WHEEL_RADIUS * 2.0f * M_PI * 1000.0f / TICKS_PER_TURN;

/* ======== Лидар ======== */
static const uint8_t HDR[4] = { 0x55, 0xAA, 0x03, 0x08 };
static const uint8_t BODY_LEN = 32;
static const uint8_t INTENSITY_MIN = 2;
static const float MAX_SPREAD_DEG = 20.0;
#define FRAME_LEN 20
#define MAX_FRAMES 64
static uint8_t scanBuf[MAX_FRAMES * FRAME_LEN];
static uint8_t *wr = scanBuf;
static uint8_t frameCount = 0;
static float prevStartAngle = -1;
volatile uint32_t stat_rx = 0;
volatile uint32_t stat_tx = 0;

/* ======== Web ======== */
AsyncWebServer *server = nullptr;
AsyncWebSocket *ws = nullptr;
AsyncWebServer *lidarServer = nullptr;
AsyncWebSocket *lidarWs = nullptr;
AsyncWebSocketClient *wsClient = nullptr;
AsyncWebSocketClient *lidarSole = nullptr;

/* ========== Вспомогательные функции ========== */
inline float decodeAngle(uint16_t raw) {
  float a = (raw - 0xA000) / 64.0f;
  if (a < 0) a += 360.0f;
  else if (a >= 360) a -= 360.0f;
  return a;
}

bool readBytes(HardwareSerial &serial, uint8_t *dst, size_t n, uint32_t timeout = 300) {
  uint32_t t0 = millis();
  for (size_t i = 0; i < n; ++i) {
    while (!serial.available()) {
      if (millis() - t0 > timeout) return false;
      vTaskDelay(1);
      esp_task_wdt_reset();
    }
    dst[i] = serial.read();
  }
  return true;
}

bool waitLidarHeader(HardwareSerial &serial) {
  uint8_t pos = 0;
  uint32_t t0 = millis();
  while (true) {
    if (serial.available()) {
      if (uint8_t(serial.read()) == HDR[pos]) {
        if (++pos == 4) return true;
      } else {
        pos = 0;
      }
    }
    if (millis() - t0 > 200) return false;
    esp_task_wdt_reset();
  }
}

inline uint16_t crc16(uint16_t crc, uint8_t v) {
  crc ^= v;
  for (uint8_t i = 0; i < 8; ++i) {
    crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

/* ========== Serial1: отправка конфига для второй платы ========== */
void sendConfigToSerial1() {
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  delay(100);
  
  String configMessage = robotConfig.toSecondBoardString();
  Serial1.println(configMessage);
  
  Serial.printf("[Config->Serial1] %s\n", configMessage.c_str());
  delay(100);
  Serial1.end();
}

/* ========== Serial1: чтение данных ========== */
void checkSerial1Data() {
  static String line = "";
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n' || c == '\r') {
      if (line.length() > 0) {
        Serial.println("[Serial1] " + line);
      }
      line = "";
    } else if (c >= 32 && c <= 126) {
      line += c;
      if (line.length() > 200) {
        line = line.substring(0, 200);
      }
    }
  }
}

/* ========== Serial: приём конфига ========== */
void checkSerialConfig() {
  static String line = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length() > 5) {
        int idx = 0, prev = 0;
        String parts[6];
        for (int i = 0; i < 6; ++i) {
          idx = line.indexOf('|', prev);
          if (idx < 0 && i < 5) { break; }
          parts[i] = line.substring(prev, (i < 5) ? idx : line.length());
          prev = idx + 1;
        }
        if (parts[0].length()) robotConfig.ssid = parts[0];
        if (parts[1].length()) robotConfig.password = parts[1];
        if (parts[2].length()) robotConfig.main_port = parts[2].toInt();
        if (parts[3].length()) robotConfig.lidar_port = parts[3].toInt();
        if (parts[4].length()) robotConfig.client_ip = parts[4];
        if (parts[5].length()) robotConfig.client_port = parts[5].toInt();
        robotConfig.save();
        Serial.println("Config updated, rebooting...");
        delay(200);
        ESP.restart();
      }
      line = "";
    } else {
      line += c;
    }
  }
}

/* ========== PCNT (энкодеры) ========== */
void pcntInit(pcnt_unit_t unit, gpio_num_t pulse_pin, gpio_num_t ctrl_pin) {
  pcnt_config_t cfg = {};
  cfg.unit = unit;
  cfg.channel = PCNT_CHANNEL_0;
  cfg.pulse_gpio_num = pulse_pin;
  cfg.ctrl_gpio_num = ctrl_pin;
  cfg.pos_mode = PCNT_COUNT_INC;
  cfg.neg_mode = PCNT_COUNT_DEC;
  cfg.lctrl_mode = PCNT_MODE_REVERSE;
  cfg.hctrl_mode = PCNT_MODE_KEEP;
  cfg.counter_h_lim = 32767;
  cfg.counter_l_lim = -32768;
  pcnt_unit_config(&cfg);
  pcnt_set_filter_value(unit, 100);
  pcnt_filter_enable(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

inline int16_t readEncoder(pcnt_unit_t unit) {
  int16_t count = 0;
  pcnt_get_counter_value(unit, &count);
  pcnt_counter_clear(unit);
  return count;
}

/* ========== PWM/Моторы ========== */
inline void setPWM(uint8_t pin, uint8_t value) {
  analogWrite(pin, value);
  switch (pin) {
    case L_A: dutyLA = value; break;
    case L_B: dutyLB = value; break;
    case R_A: dutyRA = value; break;
    case R_B: dutyRB = value; break;
  }
}

inline void stopMotors() {
  setPWM(L_A, 0);
  setPWM(L_B, 0);
  setPWM(R_A, 0);
  setPWM(R_B, 0);
}

/* ========== PID ========== */
void updatePID() {
  static uint32_t prevMillis = millis();
  static float iTermL = 0, iTermR = 0;
  static float prevErrorL = 0, prevErrorR = 0;
  static float lastTgtL = 0, lastTgtR = 0;

  uint32_t now = millis();
  float dt = (now - prevMillis) * 0.001f;
  if (dt < 0.001f) dt = 0.001f;
  prevMillis = now;

  if (tgtL != lastTgtL || fabs(tgtL) < 1.0f) { iTermL = 0; prevErrorL = 0; }
  if (tgtR != lastTgtR || fabs(tgtR) < 1.0f) { iTermR = 0; prevErrorR = 0; }
  lastTgtL = tgtL;
  lastTgtR = tgtR;

  float corr = 0.0f;
  if (alignMode) {
    int32_t dL = encTotL - alignRefL;
    int32_t dR = encTotR - alignRefR;
    float diff_mm = (float)(dL - alignSign * dR) * MM_PER_TICK;
    corr = kAlign * diff_mm;
  }
  float tgtCorrL = tgtL - corr;
  float tgtCorrR = tgtR + alignSign * corr;

  float errorL = tgtCorrL - speedL;
  float errorR = tgtCorrR - speedR;
  iTermL += errorL * dt;
  iTermR += errorR * dt;
  const float I_LIMIT = 300.0f;
  if (iTermL > I_LIMIT) iTermL = I_LIMIT;
  if (iTermL < -I_LIMIT) iTermL = -I_LIMIT;
  if (iTermR > I_LIMIT) iTermR = I_LIMIT;
  if (iTermR < -I_LIMIT) iTermR = -I_LIMIT;
  float dTermL = (errorL - prevErrorL) / dt;
  float dTermR = (errorR - prevErrorR) / dt;
  prevErrorL = errorL;
  prevErrorR = errorR;
  float outputL = kp * errorL + ki * iTermL + kd * dTermL + kff * tgtCorrL;
  float outputR = kp * errorR + ki * iTermR + kd * dTermR + kff * tgtCorrR;
  if (outputL > 255) outputL = 255;
  if (outputL < -255) outputL = -255;
  if (outputR > 255) outputR = 255;
  if (outputR < -255) outputR = -255;
  uint8_t pwmLA, pwmLB, pwmRA, pwmRB;
  if (outputL >= 0) { pwmLA = (uint8_t)outputL; pwmLB = 0; }
  else { pwmLA = 0; pwmLB = (uint8_t)(-outputL); }
  if (outputR >= 0) { pwmRA = (uint8_t)outputR; pwmRB = 0; }
  else { pwmRA = 0; pwmRB = (uint8_t)(-outputR); }
  setPWM(L_A, pwmLA);
  setPWM(L_B, pwmLB);
  setPWM(R_A, pwmRA);
  setPWM(R_B, pwmRB);
}

/* ========== Лидар Task ========= */
void lidarTask(void *param) {
  esp_task_wdt_add(NULL);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  uint8_t body[BODY_LEN];
  while (true) {
    esp_task_wdt_reset();
    vTaskDelay(1);
    if (!waitLidarHeader(Serial2)) { continue; }
    if (!readBytes(Serial2, body, BODY_LEN)) { continue; }
    float startDeg = decodeAngle(body[2] | (body[3] << 8));
    uint8_t offset = 4;
    uint16_t dist[8];
    uint8_t quality[8];
    for (int i = 0; i < 8; ++i) {
      dist[i] = body[offset] | (body[offset + 1] << 8);
      quality[i] = body[offset + 2];
      offset += 3;
    }
    float endDeg = decodeAngle(body[offset] | (body[offset + 1] << 8));
    if (endDeg < startDeg) endDeg += 360.0f;
    if (endDeg - startDeg > MAX_SPREAD_DEG) continue;
    uint16_t s = (uint16_t)(startDeg * 100 + 0.5f);
    uint16_t e = (uint16_t)(endDeg * 100 + 0.5f);
    *wr++ = s & 0xFF;
    *wr++ = s >> 8;
    *wr++ = e & 0xFF;
    *wr++ = e >> 8;
    for (int i = 0; i < 8; ++i) {
      uint16_t d = (quality[i] >= INTENSITY_MIN) ? dist[i] : 0;
      *wr++ = d & 0xFF;
      *wr++ = d >> 8;
    }
    frameCount++;
    stat_rx++;
    if (frameCount >= MAX_FRAMES) {
      frameCount = 0;
      wr = scanBuf;
      prevStartAngle = -1;
      continue;
    }
    if (prevStartAngle >= 0.0f && startDeg < prevStartAngle && frameCount >= 30) {
      size_t scanSize = frameCount * FRAME_LEN;
      uint16_t crc = 0xFFFF;
      for (size_t i = 0; i < scanSize; ++i) {
        crc = crc16(crc, scanBuf[i]);
      }
      scanBuf[scanSize] = crc & 0xFF;
      scanBuf[scanSize + 1] = crc >> 8;
      if (lidarSole && lidarSole->canSend()) {
        lidarSole->binary(scanBuf, scanSize + 2);
        stat_tx++;
      }
      wr = scanBuf;
      frameCount = 0;
    }
    prevStartAngle = startDeg;
  }
}

/* ========== Отправка состояния робота по WebSocket ========== */
void sendRobotState() {
  if (!wsClient || wsClient->status() != WS_CONNECTED) return;
  
  DynamicJsonDocument doc(1024);
  doc["type"] = "state";
  
  JsonObject duty = doc.createNestedObject("duty");
  duty["L_A"] = dutyLA;
  duty["L_B"] = dutyLB;
  duty["R_A"] = dutyRA;
  duty["R_B"] = dutyRB;
  
  JsonObject enc = doc.createNestedObject("enc");
  enc["left"] = encTotL;
  enc["right"] = encTotR;
  
  JsonObject speed = doc.createNestedObject("speed");
  speed["left"] = speedL;
  speed["right"] = speedR;
  
  JsonObject target = doc.createNestedObject("target");
  target["left"] = tgtL;
  target["right"] = tgtR;
  
  JsonObject odom = doc.createNestedObject("odom");
  odom["x"] = odomX;
  odom["y"] = odomY;
  odom["th"] = odomTh;
  
  JsonObject pid = doc.createNestedObject("pid");
  pid["kp"] = kp;
  pid["ki"] = ki;
  pid["kd"] = kd;
  pid["kff"] = kff;
  
  doc["align_mode"] = alignMode;
  doc["uptime"] = millis();
  
  String message;
  serializeJson(doc, message);
  wsClient->text(message);
}

/* ========== Основная Task ========= */
void mainTask(void*) {
  esp_task_wdt_add(NULL);
  static uint32_t t10 = 0, t20 = 0, t100 = 0;
  uint32_t now;
  
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  
  while (true) {
    now = millis();
    esp_task_wdt_reset();
    
    checkSerialConfig();
    checkSerial1Data();

    if (now - lastCmdMs > 3000) {
      if (tgtL != 0 || tgtR != 0) {
        tgtL = tgtR = 0;
        alignMode = false;
        stopMotors();
        Serial.println("[SAFE] cmd timeout → STOP");
      }
    }
    
    if (now - t10 >= 10) {
      t10 = now;
      int16_t dR = readEncoder(PCNT_UNIT_0);
      int16_t dL = readEncoder(PCNT_UNIT_1);
      encTotR += dR;
      encTotL += dL;
      float sR = dR * MM_PER_TICK / 1000.0f;
      float sL = dL * MM_PER_TICK / 1000.0f;
      float ds = 0.5f * (sR + sL);
      float dth = (sR - sL) / BASE;
      float midTh = odomTh + 0.5f * dth;
      odomX += ds * cosf(midTh);
      odomY += ds * sinf(midTh);
      odomTh += dth;
      if (odomTh > M_PI) odomTh -= 2 * M_PI;
      if (odomTh < -M_PI) odomTh += 2 * M_PI;
    }
    
    if (now - t20 >= 20) {
      float dt = (now - t20) * 0.001f;
      t20 = now;
      speedL = (encTotL - prevEncL) * MM_PER_TICK / dt;
      speedR = (encTotR - prevEncR) * MM_PER_TICK / dt;
      prevEncL = encTotL;
      prevEncR = encTotR;
    }
    
    // Отправка состояния робота каждые 100ms
    if (now - t100 >= 100) {
      t100 = now;
      sendRobotState();
    }
    
    static uint32_t tPID = 0;
    if (now - tPID >= 20) {
      tPID = now;
      updatePID();
    }
    
    if (ws) ws->cleanupClients();
    if (lidarWs) lidarWs->cleanupClients();
    vTaskDelay(1);
  }
}

/* ========== WebSocket обработчики ========= */
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    if (wsClient && wsClient->status() == WS_CONNECTED) wsClient->close();
    wsClient = client;
    wsClient->client()->setNoDelay(true);
    Serial.printf("[WS] Client #%u connected\n", client->id());
    
    // Отправляем приветственное сообщение
    DynamicJsonDocument doc(256);
    doc["type"] = "connected";
    doc["message"] = "Robot WebSocket connected";
    doc["version"] = "1.0";
    String message;
    serializeJson(doc, message);
    client->text(message);
    
  } else if (type == WS_EVT_DISCONNECT) {
    if (client == wsClient) {
      wsClient = nullptr;
      Serial.printf("[WS] Client #%u disconnected\n", client->id());
    }
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    
    if (info->opcode == WS_BINARY && len == 4) {
      // Бинарная команда управления скоростями (совместимость со старой версией)
      int16_t left = data[0] | (data[1] << 8);
      int16_t right = data[2] | (data[3] << 8);
      tgtL = (float)left;
      tgtR = (float)right;
      lastCmdMs = millis();
      
      if (fabs(fabs(tgtL) - fabs(tgtR)) < 1.0f && fabs(tgtL) > 1.0f) {
        alignMode = true;
        alignSign = (tgtL * tgtR >= 0) ? 1.0f : -1.0f;
        alignRefL = encTotL;
        alignRefR = encTotR;
      } else {
        alignMode = false;
      }
      Serial.printf("[WS] Binary cmd: left=%d, right=%d\n", left, right);
      
    } else if (info->opcode == WS_TEXT) {
      // JSON команды
      String message = String((char*)data).substring(0, len);
      DynamicJsonDocument doc(512);
      
      if (deserializeJson(doc, message) == DeserializationError::Ok) {
        String type = doc["type"];
        
        if (type == "setSpeed") {
          if (doc.containsKey("left")) tgtL = doc["left"];
          if (doc.containsKey("right")) tgtR = doc["right"];
          lastCmdMs = millis();
          
          if (fabs(fabs(tgtL) - fabs(tgtR)) < 1.0f && fabs(tgtL) > 1.0f) {
            alignMode = true;
            alignSign = (tgtL * tgtR >= 0) ? 1.0f : -1.0f;
            alignRefL = encTotL;
            alignRefR = encTotR;
          } else {
            alignMode = false;
          }
          
          // Отправляем подтверждение
          DynamicJsonDocument response(256);
          response["type"] = "ack";
          response["command"] = "setSpeed";
          response["left"] = tgtL;
          response["right"] = tgtR;
          String ack;
          serializeJson(response, ack);
          client->text(ack);
          
          Serial.printf("[WS] Speed cmd: left=%.1f, right=%.1f\n", tgtL, tgtR);
          
        } else if (type == "setCoeff") {
          if (doc.containsKey("kp")) kp = doc["kp"];
          if (doc.containsKey("ki")) ki = doc["ki"];
          if (doc.containsKey("kd")) kd = doc["kd"];
          if (doc.containsKey("kff")) kff = doc["kff"];
          
          // Отправляем подтверждение
          DynamicJsonDocument response(256);
          response["type"] = "ack";
          response["command"] = "setCoeff";
          response["kp"] = kp;
          response["ki"] = ki;
          response["kd"] = kd;
          response["kff"] = kff;
          String ack;
          serializeJson(response, ack);
          client->text(ack);
          
          Serial.printf("[WS] PID coeffs: kp=%.3f, ki=%.3f, kd=%.3f, kff=%.3f\n", kp, ki, kd, kff);
          
        } else if (type == "resetEnc") {
          encTotL = encTotR = 0;
          prevEncL = prevEncR = 0;
          speedL = speedR = 0;
          alignMode = false;
          pcnt_counter_clear(PCNT_UNIT_0);
          pcnt_counter_clear(PCNT_UNIT_1);
          
          // Отправляем подтверждение
          DynamicJsonDocument response(256);
          response["type"] = "ack";
          response["command"] = "resetEnc";
          String ack;
          serializeJson(response, ack);
          client->text(ack);
          
          Serial.println("[WS] Encoders reset");
          
        } else if (type == "resetOdom") {
          odomX = odomY = odomTh = 0;
          
          // Отправляем подтверждение
          DynamicJsonDocument response(256);
          response["type"] = "ack";
          response["command"] = "resetOdom";
          String ack;
          serializeJson(response, ack);
          client->text(ack);
          
          Serial.println("[WS] Odometry reset");
          
        } else if (type == "getState") {
          // Немедленно отправляем состояние
          sendRobotState();
          
        } else if (type == "stop") {
          tgtL = tgtR = 0;
          alignMode = false;
          stopMotors();
          lastCmdMs = millis();
          
          // Отправляем подтверждение
          DynamicJsonDocument response(256);
          response["type"] = "ack";
          response["command"] = "stop";
          String ack;
          serializeJson(response, ack);
          client->text(ack);
          
          Serial.println("[WS] Emergency stop");
          
        } else {
          // Неизвестная команда
          DynamicJsonDocument response(256);
          response["type"] = "error";
          response["message"] = "Unknown command: " + type;
          String error;
          serializeJson(response, error);
          client->text(error);
        }
      } else {
        // Ошибка парсинга JSON
        DynamicJsonDocument response(256);
        response["type"] = "error";
        response["message"] = "Invalid JSON format";
        String error;
        serializeJson(response, error);
        client->text(error);
      }
    }
  }
}

void onLidarWsEvent(AsyncWebSocket*, AsyncWebSocketClient* c, AwsEventType t, void*, uint8_t*, size_t){
  if(t==WS_EVT_CONNECT){
    if(lidarSole && lidarSole->status()==WS_CONNECTED){
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

/* ========== HTTP-роуты (минимальные для совместимости) ========= */
void setupRoutes() {
  // Главная страница с информацией о WebSocket API
  server->on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>Robot WebSocket API</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 40px; }
        .command { background: #f5f5f5; padding: 10px; margin: 10px 0; border-radius: 5px; }
        code { background: #e8e8e8; padding: 2px 5px; border-radius: 3px; }
    </style>
</head>
<body>
    <h1>Robot WebSocket API</h1>
    <p>Connect to WebSocket: <code>ws://)" + WiFi.localIP().toString() + R"(:)" + String(robotConfig.main_port) + R"(/ws</code></p>
    
    <h2>Commands (JSON format):</h2>
    
    <div class="command">
        <h3>Set Speed</h3>
        <code>{"type": "setSpeed", "left": 100.0, "right": 100.0}</code>
    </div>
    
    <div class="command">
        <h3>Set PID Coefficients</h3>
        <code>{"type": "setCoeff", "kp": 1.0, "ki": 0.8, "kd": 0.02, "kff": 0.25}</code>
    </div>
    
    <div class="command">
        <h3>Reset Encoders</h3>
        <code>{"type": "resetEnc"}</code>
    </div>
    
    <div class="command">
        <h3>Reset Odometry</h3>
        <code>{"type": "resetOdom"}</code>
    </div>
    
    <div class="command">
        <h3>Get Current State</h3>
        <code>{"type": "getState"}</code>
    </div>
    
    <div class="command">
        <h3>Emergency Stop</h3>
        <code>{"type": "stop"}</code>
    </div>
    
    <h2>Legacy Binary Command:</h2>
    <p>4 bytes: [left_low, left_high, right_low, right_high] (int16 values)</p>
    
    <h2>State Updates:</h2>
    <p>Robot automatically sends state updates every 100ms with type "state"</p>
</body>
</html>
    )";
    request->send(200, "text/html", html);
  });
  
  // Базовый API endpoint для проверки состояния
  server->on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(256);
    doc["status"] = "ok";
    doc["websocket_url"] = "ws://" + WiFi.localIP().toString() + ":" + String(robotConfig.main_port) + "/ws";
    doc["uptime"] = millis();
    doc["clients"] = ws ? ws->count() : 0;
    
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });
}

/* ========== Настройка WebSocket и HTTP ========= */
void setupWebServers() {
  ws = new AsyncWebSocket("/ws");
  ws->onEvent(onWsEvent);
  server = new AsyncWebServer(robotConfig.main_port);
  server->addHandler(ws);
  setupRoutes();
  server->begin();

  lidarWs = new AsyncWebSocket("/ws");
  lidarWs->onEvent(onLidarWsEvent);
  lidarServer = new AsyncWebServer(robotConfig.lidar_port);
  lidarServer->addHandler(lidarWs);
  lidarServer->on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "ESP32 Lidar WebSocket Server is running. Connect to /ws");
  });
  lidarServer->begin();

  Serial.printf("WebSocket servers started (main: %d, lidar: %d).\n", robotConfig.main_port, robotConfig.lidar_port);
  Serial.printf("Main WebSocket URL: ws://%s:%d/ws\n", WiFi.localIP().toString().c_str(), robotConfig.main_port);
}

/* ========== setup ========== */
void setup() {
  Serial.begin(115200);
  delay(300);

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed!");
  }
  robotConfig.load();

  pinMode(L_A, OUTPUT); pinMode(L_B, OUTPUT);
  pinMode(R_A, OUTPUT); pinMode(R_B, OUTPUT);
  stopMotors();

  pinMode(ENC_R_A, INPUT); pinMode(ENC_R_B, INPUT);
  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT);
  pcntInit(PCNT_UNIT_0, (gpio_num_t)ENC_R_A, (gpio_num_t)ENC_R_B);
  pcntInit(PCNT_UNIT_1, (gpio_num_t)ENC_L_A, (gpio_num_t)ENC_L_B);

  sendConfigToSerial1();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(robotConfig.ssid.c_str(), robotConfig.password.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print('.');
    checkSerialConfig();
  }
  Serial.printf("\nWiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());

  setupWebServers();

  // Настройка watchdog
  esp_task_wdt_config_t wdt_config = { 
    .timeout_ms = 5000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, 
    .trigger_panic = true 
  };
  esp_task_wdt_init(&wdt_config);

  // Создаем основные задачи
  xTaskCreatePinnedToCore(lidarTask, "LidarTask", 4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(mainTask,  "MainTask",  8192, nullptr, 3, nullptr, 0);
}

void loop() {
  // Вся логика в задачах
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}