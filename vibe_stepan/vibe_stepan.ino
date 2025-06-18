#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <AsyncWebSocket.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "driver/pcnt.h"
#include "esp_task_wdt.h"
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>

/* ======== Конфигурация ======== */
struct RobotConfig {
  String ssid = "robotx";
  String password = "78914040";
  uint16_t main_port = 80;
  uint16_t lidar_port = 81;
  String client_ip = "192.168.1.42";
  uint16_t client_port = 9001;
  String telegram_token = "";
  String telegram_user_id = "";

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
      telegram_token = doc["telegram_token"] | telegram_token;
      telegram_user_id = doc["telegram_user_id"] | telegram_user_id;
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
    doc["telegram_token"] = telegram_token;
    doc["telegram_user_id"] = telegram_user_id;
    serializeJson(doc, f);
    f.close();
  }
  String toString() const {
    char buf[512];
    snprintf(buf, sizeof(buf), "%s|%s|%d|%d|%s|%d|%s|%s", 
             ssid.c_str(), password.c_str(), main_port, lidar_port, 
             client_ip.c_str(), client_port, telegram_token.c_str(), telegram_user_id.c_str());
    return String(buf);
  }
  
  // Новая функция для подготовки сообщения для второй платы
  String toSecondBoardString() const {
    char buf[256];
    snprintf(buf, sizeof(buf), "%s|%s|%s|%d", 
             ssid.c_str(), password.c_str(), client_ip.c_str(), client_port);
    return String(buf);
  }
};

RobotConfig robotConfig;

/* ======== Telegram ======== */
WiFiClientSecure telegramClient;
UniversalTelegramBot *bot = nullptr;
volatile bool telegramEnabled = false;
String serial1Buffer = "";
volatile uint32_t lastTelegramCheck = 0;

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

/* ========== Telegram функции ========== */
void initTelegram() {
  if (robotConfig.telegram_token.length() > 0 && robotConfig.telegram_user_id.length() > 0) {
    telegramClient.setInsecure();
    bot = new UniversalTelegramBot(robotConfig.telegram_token, telegramClient);
    telegramEnabled = true;
    Serial.println("[TELEGRAM] Bot initialized");
    
    // Отправляем IP при запуске
    String startMessage = "🤖 Robot ESP32 started!\n";
    startMessage += "📍 Local IP: " + WiFi.localIP().toString() + "\n";
    startMessage += "🔗 Main port: " + String(robotConfig.main_port) + "\n";
    startMessage += "📡 Lidar port: " + String(robotConfig.lidar_port);
    
    sendTelegramMessage(startMessage);
  } else {
    Serial.println("[TELEGRAM] Token or User ID not configured");
  }
}

void sendTelegramMessage(const String& message) {
  if (!telegramEnabled || !bot) return;
  
  static uint32_t lastSent = 0;
  uint32_t now = millis();
  
  // Ограничиваем частоту отправки (не чаще 1 раза в секунду)
  if (now - lastSent < 1000) return;
  lastSent = now;
  
  if (bot->sendMessage(robotConfig.telegram_user_id, message, "")) {
    Serial.println("[TELEGRAM] Message sent: " + message.substring(0, 50) + "...");
  } else {
    Serial.println("[TELEGRAM] Failed to send message");
  }
}

void handleTelegramMessages() {
  if (!telegramEnabled || !bot) return;
  
  uint32_t now = millis();
  if (now - lastTelegramCheck < 2000) return; // Проверяем каждые 2 секунды
  lastTelegramCheck = now;
  
  int numNewMessages = bot->getUpdates(bot->last_message_received + 1);
  
  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot->messages[i].chat_id);
    if (chat_id != robotConfig.telegram_user_id) {
      bot->sendMessage(chat_id, "❌ Unauthorized access", "");
      continue;
    }
    
    String text = bot->messages[i].text;
    Serial.println("[TELEGRAM] Received: " + text);
    
    if (text == "/start" || text == "/help") {
      String helpMsg = "🤖 Robot ESP32 Commands:\n";
      helpMsg += "/status - Get robot status\n";
      helpMsg += "/ip - Get current IP\n";
      helpMsg += "/stop - Emergency stop\n";
      helpMsg += "/reset - Reset encoders and odometry\n";
      helpMsg += "/config - Send config to second board";
      bot->sendMessage(chat_id, helpMsg, "");
    }
    else if (text == "/status") {
      String statusMsg = "📊 Robot Status:\n";
      statusMsg += "⚡ Motors: L=" + String(tgtL, 1) + " R=" + String(tgtR, 1) + "\n";
      statusMsg += "🎯 Speed: L=" + String(speedL, 1) + " R=" + String(speedR, 1) + "\n";
      statusMsg += "📍 Position: X=" + String(odomX, 2) + " Y=" + String(odomY, 2) + "\n";
      statusMsg += "🧭 Angle: " + String(odomTh * 180.0 / M_PI, 1) + "°";
      bot->sendMessage(chat_id, statusMsg, "");
    }
    else if (text == "/ip") {
      String ipMsg = "📍 Current IP: " + WiFi.localIP().toString();
      bot->sendMessage(chat_id, ipMsg, "");
    }
    else if (text == "/stop") {
      tgtL = tgtR = 0;
      alignMode = false;
      stopMotors();
      bot->sendMessage(chat_id, "🛑 Emergency stop activated!", "");
    }
    else if (text == "/reset") {
      encTotL = encTotR = 0;
      prevEncL = prevEncR = 0;
      speedL = speedR = 0;
      odomX = odomY = odomTh = 0;
      alignMode = false;
      pcnt_counter_clear(PCNT_UNIT_0);
      pcnt_counter_clear(PCNT_UNIT_1);
      bot->sendMessage(chat_id, "🔄 Encoders and odometry reset!", "");
    }
    else if (text == "/config") {
      sendConfigToSerial1();
      bot->sendMessage(chat_id, "📤 Config sent to second board", "");
    }
    else {
      bot->sendMessage(chat_id, "❓ Unknown command. Use /help for available commands.", "");
    }
  }
}

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
  
  // Отправляем конфиг в новом формате: ssid|password|client_ip|client_port
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
        // Отправляем в Telegram если включен
        if (telegramEnabled && line.length() > 0) {
          String telegramMsg = "📡 Serial1: " + line;
          sendTelegramMessage(telegramMsg);
        }
      }
      line = "";
    } else if (c >= 32 && c <= 126) { // Только печатные символы
      line += c;
      if (line.length() > 200) { // Ограничиваем длину строки
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
        // Формат: ssid|password|main_port|lidar_port|client_ip|client_port|telegram_token|telegram_user_id
        int idx = 0, prev = 0;
        String parts[8];
        for (int i = 0; i < 8; ++i) {
          idx = line.indexOf('|', prev);
          if (idx < 0 && i < 7) { break; }
          parts[i] = line.substring(prev, (i < 7) ? idx : line.length());
          prev = idx + 1;
        }
        if (parts[0].length()) robotConfig.ssid = parts[0];
        if (parts[1].length()) robotConfig.password = parts[1];
        if (parts[2].length()) robotConfig.main_port = parts[2].toInt();
        if (parts[3].length()) robotConfig.lidar_port = parts[3].toInt();
        if (parts[4].length()) robotConfig.client_ip = parts[4];
        if (parts[5].length()) robotConfig.client_port = parts[5].toInt();
        if (parts[6].length()) robotConfig.telegram_token = parts[6];
        if (parts[7].length()) robotConfig.telegram_user_id = parts[7];
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

/* ========== Основная Task ========= */
void mainTask(void*) {
  esp_task_wdt_add(NULL);
  static uint32_t t10 = 0, t20 = 0, t2000 = 0;
  uint32_t now;
  
  // Инициализируем Serial1 для чтения
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  
  while (true) {
    now = millis();
    checkSerialConfig();
    checkSerial1Data(); // Проверяем данные от Serial1
    handleTelegramMessages(); // Обрабатываем Telegram сообщения

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
    static uint32_t tPID = 0;
    if (now - tPID >= 20) {
      tPID = now;
      updatePID();
    }
    if (ws) ws->cleanupClients();
    if (lidarWs) lidarWs->cleanupClients();
    vTaskDelay(1);
    esp_task_wdt_reset();
  }
}

/* ========== WebSocket обработчики ========= */
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    if (wsClient && wsClient->status() == WS_CONNECTED) wsClient->close();
    wsClient = client;
    wsClient->client()->setNoDelay(true);
    Serial.printf("[WS] Client #%u connected\n", client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    if (client == wsClient) {
      wsClient = nullptr;
      Serial.printf("[WS] Client #%u disconnected\n", client->id());
    }
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->opcode == WS_BINARY && len == 4) {
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
      Serial.printf("[WS] Cmd: left=%d, right=%d\n", left, right);
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

/* ========== HTTP-роуты ========= */
void setupRoutes() {
  server->on("/state", HTTP_GET, [](AsyncWebServerRequest *request) {
    char json[512];
    snprintf(json, sizeof(json),
             "{\"duty\":{\"L_A\":%u,\"L_B\":%u,\"R_A\":%u,\"R_B\":%u},"
             "\"enc\":{\"left\":%ld,\"right\":%ld},"
             "\"speed\":{\"left\":%.1f,\"right\":%.1f},"
             "\"target\":{\"left\":%.1f,\"right\":%.1f},"
             "\"odom\":{\"x\":%.3f,\"y\":%.3f,\"th\":%.3f}}",
             dutyLA, dutyLB, dutyRA, dutyRB,
             encTotL, encTotR,
             speedL, speedR, tgtL, tgtR,
             odomX, odomY, odomTh);
    request->send(200, "application/json", json);
  });
  server->on("/setSpeed", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("l")) tgtL = request->getParam("l")->value().toFloat();
    if (request->hasParam("r")) tgtR = request->getParam("r")->value().toFloat();
    lastCmdMs = millis();
    if (fabs(fabs(tgtL) - fabs(tgtR)) < 1.0f && fabs(tgtL) > 1.0f) {
      alignMode = true;
      alignSign = (tgtL * tgtR >= 0) ? 1.0f : -1.0f;
      alignRefL = encTotL;
      alignRefR = encTotR;
    } else {
      alignMode = false;
    }
    request->send(200, "text/plain", "ok");
  });
  server->on("/setCoeff", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("kp")) kp = request->getParam("kp")->value().toFloat();
    if (request->hasParam("ki")) ki = request->getParam("ki")->value().toFloat();
    if (request->hasParam("kd")) kd = request->getParam("kd")->value().toFloat();
    if (request->hasParam("kff")) kff = request->getParam("kff")->value().toFloat();
    request->send(200, "text/plain", "ok");
  });
  server->on("/resetEnc", HTTP_GET, [](AsyncWebServerRequest *request) {
    encTotL = encTotR = 0;
    prevEncL = prevEncR = 0;
    speedL = speedR = 0;
    alignMode = false;
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);
    request->send(200, "text/plain", "enc reset");
  });
  server->on("/resetOdom", HTTP_GET, [](AsyncWebServerRequest *request) {
    odomX = odomY = odomTh = 0;
    request->send(200, "text/plain", "odom reset");
  });
  server->on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain",
                  "Endpoints: /state /setSpeed /setCoeff /resetEnc /resetOdom");
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

  Serial.printf("HTTP/WebSocket server started (main: %d, lidar: %d).\n", robotConfig.main_port, robotConfig.lidar_port);
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
  initTelegram(); // Инициализируем Telegram бота

  esp_task_wdt_config_t wdt_config = { .timeout_ms = 5000, .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, .trigger_panic = true };
  esp_task_wdt_init(&wdt_config);

  xTaskCreatePinnedToCore(lidarTask, "LidarTask", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(mainTask,  "MainTask",  8192, nullptr, 1, nullptr, 0);
}

void loop() {
  // Вся логика в задачах
}