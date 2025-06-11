/****************************************************************
 * PWM + Wi-Fi + AsyncWebServer + AsyncWebSocket + PCNT-энкодеры + ОДОМЕТРИЯ + PID-регулятор
 * Управление и мониторинг по Wi-Fi (JSON API over WebSocket)
 * PID коэффициенты сохраняются в SPIFFS и загружаются при старте.
 * + Battery Voltage Monitoring
 ****************************************************************/
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <AsyncWebSocket.h> // <<< ADDED: For WebSockets
#include "driver/pcnt.h"
#include <math.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "driver/adc.h"

/* ── Аппаратные параметры ─────────────────────────────*/
#define L_A 32
#define L_B 33
#define R_A 26
#define R_B 25
#define ENC_R_A 18
#define ENC_R_B 19
#define ENC_L_A 35
#define ENC_L_B 34

#define BATT_PIN 39

constexpr float WHEEL_RADIUS = 0.0223f; // метры
constexpr float BASE         = 0.097f; // метры (ширина между колесами)
constexpr int   TICKS_PER_TURN = 2936;  // тиков энкодера на оборот

constexpr char SSID[] = "robotx"; // Your WiFi SSID
constexpr char PASS[] = "78914040"; // Your WiFi Password

/* ── Глобальные значения ──────────────────────────────*/
volatile uint8_t dutyLA = 0, dutyLB = 0, dutyRA = 0, dutyRB = 0;
volatile int32_t encTotalL = 0, encTotalR = 0;
volatile float currentBatteryVoltage = 0.0f;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); // <<< ADDED: WebSocket endpoint

/* ── ОДОМЕТРИЯ ───────────────────────────────────────*/
struct Odom {
  float x = 0, y = 0, theta = 0;
  float v = 0, w = 0;
} odom;

/* ── Скорости колес ──────────────────────────────────*/
struct WheelSpeed {
  float left = 0, right = 0; // мм/с
} wheelSpeed;

/* ── Инвертированный вид: V, W как целевые для регулятора ── */
struct VWTarget {
  float v = 0; // мм/с
  float w = 0; // рад/с
} vwTarget;

/* ── ПИД-регулятор ───────────────────────────────────*/
struct PID {
  float kp = 2.0, ki = 2.5, kd = 0.0, kff = 0.3;
  float sumL = 0, sumR = 0, prevErrL = 0, prevErrR = 0;
  float outL = 0, outR = 0;
  float targetL = 0, targetR = 0; // мм/с
} pid;

const char* PID_CONFIG_FILE = "/pid.json";

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
  doc["kp"] = pid.kp;
  doc["ki"] = pid.ki;
  doc["kd"] = pid.kd;
  doc["kff"] = pid.kff;
  File file = SPIFFS.open(PID_CONFIG_FILE, FILE_WRITE);
  if (file) {
    serializeJson(doc, file);
    file.close();
  }
}

void loadPIDFromSPIFFS() {
  if (!SPIFFS.exists(PID_CONFIG_FILE)) return;
  File file = SPIFFS.open(PID_CONFIG_FILE, FILE_READ);
  if (file) {
    DynamicJsonDocument doc(256);
    if (deserializeJson(doc, file) == DeserializationError::Ok) {
      pid.kp = doc["kp"] | pid.kp; // Use current value as default if key missing
      pid.ki = doc["ki"] | pid.ki;
      pid.kd = doc["kd"] | pid.kd;
      pid.kff = doc["kff"] | pid.kff;
    }
    file.close();
  }
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

  pid.outL = constrainf(pid.outL, -255, 255);
  pid.outR = constrainf(pid.outR, -255, 255);

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
  odom.x += dCenter * cosf(odom.theta + dTheta*0.5f); // Corrected: dTheta/2.0f
  odom.y += dCenter * sinf(odom.theta + dTheta*0.5f); // Corrected: dTheta/2.0f
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
}
void update_vw_from_targets() {
  vwTarget.v = (pid.targetL + pid.targetR) * 0.5f;
  vwTarget.w = (pid.targetR - pid.targetL) / (BASE * 1000.0f);
}

void sendStateOverWebSocket() {
  update_vw_from_targets(); // Ensure vwTarget is up-to-date
  currentBatteryVoltage = readBatteryVoltage();

  // Increased buffer size for safety, consider using ArduinoJson for construction
  // if the string becomes much larger or more complex.
  char js_buffer[800]; 
  
  snprintf(js_buffer, sizeof(js_buffer),
    "{\"type\":\"state\","
    "\"duty\":{\"L_A\":%u,\"L_B\":%u,\"R_A\":%u,\"R_B\":%u},"
    "\"enc\":{\"left\":%ld,\"right\":%ld},"
    "\"odom\":{\"x\":%.4f,\"y\":%.4f,\"theta\":%.4f,\"v\":%.4f,\"w\":%.4f},"
    "\"speed\":{\"left\":%.1f,\"right\":%.1f},"
    "\"pid_coeffs\":{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"kff\":%.3f},"
    "\"vw_target\":{\"v\":%.1f,\"w\":%.3f}," // Renamed "vw" to "vw_target" for clarity
    "\"pwm_out\":{\"L\":%.1f,\"R\":%.1f},"  // Renamed "pwm" to "pwm_out"
    "\"battery_v\":%.2f}",
    dutyLA, dutyLB, dutyRA, dutyRB, encTotalL, encTotalR,
    odom.x, odom.y, odom.theta, odom.v, odom.w,
    wheelSpeed.left, wheelSpeed.right,
    pid.kp, pid.ki, pid.kd, pid.kff, 
    vwTarget.v, vwTarget.w, // Use updated vwTarget
    pid.outL, pid.outR,
    currentBatteryVoltage
  );
  ws.textAll(js_buffer);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0; // Null-terminate
    char* msg_payload = (char*)data;
    Serial.printf("WS Rcv: %s\n", msg_payload);

    DynamicJsonDocument doc(512); // Adjust size as needed for commands
    DeserializationError error = deserializeJson(doc, msg_payload);

    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      // Optionally send an error back to the client
      // client->text("{\"type\":\"error\", \"message\":\"Invalid JSON\"}");
      return;
    }

    const char* action = doc["action"];
    if (!action) {
        // client->text("{\"type\":\"error\", \"message\":\"Action missing\"}");
        return;
    }
    
    AsyncWebSocketClient *client = (AsyncWebSocketClient*)arg; // This might not be correct, need the client who sent it.
                                                               // The onEvent handler provides the client.
                                                               // For now, we'll assume this function is called from a context where client is available
                                                               // or we send generic status. Best to handle response in onEvent.

    if (strcmp(action, "setVW") == 0) {
      float v = doc["v"] | 0.0f;
      float w = doc["w"] | 0.0f;
      set_targets_from_vw(v, w);
      if (v == 0.0f && w == 0.0f) {
          stopMotors(); 
          pid.targetL = 0; pid.targetR = 0;
          pid.sumL = 0; pid.sumR = 0;
          pid.outL = 0; pid.outR = 0;
      }
      // client->text("{\"type\":\"status\", \"action_acked\":\"setVW\", \"message\":\"VW targets updated\"}");
      Serial.println("WS: setVW processed");
    } else if (strcmp(action, "setPID") == 0) {
      bool changed = false;
      if (doc.containsKey("kp")) { pid.kp = doc["kp"]; changed = true; }
      if (doc.containsKey("ki")) { pid.ki = doc["ki"]; changed = true; }
      if (doc.containsKey("kd")) { pid.kd = doc["kd"]; changed = true; }
      if (doc.containsKey("kff")) { pid.kff = doc["kff"]; changed = true; }
      if (changed) savePIDToSPIFFS();
      // client->text("{\"type\":\"status\", \"action_acked\":\"setPID\", \"message\":\"PID updated\"}");
      Serial.println("WS: setPID processed");
    } else if (strcmp(action, "setWheelsSpeed") == 0) {
      if (doc.containsKey("left"))  pid.targetL = doc["left"];
      if (doc.containsKey("right")) pid.targetR = doc["right"];
      update_vw_from_targets();
      // client->text("{\"type\":\"status\", \"action_acked\":\"setWheelsSpeed\", \"message\":\"Wheel speed targets updated\"}");
       Serial.println("WS: setWheelsSpeed processed");
    } else if (strcmp(action, "resetAll") == 0) {
      encTotalL=encTotalR=0;
      odom = Odom(); wheelSpeed = WheelSpeed();
      pid.sumL = 0; pid.sumR = 0; pid.prevErrL = 0; pid.prevErrR = 0;
      pid.outL = 0; pid.outR = 0;
      pid.targetL = 0; pid.targetR = 0;
      vwTarget.v = 0; vwTarget.w = 0;
      stopMotors();
      pcnt_counter_clear(PCNT_UNIT_0); pcnt_counter_clear(PCNT_UNIT_1);
      // client->text("{\"type\":\"status\", \"action_acked\":\"resetAll\", \"message\":\"All reset\"}");
      Serial.println("WS: resetAll processed");
    } else if (strcmp(action, "getPID") == 0) {
        char js[128];
        snprintf(js, sizeof(js), "{\"type\":\"pid_coeffs\",\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"kff\":%.3f}", pid.kp, pid.ki, pid.kd, pid.kff);
        // client->text(js); // This needs the specific client who requested
        Serial.println("WS: getPID processed, response needs client context");
    } else if (strcmp(action, "getBattery") == 0) {
        currentBatteryVoltage = readBatteryVoltage();
        char js[64];
        snprintf(js, sizeof(js), "{\"type\":\"battery\",\"voltage\":%.2f}", currentBatteryVoltage);
        // client->text(js); // This needs the specific client who requested
        Serial.println("WS: getBattery processed, response needs client context");
    } else {
        // client->text("{\"type\":\"error\", \"message\":\"Unknown action\"}");
        Serial.printf("WS: Unknown action: %s\n", action);
    }
  }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      // Optionally send initial state or welcome message
      // client->text("{\"type\":\"welcome\", \"message\":\"Connected to ESP32 Robot\"}");
      // sendStateOverWebSocket(); // Send current state to newly connected client (or rely on periodic broadcast)
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      // Pass to the handler function. The 'client' object is available here.
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

            // Handle actions and send response to this specific client
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
                currentBatteryVoltage = readBatteryVoltage(); // Ensure it's fresh
                char js[64];
                snprintf(js, sizeof(js), "{\"type\":\"battery\",\"voltage\":%.2f}", currentBatteryVoltage);
                client->text(js);
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
  WiFi.mode(WIFI_STA); WiFi.begin(SSID,PASS);
  Serial.print("Connecting to WiFi");
  while(WiFi.status()!=WL_CONNECTED){ Serial.print('.'); delay(400);}
  Serial.printf("\nIP: %s\n",WiFi.localIP().toString().c_str());

  ws.onEvent(onWsEvent); // Attach WebSocket event handler
  server.addHandler(&ws); // Add WebSocket handler to the server

  // Basic HTTP endpoint for root, just to confirm server is up
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "ESP32 Robot WebSocket Server is running. Connect to /ws");
  });

  server.begin();
  Serial.println("HTTP server and WebSocket server started.");
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for serial

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);

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

  setupWiFiAndServer();
  currentBatteryVoltage = readBatteryVoltage();
  Serial.printf("Initial Battery Voltage: %.2f V\n", currentBatteryVoltage);
}

void loop() {
  static uint32_t tEnc=0;
  static uint32_t lastUpdate = 0; // For dt calculation
  uint32_t now_ms = millis();
  
  if(now_ms - tEnc >= 10){ // Encoder, Odometry, and PID update loop (100Hz)
    tEnc = now_ms;
    int16_t dR = snapPCNT(PCNT_UNIT_0);
    int16_t dL = snapPCNT(PCNT_UNIT_1); // Assuming ENC_L is UNIT_1
    encTotalR += dR;
    encTotalL += dL;
    
    float dt = (lastUpdate > 0) ? (now_ms - lastUpdate) / 1000.0f : 0.01f; // Avoid large dt on first run
    if(dt > 0.0001f) {
      updateOdometryAndSpeed(dL, dR, dt);
      if (vwTarget.v != 0.0f || vwTarget.w != 0.0f || pid.outL !=0 || pid.outR !=0 ) {
         pidUpdate(dt, wheelSpeed.left, wheelSpeed.right);
      } else {
         stopMotors(); // Ensure motors are stopped if targets and outputs are zero
      }
    }
    lastUpdate = now_ms;
  }

  static uint32_t tStateWs = 0;
  if(now_ms - tStateWs >= 500) { // Send state over WebSocket at ~2Hz
    tStateWs = now_ms;
    if (ws.count() > 0) { // Only send if there are connected clients
        sendStateOverWebSocket();
    }
    // Serial logging can be reduced or made conditional
    Serial.printf("Clients: %d, V:%.1f W:%.3f PWM L:%.1f R:%.1f Batt:%.2fV\n", ws.count(), vwTarget.v, vwTarget.w, pid.outL, pid.outR, currentBatteryVoltage);
  }
  
  // ws.cleanupClients(); // Periodically remove disconnected clients - handled by library
}