/****************************************************************
 * PWM + Wi-Fi + AsyncWebServer + PCNT-энкодеры + ОДОМЕТРИЯ + PID-регулятор
 * Управление и мониторинг по Wi-Fi (JSON API)
 ****************************************************************/
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "driver/pcnt.h"
#include <math.h>

/* ── Аппаратные параметры ─────────────────────────────*/
#define L_A 32
#define L_B 33
#define R_A 26
#define R_B 25
#define ENC_R_A 39
#define ENC_R_B 36
#define ENC_L_A 35
#define ENC_L_B 34

constexpr float WHEEL_RADIUS = 0.0223f; // метры
constexpr float BASE         = 0.0956f; // метры
constexpr int   TICKS_PER_TURN = 2936;  // тиков энкодера на оборот

constexpr char SSID[] = "robotx";
constexpr char PASS[] = "78914040";

/* ── Глобальные значения ──────────────────────────────*/
volatile uint8_t dutyLA = 0, dutyLB = 0, dutyRA = 0, dutyRB = 0;
volatile int32_t encTotalL = 0, encTotalR = 0;

AsyncWebServer server(80);

/* ── ОДОМЕТРИЯ ───────────────────────────────────────*/
struct Odom {
  float x = 0, y = 0, theta = 0;
  float v = 0, w = 0;
} odom;

/* ── Скорости колес ──────────────────────────────────*/
struct WheelSpeed {
  float left = 0, right = 0; // мм/с
} wheelSpeed;

/* ── ПИД-регулятор ───────────────────────────────────*/
struct PID {
  float kp = 2.0, ki = 2.5, kd = 0.0, kff = 0.3;
  float sumL = 0, sumR = 0, prevErrL = 0, prevErrR = 0;
  float outL = 0, outR = 0;
  float targetL = 0, targetR = 0; // мм/с
} pid;

/* ── PWM ─────────────────────────────────────────────*/
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

/* ── PCNT ────────────────────────────────────────────*/
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

/* ── ПИД-регулятор ───────────────────────────────────*/
float constrainf(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }

void pidUpdate(float dt, float vL, float vR) {
  // --- Левое колесо
  float errL = pid.targetL - vL;
  pid.sumL += errL * dt;
  float dErrL = (errL - pid.prevErrL) / dt;
  float ffL = pid.kff * pid.targetL;
  pid.outL = pid.kp*errL + pid.ki*pid.sumL + pid.kd*dErrL + ffL;
  pid.prevErrL = errL;
  // --- Правое колесо
  float errR = pid.targetR - vR;
  pid.sumR += errR * dt;
  float dErrR = (errR - pid.prevErrR) / dt;
  float ffR = pid.kff * pid.targetR;
  pid.outR = pid.kp*errR + pid.ki*pid.sumR + pid.kd*dErrR + ffR;
  pid.prevErrR = errR;
  // --- Ограничение PWM [0..255]
  pid.outL = constrainf(pid.outL, -255, 255);
  pid.outR = constrainf(pid.outR, -255, 255);

  // --- Драйверы H-моста: прямое/обратное направление
  if (pid.outL > 0) {
    analogWriteTrack(L_A, (uint8_t)pid.outL); analogWriteTrack(L_B, 0);
  } else {
    analogWriteTrack(L_A, 0); analogWriteTrack(L_B, (uint8_t)(-pid.outL));
  }
  if (pid.outR > 0) {
    analogWriteTrack(R_A, (uint8_t)pid.outR); analogWriteTrack(R_B, 0);
  } else {
    analogWriteTrack(R_A, 0); analogWriteTrack(R_B, (uint8_t)(-pid.outR));
  }
}

/* ── ОДОМЕТРИЯ+СКОРОСТЬ ─────────────────────────────*/
void updateOdometryAndSpeed(int32_t dTicksL, int32_t dTicksR, float dt) {
  float dL = (float)dTicksL * 2.0f * M_PI * WHEEL_RADIUS / TICKS_PER_TURN;
  float dR = (float)dTicksR * 2.0f * M_PI * WHEEL_RADIUS / TICKS_PER_TURN;

  // --- скорость колёс [мм/с]
  wheelSpeed.left  = (dt > 0.0001f) ? (dL * 1000.0f / dt) : 0.0f;
  wheelSpeed.right = (dt > 0.0001f) ? (dR * 1000.0f / dt) : 0.0f;

  // --- одометрия
  float dCenter = (dL + dR) * 0.5f;
  float dTheta = (dR - dL) / BASE;
  odom.x += dCenter * cosf(odom.theta + dTheta/2.0f);
  odom.y += dCenter * sinf(odom.theta + dTheta/2.0f);
  odom.theta += dTheta;
  odom.v = dCenter / dt;
  odom.w = dTheta / dt;
}

/* ── Wi-Fi + Web ─────────────────────────────────────*/
void setupWiFiAndRoutes() {
  WiFi.mode(WIFI_STA); WiFi.begin(SSID,PASS);
  Serial.print("Connecting");
  while(WiFi.status()!=WL_CONNECTED){ Serial.print('.'); delay(400);}
  Serial.printf("\nIP: %s\n",WiFi.localIP().toString().c_str());

  server.on("/state",HTTP_GET,[](AsyncWebServerRequest *req){
    char js[512];
    snprintf(js,sizeof(js),
      "{\"duty\":{\"L_A\":%u,\"L_B\":%u,\"R_A\":%u,\"R_B\":%u},"
      "\"enc\":{\"left\":%ld,\"right\":%ld},"
      "\"odom\":{\"x\":%.4f,\"y\":%.4f,\"theta\":%.4f,\"v\":%.4f,\"w\":%.4f},"
      "\"speed\":{\"left\":%.1f,\"right\":%.1f},"
      "\"pid\":{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"kff\":%.3f,"
      "\"targetL\":%.1f,\"targetR\":%.1f},"
      "\"pwm\":{\"L\":%.1f,\"R\":%.1f}}",
      dutyLA,dutyLB,dutyRA,dutyRB, encTotalL,encTotalR,
      odom.x, odom.y, odom.theta, odom.v, odom.w,
      wheelSpeed.left, wheelSpeed.right,
      pid.kp, pid.ki, pid.kd, pid.kff, pid.targetL, pid.targetR,
      pid.outL, pid.outR
    );
    req->send(200,"application/json",js);
  });

  // Установка коэффициентов PID
  server.on("/setPID", HTTP_GET, [](AsyncWebServerRequest *req){
    if(req->hasParam("kp")) pid.kp = req->getParam("kp")->value().toFloat();
    if(req->hasParam("ki")) pid.ki = req->getParam("ki")->value().toFloat();
    if(req->hasParam("kd")) pid.kd = req->getParam("kd")->value().toFloat();
    if(req->hasParam("kff")) pid.kff = req->getParam("kff")->value().toFloat();
    req->send(200,"text/plain","PID updated");
  });

  // Установка целевых скоростей колес
  server.on("/setWheelsSpeed", HTTP_GET, [](AsyncWebServerRequest *req){
    if(req->hasParam("left"))  pid.targetL = req->getParam("left")->value().toFloat();
    if(req->hasParam("right")) pid.targetR = req->getParam("right")->value().toFloat();
    req->send(200,"text/plain","Targets updated");
  });

  // Сброс всех значений (энкодеры, одометрия, PID-интеграторы)
  server.on("/resetAll", HTTP_GET, [](AsyncWebServerRequest *req){
    encTotalL=encTotalR=0;
    odom = Odom(); wheelSpeed = WheelSpeed();
    pid.sumL = 0; pid.sumR = 0; pid.prevErrL = 0; pid.prevErrR = 0;
    pid.outL = 0; pid.outR = 0;
    pcnt_counter_clear(PCNT_UNIT_0); pcnt_counter_clear(PCNT_UNIT_1);
    req->send(200,"text/plain","All reset");
  });

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain",
        "OK. Use /state, /setPID, /setWheelsSpeed, /resetAll");
  });
  server.begin();
}

/* ── SETUP ───────────────────────────────────────────*/
void setup() {
  Serial.begin(115200);
  pinMode(L_A,OUTPUT); pinMode(L_B,OUTPUT);
  pinMode(R_A,OUTPUT); pinMode(R_B,OUTPUT); stopMotors();

  pinMode(ENC_R_A,INPUT); pinMode(ENC_R_B,INPUT);
  pinMode(ENC_L_A,INPUT); pinMode(ENC_L_B,INPUT);
  setupEncoder(PCNT_UNIT_0,(gpio_num_t)ENC_R_A,(gpio_num_t)ENC_R_B);
  setupEncoder(PCNT_UNIT_1,(gpio_num_t)ENC_L_A,(gpio_num_t)ENC_L_B);

  setupWiFiAndRoutes();
}

/* ── LOOP ────────────────────────────────────────────*/
void loop() {
  static uint32_t tEnc=0;
  static uint32_t lastUpdate = 0;
  uint32_t now = millis();
  if(now-tEnc>=10){
    tEnc=now;
    int16_t dR = snapPCNT(PCNT_UNIT_0);
    int16_t dL = snapPCNT(PCNT_UNIT_1);
    encTotalR += dR;
    encTotalL += dL;
    float dt = (now-lastUpdate)/1000.0f;
    if(dt > 0.0001f) {
      updateOdometryAndSpeed(dL, dR, dt);
      pidUpdate(dt, wheelSpeed.left, wheelSpeed.right);
      lastUpdate = now;
    }
  }

  // Печать состояния раз в 500 мс
  static uint32_t tLog=0;
  if(now-tLog>=500){
    tLog=now;
    Serial.printf("SPD L=%.1f mm/s R=%.1f mm/s Target L=%.1f mm/s R=%.1f mm/s\n", 
      wheelSpeed.left, wheelSpeed.right, pid.targetL, pid.targetR);
  }
}