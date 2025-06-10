/****************************************************************
 *  PWM + Wi-Fi + AsyncWebServer + PCNT-энкодеры  v2 + ОДОМЕТРИЯ + СКОРОСТЬ КОЛЁС
 *  ────────────────────────────────────────────────────────────
 *  /state       → JSON со скважностями, счётчиками, одометрией, скоростью колёс
 *  /setPWM      ?la=..&lb=..&ra=..&rb=..   (0-255)  — задаёт ШИМ
 *  /resetEnc    → обнуляет оба счётчика и одометрию
 ****************************************************************/
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "driver/pcnt.h"
#include <math.h>

/* ── пины моторов ───────────────────────────────────────────*/
#define L_A 32
#define L_B 33
#define R_A 26
#define R_B 25

/* ── пины энкодеров ─────────────────────────────────────────*/
#define ENC_R_A 39
#define ENC_R_B 36
#define ENC_L_A 35
#define ENC_L_B 34

/* ── ПАРАМЕТРЫ РОБОТА ──────────────────────────────────────*/
constexpr float WHEEL_RADIUS = 0.0446f;  // Радиус колеса, метры (например, 32 мм)
constexpr float BASE        = 0.0956f;   // База робота (расстояние между колёсами), метры
constexpr int   TICKS_PER_TURN = 2936;   // Импульсов энкодера на оборот колеса

/* ── Wi-Fi ────────────────────────────────────────────*/
constexpr char SSID[] = "robotx";
constexpr char PASS[] = "78914040";

/* ── глобальные PWM-значения ──────────────────────────*/
volatile uint8_t dutyLA, dutyLB, dutyRA, dutyRB;

/* ── “длинные” счётчики энкодеров ────────────────────*/
volatile int32_t encTotalL = 0;
volatile int32_t encTotalR = 0;

/* ── AsyncWebServer ──────────────────────────────────*/
AsyncWebServer server(80);

/* ── ОДОМЕТРИЯ ───────────────────────────────────────*/
struct Odom {
  float x = 0;    // [м]
  float y = 0;    // [м]
  float theta = 0;// [рад]
  float v = 0;    // [м/с]
  float w = 0;    // [рад/с]
} odom;

/* ── Скорость колёс ──────────────────────────────────*/
struct WheelSpeed {
  float left = 0;   // мм/с
  float right = 0;  // мм/с
} wheelSpeed;

/*──────────────── PWM-утилиты ─────────────────────────*/
void analogWriteTrack(uint8_t pin, uint8_t duty)
{
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

/*──────────────── PCNT-helpers ───────────────────────*/
void setupEncoder(pcnt_unit_t unit, gpio_num_t a, gpio_num_t b)
{
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

/*─── ОДОМЕТРИЯ: вычисление ───────────────────────────*/
void updateOdometryAndSpeed(int32_t dTicksL, int32_t dTicksR, float dt)
{
  // 1. Переводим ticks -> метры (s = 2*PI*R*N / TICKS_PER_TURN)
  float dL = (float)dTicksL * 2.0f * M_PI * WHEEL_RADIUS / TICKS_PER_TURN;
  float dR = (float)dTicksR * 2.0f * M_PI * WHEEL_RADIUS / TICKS_PER_TURN;

  // --- скорость колёс [мм/с]
  wheelSpeed.left  = (dt > 0.0001f) ? (dL * 1000.0f / dt) : 0.0f;
  wheelSpeed.right = (dt > 0.0001f) ? (dR * 1000.0f / dt) : 0.0f;

  // 2. Приращения
  float dCenter = (dL + dR) * 0.5f;
  float dTheta = (dR - dL) / BASE;

  // 3. Обновление положения
  odom.x += dCenter * cosf(odom.theta + dTheta/2.0f);
  odom.y += dCenter * sinf(odom.theta + dTheta/2.0f);
  odom.theta += dTheta;

  // 4. Скорости (линейная и угловая)
  odom.v = dCenter / dt;
  odom.w = dTheta / dt;
}

/*─── Wi-Fi + Web ─────────────────────────────────────*/
void setupWiFiAndRoutes()
{
  WiFi.mode(WIFI_STA); WiFi.begin(SSID,PASS);
  Serial.print("Connecting");
  while(WiFi.status()!=WL_CONNECTED){ Serial.print('.'); delay(400);}
  Serial.printf("\nIP: %s\n",WiFi.localIP().toString().c_str());

  /* /state — текущие данные */
  server.on("/state",HTTP_GET,[](AsyncWebServerRequest *req){
    char js[384];
    snprintf(js,sizeof(js),
      "{\"duty\":{\"L_A\":%u,\"L_B\":%u,\"R_A\":%u,\"R_B\":%u},"
      "\"enc\":{\"left\":%ld,\"right\":%ld},"
      "\"odom\":{\"x\":%.4f,\"y\":%.4f,\"theta\":%.4f,\"v\":%.4f,\"w\":%.4f},"
      "\"speed\":{\"left\":%.1f,\"right\":%.1f}}",
      dutyLA,dutyLB,dutyRA,dutyRB, encTotalL,encTotalR,
      odom.x, odom.y, odom.theta, odom.v, odom.w,
      wheelSpeed.left, wheelSpeed.right
    );
    req->send(200,"application/json",js);
  });

  /* /setPWM?la=..&lb=..&ra=..&rb=.. */
  server.on("/setPWM",HTTP_GET,[](AsyncWebServerRequest *req){
    auto val=[&](const char* n){ return req->hasParam(n)?
      constrain(req->getParam(n)->value().toInt(),0,255):0; };
    analogWriteTrack(L_A,val("la")); analogWriteTrack(L_B,val("lb"));
    analogWriteTrack(R_A,val("ra")); analogWriteTrack(R_B,val("rb"));
    req->send(200,"text/plain","PWM updated");
  });

  /* /resetEnc — обнуление счётчиков и одометрии */
  server.on("/resetEnc",HTTP_GET,[](AsyncWebServerRequest *req){
    encTotalL=encTotalR=0;
    odom = Odom(); // Сбросить структуру одометрии
    wheelSpeed = WheelSpeed(); // Сбросить скорости
    pcnt_counter_clear(PCNT_UNIT_0); pcnt_counter_clear(PCNT_UNIT_1);
    req->send(200,"text/plain","Encoders and odometry reset");
  });

  server.on("/", HTTP_GET,
          [](AsyncWebServerRequest *request) {
              request->send(200, "text/plain",
                             "OK. Use /state, /setPWM, /resetEnc");
          });
  server.begin();
}

/*────────────────────────── SETUP ───────────────────*/
void setup()
{
  Serial.begin(115200);
  pinMode(L_A,OUTPUT); pinMode(L_B,OUTPUT);
  pinMode(R_A,OUTPUT); pinMode(R_B,OUTPUT); stopMotors();

  pinMode(ENC_R_A,INPUT); pinMode(ENC_R_B,INPUT);
  pinMode(ENC_L_A,INPUT); pinMode(ENC_L_B,INPUT);
  setupEncoder(PCNT_UNIT_0,(gpio_num_t)ENC_R_A,(gpio_num_t)ENC_R_B);
  setupEncoder(PCNT_UNIT_1,(gpio_num_t)ENC_L_A,(gpio_num_t)ENC_L_B);

  setupWiFiAndRoutes();
}

/*──────────────────────── LOOP ──────────────────────*/
void loop()
{
  /* ➊ каждые ~10 мс снимаем приращения, расширяем до int32 и обновляем одометрию и скорость колёс */
  static uint32_t tEnc=0;
  static int32_t prevL=0, prevR=0;
  static uint32_t lastUpdate = 0;
  uint32_t now = millis();
  if(now-tEnc>=10){
    tEnc=now;
    int16_t dR = snapPCNT(PCNT_UNIT_0);
    int16_t dL = snapPCNT(PCNT_UNIT_1);
    encTotalR += dR;
    encTotalL += dL;
    float dt = (now-lastUpdate)/1000.0f;
    if(dt > 0.0001f) { // защита от деления на ноль
      updateOdometryAndSpeed(dL, dR, dt);
      lastUpdate = now;
    }
  }

  /* ➋ раз в 500 мс выводим в Serial */
  static uint32_t tLog=0;
  if(now-tLog>=500){
    tLog=now;
    Serial.printf("Enc L: %ld\tEnc R: %ld\n",encTotalL,encTotalR);
    Serial.printf("Odom: x=%.3f y=%.3f th=%.3f v=%.3f w=%.3f\n", odom.x, odom.y, odom.theta, odom.v, odom.w);
    Serial.printf("Speed: L=%.1f mm/s   R=%.1f mm/s\n", wheelSpeed.left, wheelSpeed.right);
  }
}