/*********************************************************************
 * esp32_lidar_ws.ino  –  LIDAR → WebSocket (200 FPS, 1 клиент)
 *********************************************************************/
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <driver/uart.h>
#include "esp_task_wdt.h"

/* ---------- пользователь ---------- */
#define WIFI_SSID "robotx"
#define WIFI_PASS "78914040"
#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 17
#define LIDAR_BAUD   115200
#define DEBUG_FULL   0
/* ---------------------------------- */

/* WebSocket */
#define WS_PORT 8888
#define WS_PATH "/ws"
AsyncWebServer  server(WS_PORT);
AsyncWebSocket  ws(WS_PATH);

/* Lidar packet */
static const uint8_t HDR[4]   = {0x55,0xAA,0x03,0x08};
static const uint8_t BODY_LEN = 32;
static const uint8_t INTENSITY_MIN = 2;
static const float   MAX_SPREAD_DEG = 20.0;

/* сети */
const uint8_t NET_LEN = 20;
static QueueHandle_t qFrames;      // длина 1

/* счётчики */
volatile uint32_t fps_rx = 0, fps_tx = 0;

/* ---------- единственный клиент ---------- */
AsyncWebSocketClient* sole = nullptr;   // <<— перенесено вверх

/* ---------- utils ---------- */
static inline float decodeAngle(uint16_t r){
  float a = (r - 0xA000) / 64.0f;
  if (a < 0)      a += 360.0f;
  else if(a>=360) a -= 360.0f;
  return a;
}
static bool readBytesTO(HardwareSerial& s,uint8_t* b,size_t len,uint32_t to=500){
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
#if DEBUG_FULL
      Serial.printf("%02X ", uc);
#endif
      if(uc==HDR[pos]){ if(++pos==4) return true; }
      else pos=0;
    }else vTaskDelay(1);
    if(millis()-t0>200) return false;
    esp_task_wdt_reset();
  }
}

/* ---------- TASK 1  (Core1) ------------- */
void lidarTask(void*){
  esp_task_wdt_add(NULL); // Регистрируем задачу в WDT
  Serial1.begin(LIDAR_BAUD,SERIAL_8N1,LIDAR_RX_PIN,LIDAR_TX_PIN);
  uint8_t body[BODY_LEN], pkt[NET_LEN];

  while(true){
    if(!waitHeader(Serial1))                 continue;
    if(!readBytesTO(Serial1,body,BODY_LEN))  continue;

    float sDeg = decodeAngle(body[2] | (body[3]<<8));
    uint8_t off=4; uint16_t dist[8]; uint8_t inten[8];
    for(int i=0;i<8;i++){ dist[i]=body[off]|(body[off+1]<<8); inten[i]=body[off+2]; off+=3; }
    float eDeg = decodeAngle(body[off] | (body[off+1]<<8));
    if(eDeg<sDeg) eDeg += 360;
    if(eDeg - sDeg > MAX_SPREAD_DEG) continue;

    uint8_t* p=pkt;
    uint16_t s=(uint16_t)(sDeg*100+0.5f), e=(uint16_t)(eDeg*100+0.5f);
    *p++=s; *p++=s>>8; *p++=e; *p++=e>>8;
    for(int i=0;i<8;i++){ uint16_t d=inten[i]>=INTENSITY_MIN?dist[i]:0; *p++=d; *p++=d>>8; }

    xQueueOverwrite(qFrames, pkt);     // очередь=1, без дропов
    ++fps_rx;
    esp_task_wdt_reset(); // Сброс WDT именно в этой задаче
  }
}

/* ---------- TASK 2 (Core0) ------------- */
void wsTask(void*){
  esp_task_wdt_add(NULL); // Регистрируем задачу в WDT
  const TickType_t slot=pdMS_TO_TICKS(5);
  TickType_t prev=xTaskGetTickCount();
  uint8_t pkt[NET_LEN];

  for(;;){
    vTaskDelayUntil(&prev, slot);

    if (sole && sole->canSend() &&
        xQueueReceive(qFrames, pkt, 0) == pdTRUE) {
      sole->binary(pkt, NET_LEN);
      ++fps_tx;
    }
    esp_task_wdt_reset(); // Сброс WDT именно в этой задаче
  }
}

/* ---------- WS events ---------- */
void onWs(AsyncWebSocket*,AsyncWebSocketClient* c,
          AwsEventType t,void*,uint8_t*,size_t){
  if(t==WS_EVT_CONNECT){
    if(sole && sole->status()==WS_CONNECTED){
      Serial.printf("[WS] kick old client %u\n", sole->id());
      sole->close();
    }
    sole=c;
    Serial.printf("[WS] + client %u\n", c->id());
  }
  if(t==WS_EVT_DISCONNECT && c==sole){
    sole = nullptr;
    Serial.printf("[WS] - client %u\n", c->id());
  }
}

/* ---------- setup ---------- */
void setup(){
  Serial.begin(115200); delay(300);
  Serial.println("\n=== LIDAR WS 200 FPS (single client) ===");

  WiFi.mode(WIFI_STA); WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID,WIFI_PASS);
  while(WiFi.status()!=WL_CONNECTED){ Serial.print('.'); delay(300); }
  Serial.printf("\nIP %s\n", WiFi.localIP().toString().c_str());

  ws.onEvent(onWs); server.addHandler(&ws); server.begin();

  qFrames = xQueueCreate(/* length */1, NET_LEN);
  xTaskCreatePinnedToCore(lidarTask,"LIDAR",4096,NULL,2,NULL,1);
  xTaskCreatePinnedToCore(wsTask,   "WS_TX",4096,NULL,2,NULL,0);

  // Инициализация и регистрация главной задачи (loop) в WDT
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 5000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);     // Регистрируем loop() в WDT
}

/* ---------- loop ---------- */
uint32_t t0=0;
void loop(){
  ws.cleanupClients();
  if(millis()-t0>=1000){
    t0=millis();
    uint32_t rx=fps_rx; fps_rx=0;
    uint32_t tx=fps_tx; fps_tx=0;
    Serial.printf("[STAT] rx:%3u  tx:%3u\n", rx, tx);
  }
  esp_task_wdt_reset(); // Сброс WDT для главной задачи
}