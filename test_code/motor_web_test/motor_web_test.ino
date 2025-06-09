#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ==== НАСТРОЙКИ WiFi ====
const char* ssid = "robotx";
const char* password = "78914040";

// ==== Пины моторов ====
#define MOTOR1_A 32
#define MOTOR1_B 33
#define MOTOR2_A 26
#define MOTOR2_B 25

const int freq = 1000;
const int resolution = 8;

// ==== Переменные для хранения скоростей и направлений ====
int speed1 = 0;
int speed2 = 0;
String dir1 = "forward";
String dir2 = "forward";

// ==== Веб-сервер на 80 порту ====
WebServer server(80);

void setup() {
  Serial.begin(115200);
  // Настроить WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nConnected! IP: ");
  Serial.println(WiFi.localIP());

  // PWM пины
  ledcAttach(MOTOR1_A, freq, resolution);
  ledcAttach(MOTOR1_B, freq, resolution);
  ledcAttach(MOTOR2_A, freq, resolution);
  ledcAttach(MOTOR2_B, freq, resolution);

  stopMotors();

  // Обработчик главной страницы
  server.on("/", HTTP_GET, handleRoot);
  // Обработчик применения скоростей и направлений (POST)
  server.on("/set", HTTP_POST, handleSet);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

// ==== HTML интерфейс ====
const char HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>ESP32 Motor Control</title>
  <style>
    body { font-family: sans-serif; text-align:center; margin-top:40px; }
    .slider { width: 300px; }
    .label { font-size: 1.2em; }
    button { margin-top:20px; font-size:1.2em; }
  </style>
</head>
<body>
  <h2>Управление моторами ESP32</h2>
  <form action="/set" method="post">
    <div>
      <span class="label">Мотор левыи:</span><br>
      <input type="range" min="0" max="255" value="%SPEED1%" name="speed1" class="slider" id="slider1"
        oninput="document.getElementById('val1').innerText=this.value">
      <span id="val1">%SPEED1%</span><br>
      <label><input type="radio" name="dir1" value="forward" %DIR1FWD%>Вперёд</label>
      <label><input type="radio" name="dir1" value="backward" %DIR1BWD%>Назад</label>
    </div>
    <div>
      <span class="label">Мотор прави:</span><br>
      <input type="range" min="0" max="255" value="%SPEED2%" name="speed2" class="slider" id="slider2"
        oninput="document.getElementById('val2').innerText=this.value">
      <span id="val2">%SPEED2%</span><br>
      <label><input type="radio" name="dir2" value="forward" %DIR2FWD%>Вперёд</label>
      <label><input type="radio" name="dir2" value="backward" %DIR2BWD%>Назад</label>
    </div>
    <button type="submit">Применить</button>
  </form>
</body>
</html>
)rawliteral";

// ==== Обработчики ====
void handleRoot() {
  String html = HTML;
  html.replace("%SPEED1%", String(speed1));
  html.replace("%SPEED2%", String(speed2));
  html.replace("%DIR1FWD%", dir1 == "forward" ? "checked" : "");
  html.replace("%DIR1BWD%", dir1 == "backward" ? "checked" : "");
  html.replace("%DIR2FWD%", dir2 == "forward" ? "checked" : "");
  html.replace("%DIR2BWD%", dir2 == "backward" ? "checked" : "");
  server.send(200, "text/html", html);
}

void handleSet() {
  if (server.hasArg("speed1")) speed1 = server.arg("speed1").toInt();
  if (server.hasArg("speed2")) speed2 = server.arg("speed2").toInt();
  if (server.hasArg("dir1")) dir1 = server.arg("dir1");
  if (server.hasArg("dir2")) dir2 = server.arg("dir2");

  // Применить скорости и направления
  if (dir1 == "forward") {
    motor1Forward(speed1);
  } else {
    motor1Backward(speed1);
  }

  if (dir2 == "forward") {
    motor2Forward(speed2);
  } else {
    motor2Backward(speed2);
  }

  handleRoot();
}

// ==== Моторы ====
void motor1Forward(int speed) {
  ledcWrite(MOTOR1_A, speed);
  ledcWrite(MOTOR1_B, 0);
}

void motor1Backward(int speed) {
  ledcWrite(MOTOR1_A, 0);
  ledcWrite(MOTOR1_B, speed);
}

void motor2Forward(int speed) {
  ledcWrite(MOTOR2_A, speed);
  ledcWrite(MOTOR2_B, 0);
}

void motor2Backward(int speed) {
  ledcWrite(MOTOR2_A, 0);
  ledcWrite(MOTOR2_B, speed);
}

void stopMotors() {
  ledcWrite(MOTOR1_A, 0);
  ledcWrite(MOTOR1_B, 0);
  ledcWrite(MOTOR2_A, 0);
  ledcWrite(MOTOR2_B, 0);
}