#include "config.h"
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

CamConfig camConfig;

static const char* CONFIG_FILE = "/camconfig.txt";

// Заполнить дефолтами
void CamConfig::setDefaults() {
  ssid = "robotx";
  password = "78914040";
  main_port = 80;
  lidar_port = 8888;
  client_ip = "";
  client_port = 0;
}

// Парсинг строки "ssid|password|main_port|lidar_port|client_ip|client_port"
bool CamConfig::parseAndSet(const String& line, String& err) {
  int p1 = line.indexOf('|');
  int p2 = line.indexOf('|', p1+1);
  int p3 = line.indexOf('|', p2+1);
  int p4 = line.indexOf('|', p3+1);
  int p5 = line.indexOf('|', p4+1);
  if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0 || p5 < 0) {
    err = "Invalid format";
    return false;
  }
  String s = line.substring(0, p1);
  String p = line.substring(p1+1, p2);
  String mainPortStr = line.substring(p2+1, p3);
  String lidarPortStr = line.substring(p3+1, p4);
  String cip = line.substring(p4+1, p5);
  String cportStr = line.substring(p5+1);
  uint16_t mainP = mainPortStr.toInt();
  uint16_t lidarP = lidarPortStr.toInt();
  uint16_t cport = cportStr.toInt();

  if (!mainP || !lidarP) { err = "Port invalid"; return false; }
  ssid = s; password = p; main_port = mainP; lidar_port = lidarP; client_ip = cip; client_port = cport;
  return true;
}

void CamConfig::load() {
  setDefaults();
  if (!SPIFFS.begin(true)) return;
  File f = SPIFFS.open(CONFIG_FILE, "r");
  if (!f) return;
  String l = f.readStringUntil('\n');
  String dummy;
  parseAndSet(l, dummy);
  f.close();
}

void CamConfig::save() {
  if (!SPIFFS.begin(true)) return;
  File f = SPIFFS.open(CONFIG_FILE, "w");
  if (!f) return;
  f.println(toString());
  f.close();
}

String CamConfig::toString() const {
  return ssid + "|" + password + "|" + String(main_port) + "|" + String(lidar_port) + "|" + client_ip + "|" + String(client_port);
}

String CamConfig::toJSON() const {
  return "{\"ssid\":\"" + ssid +
    "\",\"password\":\"" + password +
    "\",\"main_port\":" + String(main_port) +
    ",\"lidar_port\":" + String(lidar_port) +
    ",\"client_ip\":\"" + client_ip +
    "\",\"client_port\":" + String(client_port) + "}";
}