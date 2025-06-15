#pragma once
#include <Arduino.h>

struct CamConfig {
  String ssid;
  String password;
  uint16_t main_port;
  uint16_t lidar_port;
  String client_ip;
  uint16_t client_port;

  void load();
  void save();
  bool parseAndSet(const String& line, String& err);
  String toString() const;
  String toJSON() const;
  void setDefaults();
};

extern CamConfig camConfig;