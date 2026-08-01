#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>
#include <esp_wifi.h>

#include "config.hpp"

class WiFiManager {
 private:
  static bool wifiConnected;
  static unsigned long lastWifiCheck;
  static unsigned long wifiFailureStartTime;
  static const unsigned long WIFI_FAILURE_TIMEOUT;

 public:
  static bool initialize();
  static bool isConnected();
  static void checkConnectionStatus();
  static void reconnect();
};

#endif  // WIFI_MANAGER_H