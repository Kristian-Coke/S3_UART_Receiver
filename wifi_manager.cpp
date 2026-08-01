#include "wifi_manager.h"

#include <Arduino.h>

// Static member definitions
bool WiFiManager::wifiConnected = true;
unsigned long WiFiManager::lastWifiCheck = 0;
unsigned long WiFiManager::wifiFailureStartTime = 0;
const unsigned long WiFiManager::WIFI_FAILURE_TIMEOUT = 30000;  // 30 seconds

bool WiFiManager::initialize() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");

  const unsigned long start = millis();
  const unsigned long timeout = 15000; // wait up to 15s
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeout) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP=");
    Serial.println(WiFi.localIP());
    wifiConnected = true;
    return true;
  } else {
    Serial.print("Failed to connect after ");
    Serial.print((millis() - start) / 1000);
    Serial.println("s. Status: " + String(WiFi.status()));
    return false;
  }
}

bool WiFiManager::isConnected() { return WiFi.status() == WL_CONNECTED; }

void WiFiManager::checkConnectionStatus() {
  unsigned long currentTime = millis();

  if (currentTime - lastWifiCheck > 5000) {  // Check every 5 seconds
    if (WiFi.status() != WL_CONNECTED) {
      if (wifiConnected) {
        // WiFi just disconnected, start failure timer
        wifiFailureStartTime = currentTime;
        wifiConnected = false;
        Serial.println("WiFi connection lost, reconnecting...");
      } else {
        // WiFi has been disconnected for a while
        if (currentTime - wifiFailureStartTime > WIFI_FAILURE_TIMEOUT) {
          Serial.println("WiFi reconnection failed for too long - REBOOTING");
          Serial.flush();
          delay(1000);
          ESP.restart();
        }
      }
      WiFi.reconnect();
    } else {
      if (!wifiConnected) {
        Serial.println("WiFi reconnected successfully");
        wifiConnected = true;
      }
    }
    lastWifiCheck = currentTime;
  }
}

void WiFiManager::reconnect() {
  Serial.println("Attempting WiFi reconnection...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi reconnected successfully!");
    wifiConnected = true;
  } else {
    Serial.println("\nFailed to reconnect to WiFi.");
  }
}
