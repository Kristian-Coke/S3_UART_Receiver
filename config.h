#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"



// WiFi Configuration
extern const char WIFI_SSID[];
extern const char WIFI_PASSWORD[];

extern const char mqtt_broker[];
extern const char topic[];
extern const char mqtt_username[];
extern const char mqtt_password[];
extern const int mqtt_port;
#endif  // CONFIG_H