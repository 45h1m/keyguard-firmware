#ifndef ESP32_NETWORK_H
#define ESP32_NETWORK_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "esp_camera.h"

// Function declarations
void setupWiFi(const char* ssid, const char* password);
String sendPhotoToServer(camera_fb_t* fb, const String& serverName, const int serverPort, const String& serverPath);

#endif // ESP32_NETWORK_H