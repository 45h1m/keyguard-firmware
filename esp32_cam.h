#ifndef ESP32_CAM_H
#define ESP32_CAM_H

#include <Arduino.h>
#include "esp_camera.h"

// Function declarations
void setupCamera();
camera_fb_t* takePicture();
void returnPicture(camera_fb_t* fb);

#endif // ESP32_CAM_H