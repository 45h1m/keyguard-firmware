/*
  Rui Santos
  Complete project details at:
  https://RandomNerdTutorials.com/esp32-cam-http-post-php-arduino/
  https://RandomNerdTutorials.com/esp32-cam-post-image-photo-server/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  *** MODIFIED FOR HTTP INSTEAD OF HTTPS ***
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h> // Changed from WiFiClientSecure
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"

const char* ssid = "SSID";          // REPLACE WITH YOUR WIFI SSID
const char* password = "okokokok";  // REPLACE WITH YOUR WIFI PASSWORD

String serverName = "192.168.113.186";   // REPLACE WITH YOUR SERVER IP OR DOMAIN NAME

String serverPath = "/upload";     // The path on the server to handle the upload

// IMPORTANT: Ensure your server is listening for HTTP (not HTTPS) on this port.
// Standard HTTP port is 80.
const int serverPort = 3000; // Server port for HTTP connection

WiFiClient client; // Changed from WiFiClientSecure

// CAMERA_MODEL_AI_THINKER PINS
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

const int timerInterval = 30000;    // time between each HTTP POST image (milliseconds)
unsigned long previousMillis = 0;   // last time image was sent

void setup() {
  Serial.begin(115200);
  Serial.print("ESP32 Cam - HTTP POST Example\n");
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector

  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("WiFi connected!\n");
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA; // Or FRAMESIZE_UXGA for higher resolution if needed
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;       // Use 2 frame buffers in PSRAM
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  Serial.println("Camera initialized successfully.");

  // Send the first photo immediately
  sendPhoto();
  previousMillis = millis(); // Set the timer baseline after the first send
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= timerInterval) {
    sendPhoto();
    previousMillis = currentMillis;
  }
}

String sendPhoto() {
  String getAll;
  String getBody;

  Serial.println("Taking picture...");
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    // Optional: Try to re-initialize or restart
    delay(1000);
    ESP.restart();
    return "Camera capture failed"; // Return error string
  }
  Serial.printf("Picture taken! Size: %u bytes\n", fb->len);

  Serial.println("Connecting to server: " + serverName + ":" + String(serverPort));

  // Connect using HTTP client
  if (client.connect(serverName.c_str(), serverPort)) {
    Serial.println("Connection successful!");
    String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";

    uint32_t imageLen = fb->len;
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;

    // Send HTTP POST request header
    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName + ":" + String(serverPort)); // Include port in Host header if non-standard
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    client.println("Connection: close"); // Advise server to close connection after response
    client.println(); // End of headers

    // Send multipart boundary and headers for the image file
    client.print(head);

    // Send image data in chunks
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    size_t chunksize = 1024; // Send in 1KB chunks
    for (size_t n = 0; n < fbLen; n = n + chunksize) {
      if (n + chunksize < fbLen) {
        client.write(fbBuf, chunksize);
        fbBuf += chunksize;
      }
      else if (fbLen % chunksize > 0) {
        size_t remainder = fbLen % chunksize;
        client.write(fbBuf, remainder);
      }
    }
    // Send the final multipart boundary
    client.print(tail);

    // Return the frame buffer to be reused
    esp_camera_fb_return(fb);
    fb = NULL; // Make sure we don't return it again if something goes wrong below

    Serial.println("Image data sent. Waiting for server response...");

    // Read the response from the server
    int timoutTimer = 10000; // 10 seconds timeout
    long startTimer = millis();
    boolean state = false; // Flag to indicate if we are reading the body

    while ((startTimer + timoutTimer) > millis()) {
      // Serial.print("."); // Optional: print dots while waiting
      // delay(100); // Be careful with delays inside loops like this

      while (client.available()) {
        char c = client.read();
         //Serial.print(c); // Uncomment for debugging full response

        // Simple logic to capture the response body after the first empty line
        if (c == '\n') {
          if (getAll.length() == 0) { // Indicates end of headers (blank line)
             state = true;
          }
          getAll = ""; // Reset line buffer
        }
        else if (c != '\r') {
          getAll += String(c); // Build the current line
        }

        if (state == true) { // If we are in the body part
          getBody += String(c);
        }
        startTimer = millis(); // Reset timeout timer with every byte received
      }
      if (getBody.length() > 0) {
          // Check if client is still connected, break if not
          if (!client.connected() && !client.available()){
              break;
          }
      }
       // Add small delay to prevent tight loop hogging CPU if client.available() is false
       delay(1);
    }
    Serial.println(); // Newline after waiting dots or response printing

    if (getBody.length() > 0) {
      Serial.println("Server Response Body:");
      Serial.println(getBody);
    } else {
      Serial.println("No response body received or timeout.");
      getBody = "Server Response Timeout";
    }

    client.stop(); // Close the connection
    Serial.println("Connection closed.");
  }
  else {
    getBody = "Connection to " + serverName + " failed.";
    Serial.println(getBody);
    if (fb) { // Make sure to return buffer even if connection failed after capture
        esp_camera_fb_return(fb);
        fb = NULL;
    }
  }
  return getBody;
}