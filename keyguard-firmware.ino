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
  *** MODIFIED TO ADD WS2811 LED STATUS VIA FASTLED ***
  *** MODIFIED TO ADD "READY" LED INDICATION ***
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>  // Changed from WiFiClientSecure
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <ArduinoJson.h>
#include <FastLED.h>     // Added for WS2811 LED control

// --- FastLED definitions ---
#define LED_PIN 2        // GPIO pin for WS2811 data line (e.g., GPIO2)
#define NUM_LEDS 1       // We have one WS2811 LED
#define BRIGHTNESS 60    // Brightness 0-255
CRGB leds[NUM_LEDS];     // Array to hold LED color data
const CRGB READY_COLOR = CRGB::Aqua; // Color for "ready to take picture" state
// --- End FastLED definitions ---

const size_t JSON_DOC_SIZE = 1024;

const char* ssid = "SSID";          // REPLACE WITH YOUR WIFI SSID
const char* password = "okokokok";  // REPLACE WITH YOUR WIFI PASSWORD

String serverName = "192.168.96.186";  // REPLACE WITH YOUR SERVER IP OR DOMAIN NAME

String serverPath = "/upload";  // The path on the server to handle the upload

const int serverPort = 3000;  // Server port for HTTP connection

WiFiClient client;

// CAMERA_MODEL_AI_THINKER PINS
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

const int timerInterval = 5000;    // time between each HTTP POST image (milliseconds)
unsigned long previousMillis = 0;  // last time image was sent

#define IR_SENSOR_PIN 13

#define RELAY_PIN_1  12
#define RELAY_PIN_2  14
#define RELAY_PIN_3  15

// --- LED Helper Functions ---
void led_set_color(CRGB color) {
    leds[0] = color;
    FastLED.show();
}

void led_off() {
    leds[0] = CRGB::Black;
    FastLED.show();
}

void led_show_status_temp(CRGB color, unsigned long duration_ms) {
    // Ensure temporary status is shown at standard brightness
    // (in case a future "ready" state uses pulsing brightness)
    uint8_t currentGlobalBrightness = FastLED.getBrightness();
    if (currentGlobalBrightness != BRIGHTNESS) FastLED.setBrightness(BRIGHTNESS);

    led_set_color(color);
    delay(duration_ms);
    led_off(); // Turns LED off

    // Restore previous brightness only if it was different,
    // otherwise main loop will handle setting BRIGHTNESS for READY_COLOR
    // For now, simpler to let the main loop always set READY_COLOR with BRIGHTNESS.
    // if (currentGlobalBrightness != BRIGHTNESS) FastLED.setBrightness(currentGlobalBrightness);
}
// --- End LED Helper Functions ---

void setup() {
  Serial.begin(115200);
  Serial.print("ESP32 Cam - HTTP POST Example with LED Status\n");
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  FastLED.addLeds<WS2811, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  led_off(); 

  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  pinMode(RELAY_PIN_3, OUTPUT);
  digitalWrite(RELAY_PIN_1, HIGH);
  digitalWrite(RELAY_PIN_2, HIGH);
  digitalWrite(RELAY_PIN_3, HIGH);

  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  unsigned long wifi_connect_start_time = millis();
  bool wifi_led_state = false;
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    leds[0] = wifi_led_state ? CRGB::Blue : CRGB::Black;
    FastLED.show();
    wifi_led_state = !wifi_led_state;
    delay(250); 

    if (millis() - wifi_connect_start_time > 30000) { 
        Serial.println("\nWiFi Connection Timeout!");
        while(true) { // Blink Red indefinitely
            led_set_color(CRGB::Red); delay(500);
            led_off(); delay(500);
        }
    }
  }
  Serial.println();
  Serial.print("WiFi connected!\n");
  led_show_status_temp(CRGB::Green, 1500); 
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

  if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    led_set_color(CRGB::Red); 
    Serial.println("Restarting...");
    delay(2000); 
    ESP.restart();
  }
  Serial.println("Camera initialized successfully.");
  led_show_status_temp(CRGB::Green, 1000); 

  pinMode(4, OUTPUT); 
  digitalWrite(4, LOW); 

  Serial.println("System setup complete. Entering ready state.");
  led_set_color(READY_COLOR); // Set to ready state after setup completes
  previousMillis = millis(); // Set the timer baseline
}

void loop() {
  unsigned long currentMillis = millis();

  // Condition to take a photo: timerInterval has passed AND IR sensor is active (LOW)
  if (currentMillis - previousMillis >= timerInterval && !digitalRead(IR_SENSOR_PIN)) {
    // --- Active processing starts ---
    FastLED.setBrightness(BRIGHTNESS); // Ensure standard brightness for active states
    led_set_color(CRGB::DeepPink);     // Indicate IR trigger / process start

    digitalWrite(4, HIGH); // Turn on onboard flash LED
    delay(100);            // Allow flash to stabilize before capture

    String serverResponse = sendPhoto(); // This function manages its own LED states during execution

    delay(100);            // Original delay from user code
    digitalWrite(4, LOW);  // Turn off onboard flash LED (before handleServerResponse, as in original)

    handleServerResponse(serverResponse); // This function manages its own LED states

    previousMillis = currentMillis;    // Reset the timer
    Serial.println("Processing complete. Returning to ready state.");
    led_set_color(READY_COLOR);        // Return to ready state indication
  } else {
    // --- Idle / Ready state ---
    // Set to READY_COLOR if not already, and ensure brightness is standard
    // This handles returning to ready after a temporary status from led_show_status_temp might have turned LED off.
    if (leds[0] != READY_COLOR || FastLED.getBrightness() != BRIGHTNESS) {
        FastLED.setBrightness(BRIGHTNESS);
        led_set_color(READY_COLOR);
    }
    // Optional small delay if loop runs too fast in idle, but usually not necessary with millis-based timing
    // delay(10); 
  }
}

String sendPhoto() {
  String getAll;
  String getBody;

  Serial.println("Taking picture...");
  FastLED.setBrightness(BRIGHTNESS); // Ensure brightness for this status
  led_set_color(CRGB::Cyan); // LED Cyan: Taking picture

  camera_fb_t* fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    led_show_status_temp(CRGB::Red, 2000); // Red for 2s, then off (main loop will restore READY_COLOR or restart)
    delay(1000); 
    ESP.restart(); 
    return "Error: Camera capture failed"; // Should be unreachable due to restart
  }
  Serial.printf("Picture taken! Size: %u bytes\n", fb->len);

  Serial.println("Connecting to server: " + serverName + ":" + String(serverPort));
  FastLED.setBrightness(BRIGHTNESS);
  led_set_color(CRGB::Yellow); 

  if (client.connect(serverName.c_str(), serverPort)) {
    FastLED.setBrightness(BRIGHTNESS);
    led_set_color(CRGB::LimeGreen); 
    Serial.println("Connection successful!");
    String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";

    uint32_t imageLen = fb->len;
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;

    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName + ":" + String(serverPort));
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    client.println("Connection: close");
    client.println();
    client.print(head);

    Serial.println("Sending image data...");
    FastLED.setBrightness(BRIGHTNESS);
    led_set_color(CRGB::Orange); 

    uint8_t* fbBuf = fb->buf;
    size_t fbLen = fb->len;
    size_t chunksize = 1024;
    for (size_t n = 0; n < fbLen; n = n + chunksize) {
      if (n + chunksize < fbLen) {
        client.write(fbBuf, chunksize);
        fbBuf += chunksize;
      } else if (fbLen % chunksize > 0) {
        size_t remainder = fbLen % chunksize;
        client.write(fbBuf, remainder);
      }
    }
    client.print(tail);
    esp_camera_fb_return(fb);
    fb = NULL;

    Serial.println("Image data sent. Waiting for server response...");
    FastLED.setBrightness(BRIGHTNESS);
    led_set_color(CRGB::SkyBlue); 

    int timoutTimer = 10000;
    long startTimer = millis();
    boolean http_state = false; // Renamed to avoid conflict with a potential global 'state'
    getAll = ""; 
    getBody = ""; 

    while ((startTimer + timoutTimer) > millis()) {
      while (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (getAll.length() == 0) { http_state = true; }
          getAll = "";
        } else if (c != '\r') {
          getAll += String(c);
        }
        if (http_state == true) { getBody += String(c); }
        startTimer = millis(); 
      }
      if (getBody.length() > 0 && (!client.connected() && !client.available())) {
        break;
      }
      delay(1); 
    }
    Serial.println(); 

    if (getBody.length() > 0) {
      Serial.println("Server Response Body:");
      Serial.println(getBody);
      // LED (SkyBlue) will be changed by handleServerResponse or loop returning to READY_COLOR
    } else {
      Serial.println("No response body received or timeout.");
      getBody = "Error: Server Response Timeout"; 
      led_show_status_temp(CRGB::DarkRed, 2000); 
    }
    client.stop();
    Serial.println("Connection closed.");
  } else {
    getBody = "Error: Connection to " + serverName + " failed.";
    Serial.println(getBody);
    led_show_status_temp(CRGB::Red, 2000); 
    if (fb) { 
      esp_camera_fb_return(fb);
      fb = NULL;
    }
  }
  return getBody;
}

void handleServerResponse(String responseBody) {
  Serial.println("\n--- Handling Server Response ---");
  FastLED.setBrightness(BRIGHTNESS); // Ensure brightness

  if (responseBody.startsWith("Error: Server Response Timeout") ||
      responseBody.startsWith("Error: Connection to ") ||
      responseBody.startsWith("Error: Camera capture failed") // Added this just in case restart fails
      ) {
    Serial.println("Error reported by sendPhoto, LED status already handled (likely shown error color then off):");
    Serial.println(responseBody);
    Serial.println("--- End Handling ---");
    // Main loop will set READY_COLOR next
    return; 
  }

  if (responseBody.startsWith("Error:")) {
    Serial.println("Generic or unexpected error string prefixed with 'Error:' received:");
    Serial.println(responseBody);
    led_show_status_temp(CRGB::Magenta, 2500); 
    Serial.println("--- End Handling ---");
    return;
  }

  Serial.println("Attempting to parse JSON response...");
  led_set_color(CRGB::Teal); 

  StaticJsonDocument<JSON_DOC_SIZE> doc;
  DeserializationError error = deserializeJson(doc, responseBody);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    Serial.println("Raw response was:");
    Serial.println(responseBody);
    led_show_status_temp(CRGB::Red, 3000); 
    Serial.println("--- End Handling ---");
    return;
  }

  Serial.println("JSON Parsing Successful!");
  led_show_status_temp(CRGB::Green, 1500); // Green for 1.5s, then off. Loop will set READY_COLOR.

  bool success = doc["success"] | false;
  const char* recognition_result = doc["recognition_result"] | "unknown";
  const char* message = doc["message"] | "";
  const char* timestamp = doc["timestamp"] | "";

  Serial.printf("Success: %s\n", success ? "true" : "false");
  Serial.printf("Recognition Result: %s\n", recognition_result);
  Serial.printf("Message: %s\n", message);
  Serial.printf("Timestamp: %s\n", timestamp);

  if (doc.containsKey("person") && !doc["person"].isNull()) {
    JsonObject person = doc["person"];
    const char* person_id = person["id"] | "N/A";
    const char* person_name = person["name"] | "N/A";
    int access_level = person["access_level"] | -1;

    Serial.println("Person Details:");
    Serial.printf("  ID: %s\n", person_id);
    Serial.printf("  Name: %s\n", person_name);
    Serial.printf("  Access Level: %d\n", access_level);
  } else {
    Serial.println("Person details not found in response.");
  }

  if (doc.containsKey("actions") && doc["actions"].is<JsonArray>()) {
    JsonArray actions = doc["actions"].as<JsonArray>();
    if (!actions.isNull() && actions.size() > 0) {
      Serial.println("Actions:");
      for (JsonObject action_item : actions) {
        const char* lock_id = action_item["lock_id"] | "N/A";
        const char* action_cmd = action_item["action"] | "N/A";
        int duration = action_item["duration"] | 0; 

        Serial.printf("  Lock ID: %s, Action: %s, Duration: %d\n", lock_id, action_cmd, duration);

        if(strcmp(action_cmd, "N/A") == 0 || strcmp(lock_id, "N/A") == 0) {
          Serial.println("Skipping unlock::No 'lock_id' or 'action'");
          led_show_status_temp(CRGB::Orange, 1500); 
        } else if (strcmp(action_cmd, "unlock") == 0) { 
          unlock(lock_id); 
        } else {
          Serial.println("Unknown action command: " + String(action_cmd));
          led_show_status_temp(CRGB::Yellow, 1500); 
        }
      }
    } else {
      Serial.println("Actions array is present but empty or null.");
    }
  } else {
    Serial.println("Actions array not found or not an array in response.");
  }

  Serial.println("--- End Handling ---");
}

void unlock(const char* lock_id) {
  Serial.println("  -> Entered unlock block");
  FastLED.setBrightness(BRIGHTNESS); // Ensure brightness
  bool unlocked_something = false;
  CRGB unlock_color = CRGB::Purple; 

  if (strcmp(lock_id, "lock_001") == 0) {
    Serial.println("  -> Action unlocking: (ID: lock_001)");
    digitalWrite(RELAY_PIN_1, LOW); 
    unlocked_something = true;
  }
  else if (strcmp(lock_id, "lock_002") == 0) {
    Serial.println("  -> Action unlocking: (ID: lock_002)");
    digitalWrite(RELAY_PIN_2, LOW); 
    unlocked_something = true;
  }
  else if (strcmp(lock_id, "lock_003") == 0) {
    Serial.println("  -> Action unlocking: (ID: lock_003)");
    digitalWrite(RELAY_PIN_3, LOW); 
    unlocked_something = true;
  }
  else {
    Serial.println("  -> Invalid lock_id: " + String(lock_id));
    led_show_status_temp(CRGB::Gold, 1500); 
    return; 
  }
  
  if (unlocked_something) {
    led_set_color(unlock_color); 
    Serial.printf("  -> Relay(s) for %s activated, LED %s\n", lock_id, "Purple");
  }
  
  delay(3000); 
  
  digitalWrite(RELAY_PIN_1, HIGH); 
  digitalWrite(RELAY_PIN_2, HIGH); 
  digitalWrite(RELAY_PIN_3, HIGH); 
  
  if (unlocked_something) {
    Serial.println("  -> Locked again (Relays OFF)");
    led_show_status_temp(CRGB::DarkCyan, 500); // Then off. Loop will set READY_COLOR.
  } else {
    led_off(); // Should not be reached if invalid IDs return, but good practice.
  }
  delay(100); 
}