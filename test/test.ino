/*
  ESP32-CAM HTTP Image Upload with Face Recognition and Lock Control
  Based on original work by Rui Santos (RandomNerdTutorials.com)
  Modified for HTTP and enhanced with face recognition and lock control
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <ArduinoJson.h>

// ===== NETWORK CONFIGURATION =====
const char* ssid = "SSID";          // REPLACE WITH YOUR WIFI SSID
const char* password = "okokokok";  // REPLACE WITH YOUR WIFI PASSWORD

// ===== SERVER CONFIGURATION =====
String serverName = "192.168.118.186";  // REPLACE WITH YOUR SERVER IP OR DOMAIN NAME
String serverPath = "/upload";          // The path on the server to handle the upload
const int serverPort = 3000;            // Server port for HTTP connection
WiFiClient client;                      // HTTP client connection

// ===== TIMER CONFIGURATION =====
const int timerInterval = 10000;   // Time between each HTTP POST image (milliseconds)
unsigned long previousMillis = 0;  // Last time image was sent

// ===== CAMERA PIN CONFIGURATION =====
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

// ===== LOCK PIN CONFIGURATION =====
struct LockConfig {
  const char* id;
  int pin;
  bool activeLow;  // Set true if the lock activates when pin is LOW
};

// Define all available locks in the system
const LockConfig LOCKS[] = {
  {"lock_001", 12, false},  // Lock 1 on GPIO12, activates on HIGH
  {"lock_002", 13, false},  // Lock 2 on GPIO13, activates on HIGH
  {"lock_003", 14, false}   // Lock 3 on GPIO14, activates on HIGH
};
const int NUM_LOCKS = sizeof(LOCKS) / sizeof(LOCKS[0]);

// ===== FUNCTION DECLARATIONS =====
void setupCamera();
void setupWiFi();
void setupLockPins();
String sendPhoto();
bool handleFaceRecognitionResponse(String jsonResponse);
void unlockSolenoid(const char* lockId, int duration);
int findLockPin(const char* lockId);

// ===== SETUP FUNCTION =====
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Cam - HTTP POST with Face Recognition");
  
  // Disable brownout detector
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  setupWiFi();
  setupCamera();
  setupLockPins();
  
  // Send the first photo immediately
  sendPhoto();
  previousMillis = millis();  // Set the timer baseline after the first send
}

// ===== MAIN LOOP =====
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= timerInterval) {
    sendPhoto();
    previousMillis = currentMillis;
  }
}

// ===== SETUP FUNCTIONS =====
void setupWiFi() {
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
  Serial.println("WiFi connected!");
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());
}

void setupCamera() {
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

  // Initialize with appropriate specs based on available memory
  if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;  // 0-63 lower number means higher quality
    config.fb_count = 2;       // Use 2 frame buffers in PSRAM
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  Serial.println("Camera initialized successfully.");
}

void setupLockPins() {
  // Initialize all lock pins as outputs and set to initial state
  for (int i = 0; i < NUM_LOCKS; i++) {
    pinMode(LOCKS[i].pin, OUTPUT);
    // Set initial state (locked)
    digitalWrite(LOCKS[i].pin, LOCKS[i].activeLow ? HIGH : LOW);
    Serial.printf("Initialized lock %s on pin %d\n", LOCKS[i].id, LOCKS[i].pin);
  }
}

// ===== LOCK CONTROL FUNCTIONS =====
int findLockPin(const char* lockId) {
  for (int i = 0; i < NUM_LOCKS; i++) {
    if (strcmp(lockId, LOCKS[i].id) == 0) {
      return i;  // Return index in the LOCKS array
    }
  }
  return -1;  // Lock ID not found
}

void unlockSolenoid(const char* lockId, int duration) {
  int lockIndex = findLockPin(lockId);
  
  if (lockIndex != -1) {
    int pinNumber = LOCKS[lockIndex].pin;
    bool isActiveLow = LOCKS[lockIndex].activeLow;
    
    Serial.printf("Unlocking solenoid %s on pin %d for %d seconds\n", 
                  lockId, pinNumber, duration);

    // Activate the solenoid (set to active state)
    digitalWrite(pinNumber, isActiveLow ? LOW : HIGH);

    // Create a delay to keep the lock open for the specified duration
    delay(duration * 1000);
    
    // Deactivate the solenoid (return to locked state)
    digitalWrite(pinNumber, isActiveLow ? HIGH : LOW);

    Serial.printf("Lock %s secured again\n", lockId);
  } else {
    Serial.printf("Unknown lock ID: %s\n", lockId);
  }
}

// ===== FACE RECOGNITION RESPONSE HANDLER =====
bool handleFaceRecognitionResponse(String jsonResponse) {
  // Check if response is empty or indicates an error
  if (jsonResponse.length() == 0 || 
      jsonResponse == "Server Response Timeout" || 
      jsonResponse.startsWith("Connection to")) {
    Serial.println("Error in server response: " + jsonResponse);
    return false;
  }

  // Find the JSON content in the response (skipping HTTP headers if present)
  int jsonStart = jsonResponse.indexOf('{');
  if (jsonStart == -1) {
    Serial.println("No valid JSON found in response");
    return false;
  }

  String jsonContent = jsonResponse.substring(jsonStart);

  // Parse the JSON response
  StaticJsonDocument<1024> doc;  // Adjust size based on expected response size
  DeserializationError error = deserializeJson(doc, jsonContent);

  if (error) {
    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());
    return false;
  }

  // Process the recognition result
  const char* status = doc["status"];
  const char* recognitionResult = doc["recognition_result"];
  const char* message = doc["message"];

  Serial.print("Status: ");
  Serial.println(status);
  Serial.print("Recognition result: ");
  Serial.println(recognitionResult);
  Serial.print("Message: ");
  Serial.println(message);

  // Check if we have a successful recognition with unlock permission
  if (strcmp(recognitionResult, "identified") == 0) {
    // Get person information
    const char* personName = doc["person"]["name"];
    int accessLevel = doc["person"]["access_level"];

    Serial.print("Access granted for: ");
    Serial.println(personName);
    Serial.print("Access level: ");
    Serial.println(accessLevel);

    // Process unlock actions
    JsonArray actions = doc["actions"];
    for (JsonVariant action : actions) {
      const char* lockId = action["lock_id"];
      const char* actionType = action["action"];
      int duration = action["duration"];

      Serial.printf("Performing %s on lock: %s\n", actionType, lockId);

      if (strcmp(actionType, "unlock") == 0) {
        // Trigger the solenoid unlock function
        unlockSolenoid(lockId, duration);
      }
    }
    return true;
  } else if (strcmp(recognitionResult, "unrecognized") == 0) {
    Serial.println("Face not recognized");
    return false;
  } else if (strcmp(recognitionResult, "no_permission") == 0) {
    Serial.println("Person recognized but has no access permission");
    return false;
  }
  return false;
}

// ===== IMAGE CAPTURE AND UPLOAD FUNCTION =====
String sendPhoto() {
  String getAll;
  String getBody;

  Serial.println("Taking picture...");
  camera_fb_t* fb = NULL;
  fb = esp_camera_fb_get();
  
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
    return "Camera capture failed";
  }
  
  Serial.printf("Picture taken! Size: %u bytes\n", fb->len);
  Serial.println("Connecting to server: " + serverName + ":" + String(serverPort));

  // Connect using HTTP client
  if (client.connect(serverName.c_str(), serverPort)) {
    Serial.println("Connection successful!");
    
    // Prepare multipart form data
    String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";

    uint32_t imageLen = fb->len;
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;

    // Send HTTP POST request header
    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName + ":" + String(serverPort));
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    client.println("Connection: close");
    client.println();

    // Send multipart boundary and headers for the image file
    client.print(head);

    // Send image data in chunks
    uint8_t* fbBuf = fb->buf;
    size_t fbLen = fb->len;
    size_t chunksize = 1024;  // Send in 1KB chunks
    
    for (size_t n = 0; n < fbLen; n = n + chunksize) {
      if (n + chunksize < fbLen) {
        client.write(fbBuf, chunksize);
        fbBuf += chunksize;
      } else if (fbLen % chunksize > 0) {
        size_t remainder = fbLen % chunksize;
        client.write(fbBuf, remainder);
      }
    }
    
    // Send the final multipart boundary
    client.print(tail);

    // Return the frame buffer to be reused
    esp_camera_fb_return(fb);
    fb = NULL;

    Serial.println("Image data sent. Waiting for server response...");

    // Read the response from the server
    int timoutTimer = 10000;  // 10 seconds timeout
    long startTimer = millis();
    boolean state = false;  // Flag to indicate if we are reading the body

    while ((startTimer + timoutTimer) > millis()) {
      while (client.available()) {
        char c = client.read();

        // Simple logic to capture the response body after the first empty line
        if (c == '\n') {
          if (getAll.length() == 0) {  // Indicates end of headers (blank line)
            state = true;
          }
          getAll = "";  // Reset line buffer
        } else if (c != '\r') {
          getAll += String(c);  // Build the current line
        }

        if (state == true) {  // If we are in the body part
          getBody += String(c);
        }
        startTimer = millis();  // Reset timeout timer with every byte received
      }
      
      if (getBody.length() > 0) {
        // Check if client is still connected, break if not
        if (!client.connected() && !client.available()) {
          break;
        }
      }
      // Add small delay to prevent tight loop hogging CPU
      delay(1);
    }

    if (getBody.length() > 0) {
      Serial.println("Server Response Body:");
      Serial.println(getBody);

      // Process the JSON response
      bool accessGranted = handleFaceRecognitionResponse(getBody);

      if (accessGranted) {
        Serial.println("Access granted - door unlocked");
      } else {
        Serial.println("Access denied");
      }
    } else {
      Serial.println("No response body received or timeout.");
      getBody = "Server Response Timeout";
    }

    client.stop();  // Close the connection
    Serial.println("Connection closed.");
  } else {
    getBody = "Connection to " + serverName + " failed.";
    Serial.println(getBody);
    
    if (fb) {  // Make sure to return buffer even if connection failed after capture
      esp_camera_fb_return(fb);
      fb = NULL;
    }
  }
  
  return getBody;
}