#include "esp32_network.h"

void setupWiFi(const char* ssid, const char* password) {
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
}

String sendPhotoToServer(camera_fb_t* fb, const String& serverName, const int serverPort, const String& serverPath) {
  String getAll;
  String getBody;
  WiFiClient client;

  // Safety check
  if (!fb) {
    return "No image to send";
  }

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
    client.println("Host: " + serverName + ":" + String(serverPort));
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    client.println("Connection: close");
    client.println();

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

    Serial.println("Image data sent. Waiting for server response...");

    // Read the response from the server
    int timoutTimer = 10000; // 10 seconds timeout
    long startTimer = millis();
    boolean state = false; // Flag to indicate if we are reading the body

    while ((startTimer + timoutTimer) > millis()) {
      while (client.available()) {
        char c = client.read();

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
      // Add small delay to prevent tight loop hogging CPU
      delay(1);
    }
    Serial.println();

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
  }
  return getBody;
}