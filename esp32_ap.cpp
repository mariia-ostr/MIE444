#include <WiFi.h>

const char* main_ssid = "iiPhone";
const char* main_pswd = "xyz";
//const char* main_ssid = "Apple";
//const char* main_pswd = "qzc579pTMW";

// access point credentials
const char* ap_ssid = "ESP32_Wifi";
const char* ap_pswd = "xyz";

void setup() {
  Serial.begin(115200);  // Start UART communication for debugging

  Serial.println("Connecting to WiFi...");
  WiFi.begin(main_ssid, main_pswd);

  unsigned long startAttemptTime = millis(); // Start time for timeout

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) { // Timeout after 15 seconds
    delay(1000);
    Serial.print("Status: ");
    Serial.println(WiFi.status());
    Serial.println("Connecting...");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected, setting up access point ...");
    Serial.print("Router IP Address: ");
    Serial.println(WiFi.localIP());
    WiFi.softAP(ap_ssid, ap_pswd);

    IPAddress AP_IP = WiFi.softAPIP();
    Serial.print("Access Point IP Address: ");
    Serial.println(AP_IP);

  } else {
    Serial.println("Failed to connect to WiFi");
  }
}


void loop() {
  // Keep the Wi-Fi connection alive
  delay(1000);
}
