#include <WiFi.h>

const char* ssid = "ESP32_Hotspot";
const char* password = "12345678";

void setup() {
  Serial.begin(115200);

  // Set ESP32 to Access Point mode
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password, 1, 0, 4); // SSID visible


  Serial.println("ESP32 Hotspot created");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("Password: ");
  Serial.println(password);
}

void loop() {
  // Keep the AP running
}
