#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "ESP32_Hotspot";       // Wi-Fi SSID
const char* password = "12345678"; // Wi-Fi password

const char* serverIP = "192.168.79.175"; // Replace with the ESP32's IP address
const int udpPort = 12345;                 // UDP port to send data to
WiFiUDP udp;

// Unique ID for this device
const char* deviceID = "ESP8266-1";        // Change to "ESP8266-2" for the second client

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi.");
}

void loop() {
  // Get RSSI value
  int rssi = WiFi.RSSI();

  // Create the message
  String message = String(deviceID) + ":" + String(rssi);

  // Send message to ESP32
  udp.beginPacket(serverIP, udpPort);
  udp.write(message.c_str());
  udp.endPacket();

  Serial.print("Sent message: ");
  Serial.println(message);

  delay(1); // Send data every 2 seconds
}
