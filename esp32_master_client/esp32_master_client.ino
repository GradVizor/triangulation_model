#include <WiFi.h>
#include <WiFiUdp.h>
#include <cmath>
#include <Arduino.h>

// Left motors :-
#define ENA 14
#define IN1 27
#define IN2 26

// Right motors :-
#define IN3 25
#define IN4 33
#define ENB 32

// Values used further :-
#define MARGIN 10
#define THRESHOLD -40
#define SPEED 250

const char* ssid = "ESP32_Hotspot";       // Wi-Fi SSID
const char* password = "12345678"; // Wi-Fi password

WiFiUDP udp;
const int udpPort = 12345;                 // UDP port to listen on
char incomingPacket[255];                 // Buffer for incoming data

// Variables to store RSSI values for each device
int rssi_esp8266_1 = 0;
int rssi_esp8266_2 = 0;
int rssi_self = 0;

void forward(int value) {
  analogWrite(ENA, value);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, value);
}
void right(int value) {
  analogWrite(ENA, value);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}
void left(int value) {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, value);
}
void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}
void rotate(int value) {
  analogWrite(ENA, value);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, value);
}

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  Serial.begin(115200);

  // Connect to Wi-Fi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi.");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // Start UDP
  udp.begin(udpPort);
  Serial.print("Listening on UDP port ");
  Serial.println(udpPort);
}

void loop() {
  int rssi_self = WiFi.RSSI();
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Read the packet
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = '\0';
    }

    // Extract RSSI value and device ID
    String data = String(incomingPacket);
    int separatorIndex = data.indexOf(':');
    if (separatorIndex != -1) {
      String deviceID = data.substring(0, separatorIndex);
      int rssi = data.substring(separatorIndex + 1).toInt();

      // Store RSSI value based on device ID
      if (deviceID == "ESP8266-1") {
        rssi_esp8266_1 = rssi;
      } else if (deviceID == "ESP8266-2") {
        rssi_esp8266_2 = rssi;
      }

      Serial.print("Device ");
      Serial.print(deviceID);
      Serial.print(" RSSI: ");
      Serial.println(rssi);

      int error = (rssi_esp8266_1 - rssi_esp8266_2);

      // Compare RSSI values if both are available
      if (rssi_esp8266_1 != 0 && rssi_esp8266_2 != 0) {
        Serial.println("Comparing RSSI values...");

        if (error > MARGIN) {
            Serial.println("............RIGHT.............");
            right(SPEED);
        } else if (error < -MARGIN) {
            Serial.println("............LEFT.............");
            left(SPEED);
        } else {
            if (rssi_esp8266_1 < THRESHOLD || rssi_esp8266_2 < THRESHOLD) {
              if (rssi_self > (rssi_esp8266_1+rssi_esp8266_2)/2){
                Serial.println("............EQUIDISTANT.............");
                forward(SPEED);
              }
              else {
                rotate(SPEED);
              }
            } else {
              Serial.println("............STOP.............");
              stop();
            }
        }
      }
    }
  }

  delay(10); // Add a small delay for stability
}
