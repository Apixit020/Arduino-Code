#include <ESP8266WiFi.h>

// Client credentials
const char* ssid = "ESP32_AP";
const char* password = "password";

// GPIO Pin for LED
// #define LED_PIN 2  // Change this pin if necessary for Client2

WiFiClient client;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  // Initialize LED as off
  Serial.begin(115200);            //เปิดใช้ Serial
  // Connect to Access Point

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());  //แสดงหมายเลข IP
  // Connect to server (BSS)
  client.connect(WiFi.gatewayIP(), 80);
}

void loop() {
  if (client.available()) {
    String command = client.readStringUntil('\n');
    command.trim();  // ตัดช่องว่างที่อยู่ข้างหน้าหรือข้างหลังออก

    Serial.println("Received command: " + command);  // ตรวจสอบค่าที่อ่านมา
    if (command == "TURN_ON_LED") {
      digitalWrite(LED_BUILTIN, HIGH);  // เปิดไฟ LED
      Serial.println("LED turned ON");
    } else if (command == "TURN_OFF_LED") {
      digitalWrite(LED_BUILTIN, LOW);  // ปิดไฟ LED
      Serial.println("LED turned OFF");
    } else {
      Serial.println("Unknown command");
    }
  }
}
