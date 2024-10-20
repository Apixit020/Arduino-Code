/*
 * ไลบรารี่ TridentTD_LineNotify version 2.1
 * ใช้สำหรับ ส่ง แจ้งเตือนไปยัง LINE สำหรับ ESP8266 และ ESP32
 * สามารถส่งได้ทั้ง ข้อความ , สติกเกอร์ และรูปภาพ(ด้วย url)
 * -----------------------------------------------------
 * ให้ save เป็น file ต่างหากก่อนถึงจะส่ง Line Notify ภาษาไทยได้
 */

#include <TridentTD_LineNotify.h>

const char* ssid = "B8D";  //--> Your wifi name
const char* password = "79797989tyzy.";

#define LINE_TOKEN "6OaVtSxixL7VXuKb3Qeg9Z28qcLMF4bDowS4DtXMJY2"

int ledPin = 13;     // ขา LED
int analogPin = 5;   // ขา analog
int val = 0;         // ตัวแปรเก็บค่าความร้อน
bool alert = false;  // ตัวแปรสถานะแจ้งเตือน

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println(LINE.getVersion());

  WiFi.begin(ssid, password);
  Serial.printf("WiFi connecting to %s\n", ssid);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(400);
  }
  Serial.printf("\nWiFi connected\nIP : ");
  Serial.println(WiFi.localIP());

  // กำหนด Line Token
  LINE.setToken(LINE_TOKEN);

  pinMode(ledPin, OUTPUT);  // ตั้งค่าขา LED เป็น output
}

void loop() {
  val = analogRead(analogPin);  // อ่านค่าความร้อนจากขา analog
  Serial.print("val = ");       // พิมพ์ข้อความ "val = "
  Serial.println(val);          // พิมพ์ค่าความร้อน

  if (val > 500 && !alert) {         // ตรวจสอบทั้ง val และ alert
    digitalWrite(ledPin, HIGH);      // เปิด LED
    LINE.notify("อุณหภูมิ เกินกำหนด!");  // แจ้งเตือนผ่านไลน์
    alert = true;                    // เปลี่ยนสถานะเป็น true
  } else {
    digitalWrite(ledPin, LOW);  // ปิด LED
    alert = false;              // รีเซ็ตสถานะเป็น false
  }
  delay(100);  // รอ 100 มิลลิวินาที
}
