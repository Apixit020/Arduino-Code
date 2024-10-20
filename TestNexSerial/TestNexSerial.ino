#include <Nextion.h>

#define nexSerial Serial3

void setup() {
  nexSerial.begin(115200);  // ตั้งค่า Baud Rate เป็น 9600 bps
  Serial.begin(115200);   // ตั้งค่า Baud Rate ของ Serial Monitor

  delay(1000);  // รอการเริ่มต้นของจอ

  // ส่งคำสั่งง่ายๆ ไปยังจอ
  nexSerial.print("page 4");
  nexSerial.write(0xFF);
  nexSerial.write(0xFF);
  nexSerial.write(0xFF);

  Serial.println("Sent command to Nextion display");
}

void loop() {
  if (nexSerial.available()) {
    Serial.write(nexSerial.read());  // อ่านข้อมูลจากจอและแสดงบน Serial Monitor
  }
}
