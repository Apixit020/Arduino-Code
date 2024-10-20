#include <Servo.h>

// กำหนดพินสำหรับเซ็นเซอร์ Ultrasonic และ Servo
const int trigPin = 13;
const int echoPin = 12;
const int servoPin = 6;
int conveyor = 2;

Servo myServo;  // สร้างอ็อบเจกต์ servo

long duration;
int distance;

// กำหนดค่าระยะทางที่ต้องการเปิดปิดฝาถังขยะ
const int distanceThreshold = 8;  // หน่วยเป็นเซนติเมตร

void setup() {
  Serial.begin(115200);
  myServo.attach(servoPin);

  pinMode(13, OUTPUT);  // กำหนด pin 13 เป็น Output สำหรับควบคุม LED

  //conveyor
  pinMode(conveyor, OUTPUT);

  //ส่งสํญญาณคอนเวย์เยอร์
  digitalWrite(conveyor, 0);

  // ตั้งค่าพินสำหรับ ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // เริ่มต้น Servo และตั้งมุมเริ่มต้นที่ฝาปิด
  myServo.write(0);  // ตั้งให้ฝาปิด
}

void loop() {

  // ส่งสัญญาณเสียงออกจากเซ็นเซอร์
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // รับสัญญาณเสียงกลับและคำนวณระยะทาง
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  // แสดงระยะทางบน Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "Red") {
      digitalWrite(13, 1);
      digitalWrite(conveyor, 1);
      myServo.write(180);  // เปิดฝาถังที่มุม 90 องศา
      // myServo.write(0);    // เปิดฝาถ
    } else if (command == "Yellow") {
      digitalWrite(13, 1);
      digitalWrite(conveyor, 1);

      myServo.write(45);  // เปิดฝาถังที่มุม 90 องศา
      // myServo.write(0);    // เปิดฝาถ
    } else if (command == "Green") {
      digitalWrite(13, 1);
    digitalWrite(conveyor, 1);

      myServo.write(90);  // เปิดฝาถังที่มุม 90 องศา
      // myServo.write(0);    // เปิดฝาถ
    }
  }

  // if (distance < distanceThreshold) {
  //   // ถ้าระยะห่างน้อยกว่า ให้หมุนไป 90 องศา
  //   digitalWrite(conveyor, 1);
  //   // digitalWrite(13, 1)





  // }
  else {
    digitalWrite(conveyor, 0);
    digitalWrite(13, 0);
    myServo.write(0);  // เปิดฝาถังที่มุม 90 องศา
  }

  delay(500);  // รอ 0.5 วินาที ก่อนเช็กครั้งต่อไป
}