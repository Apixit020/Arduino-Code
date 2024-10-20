#include <Servo.h>
Servo myservo;  //ประกาศตัวแปรแทน Servo

int pushButton1 = 2;
int pushButton2 = 4;
int pushButton3 = 3;

void setup() {
  Serial.begin(9600);
  pinMode(pushButton1, INPUT);
  pinMode(pushButton2, INPUT);
  pinMode(pushButton3, INPUT);
  myservo.attach(6);  // กำหนดขา 9 ควบคุม Servo
}

void loop() {
  int buttonState1 = digitalRead(pushButton1);
  int buttonState2 = digitalRead(pushButton2);
  int buttonState3 = digitalRead(pushButton3);


  Serial.print(buttonState1);
  Serial.print("\t");
  Serial.print(buttonState3);
  Serial.print("\t");
  Serial.println(buttonState2);
  delay(1);  // delay in between reads for stability
  if (buttonState1 == 1) {
    myservo.write(180);  // สั่งให้ Servo หมุนไปองศาที่ 180
    delay(1000);         // หน่วงเวลา 1000ms

  } else {
    myservo.write(0);  // สั่งให้ Servo หมุนไปองศาที่ 0
  }
}
