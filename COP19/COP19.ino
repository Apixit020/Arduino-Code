#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

Servo myservo;  //ประกาศตัวแปรแทน Servo

int ir = 5;

int point = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(ir, INPUT);
  Serial.begin(9600);
  myservo.attach(13);
  myservo.write(0);
  lcd.begin();
  lcd.backlight();
}

void loop() {
  // put your main code here, to run repeatedly:
  int val = digitalRead(ir);
  Serial.println(val);
  lcd.setCursor(0, 0);
  lcd.print("Point : ");
  lcd.setCursor(7, 0);
  lcd.print(point);
  if (val == 1) {
    myservo.write(180);
    point++;
  } else {
    myservo.write(0);
  }delay (1000);
}
