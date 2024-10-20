#define BLYNK_TEMPLATE_ID "TMPL6OOZ3b4IE"
#define BLYNK_TEMPLATE_NAME "golf"
#define BLYNK_AUTH_TOKEN "eKOVJi-amvIsaJCesgMR1rxILR0ZrvSL"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "iPhone";
char pass[] = "01072534";

//motorL
#define DRIVE1_IN1 16  // H-Bridge input pins
#define DRIVE1_IN2 17

//motorR
#define DRIVE2_IN1 18  // H-Bridge pins for second motor
#define DRIVE2_IN2 19

//ตรวจจับเส้น
//เขียวซ้าย ม่วงกลาง น้ำเงินขวา
#define MID_IR 34
#define LEFT_IR 35
#define RIGHT_IR 36

#define TRAY1_IN1 26
#define TRAY1_IN2 27
#define TRAY2_IN1 12
#define TRAY2_IN2 14

#define sw 5

int mid_val = 2900;

int station = 0;
bool stationReached[4] = { false, false, false, false };
int buttonState = 1;

BLYNK_WRITE(V1) {
  if (!stationReached[1]) {
    int pinValue = param.asInt();
    if (pinValue == 1) {
      Vstanby();
      uturn();
      Serial.println("Vstanby");
      stationReached[1] = true;
      // stationReached[4] = false;
      station = 2;
    }
  }
}

BLYNK_WRITE(V2) {
  if (!stationReached[2]) {  // ตรวจสอบว่ายังไม่ถึง station 2
    int pinValue = param.asInt();
    if (pinValue == 1) {
      Vstation1();
      uturn();

      Serial.println("Vstation1");
      stationReached[2] = true;
      // stationReached[1] = false;
      station = 3;

      unsigned long startTime = millis();
      while (buttonState == 1 && (millis() - startTime) < 10000) {  // รอ 20 วินาทีหรือจนกว่าปุ่มจะถูกกด
        buttonState = digitalRead(sw);
        Blynk.run();
      }

      if (buttonState == 0) {
        unsigned long startTime = millis();
        while (millis() - startTime < 8000) {
          openTray2();
          Blynk.run();
        }
        stopTray();
        startTime = millis();
        while (millis() - startTime < 10000) {
          Blynk.run();
        }
        startTime = millis();
        while (millis() - startTime < 8000) {
          closeTray2();
          Blynk.run();
        }
        stopTray();
      }
      V_back_to_stanby1();
      uturn();
      station = 1;
      stationReached[2] = false;
      // stationReached[1] = true;
    }
    Serial.print("V2 Slider value is: ");
    Serial.println(pinValue);
  }
}

BLYNK_WRITE(V3) {
  if (!stationReached[3]) {  // ตรวจสอบว่ายังไม่ถึง station 2
    int pinValue = param.asInt();
    if (pinValue == 1) {
      Vstation2();
      uturn();
      Serial.println("Vstation2");
      stationReached[3] = true;
      // stationReached[1] = false;
      station = 4;

      unsigned long startTime = millis();
      while (buttonState == 1 && (millis() - startTime) < 10000) {  // รอ 20 วินาทีหรือจนกว่าปุ่มจะถูกกด
        buttonState = digitalRead(sw);
        Blynk.run();
      }

      if (buttonState == 0) {
        unsigned long startTime = millis();
        while (millis() - startTime < 10000) {
          openTray1();
          Blynk.run();
        }
        stopTray();
        startTime = millis();
        while (millis() - startTime < 10000) {
          Blynk.run();
        }
        startTime = millis();
        while (millis() - startTime < 10000) {
          closeTray1
          ();
          Blynk.run();
        }
        stopTray();
      }
      V_back_to_stanby2();
      uturn();
      station = 1;
      stationReached[3] = false;
      // stationReached[1] = true;
    }
    Serial.print("V3 Slider value is: ");
    Serial.println(pinValue);
  }
}

BLYNK_WRITE(V4) {
  // if (!stationReached[4]) {
  int pinValue = param.asInt();
  if (pinValue == 1) {
    V_back_to_home();
    uturn();
    Serial.println("Vstanby");
    // stationReached[4] = true;
    stationReached[1] = false;
    station = 1;
    // }
    Serial.print("V4 Slider value is: ");
    Serial.println(pinValue);
  }
  // }
}

void openTray1() {
  digitalWrite(TRAY1_IN1, 0);
  digitalWrite(TRAY1_IN2, 1);
}

void closeTray1() {
  digitalWrite(TRAY1_IN1, 1);
  digitalWrite(TRAY1_IN2, 0);
}

void openTray2() {
  digitalWrite(TRAY2_IN1, 0);
  digitalWrite(TRAY2_IN2, 1);
}

void closeTray2() {
  digitalWrite(TRAY2_IN1, 1);
  digitalWrite(TRAY2_IN2, 0);
}

void stopTray() {
  digitalWrite(TRAY1_IN1, 0);
  digitalWrite(TRAY1_IN2, 0);
  digitalWrite(TRAY2_IN1, 0);
  digitalWrite(TRAY2_IN2, 0);
}

void driveMotors(int m1, int m2, int m3, int m4) {
  analogWrite(DRIVE1_IN1, m1);
  analogWrite(DRIVE1_IN2, m2);
  analogWrite(DRIVE2_IN1, m3);
  analogWrite(DRIVE2_IN2, m4);
}

void forward() {
  driveMotors(0, 255, 0, 255);
}

void goback() {
  driveMotors(255, 0, 255, 0);
}

void left() {
  driveMotors(255, 0, 0, 255);
}

void right() {
  driveMotors(0, 255, 255, 0);
}

void stop() {
  driveMotors(0, 0, 0, 0);
}

void stop_station() {
  while (1) {
    int val1 = analogRead(LEFT_IR);
    int val2 = analogRead(MID_IR);
    int val3 = analogRead(RIGHT_IR);

    Serial.print(val1);
    Serial.print("\t");
    Serial.print(val2);
    Serial.print("\t");
    Serial.println(val3);

    if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) < mid_val)  //LEFT_IR_G32 and RIGHT_IR_G25 not on line
    {
      Serial.println("move forward");
      forward();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) < mid_val)  //Sharp Left
    {
      left();
      // Serial.println("Sharp Left");
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) > mid_val)  //Sharp Right
    {
      // Serial.println("Sharp Right");
      right();
    }

    else if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) > mid_val) {
      stop();
      // uturn();
      break;
    }
    // delay(50);
  }
}

void stop_station2() {
  while (1) {
    int val1 = analogRead(LEFT_IR);
    int val2 = analogRead(MID_IR);
    int val3 = analogRead(RIGHT_IR);

    Serial.print(val1);
    Serial.print("\t");
    Serial.print(val2);
    Serial.print("\t");
    Serial.println(val3);

    if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) < mid_val)  //LEFT_IR_G32 and RIGHT_IR_G25 not on line
    {
      Serial.println("move forward");
      forward();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) < mid_val)  //Sharp Left
    {
      left();
      // Serial.println("Sharp Left");
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) > mid_val)  //Sharp Right
    {
      // Serial.println("Sharp Right");
      right();
    }

    else if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) > mid_val) {
      stop();
      // uturn();
      break;
    }
    // delay(50);
  }
}

void cross_stop() {
  while (1) {
    int val1 = analogRead(LEFT_IR);
    int val2 = analogRead(MID_IR);
    int val3 = analogRead(RIGHT_IR);

    Serial.print(val1);
    Serial.print("\t");
    Serial.print(val2);
    Serial.print("\t");
    Serial.println(val3);

    if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) < mid_val)  //LEFT_IR_G32 and RIGHT_IR_G25 not on line
    {
      Serial.println("move forward");
      forward();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) < mid_val)  //Sharp Left
    {
      left();
      // Serial.println("Sharp Left");
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) > mid_val)  //Sharp Right
    {
      // Serial.println("Sharp Right");
      right();
    }

    else if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) > mid_val) {
      forward();
      delay(300);
      left();
      while (analogRead(LEFT_IR) < mid_val)
        ;
      while (analogRead(LEFT_IR) > mid_val)
        ;
      delay(650);
      V_back_to_stanby1();
      break;
    }
    // delay(50);
  }
}

void cross_stopST3() {
  while (1) {
    int val1 = analogRead(LEFT_IR);
    int val2 = analogRead(MID_IR);
    int val3 = analogRead(RIGHT_IR);

    Serial.print(val1);
    Serial.print("\t");
    Serial.print(val2);
    Serial.print("\t");
    Serial.println(val3);

    if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) < mid_val)  //LEFT_IR_G32 and RIGHT_IR_G25 not on line
    {
      Serial.println("move forward");
      forward();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) < mid_val)  //Sharp Left
    {
      Serial.println("Sharp Left");
      left();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) > mid_val)  //Sharp Right
    {
      Serial.println("Sharp Right");
      right();
    }

    else if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) > mid_val) {
      forward();
      delay(300);
      right();
      while (analogRead(RIGHT_IR) < mid_val)
        ;
      while (analogRead(RIGHT_IR) > mid_val)
        ;
      delay(400);
      // cross_left();
      break;
    }
    // delay(100);
  }
}

void cross_left() {
  while (1) {
    int val1 = analogRead(LEFT_IR);
    int val2 = analogRead(MID_IR);
    int val3 = analogRead(RIGHT_IR);

    Serial.print(val1);
    Serial.print("\t");
    Serial.print(val2);
    Serial.print("\t");
    Serial.println(val3);

    if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) < mid_val)  //LEFT_IR_G32 and RIGHT_IR_G25 not on line
    {
      Serial.println("move forward");
      forward();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) < mid_val)  //Sharp Left
    {
      Serial.println("Sharp Left");
      left();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) > mid_val)  //Sharp Right
    {
      Serial.println("Sharp Right");
      right();
    } else if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) < mid_val) {
      forward();
      delay(300);
      left();
      while (analogRead(LEFT_IR) < mid_val)
        ;
      while (analogRead(LEFT_IR) > mid_val)
        ;
      delay(400);
      stop_station();
      break;
    }
    // delay(50);
  }
}

void cross_right() {
  while (1) {
    if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) < mid_val)  //LEFT_IR_G32 and RIGHT_IR_G25 not on line
    {
      Serial.println("move forward");
      forward();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) < mid_val)  //Sharp Left
    {
      Serial.println("Sharp Left");
      left();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) > mid_val)  //Sharp Right
    {
      Serial.println("Sharp Right");
      right();
    }

    else if (analogRead(MID_IR) > mid_val && (analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) < mid_val) || (analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) > mid_val)) {
      forward();
      delay(200);
      right();
      while (analogRead(RIGHT_IR) < mid_val)
        ;
      while (analogRead(RIGHT_IR) > mid_val)
        ;
      stop();
      break;
    }
  }
}

void uturn() {
  Serial.println("Uturn");
  goback();
  delay(400);
  right();
  while (analogRead(RIGHT_IR) < mid_val)
    ;
  while (analogRead(RIGHT_IR) > mid_val)
    ;
  delay(400);
  stop();
}

void setup() {
  pinMode(MID_IR, INPUT);
  pinMode(LEFT_IR, INPUT);
  pinMode(RIGHT_IR, INPUT);

  pinMode(DRIVE1_IN1, OUTPUT);
  pinMode(DRIVE1_IN2, OUTPUT);

  pinMode(DRIVE2_IN1, OUTPUT);
  pinMode(DRIVE2_IN2, OUTPUT);

  pinMode(TRAY1_IN1, OUTPUT);
  pinMode(TRAY1_IN2, OUTPUT);
  pinMode(TRAY2_IN1, OUTPUT);
  pinMode(TRAY2_IN2, OUTPUT);

  pinMode(sw, INPUT_PULLUP);
  stopTray();
  Serial.begin(115200);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

void loop() {
  Blynk.run();
  int value = digitalRead(sw);
  Serial.println(value);
  Blynk.virtualWrite(V0, station);
}

// home --> stanby
void Vstanby() {

  while (1) {
    int val1 = analogRead(LEFT_IR);
    int val2 = analogRead(MID_IR);
    int val3 = analogRead(RIGHT_IR);

    Serial.print(val1);
    Serial.print("\t");
    Serial.print(val2);
    Serial.print("\t");
    Serial.println(val3);

    if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) < mid_val)  //LEFT_IR_G32 and RIGHT_IR_G25 not on line
    {
      Serial.println("move forward");
      forward();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) < mid_val)  //Sharp Left
    {
      left();
      Serial.println("Sharp Left");
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) > mid_val)  //Sharp Right
    {
      Serial.println("Sharp Right");
      right();
    } else if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) < mid_val) {
      forward();
      delay(300);
      left();
      while (analogRead(LEFT_IR) < mid_val)
        ;
      while (analogRead(LEFT_IR) > mid_val)
        ;
      delay(400);
      // stop();
      // delay(1000);
      stop_station();
      break;
    }
  }
}

//stanby --> station1
void Vstation1() {
  while (1) {
    int val1 = analogRead(LEFT_IR);
    int val2 = analogRead(MID_IR);
    int val3 = analogRead(RIGHT_IR);

    Serial.print(val1);
    Serial.print("\t");
    Serial.print(val2);
    Serial.print("\t");
    Serial.println(val3);

    if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) < mid_val)  //LEFT_IR_G32 and RIGHT_IR_G25 not on line
    {
      Serial.println("move forward");
      forward();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) < mid_val)  //Sharp Left
    {
      Serial.println("Sharp Left");
      left();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) > mid_val)  //Sharp Right
    {
      Serial.println("Sharp Right");
      right();
    } else if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) > mid_val) {
      forward();
      delay(300);
      left();
      while (analogRead(LEFT_IR) < mid_val)
        ;
      while (analogRead(LEFT_IR) > mid_val)
        ;
      delay(650);
      stop_station2();
      break;
    }
  }
}

//stanby --> station2
void Vstation2() {
  while (1) {
    int val1 = analogRead(LEFT_IR);
    int val2 = analogRead(MID_IR);
    int val3 = analogRead(RIGHT_IR);

    Serial.print(val1);
    Serial.print("\t");
    Serial.print(val2);
    Serial.print("\t");
    Serial.println(val3);

    if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) < mid_val)  //LEFT_IR_G32 and RIGHT_IR_G25 not on line
    {
      Serial.println("move forward");
      forward();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) < mid_val)  //Sharp Left
    {
      Serial.println("Sharp Left");
      left();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) > mid_val)  //Sharp Right
    {
      Serial.println("Sharp Right");
      right();
    } else if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) > mid_val) {
      forward();
      delay(300);
      left();
      while (analogRead(LEFT_IR) < mid_val)
        ;
      while (analogRead(LEFT_IR) > mid_val)
        ;
      delay(650);
      cross_stopST3();
      cross_left();
      break;
    }
  }
}

void V_back_to_home() {
  while (1) {
    int val1 = analogRead(LEFT_IR);
    int val2 = analogRead(MID_IR);
    int val3 = analogRead(RIGHT_IR);

    Serial.print(val1);
    Serial.print("\t");
    Serial.print(val2);
    Serial.print("\t");
    Serial.println(val3);

    if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) < mid_val)  //LEFT_IR_G32 and RIGHT_IR_G25 not on line
    {
      Serial.println("move forward");
      forward();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) < mid_val)  //Sharp Left
    {
      left();
      // Serial.println("Sharp Left");
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) > mid_val)  //Sharp Right
    {
      // Serial.println("Sharp Right");
      right();
    } else if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) > mid_val) {
      forward();
      delay(300);
      right();
      while (analogRead(RIGHT_IR) < mid_val)
        ;
      while (analogRead(RIGHT_IR) > mid_val)
        ;
      delay(650);
      // stop();
      // delay(1000);
      stop_station2();
      station = 0;
      break;
    }
  }
}

void V_back_to_stanby1() {
  while (1) {
    int val1 = analogRead(LEFT_IR);
    int val2 = analogRead(MID_IR);
    int val3 = analogRead(RIGHT_IR);

    Serial.print(val1);
    Serial.print("\t");
    Serial.print(val2);
    Serial.print("\t");
    Serial.println(val3);

    if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) < mid_val)  //LEFT_IR_G32 and RIGHT_IR_G25 not on line
    {
      Serial.println("move forward");
      forward();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) < mid_val)  //Sharp Left
    {
      Serial.println("Sharp Left");
      left();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) > mid_val)  //Sharp Right
    {
      Serial.println("Sharp Right");
      right();
    } else if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) < mid_val)  //Sharp Right
    {
      Serial.println("Sharp Right");
      forward();
    } else if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) > mid_val) {
      forward();
      delay(300);
      right();
      while (analogRead(RIGHT_IR) < mid_val)
        ;
      while (analogRead(RIGHT_IR) > mid_val)
        ;
      delay(400);
      stop_station();
      // uturn();
      break;
    }
  }
}

void V_back_to_stanby2() {
  while (1) {
    int val1 = analogRead(LEFT_IR);
    int val2 = analogRead(MID_IR);
    int val3 = analogRead(RIGHT_IR);

    Serial.print(val1);
    Serial.print("\t");
    Serial.print(val2);
    Serial.print("\t");
    Serial.println(val3);

    if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) < mid_val)  //LEFT_IR_G32 and RIGHT_IR_G25 not on line
    {
      Serial.println("move forward");
      forward();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) > mid_val && analogRead(RIGHT_IR) < mid_val)  //Sharp Left
    {
      Serial.println("Sharp Left");
      left();
    } else if (analogRead(MID_IR) < mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) > mid_val)  //Sharp Right
    {
      Serial.println("Sharp Right");
      right();
    } else if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) > mid_val) {
      forward();
      delay(300);
      right();
      while (analogRead(RIGHT_IR) < mid_val)
        ;
      while (analogRead(RIGHT_IR) > mid_val)
        ;
      delay(400);
      cross_stop();
      forward();
      delay(200);
      left();
      while (analogRead(LEFT_IR) < mid_val)
        ;
      while (analogRead(LEFT_IR) > mid_val)
        ;
      if (analogRead(MID_IR) > mid_val && analogRead(LEFT_IR) < mid_val && analogRead(RIGHT_IR) > mid_val) {
        forward();
        delay(200);
        right();
        while (analogRead(RIGHT_IR) < mid_val)
          ;
        while (analogRead(RIGHT_IR) > mid_val)
          ; 
        delay(300);
        forward();
        delay(200);
        stop_station();
        // uturn();
      }
      break;
    }
  }
}