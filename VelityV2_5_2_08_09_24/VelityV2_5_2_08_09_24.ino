#include <Nextion.h>       // ไลบรารีสำหรับการสื่อสารกับหน้าจอ Nextion
#define nexSerial Serial2  // กำหนด nexSerial ให้เป็น Serial2 สำหรับการสื่อสารกับ Nextion
#include <EEPROM.h>        // ไลบรารีสำหรับการใช้งาน EEPROM
#include <SPI.h>           // ไลบรารีสำหรับการสื่อสาร SPI
#include <MFRC522.h>       // ไลบรารีสำหรับการใช้งาน RFID MFRC522
#include <Wire.h>          // ไลบรารีสำหรับการสื่อสาร I2C
#include <NexButton.h>     // ไลบรารีสำหรับการใช้งานปุ่มกับ Nextion HMI
#include <NexTouch.h>      // ไลบรารีสำหรับการจัดการอีเวนต์การสัมผัสกับ Nextion HMI
#include <NexHardware.h>   // ไลบรารีสำหรับการเชื่อมต่อกับฮาร์ดแวร์ของ Nextion
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;                      // ที่อยู่ I2C ของเซ็นเซอร์ MPU6050
float ax, ay, az;                          // ตัวแปรเก็บค่า Accelerometer ในแกน X, Y, Z
float gx, gy, gz;                          // ตัวแปรเก็บค่า Gyroscope ในแกน X, Y, Z
float angleX, angleY;                      // ตัวแปรสำหรับเก็บค่าองศาที่คำนวณได้จาก Accelerometer
const int address = 1;                     // ที่อยู่ในการเก็บค่าใน EEPROM
const int address_distance = sizeof(int);  // ที่อยู่ในการเก็บค่าใน EEPROM

// Define constants for sensor settings and calculations
#define GYRO_SCALE 250.0                     // Full scale range of the gyro
#define ACCEL_SCALE 2.0                      // Full scale range of the accelerometer
#define FILTER_BANDWIDTH MPU6050_BAND_21_HZ  // Filter bandwidth


// Variables for Kalman filter (optional)
float Q_angle = 0.001;        // Process noise covariance
float Q_bias = 0.003;         // Process noise covariance for bias
float R_measure = 0.03;       // Measurement noise covariance
float P[4] = { 0, 0, 0, 0 };  // Error covariance matrix
float K[4] = { 0, 0, 0, 0 };  // Kalman gain
float y, S, K_angle, K_bias;
float angle_dot, bias;

constexpr uint8_t RST_PIN = -1;  // พินรีเซ็ตสำหรับโมดูล RFID
constexpr uint8_t SS_PIN = 17;   // พิน Slave Select สำหรับโมดูล RFID
MFRC522 rfid(SS_PIN, RST_PIN);   // สร้างออบเจ็กต์สำหรับการใช้งาน RFID
String tag = "";                 // ตัวแปรสำหรับเก็บ UID ของแท็ก RFID

int j = 1;                                // ตัวแปรสำหรับตรวจสอบสถานะการเปลี่ยนหน้าใน Nextion
volatile unsigned int temp, counter = 0;  // ตัวแปรสำหรับการนับรอบและอุณหภูมิ (ถ้ามีการใช้งาน)
unsigned long time_cycle = 0;
unsigned long time_cycle1 = 0;
unsigned long start = 0;
unsigned long start1 = 0;

int Aceletor = A9;  // พินสำหรับอ่านค่าจาก Accelerator (คันเร่ง)
int buzzer = 22;    // พินสำหรับควบคุมเสียง Buzzer
int battery = A2;   // พินสำหรับอ่านค่าระดับแบตเตอรี่
int pbattery, RC;   // ตัวแปรสำหรับเก็บค่าระดับแบตเตอรี่และอื่น ๆ
int period = 200;   // ช่วงเวลาสำหรับการอัพเดทข้อมูล (หน่วยเป็นมิลลิวินาที)
int RPM_IN = 5;
int sensorValue = 0;  // ตัวแปรเก็บค่าอนาล็อก

int bat, ModeR, ModeF;
int RPM;
int KMTT = 0;
const int numReadings = 10;  // จำนวนค่าที่นำมาคำนวณค่าเฉลี่ย
int readings[numReadings];   // อาร์เรย์เก็บค่า RPM
int indexq = 0;              // ตัวชี้ตำแหน่งในอาร์เรย์
int total = 0;               // ผลรวมของค่าทั้งหมดในอาร์เรย์
float average = 0;           // ค่าเฉลี่ยของค่า RPM
bool new_cycle = true;
bool new_cycle1 = true;
bool state = false;
int RealAce;
int my_vel, KWR;
float KMT, KMT1;
int KMT2, KMT3;
int ResetKMT1;
bool aa = 0;
int LM = 0;
int Race = 0;
int M = 0;

unsigned long buttonPressTime = 0;  // เวลาเมื่อกดปุ่ม
bool buttonHeld = false;            // ตรวจสอบว่าปุ่มถูกกดค้างไว้หรือไม่
bool isRacingMode = false;          // สถานะโหมด Racing
// กำหนดพินสำหรับปุ่มเพิ่มและลดโหมด
const int buttonUpPin = 3;
const int buttonDownPin = 4;
const int RacingPin = 2;
// ตัวแปรสำหรับเก็บสถานะของปุ่ม
int lastButtonUpState = HIGH;
int lastButtonDownState = HIGH;
int lastButtonRacingState = HIGH;

unsigned long pressTime = 0;             // เวลาเมื่อสวิตช์ถูกกด
unsigned long longPressDuration = 2000;  // ระยะเวลาในการกดค้าง (2 วินาที)

// โหมดการขับขี่ที่มีอยู่
const char *modes[] = { "P", "LM", "EC", "S", "R" };  //รอปรึกษาว่าโหมด R ต้องใส่ตรงไหน
int currentModeIndex = 0;                             // เริ่มต้นที่โหมด P
int previousModeIndex = 0;

bool correctCardScanned = false;
bool onMainPage = false;

volatile int hallCount = 0;  // ตัวนับจำนวนครั้งที่ Hall sensor ส่งสัญญาณ
unsigned long previousMillis = 0;
float rpm;
const float wheelCircumference = 0.878;  // ปรับค่าให้ตรงกับเส้นรอบวงจริง

int maxSpeed = 0;    // ตัวแปรเก็บค่าความเร็วสูงสุด
float currentSpeed;  // ตัวแปรเก็บค่าความเร็วที่คำนวณได้ในแต่ละครั้ง
int km;
int hall;
// สร้างวัตถุปุ่มสำหรับ Nextion
NexButton modeToggle1 = NexButton(4, 5, "ModeToggle1");  // ปุ่มที่หน้า page 4, ID 6
NexButton enterMain = NexButton(2, 29, "enter");         // ปุ่มที่หน้า page 4, ID 6
NexButton ResetTrip = NexButton(4, 2, "Reset");          // ปุ่มที่หน้า page 4, ID 6
NexNumber rpm1 = NexNumber(4, 14, "RPM");
NexNumber Vbattery1 = NexNumber(4, 17, "Vbattery");
NexNumber speed1 = NexNumber(4, 15, "Speed");
NexNumber Trip1 = NexNumber(4, 19, "Trip");
NexNumber maxSpeed1 = NexNumber(4, 16, "Maxspd");
NexNumber state1 = NexNumber(2, 28, "state");
NexText nexMode1 = NexText(4, 6, "Gears");
NexPage main_Page = NexPage(4, 0, "MAIN");  //65 05 06 01 FF FF FF
NexPage input_Page = NexPage(2, 0, "PIN_INPUT");
NexPicture gyro = NexPicture(4, 4, "GyroLine");
NexTimer check = NexTimer(2, 21, "Check");

// gyro 4,4 "GyroLine"
// NexPage main_Racing_Page = NexPage(5, 0, "Racing_main");  // 65 04 06 01 FF FF FF
int storedMaxSpeed;
int storeKm;
float totalDistance = 0;  // ตัวแปรเก็บระยะทางสะสม (เมตร)


//MAIN
//Racing_main

// ลิสต์ของอีเวนต์ที่ Nextion จะรับฟัง
NexTouch *nex_listen_list[] = {
  &modeToggle1,
  &enterMain,
  &main_Page,
  &gyro,
  NULL  // ต้องสิ้นสุดด้วย NULL
};

void checkCallback(void *ptr) {
  Serial.println("Timer event triggered");
  // สามารถเพิ่มเงื่อนไขอื่น ๆ ตามที่ต้องการได้
}

void setup() {
  nexSerial.begin(115200);  // เริ่มการสื่อสารผ่าน Serial2 ด้วยความเร็ว 115200 bps
  Serial.begin(115200);     // เริ่มการสื่อสารผ่าน Serial ด้วยความเร็ว 115200 bps
  SPI.begin();              // เริ่มการสื่อสาร SPI
  rfid.PCD_Init();          // เริ่มต้นโมดูล RFID

  pinMode(buzzer, OUTPUT);        // กำหนดพิน buzzer เป็น OUTPUT
  pinMode(Aceletor, INPUT);       // กำหนดพิน Aceletor เป็น INPUT
  pinMode(battery, INPUT);        // กำหนดพิน battery เป็น INPUT
  pinMode(RPM_IN, INPUT_PULLUP);  // กำหนดพิน 5 เป็น INPUT พร้อมกับเปิด Pull-up resistor เพื่ออ่านค่าจาก FRPM
  pinMode(20, OUTPUT);            // รีเลย์ตัวที่ 1 cutoff
  pinMode(21, OUTPUT);            // รีเลย์ตัวที่ 2 Font Low
  pinMode(0, OUTPUT);             // รีเลย์ตัวที่ 3 Font High
  pinMode(1, OUTPUT);             // รีเลย์ตัวที่ 4 Rear Low
  pinMode(9, OUTPUT);             // รีเลย์ตัวที่ 5 Rear High

  Serial.println("tap you Key");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Set sensor parameters
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(FILTER_BANDWIDTH);
  // Initialize Kalman filter variables (if using)
  bias = 0;
  P[0] = 0;
  P[1] = 0;
  P[2] = 0;
  P[3] = 0;

  nexInit();
  // input_Page.show();
  // ตั้งค่า Interrupt สำหรับพิน 5 (FRPM) เมื่อเกิดสัญญาณ RISING จะเรียกใช้ฟังก์ชัน ai1()
  // attachInterrupt(digitalPinToInterrupt(5), ai1, RISING);
  attachInterrupt(digitalPinToInterrupt(RPM_IN), countRPM, RISING);  // นับเมื่อค่าเปลี่ยนจาก LOW เป็น HIGH
  // สัญญาณเสียงเริ่มต้น
  digitalWrite(buzzer, HIGH);
  delay(500);
  digitalWrite(buzzer, LOW);
  delay(100);
  digitalWrite(buzzer, HIGH);
  delay(500);
  digitalWrite(buzzer, LOW);

  //ค่าเริ่มต้น io ควบคุมสปีดโหมด
  digitalWrite(20, LOW);  //รีเลย์ตัวที่ 1 cutoff
  digitalWrite(21, LOW);  //รีเลย์ตัวที่ 2 Font Low
  digitalWrite(0, LOW);   //รีเลย์ตัวที่ 3 Font High
  digitalWrite(1, LOW);   //รีเลย์ตัวที่ 4 Rear Low
  digitalWrite(9, LOW);   //รีเลย์ตัวที่ 5 Rear High

  // กำหนดพินสำหรับปุ่มโหมด
  pinMode(buttonUpPin, INPUT_PULLUP);
  pinMode(buttonDownPin, INPUT_PULLUP);
  pinMode(RacingPin, INPUT_PULLUP);

  readFromEEPROM();

  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }


  // ตั้งค่าฟังก์ชัน callback เมื่อปุ่มถูกกด
  // check.attachPush(timerCallback);
  modeToggle1.attachPush(modeToggle1PushCallback);  //รอถามสอบการทำงานส่วนนี้
  enterMain.attachPush(enterMainPushCallback);      //รอถามสอบการทำงานส่วนนี้
}
void loop() {
  nexLoop(nex_listen_list);  // ฟังอีเวนต์จากหน้าจอ Nextion
  // Serial.println(digitalRead(RPM_IN));  //508 == 84.56V

  unsigned long currentMillis = millis();

  if (onMainPage || correctCardScanned) {
    // Gyro();
    // delay(1000);
    Trip1.setValue(km);

    if (currentMillis - previousMillis >= 833) {  // คำนวณ RPM ทุก 1 วินาที
      if (hallCount == 0) {
        rpm = 0;
      } else {
        rpm = (hallCount * 60.0) / 15.0;
      }
      hallCount = 0;
      previousMillis = currentMillis;
      // คำนวณระยะทางที่ล้อหมุนได้ใน 1 นาที (เมตร)
      total = total - readings[indexq];  // ลบค่าเก่าออก
      total = total + rpm;               // บวกค่าใหม่เข้าไป
      readings[indexq] = rpm;            // อัปเดตค่าในอาร์เรย์
      indexq = indexq + 1;
      if (indexq >= numReadings) { indexq = 0; }  // รีเซ็ต index
      average = total / numReadings;

      float distancePerMinute = average * wheelCircumference;

      // คำนวณระยะทางที่วิ่งได้ในหนึ่งรอบ (เมตร)
      float distancePerRevolution = distancePerMinute / (60 * average);

      // คำนวณความเร็วเป็นกิโลเมตรต่อชั่วโมง
      float speedKmh = distancePerMinute * 60 / 1000;

      Serial.print("RPM: ");
      Serial.print(rpm);
      Serial.print("\t");
      Serial.print("Kmh: ");
      Serial.print(speedKmh);
      Serial.print("\t");
      Serial.print("max speed: ");
      Serial.print(maxSpeed);
      Serial.print("\t");
      Serial.print("hall : ");
      Serial.print(hall);
      Serial.print("\t");
      Serial.print("km : ");
      Serial.println(km);

      rpm1.setValue(average);
      speed1.setValue(speedKmh);
      // Trip1.setValue(500);
      if (speedKmh > maxSpeed) {
        maxSpeed = speedKmh;
        maxSpeed1.setValue(maxSpeed);
        // ... (ส่วนที่เหลือของโค้ดเดิม)
      }

      if (hall > 17084) {
        km++;
        EEPROM.put(address_distance, km);
        hall = 0;
      }
    }

    if (j == 1) {
      digitalWrite(buzzer, HIGH);  // ให้เสียงแจ้งเตือน
      delay(500);
      digitalWrite(buzzer, LOW);
      j = 0;  // เปลี่ยนสถานะเพื่อไม่ให้เข้ามาในเงื่อนไขนี้อีก
    }

    // อ่านสถานะของปุ่มเพิ่มและลดโหมด
    int buttonUpState = digitalRead(buttonUpPin);
    int buttonDownState = digitalRead(buttonDownPin);
    int RacingState = digitalRead(RacingPin);

    // Serial.print(buttonUpState);
    // Serial.print("\t");
    // Serial.print(buttonDownState);
    // Serial.print("\t");
    // Serial.println(RacingState);


    // ตรวจสอบการกดปุ่มเพิ่มโหมด
    if (buttonUpState == LOW && lastButtonUpState == HIGH && currentModeIndex != 4) {
      Race = 0;
      if (currentModeIndex < 3)  // ตรวจสอบว่าไม่เกินโหมดสูงสุด
      {
        currentModeIndex++;
        changeMode(currentModeIndex);  // เปลี่ยนโหมดตาม index ปัจจุบัน
      }
      delay(200);  // ดีเลย์เพื่อป้องกันการกดซ้ำ
    }

    // ตรวจสอบการกดปุ่มลดโหมด
    if (buttonDownState == LOW && lastButtonDownState == HIGH && currentModeIndex != 4) {
      Race = 0;
      if (currentModeIndex > 0)  // ตรวจสอบว่าไม่ต่ำกว่าโหมดต่ำสุด
      {
        currentModeIndex--;
        changeMode(currentModeIndex);  // เปลี่ยนโหมดตาม index ปัจจุบัน
      }
      delay(200);  // ดีเลย์เพื่อป้องกันการกดซ้ำ
    }

    bool buttonPressed = digitalRead(RacingPin) == LOW;  // ตรวจสอบว่าปุ่มถูกกดหรือไม่ (สมมติ LOW คือตอนกด)

    if (buttonPressed && !buttonHeld) {
      // ถ้ากดปุ่มเป็นครั้งแรก
      buttonPressTime = millis();  // บันทึกเวลาเมื่อกดปุ่ม
      buttonHeld = true;           // ระบุว่าปุ่มถูกกดค้างไว้
    }

    if (!buttonPressed && buttonHeld) {
      // ถ้าปล่อยปุ่มหลังจากกดค้างไว้
      buttonHeld = false;  // ปุ่มถูกปล่อยแล้ว
    }

    if (buttonHeld && (millis() - buttonPressTime >= 2000)) {
      // ถ้าปุ่มถูกกดค้างไว้เกิน 2 วินาที
      // Race = 1;
      if (!isRacingMode) {
        // เข้าสู่โหมด Racing
        previousModeIndex = currentModeIndex;
        currentModeIndex = 4;  // โหมด Racing (สมมติว่าโหมด 5 คือ Racing)
        changeMode(currentModeIndex);
        nexMode1.setText(" ");
        nexSerial.print("MAIN.pic=3");  // ส่งคำสั่งไปที่ Nextion display
        nexSerial.write(0xff);          // ส่ง 0xff สามครั้งเพื่อบอกจบคำสั่ง
        nexSerial.write(0xff);
        nexSerial.write(0xff);
        isRacingMode = true;
        // main_Racing_Page.show();
      } else {
        // กลับไปโหมดก่อนหน้า
        currentModeIndex = previousModeIndex;
        nexSerial.print("MAIN.pic=2");  // ส่งคำสั่งไปที่ Nextion display
        nexSerial.write(0xff);          // ส่ง 0xff สามครั้งเพื่อบอกจบคำสั่ง
        nexSerial.write(0xff);
        nexSerial.write(0xff);
        changeMode(currentModeIndex);
        isRacingMode = false;
        // main_Page.show();
      }

      // รีเซ็ตค่าเพื่อไม่ให้สลับโหมดซ้ำจนกว่าจะปล่อยปุ่ม
      buttonHeld = false;
    }

    // อัพเดตสถานะปุ่มล่าสุด
    lastButtonUpState = buttonUpState;
    lastButtonDownState = buttonDownState;

    sensorValue = analogRead(battery);
    float voltage = sensorValue * 1.664;  // สมมติว่า ADC ใช้แรงดันอ้างอิง 5V และมี 1023 ระดับ
    // Serial.print("ค่าอนาล็อก: ");
    // Serial.print(sensorValue);
    // Serial.print("  แรงดัน: ");
    // Serial.print(voltage);
    // Serial.println(" V");
    Vbattery1.setValue(voltage);




    delay(200);

  } else {
    rfid_scan();  // Scan the card if neither card scanned nor on main page
  }
}

void readFromEEPROM() {
  EEPROM.get(address_distance, km);
}

void countRPM() {
  hallCount++;
  hall++;
}

// // ฟังก์ชัน Interrupt สำหรับนับจำนวนรอบ
// void ai1() {
//   counter++;
// }

// ฟังก์ชันสำหรับเปลี่ยนโหมดการขับขี่
void changeMode(int modeIndex) {
  Serial.print("Current Mode: ");
  Serial.println(modes[modeIndex]);

  switch (modeIndex) {
    case 0:
      modeP();
      break;
    case 1:
      modeLM();
      break;
    case 2:
      modeEc();
      break;
    // case 3:
    //   modeD();
    //   break;
    case 3:
      modeS();
      break;
    case 4:
      modeR();
      break;
  }
}

// ฟังก์ชันย่อยสำหรับแต่ละโหมด
void modeP() {
  digitalWrite(20, HIGH);  //รีเลย์ตัวที่ 1 cutoff
  digitalWrite(21, LOW);   //รีเลย์ตัวที่ 2 Font Low
  digitalWrite(0, LOW);    //รีเลย์ตัวที่ 3 Font High
  digitalWrite(1, LOW);    //รีเลย์ตัวที่ 4 Rear Low
  digitalWrite(9, LOW);    //รีเลย์ตัวที่ 5 Rear High

  Serial.println("Mode P: Parking");
  nexMode1.setText("P");
  // nexMode2_PARKING.setText("P");
  // เพิ่มโค้ดเพิ่มเติมสำหรับโหมด Parking ถ้าจำเป็น
}
void modeEc() {
  digitalWrite(20, LOW);  //รีเลย์ตัวที่ 1 cutoff
  digitalWrite(21, LOW);  //รีเลย์ตัวที่ 2 Font Low
  digitalWrite(0, LOW);   //รีเลย์ตัวที่ 3 Font High
  digitalWrite(1, LOW);   //รีเลย์ตัวที่ 4 Rear Low
  digitalWrite(9, LOW);   //รีเลย์ตัวที่ 5 Rear High

  Serial.println("Mode ECO Mode");
  nexMode1.setText("EC");

  // nexMode2_PARKING.setText("EC");


  // เพิ่มโค้ดเพิ่มเติมสำหรับโหมด Low Mode ถ้าจำเป็น
}
void modeLM() {
  digitalWrite(20, LOW);   //รีเลย์ตัวที่ 1 cutoff
  digitalWrite(21, HIGH);  //รีเลย์ตัวที่ 2 Font Low
  digitalWrite(0, LOW);    //รีเลย์ตัวที่ 3 Font High
  digitalWrite(1, HIGH);   //รีเลย์ตัวที่ 4 Rear Low
  digitalWrite(9, LOW);    //รีเลย์ตัวที่ 5 Rear High

  Serial.println("Mode LM: Low Mode");
  nexMode1.setText("LM");

  // nexMode2_PARKING.setText("LM");

  // เพิ่มโค้ดเพิ่มเติมสำหรับโหมด Low Mode ถ้าจำเป็น
}

// void modeD() {
//   digitalWrite(20, LOW);  //รีเลย์ตัวที่ 1 cutoff
//   digitalWrite(21, LOW);  //รีเลย์ตัวที่ 2 Font Low
//   digitalWrite(0, LOW);   //รีเลย์ตัวที่ 3 Font High
//   digitalWrite(1, LOW);   //รีเลย์ตัวที่ 4 Rear Low
//   digitalWrite(9, LOW);   //รีเลย์ตัวที่ 5 Rear High;

//   Serial.println("Mode D: Drive");
//   nexMode1.setText("D");

//   // nexMode2_PARKING.setText("D");
//   // เพิ่มโค้ดเพิ่มเติมสำหรับโหมด Drive ถ้าจำเป็น
// }

void modeS() {
  digitalWrite(20, LOW);  //รีเลย์ตัวที่ 1 cutoff
  digitalWrite(21, LOW);  //รีเลย์ตัวที่ 2 Font Low
  digitalWrite(0, LOW);   //รีเลย์ตัวที่ 3 Font High
  digitalWrite(1, LOW);   //รีเลย์ตัวที่ 4 Rear Low
  digitalWrite(9, HIGH);  //รีเลย์ตัวที่ 5 Rear High

  Serial.println("Mode S: Sport");
  nexMode1.setText("S");

  //  nexMode2_PARKING.setText("S");

  // เพิ่มโค้ดเพิ่มเติมสำหรับโหมด Sport ถ้าจำเป็น
}

void modeR() {
  digitalWrite(20, LOW);  //รีเลย์ตัวที่ 1 cutoff
  digitalWrite(21, LOW);  //รีเลย์ตัวที่ 2 Font Low
  digitalWrite(0, HIGH);  //รีเลย์ตัวที่ 3 Font High
  digitalWrite(1, LOW);   //รีเลย์ตัวที่ 4 Rear Low
  digitalWrite(9, HIGH);  //รีเลย์ตัวที่ 5 Rear High

  Serial.println("Mode R: Race");
  nexMode1.setText(" ");

  // nexMode2_PARKING.setText("");
  // เพิ่มโค้ดเพิ่มเติมสำหรับโหมด Sport ถ้าจำเป็น
}

// ฟังก์ชัน callback เมื่อกดปุ่ม ModeToggle1 บนหน้าจอ Nextion
void modeToggle1PushCallback(void *ptr) {
  if (!isRacingMode) {
    Serial.println("Entering Racing Mode from Nextion Button");
    previousModeIndex = currentModeIndex;
    currentModeIndex = 4;  // โหมด Racing
    isRacingMode = true;
    changeMode(currentModeIndex);
    nexMode1.setText(" ");
    nexSerial.print("MAIN.pic=3");  // ส่งคำสั่งไปที่ Nextion display
    nexSerial.write(0xff);          // ส่ง 0xff สามครั้งเพื่อบอกจบคำสั่ง
    nexSerial.write(0xff);
    nexSerial.write(0xff);
  } else {
    Serial.println("Exiting Racing Mode from Nextion Button");
    currentModeIndex = previousModeIndex;
    isRacingMode = false;
    changeMode(currentModeIndex);
    nexSerial.print("MAIN.pic=2");  // ส่งคำสั่งไปที่ Nextion display
    nexSerial.write(0xff);          // ส่ง 0xff สามครั้งเพื่อบอกจบคำสั่ง
    nexSerial.write(0xff);
    nexSerial.write(0xff);
  }
}
void enterMainPushCallback(void *ptr) {
  aa = 1;
  if (aa == 1) {
    main_Page.show();
    modeP();
    onMainPage = true;
  } else {
    Serial.println("Wrong Password");
  }
}
void rfid_scan() {

  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    tag = rfid_read();
    Serial.println(">>>> " + tag);
    if (tag == "93 E1 C7 9A") {
      main_Page.show();
      modeP();
      correctCardScanned = true;
    } else {
      Serial.println("Key not match");
    }
  }
  delay(1);
}

String rfid_read() {
  String content = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    content.concat(String(rfid.uid.uidByte[i] < 0x10 ? " 0" : " "));
    content.concat(String(rfid.uid.uidByte[i], HEX));
  }
  content.toUpperCase();
  return content.substring(1);
}

void Gyro() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  float x = a.acceleration.x;

  // Map the acceleration value (-1g to +1g) to degrees (0° to 180°)
  int degrees = map(x * 100, -100, 100, 0, 180);

  Serial.print("Tilt Degrees: ");
  Serial.println(degrees);
  // if (angleX > 175) {
  //   gyro.setPic(60);
  // } else if (angleX > 165 && angleX < 175) {
  //   gyro.setPic(59);
  // } else if (angleX > 145 && angleX < 155) {
  //   gyro.setPic(58);
  // } else if (angleX > 135 && angleX < 145) {
  //   gyro.setPic(57);
  // } else if (angleX > 125 && angleX < 135) {
  //   gyro.setPic(56);
  // } else if (angleX > 115 && angleX < 125) {
  //   gyro.setPic(55);
  // } else if (angleX > 105 && angleX < 115) {
  //   gyro.setPic(54);
  // } else if (angleX > 95 && angleX < 105) {
  //   gyro.setPic(53);
  // } else if (angleX > 85 && angleX < 95) {
  //   gyro.setPic(52);
  // } else if (angleX > 75 && angleX < 85) {
  //   gyro.setPic(51);
  // } else if (angleX > 65 && angleX < 75) {
  //   gyro.setPic(50);
  // } else if (angleX > 55 && angleX < 65) {
  //   gyro.setPic(49);
  // } else if (angleX > 45 && angleX < 55) {
  //   gyro.setPic(48);
  // } else if (angleX > 35 && angleX < 45) {
  //   gyro.setPic(47);
  // } else if (angleX > 25 && angleX < 35) {
  //   gyro.setPic(46);
  // } else if (angleX > 15 && angleX < 25) {
  //   gyro.setPic(45);
  // } else {
  //   gyro.setPic(44);
  // }
}
