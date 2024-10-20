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

Adafruit_MPU6050 mpu;  // ที่อยู่ I2C ของเซ็นเซอร์ MPU6050
float ax, ay, az;      // ตัวแปรเก็บค่า Accelerometer ในแกน X, Y, Z
float gx, gy, gz;      // ตัวแปรเก็บค่า Gyroscope ในแกน X, Y, Z
float angleX, angleY;  // ตัวแปรสำหรับเก็บค่าองศาที่คำนวณได้จาก Accelerometer


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
MFRC522::MIFARE_Key key;         // สร้างคีย์สำหรับการตรวจสอบ RFID
String tag;                      // ตัวแปรสำหรับเก็บ UID ของแท็ก RFID

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

const int numReadings = 20;               // จำนวนการอ่านค่าสำหรับการคำนวณค่าเฉลี่ย
const int TachometerDeadzoneSamples = 2;  // ค่าตัวอย่างสำหรับ Deadzone ของ Tachometer
const int TachLimitAmount = 20;           // ขีดจำกัดสำหรับ Tachometer
int TachometerWithDeadzone;
int TachLimitCounter1 = 0;
int TachLimitCounter2 = 0;
int TachometerRemaped;
int bat, ModeR, ModeF;
int RPM;
int KMTT = 0;
int RealRPM;
int readings[numReadings];  // อาเรย์สำหรับเก็บค่าการอ่านเพื่อคำนวณค่าเฉลี่ย
int readIndex = 0;
long total = 0;
int average = 0;
bool new_cycle = true;
bool new_cycle1 = true;
bool state = false;
int RealAce;
int my_vel, KWR;
float KMT, KMT1;
int KMT2, KMT3;
int ResetKMT1;
int aa = 0;
int LM = 0;
int Race = 0;
int M = 0;

unsigned long buttonPressTime = 0;  // เวลาเมื่อกดปุ่ม
bool buttonHeld = false;            // ตรวจสอบว่าปุ่มถูกกดค้างไว้หรือไม่
bool isRacingMode = false;          // สถานะโหมด Racing
// int currentModeIndex = 0;
int previousModeIndex = 0;
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
const char *modes[] = { "P", "LM", "EC", "D", "S", "R" };  //รอปรึกษาว่าโหมด R ต้องใส่ตรงไหน
int currentModeIndex = 0;                                  // เริ่มต้นที่โหมด P

// สร้างวัตถุปุ่มสำหรับ Nextion
NexButton modeToggle1 = NexButton(4, 5, "ModeToggle1");  // ปุ่มที่หน้า page 4, ID 6
NexNumber rpm1 = NexNumber(4, 14, "RPM");
NexNumber Vbattery1 = NexNumber(4, 17, "Vbattery");
NexText nexMode1 = NexText(4, 6, "Gears");
NexPage main_Page = NexPage(4, 0, "MAIN");  //65 05 06 01 FF FF FF

// gyro 4,4 "GyroLine"
// NexPage main_Racing_Page = NexPage(5, 0, "Racing_main");  // 65 04 06 01 FF FF FF

//MAIN
//Racing_main

// ลิสต์ของอีเวนต์ที่ Nextion จะรับฟัง
NexTouch *nex_listen_list[] = {
  &modeToggle1,
  &main_Page,
  NULL  // ต้องสิ้นสุดด้วย NULL
};

void changeMainPageBackground(int pictureID);
void sendCommand(const char *cmd);

void setup() {
  nexSerial.begin(115200);  // เริ่มการสื่อสารผ่าน Serial2 ด้วยความเร็ว 115200 bps
  Serial.begin(115200);     // เริ่มการสื่อสารผ่าน Serial ด้วยความเร็ว 115200 bps
  SPI.begin();              // เริ่มการสื่อสาร SPI
  rfid.PCD_Init();          // เริ่มต้นโมดูล RFID

  pinMode(buzzer, OUTPUT);   // กำหนดพิน buzzer เป็น OUTPUT
  pinMode(Aceletor, INPUT);  // กำหนดพิน Aceletor เป็น INPUT
  pinMode(battery, INPUT);   // กำหนดพิน battery เป็น INPUT
  pinMode(5, INPUT_PULLUP);  // กำหนดพิน 5 เป็น INPUT พร้อมกับเปิด Pull-up resistor เพื่ออ่านค่าจาก FRPM
  pinMode(20, OUTPUT);       // รีเลย์ตัวที่ 1 cutoff
  pinMode(21, OUTPUT);       // รีเลย์ตัวที่ 2 Font Low
  pinMode(0, OUTPUT);        // รีเลย์ตัวที่ 3 Font High
  pinMode(1, OUTPUT);        // รีเลย์ตัวที่ 4 Rear Low
  pinMode(9, OUTPUT);        // รีเลย์ตัวที่ 5 Rear High

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

  nexInit();  // เริ่มต้นการสื่อสารกับหน้าจอ Nextion

  // ตั้งค่า Interrupt สำหรับพิน 5 (FRPM) เมื่อเกิดสัญญาณ RISING จะเรียกใช้ฟังก์ชัน ai1()
  attachInterrupt(digitalPinToInterrupt(5), ai1, RISING);

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

  modeP();

  // ตั้งค่าฟังก์ชัน callback เมื่อปุ่มถูกกด
  modeToggle1.attachPush(modeToggle1PushCallback);  //รอถามสอบการทำงานส่วนนี้
}
void loop() {
  // หน่วงเวลาเพื่อดูการเปลี่ยนแปลง
  nexLoop(nex_listen_list);  // ฟังอีเวนต์จากหน้าจอ Nextion

  // int rpmValue = 0;  // ตัวอย่างค่าที่ต้องการแสดง

  // // อัปเดตค่าตัวเลขบนจอ Nextion โดยใช้ setValue()
  // if (rpm1.setValue(rpmValue)) {
  //   Serial.println("Successfully updated rpm1.");
  //   rpmValue++;
  // } else {
  //   Serial.println("Failed to update rpm1.");
  // }

  // delay(1000);

  // ตรวจสอบการสแกน RFID
  //   while (tag != "83211224182" || M !=1 ) // ตรวจสอบว่า UID ตรงกับที่ต้องการหรือไม่
  //   {
  //     Serial.println("NOT Pass");
  //     tag = "";

  //     rfid.PICC_HaltA();      // หยุดการสื่อสารกับการ์ด
  //     rfid.PCD_StopCrypto1(); // หยุดการเข้ารหัส
  //     Serial.println(tag);

  //     if (!rfid.PICC_IsNewCardPresent()) // ตรวจสอบว่ามีการ์ดใหม่หรือไม่
  //       return;

  //     if (rfid.PICC_ReadCardSerial()) // อ่าน UID จากการ์ด
  //     {
  //       for (byte i = 0; i < 4; i++)
  //       {
  //         tag += rfid.uid.uidByte[i]; // รวม UID เป็นสตริง
  //       }
  //       Serial.print("UID = ");
  //       Serial.println(tag);
  //     }
  //   }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

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

  // ตรวจสอบการกดปุ่มเพิ่มโหมด
  if (buttonUpState == LOW && lastButtonUpState == HIGH && currentModeIndex != 5) {
    Race = 0;
    if (currentModeIndex < 4)  // ตรวจสอบว่าไม่เกินโหมดสูงสุด
    {
      currentModeIndex++;
      changeMode(currentModeIndex);  // เปลี่ยนโหมดตาม index ปัจจุบัน
    }
    delay(200);  // ดีเลย์เพื่อป้องกันการกดซ้ำ
  }

  // ตรวจสอบการกดปุ่มลดโหมด
  if (buttonDownState == LOW && lastButtonDownState == HIGH && currentModeIndex != 5) {
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
      currentModeIndex = 5;  // โหมด Racing (สมมติว่าโหมด 5 คือ Racing)
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
  // ax = a.acceleration.x * ACCEL_SCALE;
  // ay = a.acceleration.y * ACCEL_SCALE;
  // az = a.acceleration.z * ACCEL_SCALE;
  // gx = g.gyro.x * GYRO_SCALE * PI / 180;  // Convert degrees/second to radians/second
  // gy = g.gyro.y * GYRO_SCALE * PI / 180;
  // gz = g.gyro.z * GYRO_SCALE * PI / 180;

  // float angle;
  // // Calculate tilt angles using complementary filter (or Kalman filter if enabled)
  // angle_dot = gx + bias;
  // angle += angle_dot * 0.01;  // Assuming a 100Hz loop rate

  // // Update Kalman filter (if enabled)
  // // ... (implement Kalman filter equations)

  // // Apply complementary filter (or Kalman filter result)
  // angleX = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2))) * 180 / PI;
  // angleY = atan2(ax, sqrt(pow(ay, 2) + pow(az, 2))) * 180 / PI;

  // // Print tilt angles
  // Serial.print("Angle X: ");
  // Serial.print(angleX);
  // Serial.print("  Angle Y: ");
  // Serial.println(angleY);

  delay(100);

  // อ่านและแมปค่าระดับแบตเตอรี่
  pbattery = 80;
  //pbattery = abs(map(analogRead(A2), 0, 1023, 0, 165));

  if (new_cycle) {
    start = millis();   // เก็บเวลาปัจจุบัน
    new_cycle = false;  // รีเซ็ตสถานะ cycle
  }

  time_cycle = millis() - start;  // คำนวณเวลาที่ผ่านไป

  if (time_cycle > period) {
    RPM = RealRPM;            // อัพเดตค่า RPM
    RealRPM = my_vel * 0.48;  // คำนวณค่า RPM จริง
    // Serial2.print("RPM.val=");  // ส่งค่า RPM ไปยังหน้าจอ Nextion
    // Serial2.print(1000);
    // rpm1.setValue(RealRPM);
    // rpm2_PARKING.setValue(RealRPM);

    // คำนวณและส่งค่าระดับแบตเตอรี่ไปยังหน้าจอ Nextion
    bat = map(pbattery, 64, 84, 247, 253);
    bat = constrain(bat, 247, 253);
    // Serial2.print("Bat.pic=");
    // Serial2.print(bat);
    // Vbattery1.setValue(bat);
    // Vbattery2_PARKING.setValue(bat);

    new_cycle = true;  // เริ่ม cycle ใหม่
    counter = 0;       // รีเซ็ตตัวนับ
  }
}

// ฟังก์ชันสำหรับคำนวณความเร็วเป็น RPM
float vel_cal_rpm(unsigned long cycle) {
  return float(counter) * 3750 / cycle;
}

// ฟังก์ชันสำหรับคำนวณความเร็วเป็น RPS
float vel_cal_rps(unsigned long cycle) {
  return counter * 0.00375 * cycle;
}

// ฟังก์ชัน Interrupt สำหรับนับจำนวนรอบ
void ai1() {
  counter++;
}

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
    case 3:
      modeD();
      break;
    case 4:
      modeS();
      break;
    case 5:
      modeR();
      break;
  }
}

// ฟังก์ชันย่อยสำหรับแต่ละโหมด
void modeP() {
  digitalWrite(20, LOW);  //รีเลย์ตัวที่ 1 cutoff
  digitalWrite(21, LOW);  //รีเลย์ตัวที่ 2 Font Low
  digitalWrite(0, LOW);   //รีเลย์ตัวที่ 3 Font High
  digitalWrite(1, LOW);   //รีเลย์ตัวที่ 4 Rear Low
  digitalWrite(9, LOW);   //รีเลย์ตัวที่ 5 Rear High

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
  digitalWrite(20, LOW);  //รีเลย์ตัวที่ 1 cutoff
  digitalWrite(21, LOW);  //รีเลย์ตัวที่ 2 Font Low
  digitalWrite(0, LOW);   //รีเลย์ตัวที่ 3 Font High
  digitalWrite(1, LOW);   //รีเลย์ตัวที่ 4 Rear Low
  digitalWrite(9, LOW);   //รีเลย์ตัวที่ 5 Rear High

  Serial.println("Mode LM: Low Mode");
  nexMode1.setText("LM");

  // nexMode2_PARKING.setText("LM");

  // เพิ่มโค้ดเพิ่มเติมสำหรับโหมด Low Mode ถ้าจำเป็น
}

void modeD() {
  digitalWrite(20, LOW);  //รีเลย์ตัวที่ 1 cutoff
  digitalWrite(21, LOW);  //รีเลย์ตัวที่ 2 Font Low
  digitalWrite(0, LOW);   //รีเลย์ตัวที่ 3 Font High
  digitalWrite(1, LOW);   //รีเลย์ตัวที่ 4 Rear Low
  digitalWrite(9, LOW);   //รีเลย์ตัวที่ 5 Rear High;

  Serial.println("Mode D: Drive");
  nexMode1.setText("D");

  // nexMode2_PARKING.setText("D");
  // เพิ่มโค้ดเพิ่มเติมสำหรับโหมด Drive ถ้าจำเป็น
}

void modeS() {
  digitalWrite(20, LOW);  //รีเลย์ตัวที่ 1 cutoff
  digitalWrite(21, LOW);  //รีเลย์ตัวที่ 2 Font Low
  digitalWrite(0, LOW);   //รีเลย์ตัวที่ 3 Font High
  digitalWrite(1, LOW);   //รีเลย์ตัวที่ 4 Rear Low
  digitalWrite(9, LOW);   //รีเลย์ตัวที่ 5 Rear High

  Serial.println("Mode S: Sport");
  nexMode1.setText("S");

  //  nexMode2_PARKING.setText("S");

  // เพิ่มโค้ดเพิ่มเติมสำหรับโหมด Sport ถ้าจำเป็น
}

void modeR() {
  digitalWrite(20, LOW);  //รีเลย์ตัวที่ 1 cutoff
  digitalWrite(21, LOW);  //รีเลย์ตัวที่ 2 Font Low
  digitalWrite(0, LOW);   //รีเลย์ตัวที่ 3 Font High
  digitalWrite(1, LOW);   //รีเลย์ตัวที่ 4 Rear Low
  digitalWrite(9, LOW);   //รีเลย์ตัวที่ 5 Rear High

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
    currentModeIndex = 5;  // โหมด Racing
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



  // if (Race == 1 && !state) {
  //   nexSerial.print("MAIN.pic=2");  // ส่งคำสั่งไปที่ Nextion display
  //   nexSerial.write(0xff);          // ส่ง 0xff สามครั้งเพื่อบอกจบคำสั่ง
  //   nexSerial.write(0xff);
  //   nexSerial.write(0xff);
  //   nexMode1.setText(currentModeIndex);
  //   // Race = 0;

  //   // changeMode(currentModeIndex);
  // }
}

// ฟังก์ชัน callback เมื่อกดปุ่ม ModeToggle2 บนหน้าจอ Nextion
void modeToggle2PushCallback(void *ptr) {
  // Race = 0;
  // M = 1;
  Serial.println("ModeToggle2_PARKING ถูกกด");
  // เพิ่มการทำงานเมื่อปุ่มถูกกดตามที่ต้องการ
  // if (Race == 0) {
  // เข้าสู่โหมด Racing
  // currentModeIndex = previousModeIndex;
  // changeMode(currentModeIndex);
  // isRacingMode = false;
  // } else {
  // กลับไปโหมดก่อนหน้า
  // }
}
