#include <Nextion.h>       // ไลบรารีสำหรับการสื่อสารกับหน้าจอ Nextion
#define nexSerial Serial2  // กำหนด nexSerial ให้เป็น Serial2 สำหรับการสื่อสารกับ Nextion
#define HWSerial Serial2   // กำหนด HWSerial ให้เป็น Serial2
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

constexpr uint8_t RST_PIN = 10;  // พินรีเซ็ตสำหรับโมดูล RFID
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
const char *modes[] = { "P", "Lm", "Ec", "D", "S" };  //รอปรึกษาว่าโหมด R ต้องใส่ตรงไหน
int currentModeIndex = 0;                             // เริ่มต้นที่โหมด P

// สร้างวัตถุปุ่มสำหรับ Nextion
NexButton modeToggle1 = NexButton(4, 6, "ModeToggle1");  // ปุ่มที่หน้า page 4, ID 6
NexButton modeToggle2 = NexButton(5, 6, "ModeToggle2");  // ปุ่มที่หน้า page 5, ID 6

// ลิสต์ของอีเวนต์ที่ Nextion จะรับฟัง
NexTouch *nex_listen_list[] = {
  &modeToggle1,
  &modeToggle2,
  NULL  // ต้องสิ้นสุดด้วย NULL
};

void setup() {
  // HWSerial.begin(115200);  // เริ่มการสื่อสารผ่าน Serial2 ด้วยความเร็ว 115200 bps
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

  Wire.begin();  // เริ่มต้นการสื่อสาร I2C

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
  Serial.println("Initial Mode: P");
  modeP();  // เริ่มต้นด้วยโหมด P

  // ตั้งค่าฟังก์ชัน callback เมื่อปุ่มถูกกด
  modeToggle1.attachPush(modeToggle1PushCallback);  //รอถามสอบการทำงานส่วนนี้
  modeToggle2.attachPush(modeToggle2PushCallback);
}

void loop() {

  nexLoop(nex_listen_list);  // ฟังอีเวนต์จากหน้าจอ Nextion

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
  if (buttonUpState == LOW && lastButtonUpState == HIGH) {
    Race = 0;
    if (currentModeIndex < 4)  // ตรวจสอบว่าไม่เกินโหมดสูงสุด
    {
      currentModeIndex++;
      changeMode(currentModeIndex);  // เปลี่ยนโหมดตาม index ปัจจุบัน
    }
    delay(200);  // ดีเลย์เพื่อป้องกันการกดซ้ำ
  }

  // ตรวจสอบการกดปุ่มลดโหมด
  if (buttonDownState == LOW && lastButtonDownState == HIGH) {
    Race = 0;
    if (currentModeIndex > 0)  // ตรวจสอบว่าไม่ต่ำกว่าโหมดต่ำสุด
    {
      currentModeIndex--;
      changeMode(currentModeIndex);  // เปลี่ยนโหมดตาม index ปัจจุบัน
    }
    delay(200);  // ดีเลย์เพื่อป้องกันการกดซ้ำ
  }

  // if(RacingState == LOW){
  //   if(pressTime == 0){
  //     pressTime = millis();
  //   }
  //   if(millis() - pressTime >= longPressDuration){
  //     Race = 1;
  //     modeR();
  //     // while(RacingState == LOW){
  //     //   pressTime = 0;
  //     // }
  //   }
  // }

  // อัพเดตสถานะปุ่มล่าสุด
  lastButtonUpState = buttonUpState;
  lastButtonDownState = buttonDownState;
  ax = a.acceleration.x * ACCEL_SCALE;
  ay = a.acceleration.y * ACCEL_SCALE;
  az = a.acceleration.z * ACCEL_SCALE;
  gx = g.gyro.x * GYRO_SCALE * PI / 180;  // Convert degrees/second to radians/second
  gy = g.gyro.y * GYRO_SCALE * PI / 180;
  gz = g.gyro.z * GYRO_SCALE * PI / 180;

  float angle;
  // Calculate tilt angles using complementary filter (or Kalman filter if enabled)
  angle_dot = gx + bias;
  angle += angle_dot * 0.01;  // Assuming a 100Hz loop rate

  // Update Kalman filter (if enabled)
  // ... (implement Kalman filter equations)

  // Apply complementary filter (or Kalman filter result)
  angleX = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2))) * 180 / PI;
  angleY = atan2(ax, sqrt(pow(ay, 2) + pow(az, 2))) * 180 / PI;

  // Print tilt angles
  Serial.print("Angle X: ");
  Serial.print(angleX);
  Serial.print("  Angle Y: ");
  Serial.println(angleY);

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
    RPM = RealRPM;              // อัพเดตค่า RPM
    RealRPM = my_vel * 0.48;    // คำนวณค่า RPM จริง
    Serial2.print("RPM.val=");  // ส่งค่า RPM ไปยังหน้าจอ Nextion
    Serial2.print(1000);

    // คำนวณและส่งค่าระดับแบตเตอรี่ไปยังหน้าจอ Nextion
    bat = map(pbattery, 64, 84, 247, 253);
    bat = constrain(bat, 247, 253);
    Serial2.print("Bat.pic=");
    Serial2.print(bat);

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
  // เพิ่มโค้ดเพิ่มเติมสำหรับโหมด Parking ถ้าจำเป็น
}
void modeEc() {
  digitalWrite(20, LOW);  //รีเลย์ตัวที่ 1 cutoff
  digitalWrite(21, LOW);  //รีเลย์ตัวที่ 2 Font Low
  digitalWrite(0, LOW);   //รีเลย์ตัวที่ 3 Font High
  digitalWrite(1, LOW);   //รีเลย์ตัวที่ 4 Rear Low
  digitalWrite(9, LOW);   //รีเลย์ตัวที่ 5 Rear High

  Serial.println("Mode ECO Mode");
  // เพิ่มโค้ดเพิ่มเติมสำหรับโหมด Low Mode ถ้าจำเป็น
}
void modeLM() {
  digitalWrite(20, LOW);  //รีเลย์ตัวที่ 1 cutoff
  digitalWrite(21, LOW);  //รีเลย์ตัวที่ 2 Font Low
  digitalWrite(0, LOW);   //รีเลย์ตัวที่ 3 Font High
  digitalWrite(1, LOW);   //รีเลย์ตัวที่ 4 Rear Low
  digitalWrite(9, LOW);   //รีเลย์ตัวที่ 5 Rear High

  Serial.println("Mode LM: Low Mode");
  // เพิ่มโค้ดเพิ่มเติมสำหรับโหมด Low Mode ถ้าจำเป็น
}

void modeD() {
  digitalWrite(20, LOW);  //รีเลย์ตัวที่ 1 cutoff
  digitalWrite(21, LOW);  //รีเลย์ตัวที่ 2 Font Low
  digitalWrite(0, LOW);   //รีเลย์ตัวที่ 3 Font High
  digitalWrite(1, LOW);   //รีเลย์ตัวที่ 4 Rear Low
  digitalWrite(9, LOW);   //รีเลย์ตัวที่ 5 Rear High;

  Serial.println("Mode D: Drive");
  // เพิ่มโค้ดเพิ่มเติมสำหรับโหมด Drive ถ้าจำเป็น
}

void modeS() {
  digitalWrite(20, LOW);  //รีเลย์ตัวที่ 1 cutoff
  digitalWrite(21, LOW);  //รีเลย์ตัวที่ 2 Font Low
  digitalWrite(0, LOW);   //รีเลย์ตัวที่ 3 Font High
  digitalWrite(1, LOW);   //รีเลย์ตัวที่ 4 Rear Low
  digitalWrite(9, LOW);   //รีเลย์ตัวที่ 5 Rear High

  Serial.println("Mode S: Sport");
  // เพิ่มโค้ดเพิ่มเติมสำหรับโหมด Sport ถ้าจำเป็น
}

void modeR() {
  digitalWrite(20, LOW);  //รีเลย์ตัวที่ 1 cutoff
  digitalWrite(21, LOW);  //รีเลย์ตัวที่ 2 Font Low
  digitalWrite(0, LOW);   //รีเลย์ตัวที่ 3 Font High
  digitalWrite(1, LOW);   //รีเลย์ตัวที่ 4 Rear Low
  digitalWrite(9, LOW);   //รีเลย์ตัวที่ 5 Rear High

  Serial.println("Mode R: Race");
  // เพิ่มโค้ดเพิ่มเติมสำหรับโหมด Sport ถ้าจำเป็น
}

// ฟังก์ชัน callback เมื่อกดปุ่ม ModeToggle1 บนหน้าจอ Nextion
void modeToggle1PushCallback(void *ptr) {
  Race = 1;
  LM = 0;
  Serial.println("ModeToggle1 ถูกกด");
  // เพิ่มการทำงานเมื่อปุ่มถูกกดตามที่ต้องการ
}

// ฟังก์ชัน callback เมื่อกดปุ่ม ModeToggle2 บนหน้าจอ Nextion
void modeToggle2PushCallback(void *ptr) {
  M = 1;
  Serial.println("ModeToggle2 ถูกกด");
  // เพิ่มการทำงานเมื่อปุ่มถูกกดตามที่ต้องการ
}
