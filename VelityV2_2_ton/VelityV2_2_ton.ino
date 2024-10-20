#include <Nextion.h>       // ไลบรารีสำหรับการสื่อสารกับหน้าจอ Nextion
#define nexSerial Serial2  // กำหนด nexSerial ให้เป็น Serial2 สำหรับการสื่อสารกับ Nextion
// #define HWSerial Serial2   // กำหนด HWSerial ให้เป็น Serial2
#include <EEPROM.h>        // ไลบรารีสำหรับการใช้งาน EEPROM
#include <SPI.h>           // ไลบรารีสำหรับการสื่อสาร SPI
#include <MFRC522.h>       // ไลบรารีสำหรับการใช้งาน RFID MFRC522
#include <Wire.h>          // ไลบรารีสำหรับการสื่อสาร I2C
#include <NexButton.h>     // ไลบรารีสำหรับการใช้งานปุ่มกับ Nextion HMI
#include <NexTouch.h>      // ไลบรารีสำหรับการจัดการอีเวนต์การสัมผัสกับ Nextion HMI
#include <NexHardware.h>   // ไลบรารีสำหรับการเชื่อมต่อกับฮาร์ดแวร์ของ Nextion

const int MPU_addr = 0x68;  // ที่อยู่ I2C ของเซ็นเซอร์ MPU6050
int16_t AcX, AcY, AcZ;      // ตัวแปรเก็บค่า Accelerometer ในแกน X, Y, Z
int16_t GyX, GyY, GyZ;      // ตัวแปรเก็บค่า Gyroscope ในแกน X, Y, Z
double x, y, z, i;          // ตัวแปรสำหรับเก็บค่าองศาที่คำนวณได้จาก Accelerometer

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

void modeToggle1PopCallback(void *ptr) {
  modeR();
}
void modeToggle2PopCallback(void *ptr) {
  modeP();
}
void setup() {
  // HWSerial.begin(115200);  // เริ่มการสื่อสารผ่าน Serial2 ด้วยความเร็ว 115200 bps
  nexSerial.begin(115200);   // เริ่มการสื่อสารผ่าน Serial2 ด้วยความเร็ว 115200 bps
  Serial.begin(115200);    // เริ่มการสื่อสารผ่าน Serial ด้วยความเร็ว 115200 bps
  SPI.begin();             // เริ่มการสื่อสาร SPI
  rfid.PCD_Init();         // เริ่มต้นโมดูล RFID

  pinMode(buzzer, OUTPUT);   // กำหนดพิน buzzer เป็น OUTPUT
  pinMode(Aceletor, INPUT);  // กำหนดพิน Aceletor เป็น INPUT
  pinMode(battery, INPUT);   // กำหนดพิน battery เป็น INPUT
  pinMode(5, INPUT_PULLUP);  // กำหนดพิน 5 เป็น INPUT พร้อมกับเปิด Pull-up resistor เพื่ออ่านค่าจาก FRPM
  pinMode(20, OUTPUT);       // รีเลย์ตัวที่ 1 cutoff
  pinMode(21, OUTPUT);       // รีเลย์ตัวที่ 2 Font Low
  pinMode(0, OUTPUT);        // รีเลย์ตัวที่ 3 Font High
  pinMode(1, OUTPUT);        // รีเลย์ตัวที่ 4 Rear Low
  pinMode(9, OUTPUT);        // รีเลย์ตัวที่ 5 Rear High



  // ตั้งค่า Interrupt สำหรับพิน 5 (FRPM) เมื่อเกิดสัญญาณ RISING จะเรียกใช้ฟังก์ชัน ai1()
  attachInterrupt(digitalPinToInterrupt(5), ai1, RISING);

  Wire.begin();  // เริ่มต้นการสื่อสาร I2C

  // เริ่มต้นการตั้งค่า MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // เขียนไปที่รีจิสเตอร์ PWR_MGMT_1
  Wire.write(0);     // ตั้งค่าให้ MPU6050 ทำงาน
  Wire.endTransmission(true);

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
  if (nexInit()) {
    Serial.println("Hi");
  }
  Serial.println("Initial Mode: P");
  // modeP();  // เริ่มต้นด้วยโหมด P
  // เริ่มต้นการสื่อสารกับหน้าจอ Nextion
  // ตั้งค่าฟังก์ชัน callback เมื่อปุ่มถูกกด
  modeToggle1.attachPop(modeToggle1PopCallback, &modeToggle1);  //รอถามสอบการทำงานส่วนนี้
  modeToggle2.attachPop(modeToggle2PopCallback, &modeToggle2);
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

  // if (j == 1) {
  //   digitalWrite(buzzer, HIGH);  // ให้เสียงแจ้งเตือน
  //   delay(500);
  //   digitalWrite(buzzer, LOW);
  //   j = 0;  // เปลี่ยนสถานะเพื่อไม่ให้เข้ามาในเงื่อนไขนี้อีก
  // }

  // // อ่านสถานะของปุ่มเพิ่มและลดโหมด
  // int buttonUpState = digitalRead(buttonUpPin);
  // int buttonDownState = digitalRead(buttonDownPin);

  // // ตรวจสอบการกดปุ่มเพิ่มโหมด
  // if (buttonUpState == LOW && lastButtonUpState == HIGH) {
  //   Race = 0;
  //   if (currentModeIndex < 4)  // ตรวจสอบว่าไม่เกินโหมดสูงสุด
  //   {
  //     currentModeIndex++;
  //     changeMode(currentModeIndex);  // เปลี่ยนโหมดตาม index ปัจจุบัน
  //   }
  //   delay(200);  // ดีเลย์เพื่อป้องกันการกดซ้ำ
  // }

  // // ตรวจสอบการกดปุ่มลดโหมด
  // if (buttonDownState == LOW && lastButtonDownState == HIGH) {
  //   Race = 0;
  //   if (currentModeIndex > 0)  // ตรวจสอบว่าไม่ต่ำกว่าโหมดต่ำสุด
  //   {
  //     currentModeIndex--;
  //     changeMode(currentModeIndex);  // เปลี่ยนโหมดตาม index ปัจจุบัน
  //   }
  //   delay(200);  // ดีเลย์เพื่อป้องกันการกดซ้ำ
  // }

  // // อัพเดตสถานะปุ่มล่าสุด
  // lastButtonUpState = buttonUpState;
  // lastButtonDownState = buttonDownState;


  // // อ่านค่าจาก MPU6050
  // Wire.beginTransmission(MPU_addr);
  // Wire.write(0x3B);  // เริ่มต้นอ่านจากรีจิสเตอร์ 0x3B (Accelerometer)
  // Wire.endTransmission(false);
  // Wire.requestFrom(MPU_addr, 14, true);  // ขอข้อมูล 14 ไบต์

  // // อ่านค่าจาก Accelerometer และ Gyroscope
  // AcX = Wire.read() << 8 | Wire.read();
  // AcY = Wire.read() << 8 | Wire.read();
  // AcZ = Wire.read() << 8 | Wire.read();
  // // GyX, GyY, GyZ สามารถอ่านต่อได้ถ้าจำเป็น

  // // คำนวณมุมจากค่า Accelerometer
  // //   int xAng = map(AcX, minVal, maxVal, -90, 90);
  // //   int yAng = map(AcY, minVal, maxVal, -90, 90);
  // //   int zAng = map(AcZ, minVal, maxVal, -90, 90);

  // //   x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  // //   y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  // //   z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  // // Serial.print("AngleX= ");
  // // Serial.println(x); // แสดงผลมุมในแกน X


  // // อ่านและแมปค่าระดับแบตเตอรี่
  // pbattery = 80;
  // //pbattery = abs(map(analogRead(A2), 0, 1023, 0, 165));

  // if (new_cycle) {
  //   start = millis();   // เก็บเวลาปัจจุบัน
  //   new_cycle = false;  // รีเซ็ตสถานะ cycle
  // }

  // time_cycle = millis() - start;  // คำนวณเวลาที่ผ่านไป

  // if (time_cycle > period) {
  //   RPM = RealRPM;              // อัพเดตค่า RPM
  //   RealRPM = my_vel * 0.48;    // คำนวณค่า RPM จริง
  //   Serial2.print("RPM.val=");  // ส่งค่า RPM ไปยังหน้าจอ Nextion
  //   Serial2.print(1000);

  //   // คำนวณและส่งค่าระดับแบตเตอรี่ไปยังหน้าจอ Nextion
  //   bat = map(pbattery, 64, 84, 247, 253);
  //   bat = constrain(bat, 247, 253);
  //   Serial2.print("Bat.pic=");
  //   Serial2.print(bat);

  //   new_cycle = true;  // เริ่ม cycle ใหม่
  //   counter = 0;       // รีเซ็ตตัวนับ
  // }
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
