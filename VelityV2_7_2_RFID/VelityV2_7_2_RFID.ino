
// ไลบรารีที่จำเป็น
#include <Nextion.h>           // ไลบรารีสำหรับการสื่อสารกับหน้าจอ Nextion
#define nexSerial Serial2      // กำหนด nexSerial ให้เป็น Serial2 สำหรับการสื่อสารกับ Nextion
#include <EEPROM.h>            // ไลบรารีสำหรับการใช้งาน EEPROM
#include <SPI.h>               // ไลบรารีสำหรับการสื่อสาร SPI
#include <MFRC522.h>           // ไลบรารีสำหรับการใช้งาน RFID MFRC522
#include <Wire.h>              // ไลบรารีสำหรับการสื่อสาร I2C
#include <NexButton.h>         // ไลบรารีสำหรับการใช้งานปุ่มกับ Nextion HMI
#include <NexTouch.h>          // ไลบรารีสำหรับการจัดการอีเวนต์การสัมผัสกับ Nextion HMI
#include <NexHardware.h>       // ไลบรารีสำหรับการเชื่อมต่อกับฮาร์ดแวร์ของ Nextion
#include <Adafruit_MPU6050.h>  // ไลบรารีสำหรับเซ็นเซอร์ MPU6050
#include <Adafruit_Sensor.h>   // ไลบรารีสำหรับเซ็นเซอร์ทั่วไปของ Adafruit


// การตั้งค่าตัวแปรสำหรับ MPU6050
Adafruit_MPU6050 mpu;  // ออบเจ็กต์สำหรับเซ็นเซอร์ MPU6050
float ax, ay, az;      // ตัวแปรเก็บค่า Accelerometer ในแกน X, Y, Z
float gx, gy, gz;      // ตัวแปรเก็บค่า Gyroscope ในแกน X, Y, Z
float angleX = 0, angleY = 0, angleZ = 0;

// การตั้งค่าที่อยู่ใน EEPROM
const int address = 1;                     // ที่อยู่ในการเก็บค่าใน EEPROM
const int address_distance = sizeof(int);  // ที่อยู่ในการเก็บค่าใน EEPROM

// กำหนดค่าคงที่สำหรับการตั้งค่าเซ็นเซอร์และการคำนวณ
// Constants for calculations
const float GYRO_SENSITIVITY = 65.5;  // LSB/(deg/s) for ±2000 deg/s range
const float dt = 0.01;                // Time step in seconds (100Hz sampling rate)

// Variables for angle calculation

#define PULSES_PER_REVOLUTION 15  // จำนวนครั้งที่เซนเซอร์อ่านค่าต่อรอบ

// ตัวแปรสำหรับ Kalman filter (ถ้าใช้งาน)
float Q_angle = 0.001;        // Process noise covariance
float Q_bias = 0.003;         // Process noise covariance for bias
float R_measure = 0.03;       // Measurement noise covariance
float P[4] = { 0, 0, 0, 0 };  // Error covariance matrix
float K[4] = { 0, 0, 0, 0 };  // Kalman gain
float y, S, K_angle, K_bias;
float angle_dot, bias;

// การตั้งค่าพินและออบเจ็กต์สำหรับ RFID
constexpr uint8_t RST_PIN = -1;  // พินรีเซ็ตสำหรับโมดูล RFID
constexpr uint8_t SS_PIN = 17;   // พิน Slave Select สำหรับโมดูล RFID
MFRC522 rfid(SS_PIN, RST_PIN);   // สร้างออบเจ็กต์สำหรับการใช้งาน RFID
String tag = "";                 // ตัวแปรสำหรับเก็บ UID ของแท็ก RFID

String tagUser1; //รอกรอก
String tagUser2; //รอกรอก
String tagUser3; //รอกรอก
String tagUser4; //รอกรอก
String tagUser5; //รอกรอก
String tagUser6; //รอกรอก


// ตัวแปรสำหรับการตรวจสอบสถานะการเปลี่ยนหน้าใน Nextion และการนับรอบ
int j = 1;                                // ตัวแปรสำหรับตรวจสอบสถานะการเปลี่ยนหน้าใน Nextion
volatile unsigned int temp, counter = 0;  // ตัวแปรสำหรับการนับรอบและอุณหภูมิ (ถ้ามีการใช้งาน)

// การตั้งค่าพินสำหรับเซ็นเซอร์ต่างๆ และ buzzer
int Aceletor = A9;  // พินสำหรับอ่านค่าจาก Accelerator (คันเร่ง)
int buzzer = 22;    // พินสำหรับควบคุมเสียง Buzzer
int battery = A2;   // พินสำหรับอ่านค่าระดับแบตเตอรี่
int pbattery, RC;   // ตัวแปรสำหรับเก็บค่าระดับแบตเตอรี่และอื่น ๆ

// ตัวแปรสำหรับการคำนวณค่า RPM และค่าเฉลี่ย
int period = 200;     // ช่วงเวลาสำหรับการอัพเดทข้อมูล (หน่วยเป็นมิลลิวินาที)
int RPM_IN = 5;       //พิน hall sensor
int sensorValue = 0;  // ตัวแปรเก็บค่าอนาล็อก
int RPM;
int KMTT = 0;

// ตัวแปรสำหรับสถานะการทำงาน

bool aa = 0;
int LM = 0;
int Race = 0;
int M = 0;

// การตั้งค่าสำหรับปุ่มและโหมดการขับขี่
unsigned long buttonPressTime = 0;  // เวลาเมื่อกดปุ่ม
bool buttonHeld = false;            // ตรวจสอบว่าปุ่มถูกกดค้างไว้หรือไม่
bool isRacingMode = false;          // สถานะโหมด Racing
const int buttonUpPin = 3;          // พินสำหรับปุ่มเพิ่มโหมด
const int buttonDownPin = 4;        // พินสำหรับปุ่มลดโหมด
const int RacingPin = 2;            // พินสำหรับสลับโหมด Racing
int lastButtonUpState = HIGH;       // สถานะล่าสุดของปุ่มเพิ่มโหมด
int lastButtonDownState = HIGH;     // สถานะล่าสุดของปุ่มลดโหมด
int lastButtonRacingState = HIGH;   // สถานะล่าสุดของปุ่ม Racing

int cut_off = 20;
int front_low = 21;
int front_high = 14;
int rear_low = 15;
int rear_high = 9;

// การตั้งค่าระยะเวลาในการกดค้าง
unsigned long pressTime = 0;             // เวลาเมื่อสวิตช์ถูกกด
unsigned long longPressDuration = 2000;  // ระยะเวลาในการกดค้าง (2 วินาที)

// ตัวแปรสำหรับโหมดการขับขี่
const char *modes[] = { "P", "LM", "EC", "S", "R" };  // รายการของโหมดการขับขี่
int currentModeIndex = 0;                             // เริ่มต้นที่โหมด P
int previousModeIndex = 0;

// ตัวแปรสำหรับตรวจสอบสถานะของการสแกนบัตร RFID และสถานะของหน้าหลัก
bool correctCardScanned = false;
bool onMainPage = false;

// ตัวแปรสำหรับการนับรอบของ Hall sensor และการคำนวณความเร็ว
volatile int hallCount = 0;  // ตัวนับจำนวนครั้งที่ Hall sensor ส่งสัญญาณ
unsigned long previousMillis = 0;
int rpm;
const float wheelCircumference = 0.878;  // ปรับค่าให้ตรงกับเส้นรอบวงจริง

int maxSpeed = 0;  // ตัวแปรเก็บค่าความเร็วสูงสุด

int km;  //ตัวแปรเก็บระยะทาง
int hall;

volatile unsigned long lastTime = 0;
volatile unsigned long pulseTime = 0;
volatile unsigned long currentRPM = 0;  // เปลี่ยนชื่อจาก rpm เป็น currentRPM

const int numReadings = 10;  // จำนวนค่าที่นำมาคำนวณค่าเฉลี่ย
int readings[numReadings];   // อาร์เรย์เก็บค่า RPM
int indexq = 0;              // ตัวชี้ตำแหน่งในอาร์เรย์
int total = 0;               // ผลรวมของค่าทั้งหมดในอาร์เรย์
float average = 0;           // ค่าเฉลี่ยของค่า RPM
int val;
// การตั้งค่าวัตถุปุ่มสำหรับ Nextion
NexButton modeToggle1 = NexButton(4, 5, "ModeToggle1");  // ปุ่มเข้าโหมด R
NexButton enterMain = NexButton(2, 29, "enter");         // ปุ่มเข้าหน้าหลัก
NexButton ResetTrip = NexButton(4, 2, "Reset");          // ปุ่มที่หน้า page 4, ID 2
NexNumber rpm1 = NexNumber(4, 13, "RPM");                //ตัวแปรความเร็วรอบ
NexNumber Vbattery1 = NexNumber(4, 16, "Vbattery");      //ตัวเแปรโวลล์แบต
NexNumber speed1 = NexNumber(4, 14, "Speed");            //ตัวแปรความเร็ว Kmh
NexNumber Trip1 = NexNumber(4, 17, "Trip");              //ตัวแปรเก็บระยะทาง
NexNumber maxSpeed1 = NexNumber(4, 15, "Maxspd");        //ตัวแปรเก็บความเร็วสูงสุด
NexNumber batper = NexNumber(4, 23, "BattPer");          //เปอร์เซนต์แบต
NexPicture gyro = NexPicture(4, 4, "GyroLine");          //เส้น gyro
NexText nexMode1 = NexText(4, 6, "Gears");               //โหมดการขับ
NexPage main_Page = NexPage(4, 0, "MAIN");               //หน้าหลัก

// NexNumber state1 = NexNumber(2, 28, "state");
// NexPage input_Page = NexPage(2, 0, "PIN_INPUT");
// NexTimer check = NexTimer(2, 21, "Check");

// ลิสต์ของอีเวนต์ที่ Nextion จะรับฟัง
NexTouch *nex_listen_list[] = {
  &modeToggle1,
  &enterMain,
  &ResetTrip,
  &main_Page,
  &gyro,
  NULL  // ต้องสิ้นสุดด้วย NULL
};

// ตัวแปรสำหรับจัดการข้อมูลที่ถูกเก็บใน EEPROM
int storedMaxSpeed;
int storeKm;
float totalDistance = 0;  // ตัวแปรเก็บระยะทางสะสม (เมตร)

const struct {
  int angle;
  int picId;
} anglePicMapping[] = {
  { 175, 60 }, { 165, 59 }, { 145, 58 }, { 135, 57 }, { 125, 56 }, { 115, 55 }, { 105, 54 }, { 95, 53 }, { 85, 52 }, { 75, 51 }, { 65, 50 }, { 55, 49 }, { 45, 48 }, { 35, 47 }, { 25, 46 }, { 15, 45 }, { 0, 44 }
};

const int mappingSize = sizeof(anglePicMapping) / sizeof(anglePicMapping[0]);

void setup() {
  nexSerial.begin(115200);  // Start communication with the Nextion display via Serial2 at 115200 bps
  Serial.begin(115200);     // Start communication via Serial at 115200 bps
  SPI.begin();              // Start SPI communication
  rfid.PCD_Init();          // Initialize the RFID module

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Set sensor parameters
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  // Setup pin modes
  pinMode(buzzer, OUTPUT);        // Buzzer pin as OUTPUT
  pinMode(Aceletor, INPUT);       // Accelerator pin as INPUT
  pinMode(battery, INPUT);        // Battery pin as INPUT
  pinMode(RPM_IN, INPUT_PULLUP);  // RPM input pin as INPUT with Pull-up resistor

  // Setup mode buttons
  pinMode(buttonUpPin, INPUT);
  pinMode(buttonDownPin, INPUT);
  pinMode(RacingPin, INPUT);

  pinMode(20, OUTPUT);  // Relay 1 cutoff pin as OUTPUT
  pinMode(21, OUTPUT);  // Relay 2 Font Low pin as OUTPUT
  pinMode(14, OUTPUT);  // Relay 3 Font High pin as OUTPUT
  pinMode(15, OUTPUT);  // Relay 4 Rear Low pin as OUTPUT
  pinMode(9, OUTPUT);   // Relay 5 Rear High pin as OUTPUT

  // Set up the RPM interrupt
  attachInterrupt(digitalPinToInterrupt(RPM_IN), hallSensorISR, FALLING);

  // Initialize Nextion display
  nexInit();

  // Attach callback functions for Nextion display buttons
  modeToggle1.attachPush(modeToggle1PushCallback);
  enterMain.attachPush(enterMainPushCallback);
  ResetTrip.attachPush(ResetPushCallback);

  for (int i = 0; i < numReadings; i++) { readings[i] = 0; }

  // Initial buzzer sound to signal start
  digitalWrite(buzzer, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW);
  delay(100);
  digitalWrite(buzzer, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW);

  // Initial relay state
  digitalWrite(cut_off, 0);     // Relay 1 cutoff
  digitalWrite(front_low, 0);   // Relay 2 Font Low
  digitalWrite(front_high, 0);  // Relay 3 Font High
  digitalWrite(rear_low, 0);    // Relay 4 Rear Low
  digitalWrite(rear_high, 0);   // Relay 5 Rear High

  // Read previous settings from EEPROM
  readFromEEPROM();
  Serial.println("Tap your Card");
}

void updateDisplay(float angle) {
  for (int i = 0; i < mappingSize; i++) {
    if (angle > anglePicMapping[i].angle) {
      gyro.setPic(anglePicMapping[i].picId);
      break;
    }
  }
}
void loop() {
  nexLoop(nex_listen_list);  // ฟังอีเวนต์จากหน้าจอ Nextion


  Trip1.setValue(km);  //ส่งค่าระยะทางไปที่จอ


  if (onMainPage || correctCardScanned) {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= 833) {  // คำนวณ RPM ทุก 1 วินาที
      if (hallCount == 0) {
        rpm = 0;
      } else {
        rpm = (hallCount * 60.0) / 15.0;
      }
      hallCount = 0;
      previousMillis = currentMillis;
      float distancePerMinute = rpm * wheelCircumference;

      // คำนวณระยะทางที่วิ่งได้ในหนึ่งรอบ (เมตร)
      float distancePerRevolution = distancePerMinute / (60 * rpm);

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
      // static float lastRpm = 0;
      // static float lastSpeed = 0;

      // if (abs(rpm - lastRpm) > 1 || abs(speedKmh - lastSpeed) > 0.1) {
        rpm1.setValue(rpm);
        speed1.setValue(speedKmh);
      //   lastRpm = rpm;
      //   lastSpeed = speedKmh;
      // }

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
    Gyro();
    cal_bat();  // delay(1000);


    if (j == 1) {
      digitalWrite(buzzer, HIGH);  // ให้เสียงแจ้งเตือน
      delay(100);
      digitalWrite(buzzer, LOW);
      j = 0;  // เปลี่ยนสถานะเพื่อไม่ให้เข้ามาในเงื่อนไขนี้อีก
    }

    // อ่านสถานะของปุ่มเพิ่มและลดโหมด
    int buttonUpState = digitalRead(buttonUpPin);
    int buttonDownState = digitalRead(buttonDownPin);
    int RacingState = digitalRead(RacingPin);

    // ตรวจสอบการกดปุ่มเพิ่มโหมด
    if (buttonUpState == 1 && lastButtonUpState == 0 && currentModeIndex != 4) {
      Race = 0;
      if (currentModeIndex < 3) {
        currentModeIndex++;
        changeMode(currentModeIndex);  // เปลี่ยนโหมดตาม index ปัจจุบัน
      }
      delay(200);  // ดีเลย์เพื่อป้องกันการกดซ้ำ
    }

    // ตรวจสอบการกดปุ่มลดโหมด
    if (buttonDownState == 1 && lastButtonDownState == 0 && currentModeIndex != 4) {
      Race = 0;
      if (currentModeIndex > 0) {
        currentModeIndex--;
        changeMode(currentModeIndex);  // เปลี่ยนโหมดตาม index ปัจจุบัน
      }
      delay(200);  // ดีเลย์เพื่อป้องกันการกดซ้ำ
    }

    bool buttonPressed = digitalRead(RacingPin) == 1;  // ตรวจสอบว่าปุ่มถูกกดหรือไม่ (สมมติ LOW คือตอนกด)

    if (buttonPressed && !buttonHeld) {
      // ถ้ากดปุ่มเป็นครั้งแรก
      buttonPressTime = millis();  // บันทึกเวลาเมื่อกดปุ่ม
      buttonHeld = true;           // ระบุว่าปุ่มถูกกดค้างไว้
    }

    if (!buttonPressed && buttonHeld) {
      // ถ้าปล่อยปุ่มหลังจากกดค้างไว้
      buttonHeld = false;  // ปุ่มถูกปล่อยแล้ว
    }

    if (buttonHeld && (millis() - buttonPressTime >= 1000)) {
      // ถ้าปุ่มถูกกดค้างไว้เกิน 2 วินาที
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
      } else {
        // กลับไปโหมดก่อนหน้า
        currentModeIndex = previousModeIndex;
        nexSerial.print("MAIN.pic=2");  // ส่งคำสั่งไปที่ Nextion display
        nexSerial.write(0xff);          // ส่ง 0xff สามครั้งเพื่อบอกจบคำสั่ง
        nexSerial.write(0xff);
        nexSerial.write(0xff);
        changeMode(currentModeIndex);
        isRacingMode = false;
      }

      // รีเซ็ตค่าเพื่อไม่ให้สลับโหมดซ้ำจนกว่าจะปล่อยปุ่ม
      buttonHeld = false;
    }

    // อัพเดตสถานะปุ่มล่าสุด
    lastButtonUpState = buttonUpState;
    lastButtonDownState = buttonDownState;
  } else {
    rfid_scan();  // Scan the card if neither card scanned nor on main page
  }
}
void cal_bat() {

  static unsigned long lastTime = 0;  // ตัวแปรเก็บเวลาล่าสุดที่คำนวณค่าเฉลี่ย
  static int readingsCount = 0;       // ตัวแปรนับจำนวนครั้งที่อ่านค่า
  static float readingsSum = 0;       // ตัวแปรเก็บผลรวมของค่าที่อ่านได้
  float averageVoltage;

  sensorValue = analogRead(battery);
  readingsSum += sensorValue;  // เพิ่มค่าที่อ่านได้เข้าไปในผลรวม
  readingsCount++;             // เพิ่มจำนวนครั้งที่อ่านค่า

  unsigned long currentTime = millis();

  // ตรวจสอบว่าครบ 2 วินาทีหรือยัง หรืออ่านค่าได้ครบจำนวนที่ต้องการ (ถ้ามีการกำหนด)
  if (currentTime - lastTime >= 15000) {
    averageVoltage = (readingsSum / readingsCount) * 1.664;  // คำนวณค่าเฉลี่ย

    Serial.print("Average Voltage: ");
    Serial.println(averageVoltage);
    float batteryPercentage = calculateBatteryPercentage(averageVoltage);
    Serial.print("Battery Percentage: ");
    Serial.println(batteryPercentage);
    batper.setValue(batteryPercentage);
    Vbattery1.setValue(averageVoltage);

    // รีเซ็ตค่าต่างๆ เพื่อเริ่มกระบวนการใหม่
    lastTime = currentTime;
    readingsCount = 0;
    readingsSum = 0;

    // คำนวณเปอร์เซ็นต์แบตเตอรี่
  }
}

float calculateBatteryPercentage(float voltage) {
  const float minVoltage = 633;
  const float maxVoltage = 900;

  float percentage = ((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100;

  // ป้องกันไม่ให้ค่า percentage เกิน 100 หรือต่ำกว่า 0
  if (percentage > 100) {
    percentage = 100;
  } else if (percentage < 0) {
    percentage = 0;
  }

  return percentage;
}

void readFromEEPROM() {
  EEPROM.get(address_distance, km);
}

void hallSensorISR() {
  hallCount++;
  hall++;
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
  digitalWrite(cut_off, 0);     // Relay 1 cutoff
  digitalWrite(front_low, 0);   // Relay 2 Font Low
  digitalWrite(front_high, 0);  // Relay 3 Font High
  digitalWrite(rear_low, 0);    // Relay 4 Rear Low
  digitalWrite(rear_high, 0);   // Relay 5 Rear High

  Serial.println("Mode P: Parking");
  nexMode1.setText("P");
}
void modeEc() {
  digitalWrite(cut_off, 1);     // Relay 1 cutoff
  digitalWrite(front_low, 0);   // Relay 2 Font Low
  digitalWrite(front_high, 0);  // Relay 3 Font High
  digitalWrite(rear_low, 0);    // Relay 4 Rear Low
  digitalWrite(rear_high, 0);   // Relay 5 Rear High

  Serial.println("Mode ECO Mode");
  nexMode1.setText("EC");
}
void modeLM() {
  digitalWrite(cut_off, 1);     // Relay 1 cutoff
  digitalWrite(front_low, 1);   // Relay 2 Font Low
  digitalWrite(front_high, 0);  // Relay 3 Font High
  digitalWrite(rear_low, 1);    // Relay 4 Rear Low
  digitalWrite(rear_high, 0);   // Relay 5 Rear High

  Serial.println("Mode LM: Low Mode");
  nexMode1.setText("LM");
}
void modeS() {
  digitalWrite(cut_off, 1);     // Relay 1 cutoff
  digitalWrite(front_low, 0);   // Relay 2 Font Low
  digitalWrite(front_high, 0);  // Relay 3 Font High
  digitalWrite(rear_low, 0);    // Relay 4 Rear Low
  digitalWrite(rear_high, 1);   // Relay 5 Rear High

  Serial.println("Mode S: Sport");
  nexMode1.setText("S");
}
void modeR() {
  digitalWrite(cut_off, 1);     // Relay 1 cutoff
  digitalWrite(front_low, 0);   // Relay 2 Font Low
  digitalWrite(front_high, 1);  // Relay 3 Font High
  digitalWrite(rear_low, 0);    // Relay 4 Rear Low
  digitalWrite(rear_high, 1);   // Relay 5 Rear High

  Serial.println("Mode R: Race");
  nexMode1.setText(" ");

  if (angleX < 55) {
    digitalWrite(20, HIGH);  //รีเลย์ตัวที่ 1 cutoff
  }
  if (angleX < 65) {
    digitalWrite(20, HIGH);  //รีเลย์ตัวที่ 1 cutoff
  }
  if (angleX < 75) {
    digitalWrite(20, HIGH);  //รีเลย์ตัวที่ 1 cutoff
  }
  if (angleX < 85) {
    digitalWrite(20, HIGH);  //รีเลย์ตัวที่ 1 cutoff
  }
  if (angleX > 85 && angleX < 95) {
    digitalWrite(20, HIGH);  //รีเลย์ตัวที่ 1 cutoff
  }
  if (angleX > 95) {
    digitalWrite(20, HIGH);  //รีเลย์ตัวที่ 1 cutoff
  }
  if (angleX > 105) {
    digitalWrite(20, HIGH);  //รีเลย์ตัวที่ 1 cutoff
  }
  if (angleX > 115) {
    digitalWrite(20, HIGH);  //รีเลย์ตัวที่ 1 cutoff
  }
  if (angleX > 125) {
    digitalWrite(20, HIGH);  //รีเลย์ตัวที่ 1 cutoff
  }
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

void ResetPushCallback(void *ptr) {
  Serial.println("Resettttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttt");
  maxSpeed = 0;
  EEPROM.write(address_distance, 0);
  Trip1.setValue(0);
  Serial.println(maxSpeed);
  maxSpeed1.setValue(0);
}


void rfid_scan() {

  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    tag = rfid_read();
    Serial.println(">>>> " + tag);
    if (tag == tagUser1 || tag == tagUser2 || tag == tagUser3 || tag == tagUser4 || tag == tagUser5 || tag == tagUser6) {
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
  float pitch = 0.0f;
  float roll = 0.0f;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  pitch = atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
  roll = atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI;
  roll = roll - 9;
  float val = map(roll, 90, -90, 0, 180);
  // Print out the calculated angles
  // Serial.print("Pitch: ");
  // Serial.print(pitch);
  // Serial.println(" degrees");

  // Serial.print("Roll: ");
  // Serial.print(roll);
  // Serial.println(" degrees");

  Serial.print("val: ");
  Serial.print(val);
  Serial.println(" degrees");
  Serial.println("");
  updateDisplay(val);
  delay(50);
}
