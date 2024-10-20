#include <SPI.h>
#include <Wire.h>
#include "WiFi.h"
#include <MFRC522.h>
#include <HTTPClient.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <TridentTD_LineNotify.h>

#define On_Board_LED_PIN 2
#define SS_PIN 5   // ESP32 pin GIOP5
#define RST_PIN 4  // ESP32 pin GIOP27

// MFRC522 rfid(SS_PIN, RST_PIN);

MFRC522 mfrc522(SS_PIN, RST_PIN);  //--> Create MFRC522 instance.

const int pingPin = 25;
int inPin = 26;

const char* ssid = "iPhone 13 Pro Max";  //--> Your wifi name
const char* password = "Pspbrp1207";       //--> Your wifi password

String Web_App_URL = "https://script.google.com/macros/s/AKfycbzj8nDjzrZOM7BttTVniQzUUpRHgdlOPXRRLtFMeyaptsfDfrjmfcjuX9VYhn6Z0d2s/exec";

#define LINE_TOKEN "exQyLb8Ig9n8Xzgu6bdKUaV77risyXxhLGJ8l8a0yg1"

const int buttonPin1 = 16;  // กำหนดขาของปุ่ม 1
const int buttonPin2 = 15;  // กำหนดขาของปุ่ม 2

int IR = 27;   //เซนเซอร์  IR
int IR2 = 14;  //เซนเซอร์  IR

Servo servo;  //หมุนขวด

int countPoint;  //นับจำนวนขวด
int irState;     //เช็คขวด

bool notified = false;  //line noti status

int readsuccess;

char str[32] = "";
String UID_Result = "--------";

int lcdColumns = 20;
int lcdRows = 4;

String reg_Info = "";

String atc_Name = "";
String atc_Code = "";
String atc_Info = "";
String point = "";

LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  // (lcd_address, lcd_Columns, lcd_Rows)

int mode = 0;  // เก็บสถานะโหมด
int state = 0;

int connecting_process_timed_out = 20;  //--> 20 = 20 seconds.

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// Subroutine to obtain UID/ID when RFID card or RFID keychain is tapped to RFID-RC522 module.
int getUID() {
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return 0;
  }
  if (!mfrc522.PICC_ReadCardSerial()) {
    return 0;
  }

  byteArray_to_string(mfrc522.uid.uidByte, mfrc522.uid.size, str);
  UID_Result = str;

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();

  return 1;
}
//________________________________________________________________________________

//________________________________________________________________________________byteArray_to_string()
void byteArray_to_string(byte array[], unsigned int len, char buffer[]) {
  for (unsigned int i = 0; i < len; i++) {
    byte nib1 = (array[i] >> 4) & 0x0F;
    byte nib2 = (array[i] >> 0) & 0x0F;
    buffer[i * 2 + 0] = nib1 < 0xA ? '0' + nib1 : 'A' + nib1 - 0xA;
    buffer[i * 2 + 1] = nib2 < 0xA ? '0' + nib2 : 'A' + nib2 - 0xA;
  }
  buffer[len * 2] = '\0';
}
//________________________________________________________________________________


void setup() {
  Serial.begin(115200);
  Serial.println();
  delay(1000);

  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(IR, INPUT);
  pinMode(IR2, INPUT);
  servo.attach(13);
  servo.write(0);

  pinMode(On_Board_LED_PIN, OUTPUT);

  SPI.begin();         // init SPI bus
  mfrc522.PCD_Init();  // init MFRC522
  lcd.init();
  lcd.backlight();


  Serial.println();
  Serial.println("-------------");
  Serial.println("WIFI mode : STA");
  WiFi.mode(WIFI_STA);
  Serial.println("-------------");

  Serial.println();
  Serial.println("------------");
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  lcd.print("connecting...");

  int connecting_process_timed_out = 20;  //--> 20 = 20 seconds.
  connecting_process_timed_out = connecting_process_timed_out * 2;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    lcd.print(".");
    digitalWrite(On_Board_LED_PIN, HIGH);
    delay(250);
    digitalWrite(On_Board_LED_PIN, LOW);
    delay(250);
    if (connecting_process_timed_out > 0) connecting_process_timed_out--;
    if (connecting_process_timed_out == 0) {
      delay(1000);
      ESP.restart();
    }
  }

  digitalWrite(On_Board_LED_PIN, LOW);

  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("------------");
  LINE.setToken(LINE_TOKEN);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi connected");
  //::::::::::::::::::
  //----------------------------------------

  delay(100);

  Serial.println();

  delay(2000);
  lcd.clear();
}

void loop() {
  Serial.print(digitalRead(buttonPin1));
  Serial.print("\t");
  Serial.println(digitalRead(buttonPin2));

  noti();
  handleMode();  // ตรวจสอบและจัดการโหมด

  readsuccess = getUID();

  delay(1000);
}

void noti() {
  long duration, cm;

  pinMode(pingPin, OUTPUT);


  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  pinMode(inPin, INPUT);
  duration = pulseIn(inPin, HIGH);

  cm = microsecondsToCentimeters(duration);

  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

  if (cm <= 10 && !notified) {
    // ถ้าใกล้เต็มแล้ว และยังไม่ได้แจ้งเตือน
    // ให้ส่งข้อความแจ้งเตือนไปยัง LINE Notify
    LINE.notify("ถังขยะใกล้เต็มแล้วครับ");
    notified = true;  // ตั้งค่าว่าได้แจ้งเตือนแล้ว
  } else if (cm > 10 && notified) {
    // หากระยะทางมากกว่า 10 เซนติเมตร และเคยแจ้งเตือนไปแล้ว
    // กำหนดให้ notified เป็น false เพื่อให้สามารถแจ้งเตือนอีกครั้งในการวน loop ถัดไป
    notified = false;
  }

  delay(1000);
}


long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

void handleMode() {

  // ตรวจสอบว่าปุ่ม 1 หรือ 2 ถูกกดหรือไม่
  if (digitalRead(buttonPin1) == 0 && mode != 1) {
    mode = 1;  // เปลี่ยนเป็นโหมดฝาก
    Serial.println("Switching to Deposit mode");
    lcd.setCursor(0, 0);
    lcd.print("               ");
    lcd.setCursor(0, 0);
    lcd.print("Deposit Mode");
    delay(1000);
    lcd.clear();
  } else if (digitalRead(buttonPin2) == 0 && mode != 2) {
    mode = 2;  // เปลี่ยนเป็นโหมดลงทะเบียน RFID
    Serial.println("Switching to RFID Registration mode");
    lcd.setCursor(0, 0);
    lcd.print("               ");
    lcd.setCursor(0, 0);
    lcd.print("Register Mode");
    delay(1000);
    lcd.clear();
  }

  // ตรวจสอบและทำงานตามโหมดปัจจุบัน
  switch (mode) {  //deflute Mode
    case 0:
      Serial.println();
      Serial.println("Welcome");
      Serial.println("1. Deposit");
      Serial.println("2. Register");
      Serial.println();
      Serial.println(countPoint);
      lcd.print("               ");
      lcd.setCursor(0, 0);
      lcd.print("Select DP / REG");

      break;
    case 1:  //collector Mode
      lcd.setCursor(0, 0);
      lcd.print("               ");
      lcd.setCursor(2, 0);
      lcd.print("DEPOSIT MODE");
      lcd.setCursor(2, 1);
      lcd.print("Tap you card");
      if (readsuccess) {
        lcd.clear();
        delay(500);
        lcd.setCursor(4, 0);
        lcd.print("Getting  UID");
        lcd.setCursor(4, 1);
        lcd.print("Successfully");
        lcd.setCursor(0, 2);
        lcd.print("");
        lcd.setCursor(3, 3);
        lcd.print("Please wait...");
        delay(1000);

        String Read_Data_URL = Web_App_URL + "?sts=dps&uid=" + UID_Result;

        Serial.println();
        Serial.println("-------------");
        Serial.println("Read data from Google Spreadsheet...");
        Serial.print("URL : ");
        Serial.println(Read_Data_URL);

        HTTPClient http;

        // HTTP GET Request.
        http.begin(Read_Data_URL.c_str());
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

        // Gets the HTTP status code.
        int httpCode = http.GET();
        Serial.print("HTTP Status Code : ");
        Serial.println(httpCode);

        String payload;
        if (httpCode > 0) {
          payload = http.getString();
          Serial.println("Payload : " + payload);
        }
        if (httpCode == 200) {
          lcd.clear();
          mode = 0;
        }


        http.end();

        String sts_Res = getValue(payload, ',', 0);

        if (sts_Res == "OK") {

          atc_Info = getValue(payload, ',', 1);
          atc_Name = getValue(payload, ',', 2);
          atc_Code = getValue(payload, ',', 3);

          int name_Lenght = atc_Name.length();

          if (name_Lenght == 0) {
            mode = 0;
          }
          // int pos = 0;
          // if (name_Lenght > 0 && name_Lenght <= lcdColumns) {
          //   pos = map(name_Lenght, 1, lcdColumns, 0, (lcdColumns / 2) - 1);
          //   pos = ((lcdColumns / 2) - 1) - pos;
          // } else if (name_Lenght > lcdColumns) {
          //   atc_Name = atc_Name.substring(0, lcdColumns);
          // }
          //::::::::::::::::::
          else {
            lcd.clear();
            delay(500);
            lcd.setCursor(0, 0);
            lcd.print("Name : ");
            lcd.setCursor(7, 0);
            lcd.print(atc_Name);
            lcd.setCursor(0, 1);
            lcd.print("ID : ");
            lcd.setCursor(5, 1);
            lcd.print(atc_Code);
            mode = 3;
          }
        }
      }
      // เพิ่มระบบป้องกันการกดปุ่มซ้ำ

      break;
    case 2:  // register mode
      Serial.println("RFID Registration mode Ready to scan");
      lcd.setCursor(0, 0);
      lcd.print("               ");
      lcd.setCursor(2, 0);
      lcd.print("REGISTRATION");
      lcd.setCursor(2, 1);
      lcd.print("Tap you card");
      // เพิ่มโค้ดสำหรับอ่านข้อมูล RFID

      // เพิ่มระบบป้องกันการกดปุ่มซ้ำ
      if (readsuccess) {
        Serial.print("Thank you");
        Serial.println("Pls. few minute");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("UID : ");
        lcd.print(UID_Result);
        delay(2000);
        lcd.clear();
        lcd.setCursor(3, 0);
        lcd.print("Thank you");
        lcd.setCursor(0, 1);
        lcd.print("Pls. few minute");
        delay(1000);
        digitalWrite(On_Board_LED_PIN, HIGH);

        // Create a URL for reading or getting data from Google Sheets.
        String Read_Data_URL = Web_App_URL + "?sts=reg&uid=" + UID_Result;

        Serial.println();
        Serial.println("-------------");
        Serial.println("Read data from Google Spreadsheet...");
        Serial.print("URL : ");
        Serial.println(Read_Data_URL);

        HTTPClient http;

        // HTTP GET Request.
        http.begin(Read_Data_URL.c_str());
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

        // Gets the HTTP status code.
        int httpCode = http.GET();
        Serial.print("HTTP Status Code : ");
        Serial.println(httpCode);

        String payload;
        if (httpCode > 0) {
          payload = http.getString();
          Serial.println("Payload : " + payload);
        }

        http.end();
        //::::::::::::::::::

        digitalWrite(On_Board_LED_PIN, LOW);
        Serial.println("-------------");

        reg_Info = getValue(payload, ',', 1);

        if (reg_Info == "R_Successful") {
          lcd.clear();
          delay(500);
          lcd.setCursor(2, 0);
          lcd.print("The UID of your");
          lcd.setCursor(0, 1);
          lcd.print("card or keychain has");
          lcd.setCursor(1, 2);
          lcd.print("been successfully");
          lcd.setCursor(6, 3);
          lcd.print("uploaded");
          delay(5000);
          lcd.clear();
          delay(500);
          delay(2000);
          lcd.clear();
          mode = 0;
        }

        if (reg_Info == "regErr01") {
          lcd.clear();
          delay(500);
          lcd.setCursor(6, 0);
          lcd.print("Error !");
          lcd.setCursor(0, 1);
          lcd.print("The UID of your card");
          lcd.setCursor(0, 2);
          lcd.print("or keychain has been");
          lcd.setCursor(5, 3);
          lcd.print("registered");
          delay(5000);
          lcd.clear();
          delay(500);
          mode = 0;
        }

        reg_Info = "";
      }
      //..................

      // //::::::::::::::::::Conditions that are executed if reading or getting data from Google Sheets is successful (HTTP Status Codes : 200).
      // if (httpCode == 200) {
      //   mode = 0;
      // }
      break;
    case 3:
      if (digitalRead(IR) == 0 && digitalRead(IR2) == 0) {
        countPoint++;
        Serial.print("Deposit mode");
        Serial.print("\t");
        Serial.println(countPoint);
        servo.write(180);
        lcd.setCursor(0, 2);
        lcd.print("Point : ");
        lcd.setCursor(8, 2);
        lcd.print(countPoint);
      } else {
        servo.write(0);
      }
      if (readsuccess) {
        Serial.print("You Point : ");
        Serial.println(countPoint);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Save you point...");
        delay(1500);
        digitalWrite(On_Board_LED_PIN, HIGH);

        // Create a URL for sending or writing data to Google Sheets.
        String Send_Data_URL = Web_App_URL + "?sts=atc&uid=" + UID_Result + "&cpt=" + countPoint;

        // Send_Data_URL += "&cpt=" + countPoint;

        Serial.println();
        Serial.println("-------------");
        Serial.println("Send data to Google Spreadsheet...");
        Serial.print("URL : ");
        Serial.println(Send_Data_URL);

        HTTPClient http;

        // HTTP GET Request.
        http.begin(Send_Data_URL.c_str());
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

        // Gets the HTTP status code.
        int httpCode = http.GET();
        Serial.print("HTTP Status Code : ");
        Serial.println(httpCode);

        // Getting response from google sheets.
        String payload;
        if (httpCode > 0) {
          payload = http.getString();
          Serial.println("Payload : " + payload);
        }

        http.end();
        //::::::::::::::::::

        digitalWrite(On_Board_LED_PIN, LOW);
        Serial.println("-------------");

        Serial.println("Saved and Exit");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Complete");
        countPoint = 0;
        mode = 0;
      }
      break;
    default:
      break;
  }
}
