#include <Keypad.h>

const byte ROWS = 4; // four rows
const byte COLS = 4; // four columns
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {9, 8, 7, 6};
byte colPins[COLS] = {5, 4, 3, 2};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

const String correctPassword = "12123456";
String inputPassword = "";
const int relayPin = 10; // ตั้งค่าหมายเลขขาของ Relay
bool isUnlocked = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Please Enter Key : ");
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW); // ปิด Relay ในตอนเริ่มต้น
}

void loop() {
  char key = keypad.getKey();
  if (key != NO_KEY) {
    Serial.println(key);

    if (key == 'C') {
      inputPassword = ""; // ล้างรหัสผ่านที่กรอก
      Serial.println("Cleared");
    } else if (key == 'B') {
      if (inputPassword == correctPassword) {
        isUnlocked = true;
        Serial.println("Correct Password");
        digitalWrite(relayPin, HIGH); // ปลดล็อก Relay
      } else {
        Serial.println("Incorrect Password");
      }
      inputPassword = ""; // ล้างรหัสผ่านหลังจากตรวจสอบแล้ว
    } else if (key == 'A' && isUnlocked) {
      Serial.println("Relay Unlocked");
      digitalWrite(relayPin, LOW); // ล็อก Relay อีกครั้ง
      isUnlocked = false;
    } else {
      inputPassword += key; // เพิ่มตัวอักษรที่กดเข้ารหัสผ่าน
    }
  }
}
