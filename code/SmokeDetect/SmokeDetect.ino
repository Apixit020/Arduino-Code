#include <TridentTD_LineNotify.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(21, 22);  // RX, TX
unsigned int pm1 = 0;
unsigned int pm2_5 = 0;
unsigned int pm10 = 0;

#define SSID "iPhone"
#define PASSWORD "123456789"
#define LINE_TOKEN "av79EwnKuEyoOKzLwXS91HwIdOoZh2ZK7XeUvQVHRxp"

int ledPin = 2;
int analogPin = 36;  //ประกาศตัวแปร ให้ analogPin แทนขา analog ขาที่5
int val = 0;
bool notified = false;  // ตัวแปรสถานะสำหรับการแจ้งเตือน

void setup() {
  Serial.begin(115200);
  Serial.println();
  mySerial.begin(9600);
  Serial.println(LINE.getVersion());

  WiFi.begin(SSID, PASSWORD);
  Serial.printf("WiFi connecting to %s\n", SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(400);
  }
  Serial.printf("\nWiFi connected\nIP : ");
  Serial.println(WiFi.localIP());

  // กำหนด Line Token
  LINE.setToken(LINE_TOKEN);
  pinMode(ledPin, OUTPUT);  // sets the pin as output
  // pinMode(sensor, INPUT);
}

void loop() {
  int index = 0;
  char value;
  char previousValue;
  while (mySerial.available()) {
    value = mySerial.read();
    if ((index == 0 && value != 0x42) || (index == 1 && value != 0x4d)) {
      Serial.println("Cannot find the data header.");
      break;
    }

    if (index == 4 || index == 6 || index == 8 || index == 10 || index == 12 || index == 14) {
      previousValue = value;
    } else if (index == 5) {
      pm1 = 256 * previousValue + value;
      Serial.print("{ ");
      Serial.print("\"pm1\": ");
      Serial.print(pm1);
      Serial.print(" ug/m3");
      Serial.print(", ");
    } else if (index == 7) {
      pm2_5 = 256 * previousValue + value;
      Serial.print("\"pm2_5\": ");
      Serial.print(pm2_5);
      Serial.print(" ug/m3");
      Serial.print(", ");
    } else if (index == 9) {
      pm10 = 256 * previousValue + value;
      Serial.print("\"pm10\": ");
      Serial.print(pm10);
      Serial.print(" ug/m3");
    } else if (index > 15) {
      break;
    }
    index++;
  }
  while (mySerial.available()) mySerial.read();
  Serial.println(" }");
  delay(1000);

  val = analogRead(analogPin);    //อ่านค่าสัญญาณ analog ขา5
  Serial.print("val = ");         // พิมพ์ข้อมความส่งเข้าคอมพิวเตอร์ "val = "
  Serial.println(val);            // พิมพ์ค่าของตัวแปร val
  if (val > 2000 && !notified) {  // สามารถกำหนดปรับค่าได้ตามสถานที่ต่างๆ
    digitalWrite(ledPin, HIGH);   // สั่งให้ LED ติดสว่าง
    LINE.notify("มีคนสูบบุหรี่");
    notified = true;  // เปลี่ยนสถานะเป็นแจ้งเตือนแล้ว
  }
  if (pm10 > 250 && !notified) {
    digitalWrite(ledPin, HIGH);  // สั่งให้ LED ติดสว่าง
    LINE.notify("มีคนสูบบุหรี่");
    notified = true;
  } else {
    digitalWrite(ledPin, LOW);  // สั่งให้ LED ดับ
    notified = false;           // เปลี่ยนสถานะเป็นแจ้งเตือนแล้ว
  }
}
