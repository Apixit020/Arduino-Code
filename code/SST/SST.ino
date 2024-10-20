#include <TridentTD_LineNotify.h>

#define SSID        "ระอุยจุ้บๆ"
#define PASSWORD    "PUNYISANNN"
#define LINE_TOKEN  "uFoR3kVR1EUnHU2KAGO9Wqwo0cDIXhYuhFhZEt2l6qS"

int sensor = 15;

void setup() {
  Serial.begin(115200); Serial.println();
  Serial.println(LINE.getVersion());
  
  WiFi.begin(SSID, PASSWORD);
  Serial.printf("WiFi connecting to %s\n",  SSID);
  while(WiFi.status() != WL_CONNECTED) { Serial.print("."); delay(400); }
  Serial.printf("\nWiFi connected\nIP : ");
  Serial.println(WiFi.localIP());  

  // กำหนด Line Token
  LINE.setToken(LINE_TOKEN);

  pinMode(sensor, INPUT);
}

void loop() {
  int sensorValue = digitalRead(sensor);
  Serial.println(sensorValue);
  
  if (sensorValue == 1) {
    // แจ้งเตือนเมื่อค่า sensor เป็น 1
    LINE.notify("มีโจรปล้น!!!");
  }
  
  delay(100);
}
