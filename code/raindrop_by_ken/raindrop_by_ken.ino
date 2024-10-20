#include <TridentTD_LineNotify.h>

#define SSID "--------------------"
#define PASSWORD "--------------------"
#define LINE_TOKEN "--------------------"

int sensorVal = 0;

void setup() {
  Serial.begin(115200);
  Serial.println();
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
}

void loop() {
  sensorVal = analogRead(A0);
  sensorVal = map(sensorVal, 0, 1023, 100, 0);
  Serial.print("Rain percentage: ");
  Serial.println(sensorVal);
  delay(100);
  if (sensorVal >= 30) {
    digitalWrite(r, 1);
    digitalWrite(g, 0);

  } else {
    digitalWrite(g, 1);
    digitalWrite(r, 0);
  }
}
