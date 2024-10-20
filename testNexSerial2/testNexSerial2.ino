// #include <Nextion.h>

#define nexSerial Serial2

void setup() {
  nexSerial.begin(115200);  // ตั้งค่า Baud Rate เป็น 9600 bps
  Serial.begin(115200);     // ตั้งค่า Baud Rate ของ Serial Monitor

  Serial.println("Setup complete. Waiting for data from Nextion...");
  sendCurrentPageRequest();
}

void loop() {
  if (nexSerial.available()) {
    int pageID = nexSerial.read();  // Read the current page ID
    Serial.print("Current Page ID: ");
    Serial.println(pageID);
    checkCurrentPage(pageID);  // Check and print page information
  }
}

void sendCurrentPageRequest() {
  nexSerial.print("page");
  nexSerial.write(0xFF);
  nexSerial.write(0xFF);
  nexSerial.write(0xFF);
}

void checkCurrentPage(int pageID) {
  if (pageID == 4) {
    Serial.println("Currently on MAIN page.");
  } else if (pageID == 2) {
    Serial.println("Currently on PIN_INPUT page.");
  } else {
    Serial.println("Unknown page.");
  }
}