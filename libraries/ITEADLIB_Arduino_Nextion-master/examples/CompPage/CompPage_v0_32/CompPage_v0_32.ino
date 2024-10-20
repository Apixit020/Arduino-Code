#include <Nextion.h>       // ไลบรารีสำหรับการสื่อสารกับหน้าจอ Nextion
#define nexSerial Serial2  // กำหนด nexSerial ให้เป็น Serial2 สำหรับการสื่อสารกับ Nextion
// #define HWSerial Serial2   // กำหนด HWSerial ให้เป็น Serial2
#include <EEPROM.h>       // ไลบรารีสำหรับการใช้งาน EEPROM
#include <SPI.h>          // ไลบรารีสำหรับการสื่อสาร SPI
#include <MFRC522.h>      // ไลบรารีสำหรับการใช้งาน RFID MFRC522
#include <Wire.h>         // ไลบรารีสำหรับการสื่อสาร I2C
#include <NexButton.h>    // ไลบรารีสำหรับการใช้งานปุ่มกับ Nextion HMI
#include <NexTouch.h>     // ไลบรารีสำหรับการจัดการอีเวนต์การสัมผัสกับ Nextion HMI
#include <NexHardware.h>  // ไลบรารีสำหรับการเชื่อมต่อกับฮาร์ดแวร์ของ Nextion

NexPage page0 = NexPage(0, 0, "page0");
NexPage page1 = NexPage(1, 0, "page1");
NexPage page2 = NexPage(2, 0, "page2");
NexPage page3 = NexPage(3, 0, "page3");

NexTouch *nex_listen_list[] = {
  &page0,
  &page1,
  &page2,
  &page3,
  NULL
};

void page0PopCallback(void *ptr) {
  dbSerialPrintln("page0PopCallback");
  page1.show();
}

void page1PopCallback(void *ptr) {
  dbSerialPrintln("page1PopCallback");
  page2.show();
}

void page2PopCallback(void *ptr) {
  dbSerialPrintln("page2PopCallback");
  page3.show();
}

void page3PopCallback(void *ptr) {
  dbSerialPrintln("page3PopCallback");
  page0.show();
}

void setup(void) {
  nexInit();
  dbSerialPrintln("setup begin");

  page0.attachPop(page0PopCallback);
  page1.attachPop(page1PopCallback);
  page2.attachPop(page2PopCallback);
  page3.attachPop(page3PopCallback);

  dbSerialPrintln("setup end");
}

void loop(void) {
  nexLoop(nex_listen_list);
}
