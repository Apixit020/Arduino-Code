int sensor = 4; //ประกาศขาใช้อ่านค่า sensor
int led = 2; //ขาที่เชื่อมต่อกับ led
void setup() {
  Serial.begin(9600); //ประกาศใช้งาน serial monitor
  pinMode(sensor,INPUT); //ให้ sensor เป็น input
  pinMode(led,OUTPUT); //ให้ led เป็น output
}
void loop() {
  int val = digitalRead(sensor); //ประกาศตัวแปร val เพื่อเก็บค่าที่อ่านได้จาก sensor
  Serial.println(val); //serial monitor แสดงค่า val
  if(val == 0){ 
    digitalWrite(led, 0); 
    Serial.println("LED ON");
  } //คำสั่ง if ถ้าค่า val มีค่าเท่ากับ 0 ให้ led ติด
  else{
    digitalWrite(led, 1);
    Serial.println("LED OFF");
  } //ถ้าไม่ใช่ ให้ led ดับ
}