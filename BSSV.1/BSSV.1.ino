#include <WiFi.h>

// Access Point credentials
const char* ssid = "ESP32_AP";
const char* password = "password";

// GPIO Pins
#define SWITCH_1 5
#define SWITCH_2 4
#define LED_1 16
#define LED_2 17

WiFiServer server(80);

WiFiClient client1;
WiFiClient client2;

bool client1_led_state = false;  // Track LED state for Client1
bool client2_led_state = false;  // Track LED state for Client2

void setup() {
  pinMode(SWITCH_1, INPUT_PULLUP);
  pinMode(SWITCH_2, INPUT_PULLUP);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);

  // Start Access Point
  Serial.begin(115200);
  Serial.println("Starting Access Point...");
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  server.begin();

  // Initialize LEDs as off
  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, LOW);
  Serial.println("Setup complete.");
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    // Identify client and assign to client1 or client2
    Serial.println("New client connected.");
    if (!client1 && client.remoteIP() == WiFi.softAPIP()) {
      client1 = client;
      Serial.println("Client1 connected.");
    } else if (!client2 && client.remoteIP() != client1.remoteIP()) {
      client2 = client;
      Serial.println("Client2 connected.");
    }
  }

  // Update LED status based on connection status
  if (client1 && client1.connected()) {
    digitalWrite(LED_1, HIGH);  // Turn on LED 1 if Client1 is connected
    Serial.println("LED 1 ON (Client1 connected)");
  } else {
    digitalWrite(LED_1, LOW);  // Turn off LED 1 if Client1 is not connected
    Serial.println("LED 1 OFF (Client1 disconnected)");
  }

  if (client2 && client2.connected()) {
    digitalWrite(LED_2, HIGH);  // Turn on LED 2 if Client2 is connected
    Serial.println("LED 2 ON (Client2 connected)");
  } else {
    digitalWrite(LED_2, LOW);  // Turn off LED 2 if Client2 is not connected
    Serial.println("LED 2 OFF (Client2 disconnected)");
  }

  // Check switch status and toggle LED state for clients
  if (digitalRead(SWITCH_1) == LOW) {
    delay(50);  // Debounce delay
    if (digitalRead(SWITCH_1) == LOW) {
      client1_led_state = !client1_led_state;  // Toggle state
      sendCommandToClient(client1, client1_led_state ? "TURN_ON_LED" : "TURN_OFF_LED");
      Serial.println(client1_led_state ? "Client1 LED ON" : "Client1 LED OFF");
      while (digitalRead(SWITCH_1) == LOW)
        ;  // Wait for button release
    }
  }

  if (digitalRead(SWITCH_2) == LOW) {
    delay(50);  // Debounce delay
    if (digitalRead(SWITCH_2) == LOW) {
      client2_led_state = !client2_led_state;  // Toggle state
      sendCommandToClient(client2, client2_led_state ? "TURN_ON_LED" : "TURN_OFF_LED");
      Serial.println(client2_led_state ? "Client2 LED ON" : "Client2 LED OFF");
      while (digitalRead(SWITCH_2) == LOW)
        ;  // Wait for button release
    }
  }
}

void sendCommandToClient(WiFiClient& client, const char* command) {
  if (client && client.connected()) {
    client.println(command);  // Send command to client
    Serial.print("Command sent to client: ");
    Serial.println(command);
  } else {
    Serial.println("Failed to send command: Client disconnected");
  }
}