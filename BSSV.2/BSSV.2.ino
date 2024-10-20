#include <WiFi.h>
// Access Point credentials
const char* ssid = "ESP32_AP";
const char* password = "password";

// GPIO Pins
#define SWITCH_1 4
#define SWITCH_2 5
#define LED_1 2
#define LED_2 17

WiFiServer server(80);

WiFiClient client1;
WiFiClient client2;

bool client1_led_state = false;  // Track LED state for Client1
bool client2_led_state = false;  // Track LED state for Client2

// Predefined IP addresses for clients (use static IP for clients or get it dynamically)
IPAddress client1_ip(192, 168, 4, 2);  // Set the correct IP for Client1
IPAddress client2_ip(192, 168, 4, 3);  // Set the correct IP for Client2

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
  // Check for new client connections
  WiFiClient client = server.available();
  if (client) {
    IPAddress client_ip = client.remoteIP();
    if (client_ip == client1_ip) {
      client1 = client;
      Serial.println("Client1 connected.");
    } else if (client_ip == client2_ip) {
      client2 = client;
      Serial.println("Client2 connected.");
    }
  }

  // Update LED status based on connection status
  if (client1 && client1.connected()) {                                                    
    digitalWrite(LED_1, HIGH);  // Turn on LED 1 if Client1 is connected
  } else {
    digitalWrite(LED_1, LOW);  // Turn off LED 1 if Client1 is not connected
  }

  if (client2 && client2.connected()) {
    digitalWrite(LED_2, HIGH);  // Turn on LED 2 if Client2 is connected
  } else {
    digitalWrite(LED_2, LOW);  // Turn off LED 2 if Client2 is not connected
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
