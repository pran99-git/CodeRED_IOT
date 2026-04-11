#include <WiFi.h>
#include <WiFiUdp.h>

// ===== WiFi credentials =====
const char* ssid = "SSID";
const char* password = "PASSWORD";

// ===== UDP setup =====
WiFiUDP udp;
const int udpPort = 5005;

// ===== Hardware pins =====
#define LIGHT_PIN 34      // Analog photoresistor input
#define LED_ONBOARD 2     // Onboard LED
#define LED_EXT 16        // External LED (brightness indicator)

// ===== LED Bar Graph Pins (10 LEDs) =====
const int LED_BAR[] = {25, 26, 27, 14, 12, 13, 15, 4, 5, 18};
const int NUM_LEDS = 10;

// ===== PWM Configuration =====
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// ===== Swarm Structure =====
struct OtherESP {
  IPAddress ip;
  int value;
  unsigned long lastSeen;
};

OtherESP otherESPs[2];  
int otherESPCount = 0;

// ===== Variables =====
int lightValue = 0;
IPAddress masterIP;
bool isMaster = false;

unsigned long lastBroadcast = 0;
unsigned long lastPacketTime = 0;
unsigned long lastMasterCheck = 0;

#define SILENCE_THRESHOLD 100     // ms
#define ESP_TIMEOUT 3000          // ms
#define MASTER_CHECK_INTERVAL 500 // ms

// ====================================================================================
// UPDATE LED BAR GRAPH
// ====================================================================================
void updateLEDBar(int value) {
  // Map value (0-4095) to number of LEDs to light (0-10)
  int numLedsOn = map(value, 0, 4095, 0, NUM_LEDS);
  
  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < numLedsOn) {
      digitalWrite(LED_BAR[i], HIGH);
    } else {
      digitalWrite(LED_BAR[i], LOW);
    }
  }
}

// ====================================================================================
// SETUP
// ====================================================================================
void setup() {
  Serial.begin(115200);

  pinMode(LED_ONBOARD, OUTPUT);
  pinMode(LIGHT_PIN, INPUT);
  digitalWrite(LED_ONBOARD, LOW);

  // Setup LED Bar Graph pins
  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(LED_BAR[i], OUTPUT);
    digitalWrite(LED_BAR[i], LOW);
  }

  // Setup PWM for external LED
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(LED_EXT, PWM_FREQ, PWM_RESOLUTION);
#else
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LED_EXT, 0);
#endif

  // WiFi Setup
  Serial.println("\n=== ESP32 Swarm Node with LED Bar ===");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting");

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("My IP: ");
  Serial.println(WiFi.localIP());

  // Start UDP
  udp.begin(udpPort);
  Serial.print("UDP started on port ");
  Serial.println(udpPort);

  lastPacketTime = millis();
}

// ====================================================================================
// CLEAN STALE ESPs
// ====================================================================================
void cleanupStaleESPs() {
  int writeIdx = 0;

  for (int i = 0; i < otherESPCount; i++) {
    if (millis() - otherESPs[i].lastSeen < ESP_TIMEOUT) {
      otherESPs[writeIdx++] = otherESPs[i];
    } else {
      Serial.printf("Removing stale ESP: %s\n",
                    otherESPs[i].ip.toString().c_str());
    }
  }

  otherESPCount = writeIdx;
}

// ====================================================================================
// DETERMINE MASTER
// ====================================================================================
void determineMaster() {
  int highestVal = lightValue;
  IPAddress highestIP = WiFi.localIP();

  for (int i = 0; i < otherESPCount; i++) {
    if (otherESPs[i].value > highestVal) {
      highestVal = otherESPs[i].value;
      highestIP = otherESPs[i].ip;
    } else if (otherESPs[i].value == highestVal) {
      if ((uint32_t)otherESPs[i].ip < (uint32_t)WiFi.localIP()) {
        highestIP = otherESPs[i].ip;
      }
    }
  }

  masterIP = highestIP;
  isMaster = (masterIP == WiFi.localIP());
  digitalWrite(LED_ONBOARD, isMaster);

  if (isMaster) {
    Serial.printf("MASTER: %s  VALUE: %d\n",
                  WiFi.localIP().toString().c_str(), lightValue);
  }
}

// ====================================================================================
// MAIN LOOP
// ====================================================================================
void loop() {
  unsigned long now = millis();

  // ---- Read sensor ----
  lightValue = analogRead(LIGHT_PIN);

  // Update LED bar graph based on light intensity
  updateLEDBar(lightValue);

  // Update PWM LED brightness
  int brightness = map(lightValue, 0, 4095, 0, 255);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(LED_EXT, brightness);
#else
  ledcWrite(0, brightness);
#endif

  // ---- Receive swarm packets ----
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char buffer[255];
    int len = udp.read(buffer, 255);
    if (len > 0) buffer[len] = '\0';

    String msg(buffer);
    IPAddress senderIP = udp.remoteIP();

    // RESET from Raspberry Pi
    if (msg.startsWith("RESET")) {
      Serial.println("Received RESET → restarting...");
      delay(300);
      ESP.restart();
    }

    // Handle swarm data packet: "DATA:1234"
    if (msg.startsWith("DATA:") && senderIP != WiFi.localIP()) {
      int val = msg.substring(5).toInt();

      bool found = false;
      for (int i = 0; i < otherESPCount; i++) {
        if (otherESPs[i].ip == senderIP) {
          otherESPs[i].value = val;
          otherESPs[i].lastSeen = now;
          found = true;
          break;
        }
      }

      if (!found && otherESPCount < 2) {
        otherESPs[otherESPCount] = {senderIP, val, now};
        otherESPCount++;
        Serial.printf("New ESP discovered: %s\n", senderIP.toString().c_str());
      }

      lastPacketTime = now;
    }
  }

  // ---- Broadcast my value during network silence ----
  if ((now - lastPacketTime >= SILENCE_THRESHOLD) &&
      (now - lastBroadcast >= SILENCE_THRESHOLD)) {

    char msg[40];
    sprintf(msg, "DATA:%d", lightValue);

    udp.beginPacket(IPAddress(255, 255, 255, 255), udpPort);
    udp.write((uint8_t*)msg, strlen(msg));
    udp.endPacket();

    lastBroadcast = now;
  }

  // ---- Master election ----
  if (now - lastMasterCheck >= MASTER_CHECK_INTERVAL) {
    cleanupStaleESPs();
    determineMaster();
    lastMasterCheck = now;
  }

  // ---- Master sends data to Raspberry Pi (JSON) ----
  if (isMaster) {
    char json[200];
    sprintf(json,
      "{\"master\":\"%s\",\"value\":%d}",
      WiFi.localIP().toString().c_str(),
      lightValue
    );

    udp.beginPacket(IPAddress(255, 255, 255, 255), udpPort);
    udp.write((uint8_t*)json, strlen(json));
    udp.endPacket();
  }

  delay(10);
}
