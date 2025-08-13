#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#define BUTTON_PIN 0

// Target peer MAC (receiver)
uint8_t peerMac[] = {0xF4, 0x12, 0xFA, 0x82, 0xE9, 0x94};

const char* kMsg = "asdjkhaklshdhjkldsa/0";

// Simple debounce
static const uint32_t DEBOUNCE_MS = 30;
bool lastStableState = HIGH;  // with INPUT_PULLUP: HIGH = not pressed
bool lastReading      = HIGH;
uint32_t lastChangeMs = 0;

void onEspNowSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optional: log send status
  Serial.print("ESP-NOW send to ");
  for (int i = 0; i < 6; i++) {
    if (i) Serial.print(':');
    Serial.printf("%02X", mac_addr[i]);
  }
  Serial.print(" -> ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

bool ensurePeerAdded(const uint8_t mac[6]) {
  // Remove if exists, then add fresh (avoids duplicates)
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 1;          // Must match receiver's channel. Use 0 to follow current channel.
  peer.encrypt = false;      // Set true only if you’ve paired and have an LTK
  if (esp_now_is_peer_exist(mac)) {
    esp_now_del_peer(mac);
  }
  return esp_now_add_peer(&peer) == ESP_OK;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("ESP-NOW button TX on GPIO0");

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // WiFi must be in STA mode for ESP-NOW
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (true) { delay(1000); }
  }

  esp_now_register_send_cb(onEspNowSend);

  if (!ensurePeerAdded(peerMac)) {
    Serial.println("Failed to add ESP-NOW peer!");
    while (true) { delay(1000); }
  }

  Serial.println("Ready. Press the button on GPIO0 to send.");
}

void loop() {
  // Debounced edge detection
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastReading) {
    lastChangeMs = millis();
    lastReading = reading;
  }

  if ((millis() - lastChangeMs) >= DEBOUNCE_MS) {
    if (reading != lastStableState) {
      lastStableState = reading;

      // Detect the press (transition to LOW)
      if (lastStableState == LOW) {
        // Send the string via ESP-NOW
        esp_err_t err = esp_now_send(peerMac, reinterpret_cast<const uint8_t*>(kMsg), strlen(kMsg));
        if (err != ESP_OK) {
          Serial.print("esp_now_send error: ");
          Serial.println(err);
        }
      }
    }
  }
}