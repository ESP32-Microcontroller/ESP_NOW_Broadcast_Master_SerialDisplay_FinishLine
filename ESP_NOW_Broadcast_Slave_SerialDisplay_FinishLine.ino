/*
    ESP-NOW Broadcast Slave
    Lucas Saavedra Vaz - 2024

    This sketch demonstrates how to receive broadcast messages from a master device using the ESP-NOW protocol.

    The master device will broadcast a message every 5 seconds to all devices within the network.

    The slave devices will receive the broadcasted messages. If they are not from a known master, they will be registered as a new master
    using a callback function.
*/

#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros
#include <vector>

#include <SPI.h>
#include <Wire.h>

#include <ezButton.h>

long startTimeMillis;
bool gateOpen = true;

/* Definitions */
#define ESPNOW_WIFI_CHANNEL 6

#define SCL0_Pin 19
#define SDA0_Pin 20

#define AT_GATE 0
#define RACING 1
#define FINISHED 2
#define RELOADING 3
#define LANES 2

int laneStatus[LANES];
long elapsedTime[LANES];
int breakBeamPin[LANES] = {6, 7};
int finishLineLED[LANES] = {9, 10};
int readyLED = 11;
bool scoresReported = false;
bool allFinished = false;
bool allAtGate = false;
bool commEstablished = false;
ezButton resetSwitch(3);

/* Classes */

// Creating a new class that inherits from the ESP_NOW_Peer class is required.
class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
public:
  // Constructor of the class
  ESP_NOW_Peer_Class(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Peer_Class() {}

  // Function to register the master peer
  bool add_peer() {
    if (!add()) {
      log_e("Failed to register the broadcast peer");
      return false;
    }
    return true;
  }

  // Function to print the received messages from the master
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    Serial.printf("Received a message from master " MACSTR " (%s)\n", MAC2STR(addr()), broadcast ? "broadcast" : "unicast");
    Serial.printf("  Message: %s\n", (char *)data);

    char *buffer = (char*)data;

    if (strcmp(buffer, "START_GATE_OPENED") == 0) {
      digitalWrite(readyLED, LOW);
      startTimeMillis = millis();
      gateOpen = true;
      scoresReported = false;
      for (int i=0 ; i < LANES ; i++) {
        laneStatus[i] = RACING;
        digitalWrite(finishLineLED[i], LOW);
      }
      Serial.println("race START");
    } else if (strcmp(buffer, "START_GATE_CLOSED") == 0) {
      gateOpen = false;
      if (allFinished) {
        for (int i=0 ; i < LANES ; i++) {
          digitalWrite(finishLineLED[i], LOW);
          laneStatus[i] = AT_GATE;
        }
        digitalWrite(readyLED, HIGH);
      }
      if (allAtGate) {
        digitalWrite(readyLED, HIGH);
      }
    }
  }
};



// List of all the masters. It will be populated when a new master is registered
std::vector<ESP_NOW_Peer_Class> masters;

/* Callbacks */

// Callback called when an unknown peer sends a message
void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
    Serial.printf("Unknown peer " MACSTR " sent a broadcast message\n", MAC2STR(info->src_addr));
    Serial.println("Registering the peer as a master");
    commEstablished = true;
    ESP_NOW_Peer_Class new_master(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);

    masters.push_back(new_master);
    if (!masters.back().add_peer()) {
      Serial.println("Failed to register the new master");
      return;
    }
  } else {
    // The slave will only receive broadcast messages
    log_v("Received a unicast message from " MACSTR, MAC2STR(info->src_addr));
    log_v("Igorning the message");
  }
}

String getDefaultMacAddress() {

  String mac = "";

  unsigned char mac_base[6] = {0};

  if (esp_efuse_mac_get_default(mac_base) == ESP_OK) {
    char buffer[18];  // 6*2 characters for hex + 5 characters for colons + 1 character for null terminator
    sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X", mac_base[0], mac_base[1], mac_base[2], mac_base[3], mac_base[4], mac_base[5]);
    mac = buffer;
  }

  return mac;
}

/* Main */
 void setup() {
  
  Serial.begin(115200);
  Wire.begin(SDA0_Pin, SCL0_Pin);
  Serial.println(F("SSD1306 allocation OK"));
  while (!Serial) {
    delay(10);
  }

  // Initialize the Wi-Fi module
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }
  
  char buffer[256];
  String text = getDefaultMacAddress();
  text.toCharArray(buffer, text.length()+1);
  Serial.println(buffer);
  delay(2000);

  Serial.println("ESP-NOW Example - Broadcast Slave");
  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  // Initialize the ESP-NOW protocol
  if (!ESP_NOW.begin()) {
    Serial.println("Failed to initialize ESP-NOW");
    Serial.println("Reeboting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  // Register the new peer callback
  ESP_NOW.onNewPeer(register_new_master, NULL);

  Serial.println("Setup complete. Waiting for a master to broadcast a message...");

  for (int i=0 ; i < LANES ; i++) {
    laneStatus[i] = AT_GATE;
    pinMode(finishLineLED[i], OUTPUT);
    digitalWrite(finishLineLED[i], LOW);
  }
  pinMode(readyLED, OUTPUT);
  resetSwitch.setDebounceTime(50);
}

void loop() {
  // check the break beam pins for cars crossing the finish line
  for (int i=0 ; i < LANES ; i++) {
    if ((analogRead(breakBeamPin[i]) < 1000) && (laneStatus[i] == RACING)) {
      laneStatus[i] = FINISHED;
      // Serial.print("Lane "); Serial.print(i); Serial.println(" FINISHED");
      long now = millis();
      elapsedTime[i] = now - startTimeMillis;
      // Serial.print("Time: "); Serial.println(elapsedTime[i] / 1000.0);
      // Serial.println();
      digitalWrite(finishLineLED[i], HIGH);
    }
  }

  resetSwitch.loop();
  if ((resetSwitch.isPressed()) && (commEstablished)) {
    digitalWrite(readyLED, HIGH);
    for (int i=0 ; i < LANES ; i++) {
      digitalWrite(finishLineLED[i], LOW);
      laneStatus[i] = AT_GATE;
    }
    digitalWrite(readyLED, HIGH);
  }


  // check to see if all cars crossed the finish line
  allFinished = false;
  int finishedLanes = 0;
  for (int i=0 ; i < LANES ; i++) {
    if (laneStatus[i] == FINISHED) {
      finishedLanes++;
    }
  }
  if (finishedLanes == LANES) { // all lanes are in FINISHED state
    allFinished = true;
  }

  allAtGate = false;
  int atGateLanes = 0;
  for (int i=0 ; i < LANES ; i++) {
    if (laneStatus[i] == AT_GATE) {
      atGateLanes++;
    }
  }
  if (atGateLanes == LANES) { // all lanes are in FINISHED state
    allAtGate = true;
  }

  if ((allFinished) && (scoresReported == false)) { // report scores
    for (int i=0 ; i < LANES ; i++) {
      Serial.print("Lane: "); Serial.print(i+1); Serial.print(", Time: "); Serial.print(elapsedTime[i] / 1000.0); Serial.println("s");
    }
    scoresReported = true;
  }



}
