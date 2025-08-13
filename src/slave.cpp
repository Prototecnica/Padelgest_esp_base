//Firmware for new version of slave devices
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <esp_wifi.h>
#include <bsp/throwball.h>

#define ESTACION_SERVICE_UUID "00000000-0000-0000-0000-00000000AA00"
#define NOMBRE_PISTA_UUID "00000000-0000-0000-0000-00000000AA12"

#define MISCELANEO_SERVICE_UUID "00000000-0000-0000-0000-00000000CC00"
#define NOMBRE_PULSERA_UUID "00000000-0000-0000-0000-00000000CC06"
#define DIRECCION_MAC_PROPIA_UUID "00000000-0000-0000-0000-00000000CC07"
#define BAND_FIRMWARE_VERSION "00000000-0000-0000-0000-00000000CC12"

#define PERIPHERAL_NAME "Pista_esclavo"
#define WEATHER_STATION_PACKET_LENGTH 36

const uint8_t pinServo = 41;
const uint8_t pinMotor = 40;
Throwball throwball(pinServo, pinMotor);

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

BLECharacteristic *nombre_pulsera;
BLECharacteristic *direccion_mac_propia;
BLECharacteristic *nombre_pista;
BLECharacteristic *firmware_version;

void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

void handleThrowball(String parameters){
    Serial.println("Throwball requested!");
}

class ServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    Serial.println("BLE Client Connected");
  }
  void onDisconnect(BLEServer *pServer)
  {
    BLEDevice::startAdvertising();
    Serial.println("BLE Client Disconnected");
  }
};


class Nombre_Pulsera_Callbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override;
};

void Nombre_Pulsera_Callbacks::onWrite(BLECharacteristic *pCharacteristic) {
  std::string rxValue = pCharacteristic->getValue();
  if (!rxValue.empty()) {
    Serial.print("Received nombre esclavo: ");
    // Assuming rxValue is a UTF-8 encoded string
    Serial.println(rxValue.c_str()); // Print the UTF-8 string
    std::string rxValue = "_" + pCharacteristic->getValue();
    Preferences preferences;
    preferences.begin("storage", false);
    preferences.putString("name", rxValue.c_str());
    preferences.end();
    Serial1.println((String) PERIPHERAL_NAME + rxValue.c_str());    

    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP.restart();
  }
}


//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.print("Bytes received: ");
  
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  Preferences preferences;
  preferences.begin("storage", true); // The second parameter is readonly
  String nvsString = preferences.getString("name", "");
  preferences.end();
  std::string peripheralNameWithNVS = std::string(PERIPHERAL_NAME) + nvsString.c_str();
  BLEDevice::init(peripheralNameWithNVS);
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService_estacion = pServer->createService(ESTACION_SERVICE_UUID);

  nombre_pista = pService_estacion->createCharacteristic(
    BLEUUID(NOMBRE_PISTA_UUID),
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY 
  );
  nombre_pista->addDescriptor(new BLE2902);
  
  //RELATIVO A MISCELANEO
  BLEService *pService_miscelaneo = pServer->createService(BLEUUID(MISCELANEO_SERVICE_UUID), 20, 0);

  nombre_pulsera = pService_miscelaneo->createCharacteristic(
    BLEUUID(NOMBRE_PULSERA_UUID),
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  nombre_pulsera->setCallbacks(new Nombre_Pulsera_Callbacks()); 

  direccion_mac_propia = pService_miscelaneo->createCharacteristic(
    BLEUUID(DIRECCION_MAC_PROPIA_UUID),
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  direccion_mac_propia->addDescriptor(new BLE2902);

  firmware_version = pService_miscelaneo->createCharacteristic(
    BLEUUID(BAND_FIRMWARE_VERSION),
    BLECharacteristic::PROPERTY_NOTIFY | 
    BLECharacteristic::PROPERTY_READ
  );
  firmware_version->addDescriptor(new BLE2902);


  pServer->setCallbacks(new ServerCallbacks());
  pService_estacion->start();
  pService_miscelaneo->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(MISCELANEO_SERVICE_UUID);
  pAdvertising->addServiceUUID(ESTACION_SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06); // Minimum connection interval: 7.5ms (0x06 * 1.25ms)
  pAdvertising->setMaxPreferred(0x12); // Maximum connection interval: 22.5ms (0x12 * 1.25ms)
  BLEDevice::setMTU(512);
  BLEDevice::startAdvertising();
  pServer->getAdvertising()->start();
  uint8_t mac[6];
  WiFi.macAddress(mac); // Gets Wi-Fi Station MAC
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", 
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  // Set the MAC address as the value of the characteristic
  direccion_mac_propia->setValue(macStr);
  direccion_mac_propia->notify();
  Serial.println("Mac address notified!");
  std::string firmwareVersionString = FIRMWARE_VERSION;
  firmware_version->setValue(firmwareVersionString);
  firmware_version->notify();

  Serial.println("BLE initialized");

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  Serial.print("[DEFAULT] This ESP32's Board MAC Address: ");
  readMacAddress();

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  nombre_pista->setValue(peripheralNameWithNVS);
  nombre_pista->notify();

  #ifdef FIRMWARE_VERSION
  Serial.printf("FIRMWARE VERSION %s\n", FIRMWARE_VERSION);
  #endif
}

// Update the command mapping structure to include the new SET_NET command.
struct CommandMap {
  const char *command;
  void (*function)(String parameters);
};

CommandMap commandTable[] = {
  {"THROW_BALL", handleThrowball}
};

void messageParser(String uart_frame) {
  Serial.printf("This is UART FRAME: %s \n", uart_frame.c_str());
  for (int i = 0; i < sizeof(commandTable) / sizeof(CommandMap); i++) {
    if (uart_frame.startsWith(commandTable[i].command)) {
      // Assumes an underscore after the command (e.g., "SET_LIGHT_0,1")
      String parameters = uart_frame.substring(strlen(commandTable[i].command) + 1);
      commandTable[i].function(parameters);
      return;
    }
  }
  Serial.println("Unknown command");
}

void loop(){
  if (Serial.available())
  {
    String receivedData = Serial1.readStringUntil('\n'); // Read until a newline character
    Serial.print("Received from PC: ");
    Serial.println(receivedData);
    messageParser(receivedData);
  }
  vTaskDelay(10 / portTICK_PERIOD_MS);
}