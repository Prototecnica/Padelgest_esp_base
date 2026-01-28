#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <OTA.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <esp_wifi.h>
#include <bsp/led_strip.h>
#include <globals.h>

#define ESTACION_SERVICE_UUID "00000000-0000-0000-0000-00000000AA00"
#define INFO_PANTALLA_UUID "00000000-0000-0000-0000-00000000AA13"
#define NOMBRE_PISTA_UUID "00000000-0000-0000-0000-00000000AA12"
#define VARIABLES_METEREOLOGICAS_UUID "00000000-0000-0000-0000-00000000AA11"

#define MISCELANEO_SERVICE_UUID "00000000-0000-0000-0000-00000000CC00"
#define NOMBRE_PULSERA_UUID "00000000-0000-0000-0000-00000000CC06"
#define DIRECCION_MAC_ESCLAVO_UUID "00000000-0000-0000-0000-00000000CC09"
#define DIRECCION_MAC_PROPIA_UUID "00000000-0000-0000-0000-00000000CC07"
#define ESTATUS_NET_UUID "00000000-0000-0000-0000-00000000CC10"
#define VOLUMEN_STATUS_UUID "00000000-0000-0000-0000-00000000CC11"
#define BAND_FIRMWARE_VERSION "00000000-0000-0000-0000-00000000CC12"

#define PERIPHERAL_NAME "Pista"
#define WEATHER_STATION_PACKET_LENGTH 36

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

BLECharacteristic *BLE_to_UART_data;
BLECharacteristic *station_data;
BLECharacteristic *nombre_pulsera;
BLECharacteristic *direccion_mac_esclavo;
BLECharacteristic *direccion_mac_propia;
BLECharacteristic *nombre_pista;
BLECharacteristic *estatus_net;
BLECharacteristic *estatus_volumen;
BLECharacteristic *firmware_version;

uint8_t* slaveMacAddress;

LedStrip led_strip(rgb_led_pin);

//pwm
const int pwmPin = dimmer_lights_pwm_pin;     // GPIO6
const int pwmFreq = 1000; // 1 kHz
const int pwmChannel = 0;
const int pwmResolution = 8; 

BEHAVIOR behavior = SLAVE;

QueueHandle_t ledCommandQueue;
TaskHandle_t power_off_task_handle;

ESTACION_CLIMATOLOGICA estacion_climatologica;

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

// Function to serialize the ESTACION_CLIMATOLOGICA struct
String serializeEstacionClimatologica(const ESTACION_CLIMATOLOGICA &estacion) {
  String serializedData = "";
  serializedData += String(estacion.temperature) + "/";
  serializedData += String(estacion.humidity) + "/";
  serializedData += String(estacion.pressure) + "/";
  serializedData += String(estacion.soil_moisture) + "/";
  serializedData += String(estacion.wind_speed) + "/";
  serializedData += estacion.wind_direction + "/";
  return serializedData;
}

class ServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    Serial.println("BLE Client Connected");
    Serial1.println("BLE Client Connected");
  }
  void onDisconnect(BLEServer *pServer)
  {
    BLEDevice::startAdvertising();
    Serial.println("BLE Client Disconnected");
    Serial1.println("BLE Client Disconnected");
  }
};

class BLE_UART_Callbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override;
};
class Nombre_Pulsera_Callbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override;
};
class Mac_Esclavo_Callbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override;
};
class Mac_Propia_Callbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override;
};
class Estatus_Volumen_Callbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override;
};


void BLE_UART_Callbacks::onWrite(BLECharacteristic *pCharacteristic) {
  std::string rxValue = pCharacteristic->getValue();
  if (!rxValue.empty()) {
    Serial.println("sending UART data: ");
    Serial.println(rxValue.c_str());  // Print the string to Serial

    // Find the position of the last '/' in the string
    size_t pos = rxValue.find_last_of('/');
    if (pos != std::string::npos && pos < rxValue.length() - 1) {
      char num = rxValue[pos + 1];  // Get the character after the last '/'
      // Check if the character is '0', '1', or '2'
      if (num == '0' || num == '1' || num == '2') {
        std::string serialValue;
        std::string espNowValue;
        if (num == '0') {
          // Send the received string as it is
          serialValue = rxValue;
          espNowValue = rxValue;
        } else {
          // Remove the last byte to get ".../"
          std::string baseValue = rxValue.substr(0, pos + 1);
          Serial.printf("Base value %s\n", baseValue.c_str());
          if (num == '1') {
            serialValue = baseValue + "1";
            espNowValue = baseValue + "0";
          } else if (num == '2') {
            serialValue = baseValue + "0";
            espNowValue = baseValue + "1";
          }
        }
        //Log the values before sending
        Serial.printf("ESP-NOW: %s | SERIAL: %s\n", espNowValue.c_str(), serialValue.c_str());
        //Send the modified string via Serial1
        Serial1.println(serialValue.c_str());
        Serial.println("Serial1 Send success");
        //Send the modified string via ESP-NOW
        esp_err_t result = esp_now_send(slaveMacAddress, (uint8_t *)espNowValue.c_str(), espNowValue.length());
        if (result == ESP_OK) {
          Serial.println("ESP-NOW Send success");
        } else {
          Serial.println("ESP-NOW Send fail");
        }
      } else {
        Serial.println("Invalid number format received");
      }
    } else {
      Serial.println("Invalid string format received");
    }
  }
}


void Nombre_Pulsera_Callbacks::onWrite(BLECharacteristic *pCharacteristic) {
  std::string rxValue = pCharacteristic->getValue();
  if (!rxValue.empty()) {
    Serial.print("Received nombre pulsera: ");
    // Assuming rxValue is a UTF-8 encoded string
    Serial.println(rxValue.c_str()); // Print the UTF-8 string
    std::string rxValue = "_" + pCharacteristic->getValue();
    Preferences preferences;
    preferences.begin("storage", false);
    preferences.putString("name", rxValue.c_str());
    preferences.end();
    Serial1.println((String) PERIPHERAL_NAME + rxValue.c_str());    

    vTaskDelay(pdMS_TO_TICKS(5000));
    //BLEDevice::deinit(false);
    ESP.restart();
  }
}

void Mac_Esclavo_Callbacks::onWrite(BLECharacteristic *pCharacteristic) {
  std::string rxValue = pCharacteristic->getValue();
  if (!rxValue.empty()) {
    Serial.print("Received mac esclavo: ");
    // Assuming rxValue is a UTF-8 encoded string
    Serial.println(rxValue.c_str()); // Print the UTF-8 string
    Preferences preferences;
    preferences.begin("storage", false);
    preferences.putString("receiver_mac", rxValue.c_str());
    preferences.end();
    //BLEDevice::deinit(false);
  }
}

void Mac_Propia_Callbacks::onWrite(BLECharacteristic *pCharacteristic) {
  std::string rxValue = pCharacteristic->getValue();
  if (!rxValue.empty()) {
    Serial.print("Received mac esclavo: ");
    // Assuming rxValue is a UTF-8 encoded string
    Serial.println(rxValue.c_str()); // Print the UTF-8 string
    Preferences preferences;
    preferences.begin("storage", false);
    preferences.putString("receiver_mac", rxValue.c_str());
    preferences.end();
    //BLEDevice::deinit(false);
  }
}

void Estatus_Volumen_Callbacks::onWrite(BLECharacteristic *pCharacteristic) {
  std::string rxValue = pCharacteristic->getValue();
  if (!rxValue.empty()) {
    // Convert std::string to Arduino String
    String rxString = String(rxValue.c_str());
    String volCommand = rxString;
    Serial1.println(volCommand);
  }
}

void send_estacion_climatologica(){
  String serializedStruct = serializeEstacionClimatologica(estacion_climatologica);
  station_data->setValue(serializedStruct.c_str());
  station_data->notify();
  Serial1.println(serializedStruct);
}

//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.print("Bytes received: ");
  Serial.println(len);
  if(len == WEATHER_STATION_PACKET_LENGTH){
    Serial.println("Paquete de estación metereologica");
    memcpy(&estacion_climatologica, incomingData, sizeof(estacion_climatologica));
    Serial.printf("Temperatura: %d\n", estacion_climatologica.temperature);
    Serial.printf("Humedad: %d\n", estacion_climatologica.humidity);
    Serial.printf("Presion: %d\n", estacion_climatologica.pressure);
    Serial.printf("Humedad del suelo: %d\n", estacion_climatologica.soil_moisture);
    Serial.printf("Velocidad del viento: %d\n", estacion_climatologica.wind_speed);
    Serial.printf("Direccion del viento: %s\n", estacion_climatologica.wind_direction.c_str());
    
    send_estacion_climatologica();
  }
  else{
    Serial.println("Paquete para RPi");
      // Print incoming data as a string
    Serial.print("Incoming data as string: ");
    Serial.write(incomingData, len);  // This handles any type of data including non-null terminated
    Serial.println();  // Ensure to move to a new line after printing the data
    Serial1.write(incomingData, len);  // This handles any type of data including non-null terminated
    Serial1.println();  // Ensure to move to a new line after printing the data
  }
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Function to parse MAC address string and return a byte array
uint8_t* parseSlaveMacAddressString(String receiver_address) {
    static uint8_t mac[6];  // Static so it stays in memory after the function returns
    int values[6];  // Temporary array to hold the parsed hex values
    // Scan the string for hexadecimal values
    if (sscanf(receiver_address.c_str(), "%x:%x:%x:%x:%x:%x%*c", 
      &values[0], &values[1], &values[2], &values[3], &values[4], &values[5]) == 6) {
      // Convert integers to byte array
      for (int i = 0; i < 6; i++) {
          mac[i] = (uint8_t)values[i];
      }
    } else {
      Serial.println("Failed to parse MAC address");
      return nullptr;  // Return nullptr if parsing fails
    }
    return mac;
}

void handleSetLight(String parameters) {
  // Expecting format: "X,Y"
  int commaIndex = parameters.indexOf(',');
  if (commaIndex == -1) {
    Serial.println("Error: Missing comma in SET_LIGHT parameters");
    return;
  }
  String xStr = parameters.substring(0, commaIndex);
  String yStr = parameters.substring(commaIndex + 1);
  int x = xStr.toInt();
  int y = yStr.toInt();

  // Validate: X must be 0 or 1; Y must be between 0 and 65
  if ((x != 0 && x != 1) || (y < 0 || y > 100)) {
    Serial.println("Error: SET_LIGHT parameters out of range");
    return;
  }

  Serial.printf("SET_LIGHT: x=%d, y=%d\n", x, y);

  if(x == 0){
    pinMode(pwmPin, OUTPUT);
    digitalWrite(pwmPin, HIGH);
  }
  else if(x == 1){
    pinMode(pwmPin, OUTPUT);
    ledcSetup(pwmChannel, pwmFreq, pwmResolution);
    ledcAttachPin(pwmPin, pwmChannel);

    // Map y from [0, 65] to [190, 255]
    int duty = map(y, 100, 0, 150, 255);
    Serial.printf("Duty cycle: %d\n", duty);
    ledcWrite(pwmChannel, duty);
  }
}

void handleSetLedEffect(String parameters) {
  // Expecting format: "R,G,B,X,Y"
  int values[5];
  int startIndex = 0;
  int currentIndex = 0;
  // Loop to extract first 4 values (R, G, B, X)
  for (int i = 0; i < 4; i++) {
    currentIndex = parameters.indexOf(',', startIndex);
    if (currentIndex == -1) {
      Serial.println("Error: Missing parameter in SET_LED_EFFECT");
      return;
    }
    String token = parameters.substring(startIndex, currentIndex);
    values[i] = token.toInt();
    startIndex = currentIndex + 1;
  }
  // Last value (Y)
  values[4] = parameters.substring(startIndex).toInt();

  int R = values[0], G = values[1], B = values[2], X = values[3], Y = values[4];
  
  Serial.printf("SET_LED_EFFECT: R=%d, G=%d, B=%d, X=%d, Y=%d\n", R, G, B, X, Y);
  // TODO: Add code to perform the light effect based on the parameters

  if(X == 1){
    Serial.println("Enciende tira");
    if(Y == 0){
      Serial.println("Efecto estatico");
      LedCommand cmd = {"STATIC", R, G, B};
      if (xQueueSend(ledCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
        Serial.println("Error: Failed to send command to LED task.");
      }
    } else if (Y == 1){
      Serial.println("Efecto 1 serpiente 3 leds");
      LedCommand cmd = {"EFFECT1", R, G, B};
      if (xQueueSend(ledCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
        Serial.println("Error: Failed to send command to LED task.");
      }
    } else if (Y == 2){
      Serial.println("Efecto 1 serpiente 6 leds");
      LedCommand cmd = {"EFFECT2", R, G, B};
      if (xQueueSend(ledCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
        Serial.println("Error: Failed to send command to LED task.");
      }
    } else if (Y == 3){
      Serial.println("Efecto 1 serpiente 6 leds");
      LedCommand cmd = {"EFFECT3", R, G, B};
      if (xQueueSend(ledCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
        Serial.println("Error: Failed to send command to LED task.");
      }
    }
  }
  else if(X == 0){
    Serial.println("Apaga tira");
    LedCommand cmd = {"OFF", 0, 0, 0};
    if (xQueueSend(ledCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
      Serial.println("Error: Failed to send command to LED task.");
    }
  } 
}

void handleSetNet(String parameters) {
  // Expecting format: "X" (a single number where X is "0" or "1")
  parameters.trim();  // Remove any extra whitespace

  // Validate: parameters must be "0" or "1"
  if (parameters != "0" && parameters != "1") {
    Serial.println("Error: SET_NET parameter out of range");
    return;
  }
  
  Serial.printf("SET_NET: %s\n", parameters.c_str());
  // Send the net status via BLE as a string
  estatus_net->setValue(parameters.c_str());
}

void handleVolStatus(String parameters){
    // Expecting format: "X,Y"
    int commaIndex = parameters.indexOf(',');
    if (commaIndex == -1) {
      Serial.println("Error: Missing comma in VOL parameters");
      return;
    }
    String xStr = parameters.substring(0, commaIndex);
    String yStr = parameters.substring(commaIndex + 1);
    int x = xStr.toInt();
    int y = yStr.toInt();
  
    // Validate: X must be 0 or 1; Y must be 1, 2, or 3
    if ((x != 0 && x != 1) || (y < 0 || y > 100)) {
      Serial.println("Error: VOL parameters out of range");
      return;
    }
    
    Serial.printf("VOL: x=%d, y=%d\n", x, y);
    
    // Properly construct the command string
    String volCommand = "VOL_" + parameters;
    estatus_volumen->setValue(volCommand.c_str());
}


void handleGetMac(String parameters){
  uint8_t mac[6];
  WiFi.macAddress(mac); // Gets Wi-Fi Station MAC
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", 
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  String message = String("MAC:") + String(macStr);
  Serial.println(macStr);
  Serial1.println(macStr);
}

CommandMap commandTable[] = {
  {"SET_LED_EFFECT", handleSetLedEffect},
  {"SET_LIGHT", handleSetLight},
  {"SET_NET", handleSetNet},
  {"VOL", handleVolStatus},
  {"GET_MAC", handleGetMac}
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


void ledStripTask(void *param) {
  LedStrip led(19);
  led.init();
  LedCommand currentCommand = {"OFF", 0, 0, 0};
  int snakeStep = 0;
  while (true) {
      LedCommand newCommand;
      if (xQueueReceive(ledCommandQueue, &newCommand, 0) == pdPASS) {
        currentCommand = newCommand;
        snakeStep = 0;  // reset animation
        led.resetFillOneByOne();
      }
      if (currentCommand.command == "OFF") {
        led.turnOff();
        vTaskDelay(pdMS_TO_TICKS(100));
      } else if (currentCommand.command == "STATIC") {
        led.effectStatic(currentCommand.r, currentCommand.g, currentCommand.b);
        vTaskDelay(pdMS_TO_TICKS(100));
      } else if (currentCommand.command == "EFFECT1") {
        led.effectSnake(currentCommand.r, currentCommand.g, currentCommand.b, snakeStep, 3);
        snakeStep = (snakeStep + 1) % NUM_LEDS;
        vTaskDelay(pdMS_TO_TICKS(60));
      } else if (currentCommand.command == "EFFECT2") {
        led.effectSnake(currentCommand.r, currentCommand.g, currentCommand.b, snakeStep, 6);
        snakeStep = (snakeStep + 1) % NUM_LEDS;
        vTaskDelay(pdMS_TO_TICKS(60));
      } else if (currentCommand.command == "EFFECT3") {
        led.fillOneByOne(currentCommand.r, currentCommand.g, currentCommand.b);
        vTaskDelay(pdMS_TO_TICKS(60));  // per frame step
      }else {
        Serial.println("Unknown LED command");
        vTaskDelay(pdMS_TO_TICKS(100));
      }
  }
}

TaskHandle_t powerButtonTaskHandle = NULL;

void IRAM_ATTR POWER_OFF_ISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(powerButtonTaskHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void powerButton_task(void *args) {
  int time_pressed;
  unsigned long buttonStartTime = 0;

  for (;;) {
    // Wait for ISR signal
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Serial.printf("POWER BUTTON TASK: %d \n", uxTaskGetStackHighWaterMark(NULL));
    Serial.println("Button PRESSED");

    buttonStartTime = millis();
    while (digitalRead(button_pin) == LOW) {
      time_pressed = millis();
      Serial.printf("%d \n", time_pressed - buttonStartTime);
      vTaskDelay(pdMS_TO_TICKS(100));  // avoid hogging CPU
    }

    int duration = time_pressed - buttonStartTime;

    if (duration >= 3000 && duration < 10000) {
      Serial.println("Reiniciando ESP32");
      for(uint8_t i = 0; i < 3; i++){
        digitalWrite(button_led_pin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(1000));
        digitalWrite(button_led_pin, LOW);
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      ESP.restart();
    } else if (duration >= 10000) {
      Serial.println("OTA requested");
      for(uint8_t i = 0; i < 30; i++){
        digitalWrite(button_led_pin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(button_led_pin, LOW);
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      BLEDevice::deinit(false);
      ota_setup();
      vTaskDelay(pdMS_TO_TICKS(2000));
      vTaskResume(ota_task_handler);
    }

    // Loop back and wait for next trigger
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  Preferences preferences;
  preferences.begin("storage", true); // The second parameter is readonly
  String nvsString = preferences.getString("name", "");
  String receiver_address = preferences.getString("receiver_mac", "X");
  preferences.end();
  std::string peripheralNameWithNVS = std::string(PERIPHERAL_NAME) + nvsString.c_str();
  BLEDevice::init(peripheralNameWithNVS);
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService_estacion = pServer->createService(ESTACION_SERVICE_UUID);
  BLE_to_UART_data = pService_estacion->createCharacteristic(
    BLEUUID(INFO_PANTALLA_UUID),
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  BLE_to_UART_data->setCallbacks(new BLE_UART_Callbacks());

  station_data = pService_estacion->createCharacteristic(
    BLEUUID(VARIABLES_METEREOLOGICAS_UUID),
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY 
  );
  station_data->addDescriptor(new BLE2902);

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

  direccion_mac_esclavo = pService_miscelaneo->createCharacteristic(
    BLEUUID(DIRECCION_MAC_ESCLAVO_UUID),
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  direccion_mac_esclavo->setCallbacks(new Mac_Esclavo_Callbacks());  

  direccion_mac_propia = pService_miscelaneo->createCharacteristic(
    BLEUUID(DIRECCION_MAC_PROPIA_UUID),
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  direccion_mac_propia->addDescriptor(new BLE2902);

  estatus_net = pService_miscelaneo->createCharacteristic(
    BLEUUID(ESTATUS_NET_UUID),
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  estatus_net->addDescriptor(new BLE2902);

  estatus_volumen = pService_miscelaneo->createCharacteristic(
    BLEUUID(VOLUMEN_STATUS_UUID),
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  estatus_volumen->addDescriptor(new BLE2902);
  estatus_volumen->setCallbacks(new Estatus_Volumen_Callbacks()); 

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
  // Start Serial1 at 9600 baud rate
  // GPIO17 (TX), GPIO16 (RX)
  Serial1.begin(115200, SERIAL_8N1, 16, 17);
  Serial.println("Serial1 initialized for TX on GPIO17 and RX on GPIO16");
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
  

  if(strcmp(receiver_address.c_str(), "X")==0){
    Serial.println("MODO ESCLAVO");
    behavior = SLAVE;
    slaveMacAddress = parseSlaveMacAddressString(receiver_address);
  }
  else{
    Serial.println("MODO MAESTRO");
    behavior = MASTER;
    //esp_now_register_send_cb(OnDataSent);
    // Set up peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    slaveMacAddress = parseSlaveMacAddressString(receiver_address);
    if (slaveMacAddress == nullptr) {
        Serial.println("Invalid MAC address");
        memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    }else{
      memcpy(peerInfo.peer_addr, slaveMacAddress, 6);
    }    
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    } else{
      Serial.printf("Maestro envia a dispositivo %s \n", receiver_address.c_str());
    }
  }

  led_strip.init();

  ledCommandQueue = xQueueCreate(5, sizeof(LedCommand));

  pinMode(button_pin, INPUT_PULLUP);
  pinMode(button_led_pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(button_pin), POWER_OFF_ISR, FALLING);

  xTaskCreate(powerButton_task, "PowerButtonTask", 4096, NULL, 5, &powerButtonTaskHandle);
  xTaskCreatePinnedToCore(ledStripTask, "LED Strip", 4096*3, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(ota_handler, "OTA_task", 1024 * 8, NULL, configMAX_PRIORITIES, &ota_task_handler, 1);
  vTaskSuspend(ota_task_handler);

  #ifdef FIRMWARE_VERSION
  Serial.printf("FIRMWARE VERSION %s\n", FIRMWARE_VERSION);
  #endif
}

void loop(){
  if (Serial1.available())
  {
    String receivedData = Serial1.readStringUntil('\n'); // Read until a newline character
    Serial.print("Received from RPi: ");
    Serial.println(receivedData);
    messageParser(receivedData);
  }
  vTaskDelay(10 / portTICK_PERIOD_MS);
}