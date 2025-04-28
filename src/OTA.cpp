#include "OTA.h"

const char* ssid = "PosteOTA";
WebServer server(80);
TaskHandle_t ota_task_handler;


void ota_setup(void) {
  esp_partition_iterator_t partitionIterator = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
  // Loop through all partitions
  while (partitionIterator != NULL) {
    const esp_partition_t *partition = esp_partition_get(partitionIterator);
    Serial.print("Type: ");
    Serial.print(partition->type);
    Serial.print(" | Subtype: ");
    Serial.print(partition->subtype);
    Serial.print(" | Address: 0x");
    Serial.print(partition->address, HEX);
    Serial.print(" | Size: ");
    Serial.print(partition->size);
    Serial.print(" bytes | Label: ");
    Serial.println(partition->label);
    // Move to the next partition
    partitionIterator = esp_partition_next(partitionIterator);
  }

  // Free the iterator
  esp_partition_iterator_release(partitionIterator);

  WiFi.mode(WIFI_AP);  
  WiFi.softAP(ssid, NULL);
  vTaskDelay(pdMS_TO_TICKS(1000));
  IPAddress IP = IPAddress (10, 10, 10, 1);
  IPAddress NMask = IPAddress (255, 255, 255, 0);
  WiFi.softAPConfig(IP, IP, NMask);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  /* SETUP YOR WEB OWN ENTRY POINTS */
  server.on("/myurl", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "Hello there!");
  });

  /* INITIALIZE ESP2SOTA LIBRARY */
  ESP2SOTA.begin(&server);
  server.begin();
}

void ota_handler(void *args) {
  while(1){
    /* HANDLE UPDATE REQUESTS */
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}