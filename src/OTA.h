#ifndef OTA_H
#define OTA_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WebServer.h>
/* INCLUDE ESP2SOTA LIBRARY */
#include <ESP2SOTA.h>

extern TaskHandle_t ota_task_handler;

void ota_setup(void);
void ota_handler(void *args);

#endif