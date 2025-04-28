#pragma once
#include <Arduino.h>

#define NUM_LEDS 36

const uint8_t rgb_led_pin = 12;
const uint8_t dimmer_lights_pwm_pin = 13;
const uint8_t on_off_dimmer_lights_relay_pin = 14;
const uint8_t button_pin = 18;
const uint8_t button_led_pin = 19;

// Update the command mapping structure to include the new SET_NET command.
struct CommandMap {
  const char *command;
  void (*function)(String parameters);
};

typedef struct {
  String command;  // "STATIC", "OFF", "EFFECT1"
  uint8_t r;
  uint8_t g;
  uint8_t b;
} LedCommand;

typedef struct{
  int temperature;
  int humidity;
  int pressure;
  int soil_moisture;
  int wind_speed;
  String wind_direction;
}ESTACION_CLIMATOLOGICA;

typedef enum{
  MASTER,
  SLAVE
}BEHAVIOR;
  

