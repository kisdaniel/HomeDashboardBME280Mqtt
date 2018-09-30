/* GarageDoor-Opener.h */ 
#ifndef GARAGEDOOR_OPENER_H
#define GARAGEDOOR_OPENER_H

#define SERIAL_BAUD 115200



// Pin layout

#define I2C_SDA             D3  // I2C data
#define I2C_SCL             D4  // I2C clock

#define BME280_DEVICE_ID    0x76  // BME280 device id

#define STATUS_LED          D0

#define WIFI_RESET_BUTTON   D1

// D3 is reserved during programming
// D4 unused


#define PRESSED             LOW     // LOW if you use built in pull up resistor
#define UNPRESSED           HIGH    // HIGH if you use built in pull up resistor
#define INPUT_PINMODE       INPUT_PULLUP    // built in pull up resistor


#define NETWORK_STATUS_NOT_CONNECTED        0
#define NETWORK_STATUS_ONLY_WIFI            256
#define NETWORK_STATUS_CONNECTING_TO_MQTT   512
#define NETWORK_STATUS_CONNECTED            1023

#define MQTT_TOPIC_REGISTRATION    "/homedashboard/device/registration"

#include <Arduino.h>

typedef enum {
    GD_OPEN,
    GD_CLOSED,
    GD_OPENING,
    GD_CLOSING,
    GD_PARTIALLY_OPENED,
    GD_UNKNOWN
} GarageDoorState;

#endif
