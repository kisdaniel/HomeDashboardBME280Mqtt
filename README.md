# HomeDashboard Garage Door Opener

Home Dashboard BME280 Arduino project for NodeMCU (ESP8266)

This program connect to an MQTT server and receives commands from mqtt and publishes status of the garage door to mqtt.

# Configuration

## Wifi, mqtt

Wifi and mqtt server is configured in an embedded captive portal, find a HomeDashboardBME280AP access point and connect to the network and use "password" for password.

You can setup wifi and mqtt server host and port.

If you want to reconfigure your nodemcu you have to follow the following instcructions:
- press reset button (wifi reset button must not be pressed)
- press wifi reset button within 2 sec after reset
- find HomeDashboardBME280AP access point and connect to it, wifi password is "password" 
- type http://192.168.4.1 to your browser

## Default Pin layout

Pin layout definied in HomeDashboardBME280Mqtt.h 

* D0 - STATUS_LED 
* D1 - WIFI_RESET_BUTTON
* D3 - I2C_SDA
* D4 - I2C_SCL

https://www.homedashboard.hu/
