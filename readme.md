# ESP32 + SH1106 + BME680 Sketch

## Description
This is my first Arduino sketch to measure air quality using the following components:
* ESP32 (NodeMCU layout) - Purchased at [Jaycar](https://www.jaycar.com.au/esp32-main-board-with-wifi-and-bluetooth-communication/p/XC3800)
* BME680 Temperature, Humidity, Pressure, Gas Resistence Sensor - Purchased at [Core Electronics](https://core-electronics.com.au/bme680-environmental-sensor-module.html)
* SH1106 OLED Display (128x64) - Purchased at [Jaycar](https://www.jaycar.com.au/duinotech-1-3-inch-monochrome-oled-display/p/XC3728?pos=4&queryId=de47190af0f85b75a6f830d85c173531&sort=relevance)

## Configuration steps
1. Download this repository
2. Make a copy of `config.h.tpl` named `config.h`
3. Slot in the your WiFi and MQTT credentials

## Features
* Displays relevant BME680 sensor readings on the OLED display
* Sends relevant BME680 sensor readings via MQTT for consumption with something like Home Assistant