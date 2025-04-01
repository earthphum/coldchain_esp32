# ColdChain+ (ESP32 + LTE + BLE + GPS)
## Overview
This project is an IoT device that collects:
* GPS location (using internal GPS or SIM-based location fallback)
* Temperature & humidity from BLE sensors
* Motion detection via MPU6050
* Sends all data over LTE (SIM7600) to an MQTT broker
Designed for low-power, edge-based monitoring use cases:
* Smart delivery boxes
* Field sensor nodes
## Hardware
Module | Purpose
----- | -----
LILYGO® T-A7670G | ESP32 + SIM7600 LTE modem
MPU6050 | Motion detection (wake on movement)
Internal GPS | Primary GPS data
BLE Sensors | Temperature + humidity from BLE devices
LTE SIM card | Internet connection via cellular network
## Features
* ✅ LTE MQTT communication (SIM7600 over TinyGSM)

* ✅ Dual BLE client connection for sensors

* ✅ GPS + SIM-based fallback location

* ✅ Motion wake-up via MPU6050

* ✅ Deep Sleep for power saving

* ✅ Offline buffer for unsent data

* ✅ Configurable via config.h
## Installation
#### Required Libraries
Install these via PlatformIO or Arduino Library Manager:
* TinyGSM

* PubSubClient

* TinyGPS++

* Adafruit_MPU6050

* Adafruit_Sensor

* BLEDevice (ESP32 BLE Arduino)
Wiring Setup

Function | PIN
----- | -----
SIM TX  |  26
SIM RX	| 27
SIM PWR	| 4
GPS TX	| 21
GPS RX	| 22
MPU6050 SDA	| 4
MPU6050 SCL	| 15
MPU INT (wake)	| 2
## Configuration
 ``` h
const char apn[] = "your_apn";
const char* mqtt_server = "your.mqtt.broker.com";
const int mqtt_port = 1883;
BLEAddress deviceA_addr("XX:XX:XX:XX:XX:XX");
BLEAddress deviceB_addr("YY:YY:YY:YY:YY:YY");
 ```
## How It Works
1. On boot or motion interrupt:

    * Wake up and check GPS location.
  
    * Scan and connect to BLE devices.
  
    * Collect temp/humidity + battery status.
  
    * Connect to LTE and publish data via MQTT.
  
    * Enter deep sleep again.

2. If data can't be sent:

    * Store in buffer (up to 20 records).
  
    * Retry next cycle.
## Example MQTT Payload
``` json
{
  "device": "A",
  "temperature": 24.50,
  "humidity": 60,
  "latitude": 13.7563,
  "longitude": 100.5018,
  "wake_source": "interrupt",
  "box_status": "opened"
}
```
## 🛡 License

This project is licensed under the **Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)**.

© 2025 Warodom Phumprasert  
Project, King Mongkut's Institute of Technology Ladkrabang

📖 [View License](https://creativecommons.org/licenses/by-nc-sa/4.0/)
