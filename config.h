#ifndef CONFIG_H
#define CONFIG_H

// Define the modem type for SIM7600
#define TINY_GSM_MODEM_SIM7600

// Pin definitions for SIM7600 (adjust these according to your hardware)
#define PIN_DTR 25
#define PIN_RI 33
#define PIN_TX 26
#define PIN_RX 27
#define PWR_PIN 4

// APN for your mobile network (replace with your own)
const char apn[] = "YOUR_APN";

// MQTT Broker Configuration
const char* mqtt_server = "YOUR_MQTT_SERVER"; // e.g., "broker.example.com" or an IP address
const int mqtt_port = 1883;
// If using authentication, uncomment and update the lines below:
// const char* mqtt_user = "YOUR_MQTT_USER";
// const char* mqtt_pass = "YOUR_MQTT_PASS";

// MQTT Topic
const char* mqtt_topic = "device/data";

// BLE Device Addresses (replace with the addresses of your devices)
#include <BLEAddress.h>
BLEAddress deviceA_addr("YOUR_DEVICE_A_ADDRESS");
BLEAddress deviceB_addr("YOUR_DEVICE_B_ADDRESS");

#endif // CONFIG_H
