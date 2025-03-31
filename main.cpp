#include "config.h"  // Include the private configuration file

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <TinyGPS++.h>
#include <esp_sleep.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <string.h>
#include <esp_task_wdt.h>  // For watchdog timer

// ----------------------- Debug Logging -----------------------
// Define DEBUG to enable detailed logs.
// In production, comment out the following line.
#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// ----------------------- SIM7600 and MQTT Configuration -----------------------
#define GSM_BAUD 115200   // Baud rate for SIM7600 communication

HardwareSerial SerialAT(1);  // Using UART1 on ESP32 for SIM7600
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
PubSubClient mqttClient(gsmClient);

// ----------------------- GPS Configuration -----------------------
#define RXPin_GPS 22
#define TXPin_GPS 21
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial ssGPS(2);

// ----------------------- BLE Configuration -----------------------
// UUIDs for BLE services and characteristics
BLEUUID serviceUUID("ebe0ccb0-7a0a-4b0c-8a1a-6ff2997da3a6");
BLEUUID charUUID("ebe0ccc1-7a0a-4b0c-8a1a-6ff2997da3a6");
BLEUUID batteryServiceUUID("0000180F-0000-1000-8000-00805F9B34FB");
BLEUUID batteryCharUUID("00002A19-0000-1000-8000-00805F9B34FB");

// BLE Clients
BLEClient* pClientA;
BLEClient* pClientB;

// ----------------------- Sensor Data Variables -----------------------
float tempA = 0, humiA = 0, tempB = 0, humiB = 0;
int batteryA = 0, batteryB = 0;
float currentLatitude = 0.0;
float currentLongitude = 0.0;

// ----------------------- Data Point Structure -----------------------
struct DataPoint {
    String device;
    float temperature;
    float humidity;
    float latitude;
    float longitude;
};

// ----------------------- Unsent Data Buffer Configuration -----------------------
#define UNSENT_DATA_POINTS 20
DataPoint unsentDataBuffer[UNSENT_DATA_POINTS];
int unsentDataIndex = 0;

// ----------------------- Timing Configuration -----------------------
#define uS_TO_S_FACTOR 1000000ULL  
#define TIME_TO_SLEEP  180  // Deep sleep time in seconds

// ----------------------- Function Prototypes -----------------------
String sendAT(String command, int timeout, bool debug);
void printNetworkInfo();
void reconnectMQTT();
bool publishWithRetry(const char* topic, const char* payload);
bool publishDataPoint(const DataPoint& dp);
bool addToUnsentBuffer(const DataPoint& dp);
void getSIMLocation(float& latitude, float& longitude);
void getGPSLocation(float& latitude, float& longitude);
void connectAndReadBLE(BLEClient* pClient, BLEAddress address, const char* deviceID, float& temp, float& humi, int& battery);
void modemPowerOn();
void modemPowerOff();
void readGPS();
DataPoint collectData(String deviceID, float temp, float humi);
void enterDeepSleep();

// ----------------------- MPU6050 Configuration -----------------------
#define SDA_MPU 4
#define SCL_MPU 15
#define MPU_INT_PIN 2  // GPIO2 for ext0 wake-up

Adafruit_MPU6050 mpu;
String wakeSource = "";

// MPU6050 function prototypes
void initMPU6050();
void setupMotionInterrupt();
void writeMPURegister(uint8_t reg, uint8_t val);
uint8_t readMPURegister(uint8_t reg);

// ----------------------- Function Implementations -----------------------

String sendAT(String command, int timeout, bool debug) {
    String response = "";
    SerialAT.println(command);
    long startTime = millis();
    while ((millis() - startTime) < timeout) {
        while (SerialAT.available() > 0) {
            response += (char)SerialAT.read();
        }
        esp_task_wdt_feed(); // Feed watchdog during long loops
    }
    SerialAT.flush();
    if (debug) {
        DEBUG_PRINT("Command: ");
        DEBUG_PRINTLN(command);
        DEBUG_PRINT("Response: ");
        DEBUG_PRINTLN(response);
    }
    return response;
}

void printNetworkInfo() {
    DEBUG_PRINTLN("\n------- Network Information --------");
    DEBUG_PRINT("Signal Quality: ");
    DEBUG_PRINTLN(modem.getSignalQuality());
    DEBUG_PRINT("Operator: ");
    DEBUG_PRINTLN(modem.getOperator());
    DEBUG_PRINT("IMEI: ");
    DEBUG_PRINTLN(modem.getIMEI());
    DEBUG_PRINT("IMSI: ");
    DEBUG_PRINTLN(modem.getIMSI());
    DEBUG_PRINT("CCID: ");
    DEBUG_PRINTLN(modem.getSimCCID());
    DEBUG_PRINTLN("------------------------------------\n");
}

void reconnectMQTT() {
    int attempt = 1;
    while (!mqttClient.connected()) {
        DEBUG_PRINT("Attempting MQTT connection... Attempt ");
        DEBUG_PRINTLN(attempt);
        if (mqttClient.connect("ESP32Client")) {
            DEBUG_PRINTLN("MQTT connected");
            break;
        } else {
            DEBUG_PRINT("Failed, rc=");
            DEBUG_PRINTLN(mqttClient.state());
            delay(1000 * attempt);  // Exponential backoff
            attempt++;
            esp_task_wdt_feed();
        }
    }
}

bool publishWithRetry(const char* topic, const char* payload) {
    if (!mqttClient.connected()) {
        reconnectMQTT();
    }
    DEBUG_PRINT("Payload: ");
    DEBUG_PRINTLN(payload);

    if (mqttClient.publish(topic, payload)) {
        DEBUG_PRINTLN("Published to MQTT");
        return true;
    } else {
        DEBUG_PRINTLN("Failed to publish data to MQTT. Attempt 1 of 2");
        DEBUG_PRINT("MQTT Client State: ");
        DEBUG_PRINTLN(mqttClient.state());
        delay(1000);
        if (!mqttClient.connected()) {
            reconnectMQTT();
        }
        if (mqttClient.publish(topic, payload)) {
            DEBUG_PRINTLN("Data published to MQTT on retry");
            return true;
        } else {
            DEBUG_PRINTLN("Failed to publish data on second attempt");
            DEBUG_PRINT("MQTT Client State: ");
            DEBUG_PRINTLN(mqttClient.state());
            return false;
        }
    }
}

bool publishDataPoint(const DataPoint& dp) {
    String jsonPayload = "{";
    jsonPayload += "\"device\": \"" + dp.device + "\", ";
    jsonPayload += "\"temperature\": " + String(dp.temperature, 2) + ", ";
    jsonPayload += "\"humidity\": " + String(dp.humidity, 2) + ", ";
    jsonPayload += "\"latitude\": " + String(dp.latitude, 6) + ", ";
    jsonPayload += "\"longitude\": " + String(dp.longitude, 6) + ", ";
    jsonPayload += "\"wake_source\": \"" + wakeSource + "\", ";
    jsonPayload += (wakeSource == "interrupt") ? "\"box_status\": \"opened\"" : "\"box_status\": \"\"";
    jsonPayload += "}";
    return publishWithRetry(mqtt_topic, jsonPayload.c_str());
}

bool addToUnsentBuffer(const DataPoint& dp) {
    if (unsentDataIndex < UNSENT_DATA_POINTS) {
        unsentDataBuffer[unsentDataIndex++] = dp;
        DEBUG_PRINTLN("Added DataPoint to unsent buffer.");
        return true;
    } else {
        DEBUG_PRINTLN("Unsent buffer is full. Dropping DataPoint.");
        return false;
    }
}

void getSIMLocation(float& latitude, float& longitude) {
    DEBUG_PRINTLN("Obtaining location from SIM...");
    String clbsResponse = sendAT("AT+CLBS=4", 10000, true);
    int index = clbsResponse.indexOf("+CLBS:");
    if (index != -1) {
        String data = clbsResponse.substring(index + 6);
        data.trim();
        int firstComma = data.indexOf(',');
        int secondComma = data.indexOf(',', firstComma + 1);
        int thirdComma = data.indexOf(',', secondComma + 1);
        if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
            String latStr = data.substring(firstComma + 1, secondComma);
            String lonStr = data.substring(secondComma + 1, thirdComma);
            latitude = latStr.toFloat();
            longitude = lonStr.toFloat();
            DEBUG_PRINT("SIM Latitude: ");
            DEBUG_PRINTLN(String(latitude, 6));
            DEBUG_PRINT("SIM Longitude: ");
            DEBUG_PRINTLN(String(longitude, 6));
        } else {
            DEBUG_PRINTLN("Failed to parse SIM location data.");
            latitude = 0.0;
            longitude = 0.0;
        }
    } else {
        DEBUG_PRINTLN("Failed to get location from SIM.");
        latitude = 0.0;
        longitude = 0.0;
    }
}

void getGPSLocation(float& latitude, float& longitude) {
    if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        DEBUG_PRINT("GPS Latitude: ");
        DEBUG_PRINTLN(String(latitude, 6));
        DEBUG_PRINT("GPS Longitude: ");
        DEBUG_PRINTLN(String(longitude, 6));
    } else {
        DEBUG_PRINTLN("GPS location invalid.");
        latitude = 0.0;
        longitude = 0.0;
    }
}

void connectAndReadBLE(BLEClient* pClient, BLEAddress address, const char* deviceID, float& temp, float& humi, int& battery) {
    const int maxRetries = 3;
    bool connected = false;
    for (int attempt = 1; attempt <= maxRetries; attempt++) {
        DEBUG_PRINT("Attempt ");
        DEBUG_PRINT(attempt);
        DEBUG_PRINT(" to connect to device ");
        DEBUG_PRINTLN(deviceID);
        if (pClient->connect(address)) {
            connected = true;
            DEBUG_PRINT("Successfully connected to device ");
            DEBUG_PRINT(deviceID);
            DEBUG_PRINTLN();
            break;
        } else {
            DEBUG_PRINT("Failed to connect to device ");
            DEBUG_PRINT(deviceID);
            DEBUG_PRINT(" on attempt ");
            DEBUG_PRINTLN(attempt);
            delay(1000 * attempt);  // Exponential backoff
            esp_task_wdt_feed();
        }
    }
    if (connected) {
        BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
        if (pRemoteService != nullptr) {
            BLERemoteCharacteristic* pChar = pRemoteService->getCharacteristic(charUUID);
            if (pChar != nullptr) {
                String value = pChar->readValue();
                DEBUG_PRINT("Raw data: ");
                for (size_t i = 0; i < value.length(); i++) {
                    DEBUG_PRINT(String((uint8_t)value[i], HEX) + " ");
                }
                DEBUG_PRINTLN();
                if (value.length() >= 3) {
                    int16_t tempRaw = (int16_t)(((uint8_t)value[1] << 8) | (uint8_t)value[0]);
                    temp = tempRaw * 0.01;
                    humi = (float)(uint8_t)value[2];
                } else {
                    DEBUG_PRINTLN("Insufficient data length for temperature and humidity.");
                    temp = 0.0;
                    humi = 0.0;
                }
            } else {
                DEBUG_PRINTLN("Failed to find characteristic for temperature and humidity.");
                temp = 0.0;
                humi = 0.0;
            }
        } else {
            DEBUG_PRINTLN("Failed to find the remote service for temperature and humidity.");
            temp = 0.0;
            humi = 0.0;
        }
        BLERemoteService* pBatteryService = pClient->getService(batteryServiceUUID);
        if (pBatteryService != nullptr) {
            BLERemoteCharacteristic* pBatteryCharacteristic = pBatteryService->getCharacteristic(batteryCharUUID);
            if (pBatteryCharacteristic != nullptr) {
                battery = pBatteryCharacteristic->readUInt8();
            } else {
                DEBUG_PRINTLN("Failed to find characteristic for battery.");
                battery = 0;
            }
        } else {
            DEBUG_PRINTLN("Failed to find the remote service for battery.");
            battery = 0;
        }
    } else {
        DEBUG_PRINT("Could not connect to device ");
        DEBUG_PRINT(deviceID);
        DEBUG_PRINTLN(" after maximum attempts.");
        temp = 0.0;
        humi = 0.0;
        battery = 0;
    }
    if (pClient->isConnected()) {
        pClient->disconnect();
    }
}

void modemPowerOn() {
    const int SUPPLY_PIN = 12;
    const int RESET_PIN = 5;
    const int POWER_PIN = PWR_PIN;

    pinMode(SUPPLY_PIN, OUTPUT);
    digitalWrite(SUPPLY_PIN, HIGH);
    pinMode(RESET_PIN, OUTPUT);
    digitalWrite(RESET_PIN, LOW);
    delay(100);
    digitalWrite(RESET_PIN, HIGH);
    delay(3000);
    digitalWrite(RESET_PIN, LOW);
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, LOW);
    delay(100);
    digitalWrite(POWER_PIN, HIGH);
    delay(1000);
    digitalWrite(POWER_PIN, LOW);
}

void modemPowerOff() {
    DEBUG_PRINTLN("Powering off modem...");
    modem.poweroff();
    digitalWrite(PWR_PIN, LOW);
}

void readGPS() {
    unsigned long startTime = millis();
    const unsigned long timeout = 5000;
    while ((millis() - startTime) < timeout) {
        while (ssGPS.available() > 0) {
            gps.encode((char)ssGPS.read());
        }
        esp_task_wdt_feed();
    }
}

DataPoint collectData(String deviceID, float temp, float humi) {
    DEBUG_PRINT("Collecting data for device ");
    DEBUG_PRINTLN(deviceID);
    DataPoint dp;
    dp.device = deviceID;
    dp.temperature = temp;
    dp.humidity = humi;
    dp.latitude = currentLatitude;
    dp.longitude = currentLongitude;
    DEBUG_PRINT("Data collected for device ");
    DEBUG_PRINTLN(deviceID);
    return dp;
}

void enterDeepSleep() {
    DEBUG_PRINTLN("Preparing to enter Deep Sleep...");

    // Disconnect MQTT if connected
    if (mqttClient.connected()) {
        mqttClient.disconnect();
    }
    // Power off modem
    modemPowerOff();
    // Deinitialize BLE and end GPS serial communication
    BLEDevice::deinit();
    ssGPS.end();

    // Enable external wake-up from MPU6050 (active high) and timer wake-up
    esp_sleep_enable_ext0_wakeup((gpio_num_t)MPU_INT_PIN, 1);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

    DEBUG_PRINTLN("Entering Deep Sleep...");
    Serial.flush();
    esp_deep_sleep_start();
}

// ----------------------- MPU6050 Setup -----------------------
void initMPU6050() {
    Wire.begin(SDA_MPU, SCL_MPU);
    if (!mpu.begin(0x68)) {
        DEBUG_PRINTLN("Failed to find MPU6050 at 0x68");
        enterDeepSleep();
    }
    DEBUG_PRINTLN("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

void setupMotionInterrupt() {
    writeMPURegister(0x1F, 5);  // Set threshold
    writeMPURegister(0x20, 3);  // Set duration
    writeMPURegister(0x38, 0x40);  // Enable motion interrupt
    writeMPURegister(0x37, 0x00);  // Configure interrupt pin
    DEBUG_PRINT("MOT_THR (0x1F) = 0x");
    DEBUG_PRINTLN(String(readMPURegister(0x1F), HEX));
    DEBUG_PRINT("MOT_DUR (0x20) = 0x");
    DEBUG_PRINTLN(String(readMPURegister(0x20), HEX));
    DEBUG_PRINT("INT_ENABLE (0x38) = 0x");
    DEBUG_PRINTLN(String(readMPURegister(0x38), HEX));
    DEBUG_PRINT("INT_PIN_CFG (0x37) = 0x");
    DEBUG_PRINTLN(String(readMPURegister(0x37), HEX));
    DEBUG_PRINT("PWR_MGMT_1 (0x6B) = 0x");
    DEBUG_PRINTLN(String(readMPURegister(0x6B), HEX));
}

void writeMPURegister(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(0x68);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t readMPURegister(uint8_t reg) {
    Wire.beginTransmission(0x68);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x68, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0xFF;
}

// ----------------------- Setup Function -----------------------
void setup() {
    Serial.begin(115200);
    delay(1000);
    DEBUG_PRINTLN("Starting...");
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    // Initialize Watchdog Timer with a 10-second timeout (adjust as needed)
    esp_task_wdt_init(10, true);
    esp_task_wdt_add(NULL);  // Add current task to WDT

    // Determine wake-up reason
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
        DEBUG_PRINTLN("Woken up by MPU6050 Motion Interrupt!");
        wakeSource = "interrupt";
    } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        DEBUG_PRINTLN("Woken up by Timer");
        wakeSource = "timer";
    } else {
        DEBUG_PRINTLN("Woken up by another source or on first boot");
        wakeSource = "other";
    }

    // Initialize MPU6050 and configure motion interrupt
    initMPU6050();
    setupMotionInterrupt();

    // Begin communication with SIM7600
    SerialAT.begin(GSM_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
    delay(3000);

    pinMode(PWR_PIN, OUTPUT);
    pinMode(PIN_DTR, OUTPUT);
    pinMode(PIN_RI, INPUT);

    modemPowerOn();
    delay(500);
    digitalWrite(PIN_DTR, HIGH);
    delay(1000);

    DEBUG_PRINTLN("Initializing modem...");
    if (!modem.init()) {
        DEBUG_PRINTLN("Failed to initialize modem");
        enterDeepSleep();
    }
    delay(1000);
    if (modem.getSimStatus() != SIM_READY) {
        DEBUG_PRINTLN("SIM not ready. Check SIM card.");
        enterDeepSleep();
    }
    DEBUG_PRINTLN("Waiting for network...");
    if (!modem.waitForNetwork()) {
        DEBUG_PRINTLN("Network registration failed");
        enterDeepSleep();
    }
    DEBUG_PRINTLN("Network registered");

    DEBUG_PRINT("Setting APN: ");
    DEBUG_PRINTLN(apn);
    modem.sendAT("+CGDCONT=1,\"IP\",\"" + String(apn) + "\"");
    if (modem.waitResponse(10000L, "OK") != 1) {
        DEBUG_PRINTLN("Failed to set APN");
    }
    DEBUG_PRINT("Connecting to LTE using APN: ");
    DEBUG_PRINTLN(apn);
    if (!modem.gprsConnect(apn, "", "")) {
        DEBUG_PRINTLN("LTE connection failed");
        enterDeepSleep();
    }
    DEBUG_PRINTLN("LTE connected successfully");

    IPAddress ip = modem.localIP();
    DEBUG_PRINT("Modem IP Address: ");
    DEBUG_PRINTLN(ip.toString());

    printNetworkInfo();

    mqttClient.setServer(mqtt_server, mqtt_port);
    reconnectMQTT();

    // Initialize BLE
    BLEDevice::init("");
    pClientA = BLEDevice::createClient();
    pClientB = BLEDevice::createClient();

    // Initialize GPS
    ssGPS.begin(GPSBaud, SERIAL_8N1, RXPin_GPS, TXPin_GPS, false);
    DEBUG_PRINTLN(TinyGPSPlus::libraryVersion());

    DEBUG_PRINTLN("Reading GPS data...");
    readGPS();

    // Get location from GPS (or SIM if GPS data is invalid)
    if (gps.location.isValid()) {
        getGPSLocation(currentLatitude, currentLongitude);
    } else {
        getSIMLocation(currentLatitude, currentLongitude);
    }

    // Attempt to send any unsent data from previous cycles
    if (unsentDataIndex > 0) {
        DEBUG_PRINTLN("Attempting to send unsent data...");
        for (int i = 0; i < unsentDataIndex; ) {
            if (publishDataPoint(unsentDataBuffer[i])) {
                for (int j = i; j < unsentDataIndex - 1; j++) {
                    unsentDataBuffer[j] = unsentDataBuffer[j + 1];
                }
                unsentDataIndex--;
                DEBUG_PRINTLN("Unsent data point sent and removed from buffer.");
            } else {
                DEBUG_PRINTLN("Failed to resend unsent data point.");
                i++;
            }
            esp_task_wdt_feed();
        }
    }

    // Read BLE sensors for both devices
    connectAndReadBLE(pClientA, deviceA_addr, "A", tempA, humiA, batteryA);
    connectAndReadBLE(pClientB, deviceB_addr, "B", tempB, humiB, batteryB);

    // Collect and publish data for Device A
    DataPoint dpA = collectData("A", tempA, humiA);
    DEBUG_PRINTLN("Publishing data for Device A to MQTT...");
    if (publishDataPoint(dpA)) {
        DEBUG_PRINTLN("Data for Device A published successfully.");
    } else {
        DEBUG_PRINTLN("Failed to publish data for Device A. Adding to unsent buffer.");
        if (!addToUnsentBuffer(dpA)) {
            DEBUG_PRINTLN("Unsent buffer is full. Data point cannot be added.");
        }
    }

    // Collect and publish data for Device B
    DataPoint dpB = collectData("B", tempB, humiB);
    DEBUG_PRINTLN("Publishing data for Device B to MQTT...");
    if (publishDataPoint(dpB)) {
        DEBUG_PRINTLN("Data for Device B published successfully.");
    } else {
        DEBUG_PRINTLN("Failed to publish data for Device B. Adding to unsent buffer.");
        if (!addToUnsentBuffer(dpB)) {
            DEBUG_PRINTLN("Unsent buffer is full. Data point cannot be added.");
        }
    }

    // Enter deep sleep mode
    enterDeepSleep();
}

void loop() {
    // All operations are handled in setup()
}
