/*
MAKE SURE TO ADD THIS IN YOUR ADDTIONAL BOARD MANAGER URL IF NEVER USED AN ESP 32 
FILE--->PREFENCES--->SETTINGS
https://dl.espressif.com/dl/package_esp32_index.json
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

________________________________________________________________________________________________________________________________________________________________________________________________
Imported library need for this project, make sure to download all the libraries before using (On the left side tabs)

*BOARD MANAGER INSTALL*
-esp32 by Espressif Systems

*/
//_________________________________________________________________________________________________________________________________________________________________________________________________

#include <esp_now.h>
#include <WiFi.h>

//--------------------------------------------------------------------------------
//PLUG IN YOU INFROMATION HERE " --HERE-- "

// Configuration of wifi and adafruit
// WiFi Credentials (MUST MATCH SENDER)
#define WIFI_SSID "WIFI"                  //<--- " --HERE-- "
#define WIFI_PASS "PASSWORD"              //<--- " --HERE-- "

//--------------------------------------------------------------------------------
//THIS MUST MATCH WITH SS.ESP

#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for microseconds to seconds
#define TIME_TO_SLEEP 20 // Sleep time in seconds

//--------------------------------------------------------------------------------

const byte Enable_Pin = 18;  // LED/Watering system control pin

//--------------------------------------------------------------------------------
//MUST MATCH WITH SS.ESP
typedef struct __attribute__((packed)) SensorData {
  int16_t airTemp;
  uint8_t airHum;
  uint8_t soilMoist;
  uint8_t power;
} SensorData;

SensorData sensorData;
//--------------------------------------------------------------------------------
//This what allows how long and what vaule the plants should be watered 
//LED COMMENTS CAN BE USED FOR ENABLE FOR RELAY OR MOTOR DRIVER 

void Light() {
  if (sensorData.soilMoist >= 0 && sensorData.soilMoist < 40) {
    digitalWrite(Enable_Pin, HIGH);  // Turn on the LED (watering system)
    delay(2000); // 3 minutes watering
    digitalWrite(Enable_Pin, LOW);   // Turn off the LED
  } 
  else if (sensorData.soilMoist >= 40 && sensorData.soilMoist < 60) {
    digitalWrite(Enable_Pin, HIGH);  // Turn on the LED (watering system)
    delay(120000); // 2 minutes watering
    digitalWrite(Enable_Pin, LOW);   // Turn off the LED
  } 
  else if (sensorData.soilMoist >= 60 && sensorData.soilMoist < 80) {
    digitalWrite(Enable_Pin, HIGH);  // Turn on the LED (watering system)
    delay(60000); // 1 minute watering
    digitalWrite(Enable_Pin, LOW);   // Turn off the LED
  } 
  else if (sensorData.soilMoist >= 80) {
    digitalWrite(Enable_Pin, LOW);   // No watering if soil moisture is above 80%
  }
}
//--------------------------------------------------------------------------------

// Callback function to receive data via ESP-NOW
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&sensorData, incomingData, sizeof(sensorData));
  unsigned int timer = micros();
  // Convert temperature back from fixed-point representation
  float actualTemp = sensorData.airTemp / 100.0;
  
  Serial.println("Received data:");
  Serial.print("Air Temperature: "); 
  Serial.println(actualTemp);
  Serial.print("Air Humidity: ");
  Serial.println(sensorData.airHum);
  Serial.print("Soil Moisture: ");
  Serial.println(sensorData.soilMoist);

  Light();  // Call the Light function to control the LED based on soil moisture

  sensorData.power = 0;  // Sleep for a defined amount of time after processing data
  esp_sleep_enable_timer_wakeup((TIME_TO_SLEEP * uS_TO_S_FACTOR) - timer);
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();  // Deep sleep
}

void setup() {
  Serial.begin(115200);
  btStop();  // Stop Bluetooth to free up resources

  pinMode(Enable_Pin, OUTPUT);  // Set Enable_Pin as output for controlling the LED
  digitalWrite(Enable_Pin, LOW);  // Ensure the LED starts off

  // Set device as Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Connect to WiFi first
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");

  // Get and print WiFi channel (Optional, if you need to check the channel)
  int currentChannel = WiFi.channel();
  Serial.print("Current WiFi Channel: ");
  Serial.println(currentChannel);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback function for receiving data
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  Serial.println("ESP-NOW Receiver Ready");
}

void loop() {
  // Nothing to do here since we are using deep sleep and ESP-NOW
}
