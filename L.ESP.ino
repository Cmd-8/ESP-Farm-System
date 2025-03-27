/*
MAKE SURE TO ADD THIS IN YOUR ADDTIONAL BOARD MANAGER URL IF NEVER USED AN ESP 32 
FILE--->PREFENCES--->SETTINGS
https://dl.espressif.com/dl/package_esp32_index.json
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

________________________________________________________________________________________________________________________________________________________________________________________________
Imported library need for this project, make sure to download all the libraries before using (On the left side tabs)

*BOARD MANAGER INSTALL*
-esp32 by Espressif Systems
------------------------------------------
*LIBRARY MANAGER INTSTALL*
-Adafruit BME280 by Adafruit
-Adafruit BusIO by  Adafruit
-Adafuit IO Arduino by Adafruit
-Adafruit MQTT by Adafruit

*/
//_________________________________________________________________________________________________________________________________________________________________________________________________

//#include <esp_now.h>
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds 
#define TIME_TO_SLEEP  86400       // Time ESP32 will go to sleep for 24 hours

//Only Relay  Ligt sensor used only to record data before setting up 
//Define I2C pins (you can change these if needed)
//#define SDA_PIN 21
//#define SCL_PIN 22

//______________________________________________________________________________________________________________________________________________________________________________________________________________

//This Program takes data from Adafruit IO interface to set up a timer for the lights for the plants and always turn on the dame time c 

//______________________________________________________________________________________________________________________________________________________________________________________________________________



//The configuration for wifi and adafruit IO
//______________________________________________________________________________________________________________________________________________________________________________________________________________

//--------------------------------------------------------------------------------
//PLUG IN YOU INFROMATION HERE " --HERE-- "

const char* WIFI_SSID = "WIFI";           //<--- " --HERE-- "
const char* WIFI_PASSWORD = "PASSWORD";   //<--- " --HERE-- "

// Adafruit IO Configuration
#define AIO_SERVER "io.adafruit.com"
#define AIO_PORT 1883
#define AIO_USERNAME "USERNAME"           //<--- " --HERE-- "
#define AIO_KEY "API KEY"                 //<--- " --HERE-- "

//--------------------------------------------------------------------------------
//PLEASE PUT YOUR INFORMATION INTO THE RIGHT FEEDS

//USED to send data for adadruit which is connceted to the feed you made in it
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_PORT, AIO_USERNAME, AIO_KEY);

// Feed Declarations (verify exact feed names in Adafruit IO)
Adafruit_MQTT_Subscribe input1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/light.input-1");
Adafruit_MQTT_Subscribe input2 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/light.input-2");
Adafruit_MQTT_Subscribe input3 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/light.input-3");

//______________________________________________________________________________________________________________________________________________________________________________________________________________

//Adafruit_TSL2561_Unified tsl(TSL2561_ADDR_FLOAT, &Wire);
//USE EXTERNAL POWER FOR RELAY 3.3V SO WHEN THE ESP IS OFF IT STILL LIGHTS
//TSL2561 SENSOR
uint32_t Input1 = 0;
uint32_t Input2 = 0;
uint32_t Input3 = 0;

//const byte RELAY_PIN = 23;  //Relay enable pin or can be used for testing with LED Light 
const int RELAY_PIN = 2; //

void setup() {
  Serial.begin(115200);
  btStop(); //Turns off bluetooth to save power
  Serial.println("Starting System");
  pinMode(RELAY_PIN, OUTPUT);  // Set the relay control pin as output
  digitalWrite(RELAY_PIN, LOW);  // Initialize the relay to off

  //Wire.begin(SDA_PIN, SCL_PIN);

  //tsl.setGain(TSL2561_GAIN_1X);  // 1x gain (for bright conditions)
  //tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);  // 101ms integration time (good balance for indoor/outdoor use)

//This begins wifi and setup input time for adafruit IO with Input1(seconds), Input2(minute), Input3(hours)
//______________________________________________________________________________________________________________________________________________________________________________________________________________

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");

  // Initialize MQTT
  mqtt.subscribe(&input1);
  mqtt.subscribe(&input2);
  mqtt.subscribe(&input3);

  // Connect to MQTT
  Serial.println("Connecting to Adafruit IO");
  int8_t mqttRetries = 5;
  while (mqtt.connect() != 0 && mqttRetries-- > 0) {
    mqtt.disconnect();
    delay(5000);
    Serial.println("Retrying MQTT connection...");
  }
  
  if (!mqtt.connected()) {
    Serial.println("MQTT Connection Failed!");
    esp_deep_sleep_start();
  }
  Serial.println("Adafruit IO Connected!");

  // Message Processing with Timeout
  Serial.println("Waiting for messages (60 seconds)...");
  uint32_t startTime = millis();
  while (millis() - startTime < 60000) {
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(10000))) {
      if (subscription == &input1) {
        Input1 = atoi((char *)input1.lastread);
        Serial.print("Received Input1: "); Serial.println(Input1);
      }
      else if (subscription == &input2) {
        Input2 = atoi((char *)input2.lastread);
        Serial.print("Received Input2: "); Serial.println(Input2);
      }
      else if (subscription == &input3) {
        Input3 = atoi((char *)input3.lastread);
        Serial.print("Received Input3: "); Serial.println(Input3);
      }
    }
    
    // Exit early if all values received
    if (Input1 + Input2 + Input3 > 0) break;
  }

//Does the calculation for Deep-Sleep and Active light time
//______________________________________________________________________________________________________________________________________________________________________________________________________________

  // Calculate Active Time
  //3600 hours to sec    60 mintues to seconds 
  uint32_t totalActive = (Input3 * 3600) + (Input2 * 60) + Input1;
  Serial.print("Total Active Seconds: "); Serial.println(totalActive);

  // Active Period
  uint32_t activeMillis = totalActive * 1000;
  Serial.print("Active for: "); Serial.print(activeMillis); Serial.println("ms");
  //WiFi.disconnect(); //WIFI OFF HOPEFULLY IT RESARTS WHEN WAKING UP
  digitalWrite(RELAY_PIN, HIGH);
  delay(activeMillis);

  // Sleep Calculation
  uint64_t sleepMicros = (TIME_TO_SLEEP * uS_TO_S_FACTOR) - (activeMillis * 1000ULL);  // 1000 used for millis to micro
  if (sleepMicros > UINT64_MAX / 2) sleepMicros = 0;
  
  Serial.println("Entering Deep Sleep");
  esp_sleep_enable_timer_wakeup(sleepMicros);
  esp_deep_sleep_start();
}
 
void loop(){
}