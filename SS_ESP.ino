/*
MAKE SURE TO ADD THIS IN YOUR ADDTIONAL BOARD MANAGER URL IF NEVER USED AN ESP 32 
FILE--->PREFENCES--->SETTINGS
https://dl.espressif.com/dl/package_esp32_index.json
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

_____________________________________________________________________________________________
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

#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Adafruit_MQTT.h>            
#include <Adafruit_MQTT_Client.h>     

//--------------------------------------------------------------------------------
//PLUG IN YOU INFROMATION HERE " --HERE-- "

// Configuration of wifi and adafruit
#define WIFI_SSID "WIFI"                //<--- " --HERE-- "
#define WIFI_PASS "PASSWORD"            //<--- " --HERE-- "
#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883
#define AIO_USERNAME "USERNAME"         //<--- " --HERE-- "
#define AIO_KEY "API KEY"               //<--- " --HERE-- "

//--------------------------------------------------------------------------------
//TIME_TO_SLEEP IS HOW OFTEN YOU WANT DATA RO BE TAKEN
//3600 = 1 HR   7200 = 2 HR       1800 = 30 MINS      900 = 15 MINS
//DO 10 FOR TESTING (10 SECONDS) 

//The sleep configraution for how long it goes to power sbaing mode (deep-sleep)
#define uS_TO_S_FACTOR 1000000ULL     //conversion from microseconds to seconds 
#define TIME_TO_SLEEP 900             //THIS DEFAULT AT 15 MINUTES  
#define MAX_RETRIES 5                 //connection retries
#define RETRY_DELAY 500               //ms between retries
//--------------------------------------------------------------------------------

//ULL is unsignes long long which allowes to sleep longer than 32 minutes max, the longest max known would be 48 hours 

//--------------------------------------------------------------------------------
//YOU CAN CHANGE THE PIN ANY OF THESE IF YOU WANT 
//THIS WHERE THE NUMBER PIN ON ESP-32 YOU WOULD CONNECT

// SPI pins for BME280
#define BME_CS    5       // Chip select pin for SPI
#define BME_SCK   18      // Clock pin
#define BME_MISO  19      // MISO pin
#define BME_MOSI  23      // MOSI pin

Adafruit_BME280 bme;  // Create an instance of the BME280 sensor

// Soil sensor analog pins
const int soilSensor1Pin = 32;
const int soilSensor2Pin = 35;
const int soilSensor3Pin = 34;
const int soilSensor4Pin = 36;
//--------------------------------------------------------------------------------

//_________________________________________________________________________________________________________________________________________________________________________________________________
//PLEASE PUT YOUR INFORMATION INTO THE RIGHT FEEDS

//USED to send data for adadruit which is connceted to the feed you made in it
// MQTT Client
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

//*MQTT Feeds*
Adafruit_MQTT_Publish humidityFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish soilMoistureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soil-moisture");
Adafruit_MQTT_Publish temperatureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");

//_________________________________________________________________________________________________________________________________________________________________________________________________

// Parameters for averaging and low-pass filter
const int numSamples = 10;  // Number of samples to take for each sensor
float alpha = 0.1;           // Smoothing factor for low-pass filter CAN EDIT TO BE FROM 0.1 - .9

// Arrays for storing readings
float temperatureSamples[numSamples];
float humiditySamples[numSamples];
float pressureSamples[numSamples];
int soilMoistureSamples[4][numSamples];

// Variables to store filtered data
float filteredTemperature = 0;
float filteredHumidity = 0;
float filteredPressure = 0;
float smoothedSoilMoisture[4] = {0.0, 0.0, 0.0, 0.0};

// Summing variables for averaging
long tempSum = 0;
long humSum = 0;
long pressSum = 0;
long soilSum[4] = {0, 0, 0, 0};

int sampleIndex = 0;
bool dataCollected = false;

RTC_DATA_ATTR unsigned int bootCount = 0;
volatile bool dataSent = false;

//_________________________________________________________________________________________________________________________________________________________________________________________________
//This section includes all the other congfigurations for adafruit and esp-now protocol, like cheching for connection and confirmation on data being sent 

// REPLACE WITH YOUR RECEIVER MAC ADDRESSES
uint8_t broadcastAddress1[] = {0x  , 0x  , 0x  , 0x  , 0x  , 0x  };  //{0x3C, 0xBC, 0xBB, 0x06, 0x38, 0xB0};


// Struct for sensor data transmission THIS MUST MATCH WITH THE OTHER ESP-32
//---------------------------------------------------------------------------
typedef struct __attribute__((packed)) SensorData {
  int16_t airTemp;
  uint8_t airHum;
  uint8_t soilMoist;
  uint8_t power;
} SensorData;

SensorData sensorData;
esp_now_peer_info_t peerInfo;
//---------------------------------------------------------------------------


// MQTT Connection Function
void connectToAdafruitIO() {
  Serial.println("Connecting to Adafruit IO...");
  
  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  // Get and print WiFi channel
  int currentChannel = WiFi.channel();
  Serial.print("Current WiFi Channel: ");
  Serial.println(currentChannel);

  // Connect to MQTT
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);
  }
  Serial.println("Adafruit IO Connected!");
}


// MQTT Publish Function
bool publishToAdafruitIO() {
  // Ensure MQTT connection
  if (!mqtt.connected()) {
    connectToAdafruitIO();
  }

  // Publish sensor data
  bool publishSuccess = true;
  
  publishSuccess &= temperatureFeed.publish(filteredTemperature);
  publishSuccess &= humidityFeed.publish(filteredHumidity);
  publishSuccess &= soilMoistureFeed.publish(smoothedSoilMoisture[1]);

  if (publishSuccess) {
    Serial.println("Data published successfully to Adafruit IO!");
    return true;
  } else {
    Serial.println("Failed to publish data to Adafruit IO.");
    return false;
  }
}


// ESP-NOW Peer Addition Function
bool addPeer(const uint8_t* peerAddr) {
  memset(&peerInfo, 0, sizeof(peerInfo));
  
  // Use the current WiFi channel
  peerInfo.channel = WiFi.channel();
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, peerAddr, 6);
  
  esp_err_t result = esp_now_add_peer(&peerInfo);
  switch(result) {
    case ESP_OK:
      Serial.printf("Peer added successfully: %02x:%02x:%02x:%02x:%02x:%02x\n",
                    peerAddr[0], peerAddr[1], peerAddr[2], 
                    peerAddr[3], peerAddr[4], peerAddr[5]);
      return true;
    case ESP_ERR_ESPNOW_NOT_INIT:
      Serial.println("ESP-NOW not initialized");
      break;
    case ESP_ERR_ESPNOW_ARG:
      Serial.println("Invalid argument");
      break;
    case ESP_ERR_ESPNOW_FULL:
      Serial.println("Peer list full");
      break;
    default:
      Serial.printf("Unknown error: %d\n", result);
  }
  return false;
}


// ESP-NOW Send Callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  
  Serial.printf("Sent to %s - Status: %s\n", 
                macStr,
                status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
  dataSent = (status == ESP_NOW_SEND_SUCCESS);
}

// Send Data to Specific Device
bool sendData(const uint8_t* targetAddress, const SensorData& data) {
    int retryCount = 0;
    bool success = false;
    
    while (retryCount < MAX_RETRIES && !success) {
        dataSent = false;
        Serial.printf("Sending to %02X:%02X:%02X:%02X:%02X:%02X - Attempt %d of %d\n",
                     targetAddress[0], targetAddress[1], targetAddress[2],
                     targetAddress[3], targetAddress[4], targetAddress[5],
                     retryCount + 1, MAX_RETRIES);
        
        // Ensure we're on the correct channel before sending
        esp_wifi_set_channel(WiFi.channel(), WIFI_SECOND_CHAN_NONE);
        
        esp_err_t result = esp_now_send(targetAddress, (uint8_t*)&data, sizeof(data));
        
        if (result != ESP_OK) {
            Serial.println("Send Init Failed");
            retryCount++;
            delay(RETRY_DELAY);
            continue;
        }
        
        // Wait for callback with timeout
        unsigned long start = millis();
        while (!dataSent && (millis() - start) < 1000) {
            delay(10);
        }
        
        if (dataSent) {
            success = true;
            Serial.println("Data sent successfully");
            break;
        } else {
            Serial.println("Send timeout or failed");
            retryCount++;
            delay(RETRY_DELAY);
        }
    }
    return success;
}

//_________________________________________________________________________________________________________________________________________________________________________________________________
//This sections brings it all together and actually what runs

void setup() {
  int Timer = micros();                 //Timer 
  Serial.begin(115200);                 //Allow communcation through usb with the PC
  WiFi.setTxPower(WIFI_POWER_15dBm);    //This to increase wifi range of esp-32 
  btStop();                             //Turn off bluetooth to save power 

  // Initialize SPI and BME280 sensor
  SPI.begin(BME_SCK, BME_MISO, BME_MOSI, BME_CS);
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1); // Stop if sensor is not found
  }

//---------------------------------------------------------------------------------------------------------------------  
//THIS WHERE THE CALCULTAIONS FOR MOVING AVERAGE AND LOW PASS FILTER IS ADD TO SENSOR DATA
  
  // Collect 10 samples for each sensor
  while (!dataCollected) {
    // --- Collecting Data from BME280 ---  To test do random(0, 100); instead of bme.read
    float temp = bme.readTemperature();                                 //random(0, 100);
    float humidity = bme.readHumidity();                                //random(0, 100);
    float pressure = bme.readPressure() / 100.0F;  // Pressure in hPa   //random(0, 100);

    // Store the raw readings
    temperatureSamples[sampleIndex] = temp;
    humiditySamples[sampleIndex] = humidity;
    pressureSamples[sampleIndex] = pressure;
    
    /*
    for (int i = 0; i < 4; i++) {
      soilMoistureSamples[i][sampleIndex] = random(0,100);  // Analog pins A0, A1, A2, A3 for soil sensors
    }
    */
    
    // Collect soil moisture readings
    for (int i = 0; i < 4; i++) {
      soilMoistureSamples[i][sampleIndex] = analogRead(i + A0);  // Analog pins A0, A1, A2, A3 for soil sensors
    }
    
    sampleIndex++;  // Move to the next sample index

    // When all samples are collected, set the flag
    if (sampleIndex >= numSamples) {
      dataCollected = true;
    }

    // Delay between readings
    delay(100);  // 100ms delay
  }

  // Process and average the 10 samples
  for (int i = 0; i < numSamples; i++) {
    tempSum += temperatureSamples[i];
    humSum += humiditySamples[i];
    pressSum += pressureSamples[i];

    for (int j = 0; j < 4; j++) {
      soilSum[j] += soilMoistureSamples[j][i];
    }
  }

  // Calculate the averages for each sensor
  float avgTemperature = tempSum / numSamples;
  float avgHumidity = humSum / numSamples;
  float avgPressure = pressSum / numSamples;

  // Apply Low-Pass Filter
  filteredTemperature = (alpha * avgTemperature) + (1 - alpha) * filteredTemperature;
  filteredHumidity = (alpha * avgHumidity) + (1 - alpha) * filteredHumidity;
  filteredPressure = (alpha * avgPressure) + (1 - alpha) * filteredPressure;

  for (int i = 0; i < 4; i++) {
    float avgSoilMoisture = soilSum[i] / numSamples;
    smoothedSoilMoisture[i] = (alpha * avgSoilMoisture) + (1 - alpha) * smoothedSoilMoisture[i];
  }

  // Print soil sensor readings
  for (int i = 0; i < 4; i++) {
    Serial.print("Soil Sensor "); Serial.print(i+1); Serial.print(": ");
    Serial.print(smoothedSoilMoisture[i]); Serial.println(" %");
  }

//---------------------------------------------------------------------------------------------------------------------  

  // WiFi and ESP-NOW Setup
  WiFi.mode(WIFI_STA);
  int currentChannel = WiFi.channel();
  esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
  
  // Connect to Adafruit IO
  connectToAdafruitIO();

  // Publish data to Adafruit IO
  publishToAdafruitIO();
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    esp_deep_sleep_start();
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
  
  // Add peers
  bool peer1Added = addPeer(broadcastAddress1);
  
  if (!peer1Added) {
    Serial.println("No peers added successfully. Entering sleep.");
    esp_deep_sleep_start();
    return;
  }
  
  // Map soil moisture values
  for (int i = 0; i < 4; i++) {
    smoothedSoilMoisture[i] = map(smoothedSoilMoisture[i], 0, 4096, 0, 100);
  }

  // Prepare sensor data
  sensorData.airTemp = filteredTemperature * 100;  // Multiply by 100 for precision
  sensorData.airHum = filteredHumidity;
  sensorData.soilMoist = smoothedSoilMoisture[1]; // Using first soil moisture sensor
  sensorData.power = 1;  // Battery percentage or power status
  
  // Send to device
  bool device1Success = sendData(broadcastAddress1, sensorData);
  
  // Log results
  Serial.printf("Device 1 send %s\n", device1Success ? "successful" : "failed");

  // Sleep configuration
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.flush();
  delay(100);
  esp_deep_sleep_start();
}

void loop() {
  // No need to loop; data is collected and processed only once in setup
}