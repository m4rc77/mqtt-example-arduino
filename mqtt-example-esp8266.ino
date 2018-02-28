// Board Libraries
#include <ESP8266WiFi.h>

// Required libraries:
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <PubSubClient.h>

// The Wemos D1 mini's pins do not map to arduino pin numbers accurately ... see
// https://github.com/esp8266/Arduino/issues/1243
#define DHT_PIN 2 // => D3
#define LED_PIN 0 // => D4

// Define DHT Temperature/Humidity Sensor type ...
#define DHTTYPE DHT22

#include "config.h"
// File config.h sould contain the configuration in the following form ...
/*
const char *ssid       = "your WLAN SSID";
const char *password   = "your WLAN password";
const char *mqttServer = "the broker url";

// see https://github.com/mqtt-smarthome/mqtt-smarthome/blob/master/Architecture.
const char *mqttTopicTemperature = "/REPLACE_ME/status/temperature";
const char *mqttTopicHumidity    = "/REPLACE_ME/status/humidity";
const char *mqttTopicLedStatus   = "/REPLACE_ME/status/led";
const char *mqttTopicLedSet      = "/REPLACE_ME/set/led";
*/
const int PUBLISH_SENSOR_DATA_DELAY = 10000; //milliseconds

// Variables ...
long sensorDelay;
long lastDhtRead;

float temperature = -1234;
float humidity    = -1;
boolean ledSwitchedOn = false;

DHT_Unified  dht(DHT_PIN, DHTTYPE);
WiFiClient   espClient;
PubSubClient mqttClient(espClient);

void setup() {
  Serial.begin(9600);

  // setup io-pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(DHT_PIN, INPUT);
  ledOff();

  setupDhtSensor();
  setupWifi();
  setupMqtt();
}

void setupDhtSensor() {
 // Initialize device.
  dht.begin();
  
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(" ");
  Serial.println("=================================================");
  Serial.println("Temperature: ------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Min Delay:    "); Serial.print(sensor.min_delay/1000); Serial.println(" ms");
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");
  Serial.println("-------------------------------------");
  dht.humidity().getSensor(&sensor);
  Serial.println("Humidity: ---------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Min Delay:    "); Serial.print(sensor.min_delay/1000); Serial.println(" ms");
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");
  Serial.println("-------------------------------------");
  
  // Set delay between sensor readings based on sensor details.
  sensorDelay = max(sensor.min_delay / 1000, PUBLISH_SENSOR_DATA_DELAY);
  lastDhtRead = millis();
  Serial.print("Checking DHT Sensor every ");Serial.print(sensorDelay); Serial.println("ms");
  Serial.println("=================================================");

}

void setupWifi() {
  delay(10);
  Serial.println();Serial.print("Connecting to ");Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");Serial.println(WiFi.localIP());
}

void setupMqtt() {
  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(mqttCallback);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  Serial.print("Message arrived [");Serial.print(topicStr);Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (topicStr == mqttTopicLedSet) {
    Serial.println("Got LED switching command ...");
    // Switch on the LED if an 1 was received as first character
    if ((char)payload[0] == '1') {
      ledOn();
    } else {
      ledOff();
    }
    mqttPublishLedState();
  }
}

void loop() {
  checkWifi();
  checkMqtt();
  mqttClient.loop();

  if(lastDhtRead + sensorDelay <= millis()) {
    getTemperatureAndHumidity();
    Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" *C"); 
    Serial.print("Humidity: "); Serial.print(humidity); Serial.println("%");
    mqttPublishTemperatureAndHumidity();
  }
}

void checkWifi() {
  if (WiFi.status() != WL_CONNECTED) {
    setupWifi();  
  }
}

void checkMqtt() {
  if (!mqttClient.connected()) {
    while (!mqttClient.connected()) {
      Serial.print("Attempting to open MQTT connection...");
      if (mqttClient.connect("ESP8266_Client")) {
        Serial.println("connected");
        mqttClient.subscribe(mqttTopicLedSet);
        flashLed();
      } else {
        Serial.print("MQTT connection failed, retry count: ");
        Serial.print(mqttClient.state());
        Serial.println(" try again in 5 seconds");
        delay(5000);
      }
    }
  }  
}

void mqttPublishTemperatureAndHumidity() {
  Serial.print("Publishing to server: ");Serial.println(mqttServer);
  
  Serial.print("Publishing ");Serial.print(temperature);Serial.print(" to topic ");Serial.println(mqttTopicTemperature);
  String temperatureStr = String(temperature); //converting temperature (the float variable above) to a string 
  char charBufTemperature[temperatureStr.length()];
  temperatureStr.toCharArray(charBufTemperature, temperatureStr.length()); //packaging up the data to publish to mqtt ...
  mqttClient.publish(mqttTopicTemperature, charBufTemperature);

  Serial.print("Publishing ");Serial.print(humidity);Serial.print(" to topic ");Serial.println(mqttTopicHumidity);
  String humidityStr = String(humidity); //converting humidity (the float variable above) to a string 
  char charBufHumidity[humidityStr.length()];
  humidityStr.toCharArray(charBufHumidity, humidityStr.length()); //packaging up the data to publish to mqtt ...
  mqttClient.publish(mqttTopicHumidity, charBufHumidity);
}

void mqttPublishLedState() {
  Serial.print("Publishing to server: ");Serial.println(mqttServer);
  
  String ledStateStr = ledSwitchedOn ? "1" : "0"; 
  Serial.print("Publishing ");Serial.print(ledStateStr);Serial.print(" to topic ");Serial.println(mqttTopicLedStatus);
  char charBufLed[ledStateStr.length()];
  ledStateStr.toCharArray(charBufLed, ledStateStr.length());
  mqttClient.publish(mqttTopicLedStatus, charBufLed); 
}

// =================================================================================================================================
// Helper methods 
// =================================================================================================================================

void ledOn() {
  ledSwitchedOn = true;
  digitalWrite(LED_PIN, HIGH);
}

void ledOff() {
  ledSwitchedOn = false;
  digitalWrite(LED_PIN, LOW);
}

void flashLed() {
  for (int i=0; i < 5; i++){
      ledOn();
      delay(100);
      ledOff();
      delay(100);
   }  
   ledOff();
}

void getTemperatureAndHumidity() {
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    temperature = -9999;
  } else {
    temperature = event.temperature;
  }
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    humidity = -1;
  } else {
    humidity = event.relative_humidity;
  }
  lastDhtRead = millis();
}
  
