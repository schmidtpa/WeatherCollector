
/*
** Weather Collector
** Reads data from an BMP280 sensor via i2c and sends the received data to an mqtt broker.
** 
** Author: Patrick Schmidt <patrick@ealp-net.at>
** License: Apache License, Version 2.0
*/

#include <ESP8266WiFi.h> 
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#include "config.h"

Adafruit_BMP280 bmp; // I2C
WiFiClient espWifiClient;
PubSubClient mqttClient(espWifiClient); // MQTT

const char* wlanSSID = WIFISSID;
const char* wlanPassword = WIFIPASSWORD;

const char* mqttServer = MQTTSERVER;
const short mqttPort = MQTTPORT;
const char* mqttClientId = MQTTCLIENTID;
const char* mqttUsername =  MQTTUSERNAME;
const char* mqttPassword = MQTTPASSWORD;

float temperature;
float pressure;
char message[200];

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); // Set wifi mode to client 
  mqttClient.setServer(mqttServer, mqttPort);

  if(!bmp.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
}

void loop() {
  checkServerConnection();
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();
  
  printMeasurement();
  sendMeasurement();
  
  delay(10000);
}

void printMeasurement() {
  Serial.print("Temperature  = ");
  Serial.print(temperature);
  Serial.println(" *C");
  
  Serial.print("Pressure     = ");
  Serial.print(pressure);
  Serial.println(" Pa");
}

void sendMeasurement(){
  snprintf (message, 200, "{\"temperature\":%.8f,\"pressure\":%.8f}", temperature, pressure);
  mqttClient.publish("house/garage/weather", message);
  mqttClient.loop();
}

void checkServerConnection() {
  if(!mqttClient.connected()) {
    if(WiFi.status() != WL_CONNECTED){ 
      connectWifi();
    }
    
    connectMqtt();
  } 
}

void connectMqtt() {
  Serial.print("Connecting to MQTT broker at ");
  Serial.print(mqttServer);
  
  if (mqttClient.connect(mqttClientId, mqttUsername, mqttPassword)) {
    Serial.println(": connected!");
  } else {
    Serial.print(": Error, rc=");
    Serial.println(mqttClient.state());
    delay(500);
  }
}

void connectWifi() {
  WiFi.begin(wlanSSID, wlanPassword);
  
  Serial.print("WiFi connecting to ");
  Serial.print(wlanSSID);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  
  Serial.println(" connected!");
  Serial.print("WiFi IP: ");
  Serial.println(WiFi.localIP());
}

