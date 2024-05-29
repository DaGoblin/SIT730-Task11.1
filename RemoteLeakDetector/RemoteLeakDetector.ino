#include <ArduinoMqttClient.h>
#include "arduino_secrets.h"
#include <WiFiNINA.h>
#include <Arduino.h>
#include <millisDelay.h>


// Sensor pins
#define sensorPower 2
#define sensorPin A1
#define LED 3
#define connectionLED 4
#define buzzerPin 5

char ssid[] = SECRET_SSID; // your network SSID (name)
char pass[] = SECRET_PASS; // your network password (use for WPA, or use as key for WEP)
char mqttUser[] = SECRET_MQTT_USER; // MQTT username
char mqttPass[] = SECRET_MQTT_PASS; // MQTT password

WiFiClient wifiClient;

//MQTT Connection Details
const char broker[] = "192.168.1.10";
int port = 1883;
MqttClient mqttClient(wifiClient);
const String topicConfig = "homeassistant/binary_sensor/leaksensor1/config";
const String configPayload = R"(
{
   "name":null,
   "device_class":"moisture",
   "state_topic":"homeassistant/binary_sensor/leaksensor1/state",
   "unique_id":"leaksensor1",
   "device":{
      "identifiers":[
         "LS01"
      ],
      "name":"leaksensor"
   }
}
)";

const int buzzerToneFre = 1000; //1Khz for buzzer
const String topicState = "homeassistant/binary_sensor/leaksensor1/state";
const String topicEWSSet = "homeassistant/switch/ews/set";

bool waterDetected = false;
bool wifiFailed = false;
bool wifiConnected = false;
const unsigned long serialTimeout = 2000;
millisDelay sensorReadTimer;
millisDelay heartbeatMQTTTimer;
millisDelay WiFiCheckTimer;
millisDelay MQTTCheckTimer;
millisDelay buzzerTimer;
const unsigned long sensorReadTime = 1000;
const unsigned long heartbeatMQTTTime = 30000;
const unsigned long WiFiCheckTime = 60000;
const unsigned long MQTTCheckTime = 60000;
const unsigned long buzzerTime = 500;

// Tone buzzerTone;


void setup() {
  
  //setup Pins
  pinMode(sensorPower, OUTPUT);
  pinMode(LED, OUTPUT); 
	pinMode(connectionLED, OUTPUT); 
  digitalWrite(connectionLED, LOW);
  digitalWrite(LED, LOW);
  digitalWrite(sensorPower, LOW);
  noTone(buzzerPin);


  //setup connectivity
  serialSetup();
  WiFiConnect();
  connectMQTT();
  

  //Start Timers
  sensorReadTimer.start(sensorReadTime);
  heartbeatMQTTTimer.start(heartbeatMQTTTime);
  WiFiCheckTimer.start(WiFiCheckTime);
  MQTTCheckTimer.start(MQTTCheckTime);
  buzzerTimer.start(buzzerTime);
	
}

void checkWiFi()
{
  
  if(WiFiCheckTimer.justFinished())
  {
    Serial.println("Running WiFi Check");
    Serial.println(WiFi.status());

    int statusWiFi = WiFi.status();
    if(statusWiFi==WL_CONNECTION_LOST || statusWiFi==WL_DISCONNECTED || statusWiFi==WL_CONNECT_FAILED || statusWiFi==WL_IDLE_STATUS) //if no connection
      {
        WiFiConnect();
      }
    WiFiCheckTimer.repeat();  
  }

  if(WiFi.status()==WL_CONNECTED && !wifiConnected)
    {
      Serial.println("You're connected to the network");
      digitalWrite(connectionLED, HIGH);
      wifiConnected = true;
    }

  if(WiFi.status()== WL_CONNECT_FAILED && !wifiFailed)
  {
      Serial.println("Unable to connect to WiFi");
      Serial.println();
      digitalWrite(connectionLED, LOW);
      wifiFailed = true;
  }
}

void WiFiConnect()
{
  digitalWrite(9, LOW); //LED OFF to show disconnected
  wifiFailed = false;
  wifiConnected = false;

  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);
  WiFi.begin(ssid,pass);
  checkWiFi();
}

void serialSetup()
{
  Serial.begin(9600);
  unsigned long start = millis();
  while (!Serial) {
    if (millis() - start > serialTimeout)         
      break;
  }
}

void checkMQTT()
{
  if(MQTTCheckTimer.justFinished())
  {
    if (!mqttClient.connected())
    {
      digitalWrite(9, LOW); 
      connectMQTT();
    } 
    MQTTCheckTimer.repeat();
  }
}


void connectMQTT()
{
    if(WiFi.status()!=WL_CONNECTED) return;
    Serial.print("Attempting to connect to the MQTT broker: ");
    Serial.println(broker);
    mqttClient.setUsernamePassword(mqttUser, mqttPass);
      
    if (!mqttClient.connect(broker, port))
    {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());
        digitalWrite(connectionLED, LOW);
    }
    if (mqttClient.connected()) 
    {
      Serial.println("You're connected to the MQTT broker!");
      Serial.println();
      digitalWrite(connectionLED, HIGH);
      MQTTSend(topicConfig, configPayload); //send Discovery to Home Assistant
    }
    // mqttClient.onMessage(onMqttMessage);
}

void MQTTSend(String topic, String messageMQTT)
{
    mqttClient.beginMessage(topic);
    mqttClient.print(messageMQTT);
    mqttClient.endMessage();
    Serial.println("Message sent to: " + topic);
    Serial.println("Payload: " + messageMQTT);
}

void loop() {
	//get the reading from the function below and print it
	int level = readSensor();
	
  if(level != -1) processSensor(level);
  checkWiFi();
  checkMQTT();
  heartbeatMQTT();
}

void heartbeatMQTT()
{
  if(heartbeatMQTTTimer.justFinished())
  {
    MQTTSend(topicState, waterDetected ? "ON" : "OFF");
    heartbeatMQTTTimer.repeat();
  }
}


void processSensor(int level)
{
  bool currentvalue = waterDetected;
  Serial.print("Water level: ");
	Serial.println(level);

  if (level > 70) {
    waterDetected = true;
    digitalWrite(LED,HIGH);

    if(buzzerTimer.justFinished())
    {
      tone(buzzerPin,buzzerToneFre,250);
      buzzerTimer.repeat();
    }

  }
  else
  {
    waterDetected = false;
    digitalWrite(LED,LOW);
    noTone(buzzerPin);
  }

  if (currentvalue != waterDetected)
  {
    MQTTSend(topicState, waterDetected ? "ON" : "OFF");
    if(waterDetected) MQTTSend(topicEWSSet, "OFF");
  }
}


//This is a function used to get the reading
int readSensor() {
  int val = 0;
  if(sensorReadTimer.justFinished())
  {
    digitalWrite(sensorPower, HIGH);	// Turn the sensor ON
    delay(10);							// wait 10 milliseconds
    val = analogRead(sensorPin);		// Read the analog value form sensor
    digitalWrite(sensorPower, LOW);		// Turn the sensor OFF
    sensorReadTimer.repeat();
  }
  else
  {
    val = -1;
  }
	return val;							// send current reading
}