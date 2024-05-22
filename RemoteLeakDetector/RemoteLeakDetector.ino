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

int statusWiFi = WL_IDLE_STATUS;  

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
const unsigned long serialTimeout = 1000;
const unsigned long WiFiTimeout = 5000;
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
	// setupWiFi();

  
  // Set D7 as an OUTPUT
	
  
  //setup Pins
  pinMode(sensorPower, OUTPUT);
  pinMode(LED, OUTPUT); 
	pinMode(connectionLED, OUTPUT); 
  digitalWrite(connectionLED, LOW);
  digitalWrite(LED, LOW);
  digitalWrite(sensorPower, LOW);
  // buzzerTone.begin(buzzerPin);
  noTone(buzzerPin);
  //setup connectivity
  serialSetup();
  WiFiConnect();
  connectMQTT();
  MQTTSend(topicConfig, configPayload); //send Discovery to Home Assistant
	
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
    statusWiFi = WiFi.status();
    if(statusWiFi==WL_CONNECTION_LOST || statusWiFi==WL_DISCONNECTED) //if no connection
      {
        digitalWrite(9, LOW); //LED OFF to show disconnected
        WiFiConnect(); //call connect to wifi
      }
    WiFiCheckTimer.repeat();  
  }

}


void WiFiConnect()
{
  unsigned long start = millis();
  statusWiFi = WL_IDLE_STATUS;
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);
  while(statusWiFi!=WL_CONNECTED)
  {
    if (millis() - start > WiFiTimeout)         
    break;
    statusWiFi = WiFi.begin(ssid,pass);
    if(statusWiFi==WL_CONNECTED) 
    {
      Serial.println("You're connected to the network");
      digitalWrite(connectionLED, HIGH);
    }
    else delay(500);
  }

  if(WiFi.status()== WL_CONNECT_FAILED)
  {
      Serial.println("Unable to connect to WiFi");
      Serial.println();
      digitalWrite(connectionLED, LOW);
  }
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
  if(statusWiFi!=WL_CONNECTED) return;
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
      // buzzerTone.play(buzzerToneFre,250);
      buzzerTimer.repeat();
    }   
  }
  else
  {
    waterDetected = false;
    digitalWrite(LED,LOW);
    // buzzerTone.stop();
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