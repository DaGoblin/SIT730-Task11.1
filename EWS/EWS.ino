#include <Arduino.h>
#include <FlowSensor.h>
#include <millisDelay.h>
#include <ArduinoMqttClient.h>
#include "arduino_secrets.h"
#include <WiFiNINA.h>
#include <Wire.h>
#include "Waveshare_LCD1602_RGB.h"

#define type YFS201
#define BOUNCE_DURATION 250

const int relayPin = 2;
const int buttonPin = 21;
const int ledPin = 5;
const int flowSensorPin = 3;

char ssid[] = SECRET_SSID; // your network SSID (name)
char pass[] = SECRET_PASS; // your network password (use for WPA, or use as key for WEP)
char mqttUser[] = SECRET_MQTT_USER; // MQTT username
char mqttPass[] = SECRET_MQTT_PASS; // MQTT password

Waveshare_LCD1602_RGB lcd(16,2);  //16 characters and 2 lines of show
WiFiClient wifiClient;

//MQTT Connection Details
const char broker[] = "192.168.1.10";
int port = 1883;
MqttClient mqttClient(wifiClient);

const String topicConfig = "homeassistant/switch/ews/config";
const String configPayload = R"(
{
   "name":null,
   "command_topic":"homeassistant/switch/ews/set",
   "state_topic":"homeassistant/switch/ews/state",
   "unique_id":"ews12341",
   "device":{
      "identifiers":[
         "EWS101"
      ],
      "name":"EWS"
   }
}
)";


const String topicState = "homeassistant/switch/ews/state";
const String topicSet = "homeassistant/switch/ews/set";

bool wifiFailed = false;
bool wifiConnected = false;

volatile byte buttonPressed = false;
bool waterON = false;
bool waterPreviousState = false;
unsigned long serialTimeout = 1000;
const unsigned long waterCutoffTime = 2 * 60000; //set to 2mins initially
const unsigned long flowReadTime = 1000; //read the flow reade once a second
const unsigned long flowResetTime = 60000; //reset the flow timer once a minuter this is used to show liters per min.
const unsigned long heartbeatMQTTTime = 30000;
volatile unsigned long buttonBounceTime=0; 
const unsigned long WiFiCheckTime = 60000;
const unsigned long MQTTCheckTime = 60000;

millisDelay flowReadTimer;
millisDelay flowRunTimer;
millisDelay flowResetTimer;
millisDelay heartbeatMQTTTimer;
millisDelay WiFiCheckTimer;
millisDelay MQTTCheckTimer;

FlowSensor Sensor(type, flowSensorPin);


void setup()
{
 
  pinMode(buttonPin, INPUT);         
  pinMode(relayPin, OUTPUT);
  pinMode(ledPin, OUTPUT);    
  digitalWrite(relayPin, LOW);
  digitalWrite(ledPin, LOW);
  
  lcd.init();
  

  serialSetup();
  WiFiConnect();
  connectMQTT();
  mqttClient.subscribe(topicSet);
  Sensor.begin(flowCount);

  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, FALLING);
  Serial.println("Program Start");

  //setup timers
  
  flowReadTimer.start(flowReadTime);
  flowResetTimer.start(flowResetTime);
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
      lcd.clear();
      lcd.setCursor(0,0);
      Serial.println("You're connected to the network");
      lcd.send_string("Wifi Connected");
      wifiConnected = true;
    }

  if(WiFi.status()== WL_CONNECT_FAILED && !wifiFailed)
  {
      lcd.clear();
      lcd.setCursor(0,0);
      Serial.println("Unable to connect to WiFi");
      Serial.println();
      lcd.send_string("No Wifi");
      wifiFailed = true;
  }
}

void WiFiConnect()
{
  wifiFailed = false;
  wifiConnected = false;
  lcd.clear();
  lcd.setCursor(0,0);
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);
  lcd.send_string("Try Wifi");
  WiFi.begin(ssid,pass);
  checkWiFi();
}

void checkMQTT()
{
  if(MQTTCheckTimer.justFinished())
  {
    if (!mqttClient.connected())
    {
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
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.send_string("Try MQTT");
    mqttClient.setUsernamePassword(mqttUser, mqttPass);

    if (!mqttClient.connect(broker, port))
    {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());
        lcd.send_string(("MQTT Err:" + String(mqttClient.connectError())).c_str());
    }

    if (mqttClient.connected()) 
    {
      Serial.println("You're connected to the MQTT broker!");
      Serial.println();
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.send_string("MQTT Connected");
      MQTTSend(topicConfig, configPayload); //send Discovery to Home Assistant 
    }

    mqttClient.onMessage(onMqttMessage);
}

void MQTTSend(String topic, String messageMQTT)
{
    mqttClient.beginMessage(topic);
    mqttClient.print(messageMQTT);
    mqttClient.endMessage();
    Serial.println("Message sent to: " + topic);
} 

void flowCount()
{
	Sensor.count();
}

void loop()
{
  //  if (buttonPressed) {
  //   Serial.println("Button Pressed");
  //   buttonPressed = false;
  //   waterON = !waterON;
  //  }
   mqttClient.poll();
  //  waterToggle();
   flowFunction();
   heartbeatMQTT();
}

void handleButtonPress()
{
  if (millis() - buttonBounceTime > BOUNCE_DURATION)  
  {
    // buttonPressed = true; 
    waterToggle("TOGGLE");
    buttonBounceTime = millis();
  }
}

void waterToggle(String waterState)
{
  if (waterState = "TOGGLE")
  {
    waterON = !waterON;
  }
  else if (waterState = "ON")
  {
    waterON = true;
  }
  else if (waterState = "OFF")
  {
    waterON = false;
  }

  if (waterPreviousState != waterON)
  {
    buttonBounceTime = millis(); //stop the button triggering on strange power scenarios
    if(waterON)
    {
      digitalWrite(relayPin, HIGH);
      digitalWrite(ledPin, HIGH);
    }
    else 
    {
      digitalWrite(relayPin, LOW);
      digitalWrite(ledPin, LOW);
    }
    MQTTSend(topicState, waterON ? "ON" : "OFF");
    waterPreviousState = waterON;
  }
}

void flowFunction()
{

	if (flowReadTimer.justFinished())
	{
		Sensor.read();
    float flowRate = Sensor.getFlowRate_m();
    if (flowRate > 1.0) {
      if (!flowRunTimer.isRunning())
        {
          flowRunTimer.start(waterCutoffTime);
        }
    }
    else
    {
      flowRunTimer.stop();
    }
    // lcd.clear();
    lcd.setCursor(0,0);
    if(waterON)
    {
      Serial.println("Water On, Time untill shutoff:" + String(flowRunTimer.remaining()/1000));
      lcd.send_string(charLCD16("W On Time R:" + String(flowRunTimer.remaining()/1000)).c_str());
      
    }
    else {
      Serial.println("Water Off");
      // lcd.send_string("W Off                              ");
      lcd.send_string(charLCD16("W Off").c_str());
    }
    lcd.setCursor(0,1);
    lcd.send_string(charLCD16("Flow(L/m):" + String(flowRate)).c_str());
    
    Serial.println("Flow rate (L/minute): " + String(flowRate));
    Serial.println("Volume (L): " + String(Sensor.getVolume()));
    Serial.println();

    flowReadTimer.repeat();
	}

  // Reset Volume
	if (flowResetTimer.justFinished())
	{
		Sensor.resetVolume();
		flowResetTimer.repeat();
	}

  if (flowRunTimer.justFinished())
    {
      // waterON = false;
      waterToggle("OFF");
    }

}

String charLCD16(String message)
{
  while (message.length() <16)
  {
    message += " ";
  }

  return message;
}

void onMqttMessage(int messageSize)
{   
    String topic =mqttClient.messageTopic();
    // we received a message, print out the topic and contents
    Serial.print("Received a message with topic '");
    Serial.print(topic);
    Serial.print("', length ");
    Serial.print(messageSize);
    Serial.println(" bytes:");
  // Read the message
    char message[messageSize + 1];
    mqttClient.readBytes(message, messageSize);
    message[messageSize] = '\0'; // Null-terminate the char array


    if (topic == topicSet)
    {
      if(strcmp(message,"ON") == 0)
      {
        Serial.println("Turing Water on");
        // waterON = true;
        waterToggle("ON");
      }
      else if(strcmp(message,"OFF") == 0)
      {
        Serial.println("Turing Water off");
        // waterON = false;
        waterToggle("OFF");
      }
    }

    // use the Stream interface to print the contents

    Serial.println();
    Serial.println();
}

void heartbeatMQTT()
{
  if(heartbeatMQTTTimer.justFinished())
  {
    MQTTSend(topicState, waterON ? "ON" : "OFF");
    heartbeatMQTTTimer.repeat();
  }

}
