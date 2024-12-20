/*
FW for "Manges Mojäng" ESP32 Decor String Light Controller (esp_dslc)

Variant: MQTT
Requires: Home Assistant, MQTT Broker (Mosquitto), Automation in Home Assistant that triggers on MQTT Received message, Static IP (preferably)
Version: 0.1

***Concept***
Wakes up each 10 min and connects to MQTT Broker to receive on/off decision

***At cold start/first boot***
At cold start, output is default ON and Button toggles between 15/30mA output.
Goes to sleep after 30s of button inactivity. 
Turns off output if no connection to wifi/mqtt occured, or received "off" state from mqtt.

***At wake up***
Connects to WiFi, then MQTT broker, take actions (on/off), then go to Sleep

MQTT Publish topic "esp_dslc_XXXX/online" where XXXX is two last bytes in MAC adress (use to trigger automation in HA)
MQTT Subscribe topic "esp_dslc/output"


NOTE: 
DEBUG is defined per default in this version to be able to retrieve MAC adress and see that everything works.
Make sure "USB CDC On Boot" is Enabled under Tools menu
and look in Serial Monitor (Ctrl+Shift+M) for MAC adress etc.
When done, disable "USB CDC On Boot" and undefine #DEBUG (put // in front of it...)


In Home Assistant:
1. Install Mosquitto MQTT in Home Assistant: https://www.youtube.com/watch?v=VbHgn8-vFpc
2. By adding the following in configuration.yaml you will get an binary sensor entity that indicates availability and status:

   mqtt:
      binary_sensor:
       - name: "esp_dslc_XXXX"
         unique_id: "esp_dslc_XXXX"
         state_topic: "esp_dslc/output"
         payload_on: 'on'
         payload_off: 'off'      
         availability:
           - topic: "esp_dslc_XXXX/status"
         expire_after: 960
         
   where XXXX is the last two bytes of MAC address

3. Example Automation:
   Trigger: Received MQTT message in topic "esp_dslc_XXXX/status" with payload "online"
   Condition: e.g. "a given time", "sun is below horizon", "Bedroom Light is On" etc.
   Action: Publish MQTT message in topic "esp_dslc/output" preferrably with "Retained" flag set to true with payload 'on'.
   
   Create an addition automation for payload 'off' at some given conditions. Or simply combine into one with conditional actions.
   
*/


#include <WiFi.h>
#include <PubSubClient.h>  //PubSubClient by Nick O'Leary
#include <EEPROM.h>
#include "driver/gpio.h"

//##################### SETTINGS ######################

// WiFi credentials
const char *ssid = "YOUR_WIFI_SSID";
const char *password = "YOUR_WIFI_PWD";

// Set your Static IP address (shorter time awake compared to DHCP)
IPAddress local_IP(192, 168, 1, XX); //Static IP of this unique device
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// MQTT Broker adress and credentials
const char *mqtt_broker = "homeassistant.local";
const char *mqtt_username = "username";
const char *mqtt_password = "userpwd";
const int mqtt_port = 1883;
const String output_topic = "esp_dslc/output";

//##################### DEBUG ######################
// Make sure "USB CDC On Boot" is Enabled if DEBUG is defined.
// DEBUG mode will not work with power banks

#define DEBUG   

//###################################################

const unsigned long sleeptime_us = 10*60*1000*1000; //(10 min)
const unsigned long button_exit_time_ms = 30000;    //Timeout after last button press at cold start
const unsigned long max_runtime_ms = 5000;          //If no message from MQTT within 5s, go to DeepSleep after 5s

// Pins
#define BUCK_EN   3 //RTC gpio (retained during deep sleep)
#define HIGH_CUR  5 //RTC gpio (retained during deep sleep)
#define LED       8 //Regular gpio
#define BUTTON    9 //Regular gpio

#ifndef DEBUG
#define PBWAKE   19 //USB D+ wake up Powerbank and sense charging state (Not used if DEBUG defined)
#endif

// EEPROM
#define EEPROMsize 1

// RTC Fast Memory Variables (Retained during Deep Sleep)
RTC_DATA_ATTR bool cold_start = true; //Initial value is true to detect cold start.
//RTC_DATA_ATTR int wake_count = -1;

//Variables
bool goToDeepSleep = false;
bool currentSetModeActive = false;
bool blinkLed = false;
bool receivedMsg = false;
unsigned char ledblink[8] = {0,1,1,1,1,1,1,1};
unsigned char ledptr = 0;
unsigned char currentMode = 0;
unsigned int timeout = 0;
unsigned int newButtonState = HIGH;
unsigned long startMillis = 0;
unsigned long currentMillis = 0;
unsigned long timeMillis = 0;
unsigned long buttonReleaseMillis = 0;
unsigned long led_blink_ms = 250; //Fast blink


WiFiClient espClient;
PubSubClient client(espClient);

void setup() {

  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  pinMode(BUCK_EN, OUTPUT);
  digitalWrite(BUCK_EN, LOW);
  pinMode(HIGH_CUR, OUTPUT);
  digitalWrite(HIGH_CUR, LOW);
  
  digitalWrite(LED, LOW); //Indicate alive...

#ifndef DEBUG
  //This is done to wake Powerbanks and control if the charging works (when using pbalive-dongle)
  pinMode(PBWAKE, INPUT_PULLUP);
  int chargingDetect = digitalRead(PBWAKE);
  pinMode(PBWAKE, INPUT_PULLDOWN);
  // if(chargingDetect == 1) {
  //   //No charging detected
  //   ledblink[0] = 0;
  //   ledblink[1] = 1;
  //   ledblink[2] = 0;
  //   ledblink[3] = 1;
  //   ledblink[4] = 0;
  //   ledblink[5] = 1;
  //   ledblink[6] = 0;
  //   ledblink[7] = 1;
  //   led_blink_ms = 100; //Even faster blink  
  //   blinkLed = true;
  //   max_runtime_ms = 20000;        
  // }
#endif


  //Things done only at cold start, e.g. read previous currentMode
  if(cold_start) {
    EEPROM.begin(EEPROMsize);
    currentMode = EEPROM.read(0);
    //If not a valid currentMode, or never been set
    if(currentMode > 1) {
      currentMode=0;
      EEPROM.write(0, currentMode);
      EEPROM.commit();
    }
    setCurrent(currentMode);
  }

  //All other things done every WakeUp 
#ifdef DEBUG
  // Set software serial baud to 115200;
  Serial.begin(115200);
  Serial.println("");
  Serial.println("I'm alive");
#endif
  WiFi.mode(WIFI_STA);
  String newHostname = "esp_dslc_";
  newHostname += String(WiFi.macAddress())[12];
  newHostname += String(WiFi.macAddress())[13];
  newHostname += String(WiFi.macAddress())[15];
  newHostname += String(WiFi.macAddress())[16];
  WiFi.setHostname(newHostname.c_str());
  if (!WiFi.config(local_IP, gateway, subnet)) {
#ifdef DEBUG
    Serial.println("STA Failed to configure");
#endif
  }
  // connecting to a WiFi network
#ifdef DEBUG
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Connecting to WiFi..");
#endif
  WiFi.begin(ssid, password);
  if(!WiFi.getAutoReconnect()) {
    WiFi.setAutoReconnect(true);
  }
  timeout = 20; //Timeout if not connected in 20*0.25s delay=5s
  while ((WiFi.status() != WL_CONNECTED) && (timeout > 0)) {
      delay(250);
      timeout--;
  }
  if(timeout == 0){
#ifdef DEBUG
    Serial.println("Failed connecting to the WiFi network!");
#endif
    goToDeepSleep = true;
  } else {
#ifdef DEBUG
    Serial.println("Connected to the WiFi network");
    Serial.println(WiFi.localIP());
    Serial.println(newHostname.c_str());  
#endif
      //connecting to a mqtt broker
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);
    timeout = 2; ////Timeout if not connected in 2*3s delay=6s
    String client_id = "esp32c3-";
    client_id += "dslc-";
    client_id += String(WiFi.macAddress());
    while (!client.connected() && timeout > 0 && WiFi.status() == WL_CONNECTED) {
#ifdef DEBUG
      Serial.printf("The client %s connects to the mqtt broker", client_id.c_str());
      Serial.println("");
#endif
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
#ifdef DEBUG
        Serial.println("mqtt broker connected");
#endif
      } else {
#ifdef DEBUG
        Serial.print("failed with state ");
        Serial.println(client.state());
#endif
        timeout--;
        delay(3000);
      }
      client.loop();
    }
  }
  
  // publish and subscribe at MQTT Broker if connected
  if (client.state() == 0) {
    client.publish((newHostname + "/status").c_str(), "online");
    client.loop();
    client.subscribe(output_topic.c_str(),1);
    client.loop();
    //delay(100);
  } else {
#ifdef DEBUG
    Serial.println("failed connecting to mqtt server");
#endif
    goToDeepSleep = true;
  }

  startMillis = millis();  //initial start time
  timeMillis = startMillis;
}

//MQTT callback. Runs when received a message from Broker
void callback(char *topic, byte *payload, unsigned int length) {
  payload[length] = '\0'; // Null terminator used to terminate the char array
  String message = (char*)payload;
  String topic_str = topic;
#ifdef DEBUG
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  Serial.println(message);
  Serial.println("-----------------------");
#endif
 
  if ( topic_str == output_topic && message == "on") {
    turnON();
    goToDeepSleep = true;
  }
  if ( topic_str == output_topic && message == "off") {   
    turnOFF();  
    goToDeepSleep = true;
  }
  if (goToDeepSleep) {
    receivedMsg = true;
    client.disconnect();
    WiFi.disconnect();
  }
}

void turnON() {
  digitalWrite(BUCK_EN, HIGH);
}

void turnOFF() {
  digitalWrite(BUCK_EN, LOW);
}

void setCurrent(unsigned char mode) {
  switch (mode) {
    case 0:
      // Low limit
      digitalWrite(HIGH_CUR, LOW);
      ledblink[0] = 0;
      ledblink[1] = 1;
      ledblink[2] = 1;
      ledblink[3] = 1;
      ledblink[4] = 1;
      ledblink[5] = 1;
      ledblink[6] = 1;
      ledblink[7] = 1;      
      break;
    case 1:
      // High limit
      digitalWrite(HIGH_CUR, HIGH);
      ledblink[0] = 0;
      ledblink[1] = 1;
      ledblink[2] = 0;
      ledblink[3] = 1;
      ledblink[4] = 1;
      ledblink[5] = 1;
      ledblink[6] = 1;
      ledblink[7] = 1;
      break;
  }
  gpio_hold_dis((gpio_num_t) HIGH_CUR);
#ifdef DEBUG
  Serial.print("CurrentMode set to: ");
  Serial.println(mode);
#endif
}

void loop() {
  currentMillis = millis();
  client.loop();

  if(cold_start) {
    cold_start = false;
    currentSetModeActive = true;
    blinkLed = true;
    turnON();
  }

  if(currentSetModeActive) {
    //Read Button
    newButtonState = digitalRead(BUTTON);
    //Handle Button press
    if(newButtonState == LOW) {
      if(buttonReleaseMillis > 100) {
        currentMode++;
        if(currentMode > 1) {
          currentMode = 0;
        }
        setCurrent(currentMode);
      }
      buttonReleaseMillis = 0;
    } else {
      if(buttonReleaseMillis == 0) {
        buttonReleaseMillis = currentMillis;
      } else {
        if(currentMillis - buttonReleaseMillis >= button_exit_time_ms) {
          //If no message from MQTT after button_exit_time, make sure to turn off and go to sleep.
          if(!receivedMsg) {
            turnOFF();
          }
          currentSetModeActive = false;
          blinkLed = false;
          EEPROM.write(0, currentMode);
          EEPROM.commit();
          goToDeepSleep = true;
        }
      }
    }
  }

  if (blinkLed) {
    if(currentMillis - timeMillis >= led_blink_ms) {
      timeMillis = currentMillis;
      if(ledptr > 7) {
        ledptr = 0;
      }
      digitalWrite(LED, ledblink[ledptr]);
      ledptr++;
    }
  } else {
    digitalWrite(LED, LOW); //Static lit
  }
  
  if ((goToDeepSleep || (currentMillis - startMillis >= max_runtime_ms)) && !currentSetModeActive && newButtonState) {
    digitalWrite(LED, HIGH); //Turn off LED if left on..
    
    if (receivedMsg) {
      //release gpio before modification
      gpio_hold_dis((gpio_num_t) BUCK_EN);
    }
    
    //preserve gpios during deep sleep
    gpio_hold_en((gpio_num_t) BUCK_EN);
    gpio_hold_en((gpio_num_t) HIGH_CUR);
    gpio_deep_sleep_hold_en();


#ifdef DEBUG
    Serial.println("Enter deep sleep...");
#endif
    
    ESP.deepSleep(sleeptime_us);
  }
}

//Function that runs directly at wake up
void RTC_IRAM_ATTR esp_wake_deep_sleep(void) {
    esp_default_wake_deep_sleep();
    // Add additional functionality here
    // if(wake_count > 0) {
    //   wake_count--;
    // }
}