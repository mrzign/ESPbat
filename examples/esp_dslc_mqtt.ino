/*
FW for "Manges Mojäng" ESP32 Decor String Light Controller (esp_dslc)

Variant: MQTT
Requires: Home Assistant, MQTT Broker (Mosquitto), Automation in Home Assistant that triggers on MQTT Received message
Version: 0.3

***Concept***
Wakes up each 10 min and connects to MQTT Broker to receive on/off decision

***At cold start/first boot***
If no wifi credentials has been stored, the device boots up as an Access Point (esp_dslc_XXXX)
Connect to it and configure wifi and MQTT settings.
Once saved and device successfully connects to wifi, the AP should disappear.

At cold start, if WiFi have been stored, a short press on the button within 5s will force the Access Point
to become available for change in config. A long press(>5s) within 5s from cold start will clear wifi settings
and the Access Point will become available.

If button is not pressed within 5s from cold start, the LED will blink and the output will turn ON and Button 
toggles between 15/30mA output. It goes to sleep after 30s of button inactivity. 

Turns off output if no connection to wifi/mqtt occured, or received "off" state from mqtt.

***At wake up***
Connects to WiFi, then MQTT broker, take actions (on/off), then go to Sleep

MQTT Publish Status topic "esp_dslc_XXXX/online" where XXXX is two last bytes in MAC adress (use to trigger automation in HA)
MQTT Subscribe Output topic "esp_dslc/output"

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


#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>  //PubSubClient by Nick O'Leary
#include "driver/gpio.h"
#include "Preferences.h"

//##################### DEBUG ######################
// Make sure "USB CDC On Boot" is Enabled if DEBUG is defined.
// DEBUG mode will not work with power banks

//#define DEBUG   

//###################################################

const uint8_t sleeptime_m = 10; //(10 min)
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

#define uS_TO_S_FACTOR 1000000ULL

// RTC Fast Memory Variables (Retained during Deep Sleep)
RTC_DATA_ATTR char mqtt_broker[40] = "homeassistant.local";
RTC_DATA_ATTR char mqtt_username[20] = "username";
RTC_DATA_ATTR char mqtt_password[20] = "userpwd";
RTC_DATA_ATTR uint16_t mqtt_port = 1883;
RTC_DATA_ATTR char mqtt_topic[40] = "esp_dslc/output";
RTC_DATA_ATTR char mqtt_status_topic[40] = "esp_dslc_XXXX/status";


//Variables
bool cold_start = false;
bool goToDeepSleep = false;
bool currentSetModeActive = false;
bool blinkLed = false;
bool receivedMsg = false;
bool force_AP_mode = false;
bool active_AP_mode = false;
bool wait_for_config = false;
uint8_t boot_up_reason = 0;
uint8_t ledblink[8] = {0,1,1,1,1,1,1,1};
uint8_t ledptr = 0;
uint8_t currentMode = 0;
uint16_t timeout = 0;
uint16_t newButtonState = HIGH;
uint32_t startMillis = 0;
uint32_t currentMillis = 0;
uint32_t timeMillis = 0;
uint32_t buttonReleaseMillis = 0;
uint32_t led_blink_ms = 250; //Fast blink
String newHostname; 
WiFiManager wm; // global wm instance
WiFiManagerParameter custom_mqtt_broker;
WiFiManagerParameter custom_mqtt_username;
WiFiManagerParameter custom_mqtt_password;
WiFiManagerParameter custom_mqtt_topic;
WiFiManagerParameter custom_mqtt_port;
WiFiManagerParameter custom_mqtt_status_topic;
WiFiManagerParameter custom_mac;
char wm_mac[40];
Preferences nvmPrefs;

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

  boot_up_reason = esp_rom_get_reset_reason(0);

#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("");
#endif

#ifdef DEBUG
  Serial.print("Boot up reason: ");
  Serial.print(boot_up_reason + " - ");
  if(boot_up_reason == 1) {
    Serial.println("Cold Start");
  } else if (boot_up_reason == 5) {
    Serial.println("Deep Sleep");
  } else if (boot_up_reason == 12) {
    Serial.println("Reset");
  } else {
    Serial.println("Unknown");
  }
#endif

  WiFi.mode(WIFI_STA);
  newHostname = "esp_dslc_";
  newHostname += String(WiFi.macAddress())[12];
  newHostname += String(WiFi.macAddress())[13];
  newHostname += String(WiFi.macAddress())[15];
  newHostname += String(WiFi.macAddress())[16];
  WiFi.setHostname(newHostname.c_str());

  if(boot_up_reason == 1) {
    cold_start = true;
#ifdef DEBUG
    Serial.println("Cold Boot detected. Waiting for button press...");
#endif
    bool done = false;
    uint16_t runtime = 400; //10s
    uint16_t buttonpress = 0;
    while (runtime > 0 && !done) {
      if(digitalRead(BUTTON) == LOW) {
        delay(25); //Debounce
        if(digitalRead(BUTTON) == LOW) {
          buttonpress++;
        }
      } else {
        delay(25); //Debounce
        if(digitalRead(BUTTON) == HIGH && buttonpress > 2) {
          done = true;  //button released after beeing pressed > 50ms
        }
      }
      runtime--;
      if(runtime < 200 && buttonpress < 2) done = true; //If not pressed within 5s
      if(buttonpress > 200) done = true; //Pressed for more than 5s
    }
    if(buttonpress > 200) {
      //Button Long pressed for 5s within 5s from cold boot = clear all settings
#ifdef DEBUG
      Serial.println("Long press");
      Serial.println("Erasing Config, restarting");
#endif
      wm.resetSettings();
      // nvmPrefs.begin("thePrefs", false);
      // nvm.clear();
      // nvmPrefs.end();
      for(int i = 0; i < 15; i++) {
        digitalWrite(LED, !digitalRead(LED));
        delay(200);
      }
      ESP.restart();
    } else if (buttonpress > 2 && wm.getWiFiIsSaved()) {
      //Button short pressed within 5s from cold boot = flag for start AP for Config
#ifdef DEBUG
      Serial.println("Short press");
      Serial.println("Starting config portal after connection");
#endif
      force_AP_mode = true;
    }
  }

  //Things done only at cold start or reset, e.g. read previous currentMode
  if(boot_up_reason == 1 || boot_up_reason == 12) {
    nvmPrefs.begin("thePrefs", false);
    if(nvmPrefs.isKey("currentMode")) {
      currentMode = nvmPrefs.getUChar("currentMode");
    } else {
      nvmPrefs.putUChar("currentMode", 0);
    }
    if(nvmPrefs.isKey("mqtt_broker")) {
      strcpy(mqtt_broker, nvmPrefs.getString("mqtt_broker", mqtt_broker).c_str());
    } else {
      nvmPrefs.putString("mqtt_broker", mqtt_broker);
    }
    if(nvmPrefs.isKey("mqtt_username")) {
      strcpy(mqtt_username, nvmPrefs.getString("mqtt_username", mqtt_username).c_str());
    } else {
      nvmPrefs.putString("mqtt_username", mqtt_username);
    }
    if(nvmPrefs.isKey("mqtt_password")) {
      strcpy(mqtt_password, nvmPrefs.getString("mqtt_password", mqtt_password).c_str());
    } else {
      nvmPrefs.putString("mqtt_password", mqtt_password);
    }
    if(nvmPrefs.isKey("mqtt_topic")) {
      strcpy(mqtt_topic, nvmPrefs.getString("mqtt_topic", mqtt_topic).c_str());
    } else {
      nvmPrefs.putString("mqtt_topic", mqtt_topic);
    }
    if(nvmPrefs.isKey("mqtt_status_topic")) {
      strcpy(mqtt_status_topic, nvmPrefs.getString("mqtt_status_topic", mqtt_status_topic).c_str());
    } else {
      nvmPrefs.putString("mqtt_status_topic", (newHostname + "/status"));
      strcpy(mqtt_status_topic, (newHostname + "/status").c_str());
    }
    if(nvmPrefs.isKey("mqtt_port")) {
      mqtt_port = nvmPrefs.getUShort("mqtt_port", mqtt_port);
    } else {
      nvmPrefs.putUShort("mqtt_port", mqtt_port);
    }
    if(currentMode > 1) {
      currentMode=0;
      nvmPrefs.putUChar("currentMode", currentMode);
    }
    nvmPrefs.end();
    setCurrent(currentMode);
  }

  wm.setConfigPortalBlocking(false);

  new (&custom_mqtt_broker) WiFiManagerParameter("broker", "MQTT Broker", mqtt_broker, 40);
  new (&custom_mqtt_port) WiFiManagerParameter("port", "MQTT Port", String(mqtt_port).c_str(), 6);
  new (&custom_mqtt_username) WiFiManagerParameter("username", "MQTT Username", mqtt_username, 20);
  new (&custom_mqtt_password) WiFiManagerParameter("password", "MQTT Password", mqtt_password, 20);
  new (&custom_mqtt_topic) WiFiManagerParameter("topic", "MQTT Output Topic", mqtt_topic, 40);
  new (&custom_mqtt_status_topic) WiFiManagerParameter("status_topic", "MQTT Status Topic", mqtt_status_topic, 40);

  wm.addParameter(&custom_mqtt_broker);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_username);
  wm.addParameter(&custom_mqtt_password);
  wm.addParameter(&custom_mqtt_topic);
  wm.addParameter(&custom_mqtt_status_topic);
  sprintf(wm_mac, "<hr><br>MAC: %s</br><br>", WiFi.macAddress().c_str());
  new (&custom_mac) WiFiManagerParameter(wm_mac);
  wm.addParameter(&custom_mac);


  wm.setSaveParamsCallback(saveParamCallback);
  wm.setSaveConfigCallback(saveConfigCallback);
  wm.setConfigPortalTimeoutCallback(configPortalTimeoutCallback);

  std::vector<const char *> menu = {"wifi","sep","restart","exit"};
  wm.setMenu(menu);

  wm.setClass("invert");

  wm.setConnectTimeout(5); // how long to try to connect for before continuing
  wm.setSaveConnectTimeout(5);
  wm.setConfigPortalTimeout(120); // auto close configportal after n seconds
  wm.setAPClientCheck(true); // avoid timeout if client connected to softap

  //If WiFi credentials saved, skip AP if WiFi connection failed
  if(wm.getWiFiIsSaved() && !force_AP_mode) {
    wm.setEnableConfigPortal(false);
#ifdef DEBUG
    Serial.println("WiFi Stored, disable auto-AP");
#endif
  } else {
    wm.setEnableConfigPortal(true);
  }

#ifdef DEBUG
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());
#endif
  wm.setTitle("ESP Decor String Light Controller");
  bool res;
  res = wm.autoConnect(newHostname.c_str());

  if(!res) {
    if(wm.getConfigPortalActive() == 1) {
      active_AP_mode = true;
    } else {
#ifdef DEBUG
      Serial.println("Failed connecting to WiFi!");
#endif
    }
  } else {
#ifdef DEBUG
    Serial.println("Connected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Hostname: ");
    Serial.println(newHostname.c_str());  
#endif
    if(!force_AP_mode) {    
      client.setServer(mqtt_broker, mqtt_port);
      client.setCallback(callback);
      String client_id = "esp32c3-";
      client_id += "dslc-";
      client_id += String(WiFi.macAddress());
      timeout = 2; ////Timeout if not connected in 2*3s delay=6s
      while (!client.connected() && timeout > 0) {
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
      // publish and subscribe at MQTT Broker if connected
      if (client.state() == 0) {
        client.publish(mqtt_status_topic, "online");
        client.loop();
        client.subscribe(mqtt_topic,1);
        client.loop();
        //delay(100);
      } else {
  #ifdef DEBUG
        Serial.println("failed connecting to mqtt server");
  #endif
        goToDeepSleep = true;
      }
    }
  }


  // Get some uptime timestamps initialized
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
 
  if ( topic_str == mqtt_topic && message == "on") {
    turnON();
    goToDeepSleep = true;
  }
  if ( topic_str == mqtt_topic && message == "off") {   
    turnOFF();  
    goToDeepSleep = true;
  }
  if (goToDeepSleep) {
    receivedMsg = true;
    client.disconnect();
    wm.disconnect();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
#ifdef DEBUG
    Serial.println("Disconnected from WiFi");
#endif
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

String getParam(String name){
  //read parameter from server, for customhmtl input
  String value;
  if(wm.server->hasArg(name)) {
    value = wm.server->arg(name);
  }
  return value;
}

void saveParamCallback(){
  strcpy(mqtt_broker, custom_mqtt_broker.getValue());
  mqtt_port = atoi(custom_mqtt_port.getValue());
  strcpy(mqtt_username, custom_mqtt_username.getValue());
  strcpy(mqtt_password, custom_mqtt_password.getValue());
  strcpy(mqtt_topic, custom_mqtt_topic.getValue());
  strcpy(mqtt_status_topic, custom_mqtt_status_topic.getValue());
#ifdef DEBUG
  Serial.println("[CALLBACK] saveParamCallback fired");
  Serial.println("PARAM MQTT Broker = " + String(mqtt_broker));
  Serial.println("PARAM MQTT Port = " + String(mqtt_port));
  Serial.println("PARAM MQTT Username = " + String(mqtt_username));
  Serial.println("PARAM MQTT Password = " + String(mqtt_password));
  Serial.println("PARAM MQTT Output Topic = " + String(mqtt_topic));
  Serial.println("PARAM MQTT Status Topic = " + String(mqtt_status_topic));
#endif

  nvmPrefs.begin("thePrefs", false);
  nvmPrefs.putString("mqtt_broker", mqtt_broker);
  nvmPrefs.putUShort("mqtt_port", mqtt_port);
  nvmPrefs.putString("mqtt_username", mqtt_username);
  nvmPrefs.putString("mqtt_password", mqtt_password);
  nvmPrefs.putString("mqtt_topic", mqtt_topic);
  nvmPrefs.putString("mqtt_status_topic", mqtt_status_topic);
  nvmPrefs.end();
  if(wait_for_config) {
    wait_for_config = false;
    wm.setAPClientCheck(false);
    wm.setConfigPortalTimeout(1);
  }
}

void saveConfigCallback() {
#ifdef DEBUG
  Serial.println("[CALLBACK] saveConfigCallback fired");
#endif
}

void configPortalTimeoutCallback() {
#ifdef DEBUG
  Serial.println("[CALLBACK] configPortalTimeoutCallback fired");
  delay(100);
#endif
  ESP.restart();
}

void loop() {
  currentMillis = millis();
  client.loop();

  if(cold_start) {
    cold_start = false;
    currentSetModeActive = true;
    blinkLed = true;
    turnON();
#ifdef DEBUG
    Serial.println("CurrentSetMode Active for 30s...");
#endif
  }

  if(!wait_for_config && active_AP_mode && WiFi.status() == WL_CONNECTED) {
    active_AP_mode = false;
#ifdef DEBUG
    Serial.println("WE HAVE WIFI!");
    delay(1000);
#endif
    ESP.restart();  //Restart to get a clean start
  }

  wm.process();

  if(force_AP_mode) {
    force_AP_mode = false;
    wait_for_config = true;
    if(wm.getConfigPortalActive() == 0) {
#ifdef DEBUG
      Serial.println("Starting Config Portal");
#endif
      wm.startConfigPortal(newHostname.c_str());
      active_AP_mode = true;
      goToDeepSleep = false;
    } else {
#ifdef DEBUG
      Serial.println("Config Portal allready running");
#endif
    }
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
          nvmPrefs.begin("thePrefs", false);
          nvmPrefs.putUChar("currentMode", currentMode);
          nvmPrefs.end();
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
  
  if ((goToDeepSleep || (currentMillis - startMillis >= max_runtime_ms)) && !currentSetModeActive && newButtonState && !active_AP_mode) { 
    
    if (receivedMsg) {
      //release gpio before modification
      gpio_hold_dis((gpio_num_t) BUCK_EN);
    }
    digitalWrite(LED, HIGH); //Turn off LED if left on..
    
    //preserve gpios during deep sleep
    gpio_hold_en((gpio_num_t) BUCK_EN);
    gpio_hold_en((gpio_num_t) HIGH_CUR);
    gpio_deep_sleep_hold_en();

    uint64_t sleeptime_us = sleeptime_m*60*uS_TO_S_FACTOR;
#ifdef DEBUG
    Serial.println("Enter deep sleep...");
    Serial.print("Sleeptime(min): ");
    Serial.println(sleeptime_m);
    delay(1000);
    ESP.deepSleep(sleeptime_us);
#else
    ESP.deepSleep(sleeptime_us);
#endif
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