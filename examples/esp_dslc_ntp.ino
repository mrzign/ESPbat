/*
FW for "Manges Mojäng" ESP32 Decor String Light Controller (esp_dslc)

Variant: Timestamp
Requires: Wifi
Version: 0.1

***Concept***
Wakes up at a given time and turns on/off

***At cold start/first boot***
At cold start, output is default ON and Button toggles between 15/30mA output.
Goes to sleep after 30s of button inactivity. 
Turns off output if no connection to wifi or if not time to be lit

***At wake up***
Connects to WiFi, calibrates local time against NTP server, take actions (on/off), then go to Sleep

*/

//##################### SETTINGS ######################

// WiFi credentials
const char *ssid = "YOUR_WIFI_SSID";
const char *password = "YOUR_WIFI_PASSWORD";

const String timeOn =   "17:00";    //Time when to turn on the Light String. Format HH:MM
const String timeOff =  "23:00";    //Time when to turn off the Light String. Format HH:MM

//######################################################

//##################### DEBUG ######################
// Make sure "USB CDC On Boot" is Enabled if DEBUG is defined.
// DEBUG mode will not work with power banks

//#define DEBUG   

//###################################################

#include <WiFi.h>
#include <EEPROM.h>
#include "driver/gpio.h"
#include "time.h"

const char *ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

const unsigned long button_exit_time_ms = 30000;    //Timeout after last button press at cold start

// Pins
#define BUCK_EN   3 //RTC gpio (retained during deep sleep)
#define HIGH_CUR  5 //RTC gpio (retained during deep sleep)
#define LED       8 //Regular gpio
#define BUTTON    9 //Regular gpio

#ifndef DEBUG
#define PBWAKE   19 //USB D+ wake up Powerbank and sense charging state (Not used if DEBUG defined)
#endif

#define OFF 0
#define ON  1

// EEPROM
#define EEPROMsize 1

// RTC Fast Memory Variables (Retained during Deep Sleep)
RTC_DATA_ATTR bool cold_start = true; //Initial value is true to detect cold start.
//RTC_DATA_ATTR int wake_count = -1;


//Variables
bool goToDeepSleep = false;
bool currentSetModeActive = false;
bool blinkLed = false;
bool receivedTime = false;
unsigned char outputState = OFF;
unsigned char ledblink[8] = {0,1,1,1,1,1,1,1};
unsigned char ledptr = 0;
unsigned char currentMode = 0;
unsigned int timeout = 0;
unsigned int newButtonState = HIGH;
unsigned int sleeptime_m = 10; //(10 min) default (if failed wifi or failed NTP)
unsigned long startMillis = 0;
unsigned long currentMillis = 0;
unsigned long timeMillis = 0;
unsigned long buttonReleaseMillis = 0;
unsigned long led_blink_ms = 250;

struct tm timeinfo;


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

#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("");
  Serial.println("I'm alive");
  delay(1000);
#endif
  WiFi.mode(WIFI_STA);
  String newHostname = "esp_dslc_";
  newHostname += String(WiFi.macAddress())[12];
  newHostname += String(WiFi.macAddress())[13];
  newHostname += String(WiFi.macAddress())[15];
  newHostname += String(WiFi.macAddress())[16];
  WiFi.setHostname(newHostname.c_str());
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
    Serial.println("Failed connecting to WiFi!");
#endif
  } else {
#ifdef DEBUG
    Serial.println("Connected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Hostname: ");
    Serial.println(newHostname.c_str());  
#endif
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    if(!getLocalTime(&timeinfo)){
#ifdef DEBUG
      Serial.println("Failed to obtain time!");
#endif
    } else {
      receivedTime = true;
      sleeptime_m = eval_time();
    }
  }
  //We are done with Wifi
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
#ifdef DEBUG
  Serial.println("Disconnected from WiFi");
#endif
  goToDeepSleep = true;
  // Init and get the time
  startMillis = millis();  //initial start time
  timeMillis = startMillis;
}

//Function to see if we are within time slot
unsigned int eval_time(void) {
  unsigned int sleep_m = 0;
  unsigned int timeOn_m = 0;
  unsigned int timeOff_m = 0;
  unsigned int timeNow_m = 0;
  
  timeOn_m = timeOn.substring(3,5).toInt() + (timeOn.substring(0,2).toInt())*60;
  timeOff_m = timeOff.substring(3,5).toInt() + (timeOff.substring(0,2).toInt())*60;
  timeNow_m = timeinfo.tm_min + timeinfo.tm_hour*60;
  if(timeOff_m < timeOn_m) timeOff_m += 24*60;
#ifdef DEBUG  
  Serial.print("TimeOn: ");
  Serial.println(timeOn_m);
  Serial.print("TimeOff: ");
  Serial.println(timeOff_m);
  Serial.print("TimeNow: ");
  Serial.println(timeNow_m);
#endif
  if(timeNow_m >= timeOn_m && timeNow_m < timeOff_m) {
    //We are within time slot
    sleep_m = (timeOff_m - timeNow_m);
    outputState = ON;
  } else {
    //We are outside time slot
    sleep_m = (timeOn_m - timeNow_m);
    outputState = OFF;
  }
  return sleep_m;
}

void turnON() {
  digitalWrite(BUCK_EN, HIGH);
#ifdef DEBUG
  Serial.println("Turn ON");
#endif
}

void turnOFF() {
  digitalWrite(BUCK_EN, LOW);
#ifdef DEBUG
  Serial.println("Turn OFF");
#endif
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

//Never ending loop
void loop() {
  currentMillis = millis();

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
          //If no response from NTP after button_exit_time, make sure to turn off and go to sleep.
          if(!receivedTime) {
            turnOFF();
          } else {
            sleeptime_m = eval_time();
          }
          currentSetModeActive = false;
          blinkLed = false;
          EEPROM.write(0, currentMode);
          EEPROM.commit();
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
  
  if (goToDeepSleep && !currentSetModeActive && newButtonState) {
    if(receivedTime) {
      if(outputState == ON) {
        turnON();
      } else {
        turnOFF();
      }
      gpio_hold_dis((gpio_num_t) BUCK_EN);
    }
#ifdef DEBUG
    delay(1000);
#endif   
    digitalWrite(LED, HIGH); //Turn off LED if left on..
    
    //preserve gpios during deep sleep
    gpio_hold_en((gpio_num_t) BUCK_EN);
    gpio_hold_en((gpio_num_t) HIGH_CUR);
    gpio_deep_sleep_hold_en();

    if(sleeptime_m < 1) sleeptime_m = 1;
    if(sleeptime_m > 60) sleeptime_m = 60;
#ifdef DEBUG
    Serial.println("Enter deep sleep...");
    Serial.print("Sleeptime(min): ");
    Serial.println(sleeptime_m);
    delay(1000);
#endif
    
    ESP.deepSleep(sleeptime_m*60*1000*1000);
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

