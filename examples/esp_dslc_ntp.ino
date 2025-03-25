/*
FW for "Manges Mojäng" ESP32 Decor String Light Controller (esp_dslc)

Variant: Timestamp
Requires: Wifi
Version: 0.3

***Concept***
Wakes up at a given time and turns on/off

***At cold start/first boot***
If no wifi credentials has been stored, the device boots up as an Access Point (esp_dslc_XXXX)
Connect to it and configure wifi and two timestamps, one for ON and one for OFF.
Once saved and device successfully connects to wifi, the AP should disappear.

At cold start, if WiFi have been stored, a short press on the button within 5s will force the Access Point
to become available for change in config. A long press(>5s) within 5s from cold start will clear wifi settings
and the Access Point will become available.

If button is not pressed within 5s from cold start, the LED will blink and the output will turn ON and Button 
toggles between 15/30mA output. It goes to sleep after 30s of button inactivity. 
Turns off output if no connection to wifi or if not time to be lit

***At wake up***
Connects to WiFi, calibrates local time against NTP server, take actions (on/off), then go to Sleep

*/

//##################### DEBUG ######################
// Make sure "USB CDC On Boot" is Enabled if DEBUG is defined.
// DEBUG mode will not work with power banks

//#define DEBUG   

//###################################################

#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include "driver/gpio.h"
#include "time.h"
#include "Preferences.h"

const char *ntpServer = "pool.ntp.org";
const uint32_t gmtOffset_sec = 3600;
const uint16_t daylightOffset_sec = 0;

const uint32_t button_exit_time_ms = 30000;    //Timeout after last button press at cold start

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

#define uS_TO_S_FACTOR 1000000ULL

//Variables
bool cold_start = false;
bool goToDeepSleep = false;
bool currentSetModeActive = false;
bool blinkLed = false;
bool receivedTime = false;
bool force_AP_mode = false;
bool active_AP_mode = false;
bool wait_for_config = false;
uint8_t boot_up_reason = 0;
uint8_t outputState = OFF;
uint8_t ledblink[8] = {0,1,1,1,1,1,1,1};
uint8_t ledptr = 0;
uint8_t currentMode = 0;
uint16_t timeout = 0;
uint16_t newButtonState = HIGH;
uint16_t sleeptime_m = 10; //(10 min) default (if failed wifi or failed NTP)
uint32_t startMillis = 0;
uint32_t currentMillis = 0;
uint32_t timeMillis = 0;
uint32_t buttonReleaseMillis = 0;
uint32_t led_blink_ms = 250;
String newHostname; 
struct tm timeinfo;
WiFiManager wm; // global wm instance
WiFiManagerParameter wmparam_timeinput; // global param ( for non blocking w params )
char wm_custom_param[270];

// RTC Fast Memory Variables (Retained during Deep Sleep)
RTC_DATA_ATTR uint8_t timeSetOn_h = 0;
RTC_DATA_ATTR uint8_t timeSetOn_m = 0;
RTC_DATA_ATTR uint8_t timeSetOff_h = 0;
RTC_DATA_ATTR uint8_t timeSetOff_m = 0;

Preferences nvmPrefs;


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
    if(nvmPrefs.isKey("timeSetOn_h")) {
      timeSetOn_h = nvmPrefs.getUChar("timeSetOn_h");
    } else {
      nvmPrefs.putUChar("timeSetOn_h", 17);
    }
    if(nvmPrefs.isKey("timeSetOn_m")) {
      timeSetOn_m = nvmPrefs.getUChar("timeSetOn_m");
    } else {
      nvmPrefs.putUChar("timeSetOn_m", 0);
    }
    if(nvmPrefs.isKey("timeSetOff_h")) {
      timeSetOff_h = nvmPrefs.getUChar("timeSetOff_h");
    } else {
      nvmPrefs.putUChar("timeSetOff_h", 23);
    }
    if(nvmPrefs.isKey("timeSetOff_m")) {
      timeSetOff_m = nvmPrefs.getUChar("timeSetOff_m");
    } else {
      nvmPrefs.putUChar("timeSetOff_m", 0);
    }
    if(currentMode > 1) {
      currentMode=0;
      nvmPrefs.putUChar("currentMode", currentMode);
    }
    if(timeSetOn_h > 23) {
      timeSetOn_h = 17;
      nvmPrefs.putUChar("timeSetOn_h", timeSetOn_h);
    }
    if(timeSetOn_m > 59) {
      timeSetOn_m = 00;
      nvmPrefs.putUChar("timeSetOn_m", timeSetOn_m);
    }
    if(timeSetOff_h > 23) {
      timeSetOff_h = 23;
      nvmPrefs.putUChar("timeSetOff_h", timeSetOff_h);
    }
    if(timeSetOff_m > 59) {
      timeSetOff_m = 00;
      nvmPrefs.putUChar("timeSetOff_m", timeSetOff_m);
    }
    nvmPrefs.end();
    setCurrent(currentMode);
  }

  wm.setConfigPortalBlocking(false);

  sprintf(wm_custom_param, "<br/><label for='timeinputid_on'>Select ON Time: </label><input type='time' name='timeinputid_on' value='%02d:%02d' required><br><label for='timeinputid_off'>Select OFF Time: </label><input type='time' name='timeinputid_off' value='%02d:%02d' required><br>MAC: %s", timeSetOn_h, timeSetOn_m, timeSetOff_h, timeSetOff_m, WiFi.macAddress().c_str());
  new (&wmparam_timeinput) WiFiManagerParameter(wm_custom_param);
  wm.addParameter(&wmparam_timeinput);

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

  //We are done with Wifi, if not AP
  if(!active_AP_mode) {
    wm.disconnect();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
#ifdef DEBUG
    Serial.println("Disconnected from WiFi");
#endif
    goToDeepSleep = true;
  }
  // Get some uptime timestamps initialized
  startMillis = millis();  //initial start time
  timeMillis = startMillis;
}

//Function to see if we are within time slot
uint16_t eval_time(void) {
  uint16_t sleep_m = 0;
  uint16_t timeOn_m = 0;
  uint16_t timeOff_m = 0;
  uint16_t timeNow_m = 0;
  bool dst = false;

  //Turn timestamps to minutes
  timeOn_m = timeSetOn_m + timeSetOn_h*60; 
  timeOff_m = timeSetOff_m + timeSetOff_h*60;
  timeNow_m = timeinfo.tm_min + timeinfo.tm_hour*60;
  

//DST magic
  uint8_t month, mday, d, n;
  month = timeinfo.tm_mon + 1; // (0-11) + 1
  mday = timeinfo.tm_mday;

  d = (mday - timeinfo.tm_wday) % 7;  //First Sunday
#ifdef DEBUG  
  Serial.print("First Sunday: ");
  Serial.println(d);
#endif
  n = (31 - d) / 7;
  d = d + 7 * n;  //Last Sunday
#ifdef DEBUG  
  Serial.print("Last Sunday: ");
  Serial.println(d);
#endif

  if((month > 3) && (month < 10)) {
    dst = true;
  } else if(month == 3) {
    if (d < mday || (d == mday && timeinfo.tm_hour > 1)) dst = true;
  } else if(month == 10) {
    if(d > mday || (d == mday && timeinfo.tm_hour < 2)) dst = true;
  }

  if(dst) {
    timeNow_m += 60;
    if(timeNow_m >= 1440) timeNow_m -= 1440;
  }



#ifdef DEBUG  
  Serial.print("TimeOn: ");
  Serial.println(timeOn_m);
  Serial.print("TimeOff: ");
  Serial.println(timeOff_m);
  Serial.print("TimeNow: ");
  Serial.println(timeNow_m);
  Serial.print("DST: ");
  Serial.println(dst);
#endif
  if(timeOff_m > timeOn_m) {
    if(timeNow_m >= timeOn_m && timeNow_m < timeOff_m) {
      //We are within time slot
      sleep_m = (timeOff_m - timeNow_m);
      outputState = ON;
    } else {
      //We are outside time slot
      sleep_m = (timeOn_m - timeNow_m);
      outputState = OFF;
    }
  } else {
    if(timeNow_m >= timeOn_m || timeNow_m < timeOff_m) {
      //We are within time slot
      if(timeNow_m >= timeOn_m) sleep_m = (timeOff_m+24*60 - timeNow_m);
      if(timeNow_m < timeOff_m) sleep_m = (timeOff_m - timeNow_m);
      outputState = ON;
    } else {
      //We are outside time slot
      sleep_m = (timeOn_m - timeNow_m);
      outputState = OFF;
    }
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

void setCurrent(uint8_t mode) {
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
  String timeOn_str = getParam("timeinputid_on");
  String timeOff_str = getParam("timeinputid_off");
#ifdef DEBUG
  Serial.println("[CALLBACK] saveParamCallback fired");
  Serial.println("PARAM timeinputid_on = " + timeOn_str);
  Serial.println("PARAM timeinputid_off = " + timeOff_str);
#endif
  timeSetOn_h = timeOn_str.substring(0,2).toInt();
  timeSetOn_m = timeOn_str.substring(3,5).toInt();
  timeSetOff_h = timeOff_str.substring(0,2).toInt();
  timeSetOff_m = timeOff_str.substring(3,5).toInt();

  nvmPrefs.begin("thePrefs", false);
  nvmPrefs.putUChar("timeSetOn_h", timeSetOn_h);
  nvmPrefs.putUChar("timeSetOn_m", timeSetOn_m);
  nvmPrefs.putUChar("timeSetOff_h", timeSetOff_h);
  nvmPrefs.putUChar("timeSetOff_m", timeSetOff_m);
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


//Never ending loop
void loop() {
  currentMillis = millis();

  if(cold_start) {
    cold_start = false;
    currentSetModeActive = true;
    blinkLed = true;
    turnON();
  }
  
  if(active_AP_mode && WiFi.status() == WL_CONNECTED) {
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
          //If no response from NTP after button_exit_time, make sure to turn off and go to sleep.
          if(!receivedTime) {
            turnOFF();
          } else {
            sleeptime_m = eval_time();
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

  if (goToDeepSleep && !currentSetModeActive && newButtonState) {
    //Set the output to correct state
    if(receivedTime) {
      if(outputState == ON) {
        turnON();
      } else {
        turnOFF();
      }
      gpio_hold_dis((gpio_num_t) BUCK_EN);
    }
    digitalWrite(LED, HIGH); //Turn off LED if left on..
    //preserve gpios during deep sleep
    gpio_hold_en((gpio_num_t) BUCK_EN);
    gpio_hold_en((gpio_num_t) HIGH_CUR);
    gpio_deep_sleep_hold_en();

#ifndef DEBUG
	//This is done to wake Powerbanks and control if the charging works (when using pbalive-dongle)
	pinMode(PBWAKE, INPUT_PULLDOWN);
#endif


    if(sleeptime_m < 1) sleeptime_m = 1;
    if(sleeptime_m > 60) sleeptime_m = 60;
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


