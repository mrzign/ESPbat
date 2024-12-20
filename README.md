# ESP32 Decor String Light Controller
![](https://github.com/mrzign/ESPbat/blob/main/resources/image.png)

## Specification
On board: SoC: ESP32-C3, 1st Green LED, 1st Button (sampled during boot up for setting boot-mode (for flashing), regular usage in fw)

Output modes: Current output 20mA/40mA (Typically ~2-3V)*

Power In: 3.5V-5.5V via USB Typ-A

Sleep Current: ~7uA from VBUS

Active current: WiFi ~80mA from VBUS + ~15mA when Light output is on.

Battery/Powerbank duration will heavely depend on how often/long the light is on. Secondly how often the device interacts over wifi, and for how long.

*) The output is designed to be a current output and not a voltage output. Hense, the light intensity of a LED is proportional to the current, not the voltage.

## Get started
* Download Arduino IDE 2.3.x, https://www.arduino.cc/en/software

* Once installed, goto File->Preferences...
	Under "Additional boards manager URLs", add https://espressif.github.io/arduino-esp32/package_esp32_dev_index.json

*	Goto Tools->Boards:->Boards Manager...
	Search "esp32", install latest version of "esp32 by espressif systems"

*	Then File->Open... and open ".ino" file from examples folder

*	Read through the description and modify parts under "Settings" to fit your needs.

*	Insert "Manges Mojäng" in USB port while keeping button pressed (=Enter FW-Download mode). 
	Once inserted, release button

* Under Tools menu, configure the following:
  
  Tools->Boards:->esp32->"ESP32C3 Dev Module"
  
	Tools->Port:->"COMxx (ESP32 Family Device)"

	Tools->USB CDC On Boot:->"Disabled"		//Choose Enabled if you want UART prints when debuging (uncomment #define DEBUG)

	Tools->Flash Mode:->"DIO"
	
*	Compile and upload by menu Sketch->Upload

*	After successfull upload, the program should automatically start. If not, power cycle by unplug and reinsert (without pressing button).

