# ESP32 Decor String Light Controller

## Get started
* Download Arduino IDE 2.3.x, https://www.arduino.cc/en/software

* Once installed, goto File->Preferences...
	Under "Additional boards manager URLs", add https://espressif.github.io/arduino-esp32/package_esp32_dev_index.json

*	Goto Tools->Boards:->Boards Manager...
	Search "esp32", install latest version of "esp32 by espressif systems"

*	Then File->Open... and open ".ino" file

*	Insert "Manges Mojäng" in USB port while keeping button pressed (=Enter FW-Download mode). 
	Once inserted, release button

* Under Tools menu, configure the following:
  
  Tools->Boards:->esp32->"ESP32C3 Dev Module"
  
	Tools->Port:->"COMxx (ESP32 Family Device)"

	Tools->USB CDC On Boot:->"Disabled"		//Choose Enabled if you want UART prints when debuging (uncomment #define DEBUG)

	Tools->Flash Mode:->"DIO"
	
*	Compile and upload by menu Sketch->Upload

*	After successfull upload, the program should automatically start. If not, power cycle by unplug and reinsert (without pressing button).

