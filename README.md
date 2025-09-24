# Aerobotix_Arduino_nav

**Aerobotix_Arduino_nav** is an Arduino library for **autonomous differential drive robots**.  
It supports odometry, basic movement functions (`moveDistance`, `rotate`, `go`), PID control, and customizable motor parameters.

## Features
- Move forward/backward specific distance  
- Rotate or turn with precise angle  
- Go to a target coordinate  
- Adjustable PID parameters  
- Odometry update and speed calculation  
- Easy pin configuration  

## Requirements
- Works only on **AVR-based Arduino boards** (e.g. Uno, Mega, Nano).  
- Requires **TimerOne library (v1.1.1)** → [TimerOne on Arduino Library Manager](https://www.arduino.cc/reference/en/libraries/timerone/).  
  > ⚠️ Make sure you install **TimerOne 1.1.1** (not 1.1.0) for full compatibility.  
- Not compatible with ESP8266 / ESP32.  

## Example
```cpp
#include "Aerobotix_Arduino_nav.h"

Aerobotix_Arduino_nav robot;

void setup() {
  Serial.begin(9600);

  // Motor pins
  robot.setIN1(6); robot.setIN2(7);
  robot.setIN3(10); robot.setIN4(11);

  // Encoder pins
  robot.setInterruptPinRA(2); robot.setInterruptPinRB(3);
  robot.setInterruptPinLA(18); robot.setInterruptPinLB(19);

  // PID tuning
  robot.setKp(0.12);
  robot.setKi(0.06);

  // Init robot
  robot.begin();

  // Example movements
  robot.moveDistance(40, 100);
  robot.rotate(90, 80);
}

void loop() {
  // Nothing here, movements are blocking
}
