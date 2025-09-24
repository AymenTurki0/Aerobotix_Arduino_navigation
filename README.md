# Aerobotix_Arduino_nav

**Aerobotix_Arduino_nav** is an Arduino library for **autonomous differential drive robots**.  
It supports odometry, basic movement functions (`moveDistance`, `rotate`, `go`, `dour`), PID control, and customizable motor parameters.

## Features
- Move forward/backward specific distance
- Rotate or turn with precise angle
- Go to a target coordinate
- Adjustable PID parameters
- Odometry update and speed calculation
- Easy pin configuration

## Example
```cpp
#include "Aerobotix_Arduino_nav.h"

Aerobotix_Arduino_nav robot;

void setup() {
  Serial.begin(9600);
  robot.setIN1(6); robot.setIN2(7);
  robot.setIN3(10); robot.setIN4(11);
  robot.setInterruptPinRA(2); robot.setInterruptPinRB(3);
  robot.setInterruptPinLA(18); robot.setInterruptPinLB(19);
  robot.setKp(0.12); robot.setKi(0.06);
  robot.begin();

  robot.moveDistance(40, 100);
  robot.rotate(90, 80);
}

void loop() {}
