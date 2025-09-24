#include "Aerobotix_Arduino_nav.h"

// Create robot instance
Aerobotix_Arduino_nav robot;

// Example codes 
bool code1 = true;
bool code2 = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting Aerobotix_Arduino_nav Basic Example");
  
  // Initialize the robot
  robot.begin();
  
  // Optional: Modify some parameters before starting
  robot.setKp(0.12);           // Tune PID parameters
  robot.setKi(0.06);
  robot.setPWMMax(200);        // Increase max PWM
  robot.setMaxSpeed(150);      // Increase max speed
  
  Serial.println("Robot initialized. Starting navigation sequence...");
  delay(1000);

  // ===== EXAMPLE 1: Basic Movement =====
  Serial.println("=== Example 1: Moving forward 50cm ===");
  robot.moveDistance(50, 100);
  delay(1000);
  
  Serial.println("=== Example 2: Rotating 90 degrees ===");
  robot.rotate(90, 80);
  delay(1000);
  
  Serial.println("=== Example 3: Moving backward 30cm ===");
  robot.moveDistance(-30, 80);
  delay(1000);

  // ===== EXAMPLE 2: Complex Sequence (similar to your original code) =====
  Serial.println("=== Starting Complex Sequence ===");
  
  // Move forward 109.5cm
  robot.moveDistance(109.5, 100);
  delay(1000);
  
  // Code 1 action (like activating a mechanism)
  if (code1) {
    Serial.println("Code 1 activated");
    // You can add your mechanism control here
    delay(2000);
  }
  
  // Move backward 99.5cm
  robot.moveDistance(-99.5, 100);
  delay(1000);
  
  // Rotate 190 degrees (without stopping if obstacle)
  robot.dour(190, 80, false);
  delay(1000);
  
  // Move forward 35cm
  robot.moveDistance(35, 100);
  delay(1000);
  
  // Code 2 action
  if (code2) {
    Serial.println("Code 2 activated");
    delay(2000);
  }
  
  // Rotate 80 degrees (with stopping)
  robot.rotate(80, 80);
  delay(1000);
  
  // Move forward 92cm
  robot.moveDistance(92, 100);
  delay(1000);
  
  Serial.println("=== Complex Sequence Completed ===");

  // ===== EXAMPLE 3: Go to Coordinate =====
  Serial.println("=== Example 4: Go to coordinate (100, 50) ===");
  robot.go(100, 50, 100);  // Go to x=100cm, y=50cm at speed 100
  delay(1000);
}

void loop() {
  // Display current robot status
  Serial.println("=== Robot Status ===");
  Serial.print("Distance traveled: ");
  Serial.print(robot.getDSTotal());
  Serial.println(" cm");
  
  Serial.print("Current orientation: ");
  Serial.print(robot.getTheta() * 180.0 / PI);
  Serial.println(" degrees");
  
  Serial.print("Right wheel speed: ");
  Serial.print(robot.getCurrentVelocityRight());
  Serial.println(" cm/s");
  
  Serial.print("Left wheel speed: ");
  Serial.print(robot.getCurrentVelocityLeft());
  Serial.println(" cm/s");
  
  Serial.print("Right encoder: ");
  Serial.println(robot.getEncoderRightCount());
  
  Serial.print("Left encoder: ");
  Serial.println(robot.getEncoderLeftCount());
  
  Serial.println("====================");
  Serial.println();
  
  delay(2000);  // Update every 2 seconds

  // Optional: Add interactive control via Serial
  if (Serial.available() > 0) {
    char command = Serial.read();
    handleCommand(command);
  }
}

// Function to handle serial commands for manual control
void handleCommand(char cmd) {
  switch(cmd) {
    case 'f':  // Move forward
      robot.moveDistance(20, 80);
      break;
    case 'b':  // Move backward
      robot.moveDistance(-20, 80);
      break;
    case 'l':  // Rotate left
      robot.rotate(45, 60);
      break;
    case 'r':  // Rotate right
      robot.rotate(-45, 60);
      break;
    case 's':  // Stop
      robot.stopmotors();
      break;
    case '1':  // Tune PID
      robot.setKp(robot.getKp() + 0.01);
      Serial.print("Kp increased to: ");
      Serial.println(robot.getKp());
      break;
    case '2':  // Tune PID
      robot.setKp(robot.getKp() - 0.01);
      Serial.print("Kp decreased to: ");
      Serial.println(robot.getKp());
      break;
    case 'i':  // Display info
      displayRobotInfo();
      break;
    case 'h':  // Help
      printHelp();
      break;
  }
}

void displayRobotInfo() {
  Serial.println("=== Robot Configuration ===");
  Serial.print("Wheel radius: ");
  Serial.println(robot.getWheelRadius());
  Serial.print("Wheelbase: ");
  Serial.println(robot.getEntreaxe());
  Serial.print("Kp: ");
  Serial.println(robot.getKp());
  Serial.print("Ki: ");
  Serial.println(robot.getKi());
  Serial.print("KTheta: ");
  Serial.println(robot.getKTheta());
  Serial.print("PWM Min/Max: ");
  Serial.print(robot.getPWMMin());
  Serial.print(" / ");
  Serial.println(robot.getPWMMax());
  Serial.println("===========================");
}

void printHelp() {
  Serial.println("=== Available Commands ===");
  Serial.println("f - Move forward 20cm");
  Serial.println("b - Move backward 20cm");
  Serial.println("l - Rotate left 45°");
  Serial.println("r - Rotate right 45°");
  Serial.println("s - Stop motors");
  Serial.println("1 - Increase Kp");
  Serial.println("2 - Decrease Kp");
  Serial.println("i - Display robot info");
  Serial.println("h - Show this help");
  Serial.println("==========================");
}

// Optional: Add functions for specific tasks
void runMission() {
  Serial.println("Starting autonomous mission...");
  
  // Square pattern
  for(int i = 0; i < 4; i++) {
    robot.moveDistance(100, 120);  // Move 1 meter
    delay(500);
    robot.rotate(90, 100);         // Turn 90 degrees
    delay(500);
  }
  
  Serial.println("Mission completed!");
}

void testOdometry() {
  Serial.println("Testing odometry accuracy...");
  
  // Move in a triangle pattern
  robot.moveDistance(100, 100);
  robot.rotate(120, 80);
  robot.moveDistance(100, 100);
  robot.rotate(120, 80);
  robot.moveDistance(100, 100);
  robot.rotate(120, 80);
  
  Serial.println("Odometry test completed");
  Serial.print("Final position error: ");
  Serial.print(robot.getDSTotal());
  Serial.println(" cm");
}
