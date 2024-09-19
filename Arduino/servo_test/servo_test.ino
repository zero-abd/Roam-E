#include <Servo.h>

Servo servoMotor; // Create a Servo object to control the servo motor

void setup() {
  servoMotor.attach(3); // Attach the servo to pin 9
}

void loop() {
  // Move the servo to its minimum angle (0 degrees)
  servoMotor.write(0);
  
  delay(1000); // Wait for 1 second
  
  // Move the servo to its maximum angle (180 degrees)
  servoMotor.write(150);
  delay(1000); // Wait for 1 second
}
