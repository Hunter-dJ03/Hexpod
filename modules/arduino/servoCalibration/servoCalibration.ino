#include <Servo.h>

Servo coxa;  // create servo object
Servo femur;
Servo tibia;

int currentPosition = 1500; // initial position in microseconds (typically 1500 is centered)
const int stepSize = 10;    // amount to change per increment

void setup() {
  Serial.begin(9600);     // initialize serial communication


  coxa.attach(25);      // attach the servo on pin 9 (change as needed)
  femur.attach(27);      // attach the servo on pin 9 (change as needed)
  tibia.attach(29);      // attach the servo on pin 9 (change as needed)

  coxa.writeMicroseconds(1560);  // set servo to the initial position
  femur.writeMicroseconds(2375);
  tibia.writeMicroseconds(currentPosition);

  Serial.println("Servo Calibration Ready."); 
  Serial.println("Send a positive or negative number to adjust the servo position.");
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming data as a string
    String input = Serial.readStringUntil('\n');
    int increment = input.toInt();  // Convert input to integer

    // Update the servo position based on the input
    currentPosition = increment;

    // Constrain the position to a safe range for the servo
    currentPosition = constrain(currentPosition, 500, 2500);

    // Write the new position to the servo
    tibia.writeMicroseconds(currentPosition);

    // Feedback to the serial monitor
    Serial.print("New Position: ");
    Serial.println(currentPosition);
  }
}
