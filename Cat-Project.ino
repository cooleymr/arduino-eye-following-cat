#include "HCPCA9685.h"  //For servo controller
#include <cppQueue.h>

/* I2C slave address for the device/module. For the HCMODU0097 the default I2C address is 0x40 */
#define I2CAdd 0x40

boolean machineOn = false;

int trigPin = 4;  // Trigger
int echoPin = 5;  // Echo

int leftTrigPin = 3;  // Trigger
int leftEchoPin = 2;

int rightTrigPin = 11;  // Trigger
int rightEchoPin = 10;

unsigned int eyePos;
unsigned int armPos;

boolean armUp = true;

// Button and LED at buttom of machine
const int buttonPin = 8;  // the number of the pushbutton pin
const int ledPin = 9;     // the number of the LED pin
int buttonState = 0;

// Degrees for rotating eyes
const int leftGoalPos = 300;
const int centerGoalPos = 230;
const int rightGoalPos = 150;
int goalPos;

//Creating arrays that hold values for each sensor
const long ARRAY_SIZE = 3;
int arrPos = 0;
int centerArrPos = 0;
int leftArrPos = 0;
int rightArrPos = 0;
long centerArr[ARRAY_SIZE];
long leftArr[ARRAY_SIZE];
long rightArr[ARRAY_SIZE];

/* Create an instance of the library */
HCPCA9685 HCPCA9685(I2CAdd);

void setup() {
  //Serial Port begin
  Serial.begin(9600);

  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(leftTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);
  pinMode(rightTrigPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);

  HCPCA9685.Init(SERVO_MODE);
  HCPCA9685.Sleep(false);

  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
}

void loop() {
  // Getting distance at each sensor
  long centerDist = distance(trigPin, echoPin);
  // updateArr(centerArr, centerArrPos, centerDist);
  updateArr(centerArr, centerDist);
  long leftDist = distance(leftTrigPin, leftEchoPin);
  // updateArr(leftArr, leftArrPos, leftDist);
  updateArr(leftArr, leftDist);
  long rightDist = distance(rightTrigPin, rightEchoPin);
  // updateArr(rightArr, rightArrPos, rightDist);
  updateArr(rightArr, rightDist);
  arrPos += 1;
  if (arrPos == ARRAY_SIZE) {
    arrPos = 0;
  }

  // Getting average of each sensor
  long centerAverage = average(centerArr, ARRAY_SIZE);
  long leftAverage = average(leftArr, ARRAY_SIZE);
  long rightAverage = average(rightArr, ARRAY_SIZE);

  // Finding which average is shortest
  long shortest = min(min(centerAverage, rightAverage), leftAverage);
  if (shortest == centerAverage) {
    goalPos = centerGoalPos;
  } else if (shortest == leftAverage) {
    goalPos = leftGoalPos;
  } 
  else if (shortest == rightAverage) {
    goalPos = rightGoalPos;
  }

  // Serial.println(show(centerDist));
  // Serial.println(show(leftDist));
  // Serial.println(show(rightDist));
  // Serial.println(arrPos);

  // Serial.print(leftAverage);
  // Serial.print(",");
  // Serial.print(centerAverage);
  // Serial.print(",");
  // Serial.print(rightAverage);
  // Serial.print(",");
  // Serial.print(goalPos);
  // Serial.println("");

  // checkButton();
  // if (machineOn == true) {
    updateEyeGears();

    updateArmGear();      
  // }
delay(10);
}

void checkButton() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // Serial.println("HIGH");
    if (machineOn == true) {
      digitalWrite(ledPin, LOW);
      machineOn = false;
    } else if (machineOn == false) {
      digitalWrite(ledPin, HIGH);  // Turning machine on
      machineOn = true;
    }
  }
  if (buttonState == LOW) {
    // Serial.println("LOW");
  }
}

// Rotating eye gears 10 degrees closer to goal
void updateEyeGears() {
  const int leftServo = 15;
  const int rightServo = 11;
  // Serial.println(goalPos);
  if (eyePos < goalPos) {
    eyePos += 10;
    HCPCA9685.Servo(leftServo, eyePos);
    HCPCA9685.Servo(rightServo, eyePos);
  }
  if (eyePos > goalPos) {
    eyePos -= 10;
    HCPCA9685.Servo(leftServo, eyePos);
    HCPCA9685.Servo(rightServo, eyePos);
  }
  delay(5);
}

//Rotate gear funciton for the constant moving cat arm
void updateArmGear() {
  const int armServo = 8;
  int topAngle = 180;
  int bottomAngle = 0;
  int armSpeed = 3;

  // Serial.print(armPos);
  // Serial.print(",");
  // Serial.print(armUp);
  // Serial.println("");

  if (armUp == true) {
    if (armPos < topAngle) {
      armPos += armSpeed;
      HCPCA9685.Servo(armServo, armPos);
      delay(10);
    } else if (armPos >= topAngle) {
      armUp = false;
    }
  } else if (armUp == false) {
    if (armPos > bottomAngle) {
      armPos -= armSpeed;
      HCPCA9685.Servo(armServo, armPos);
      delay(10);
    } else if (armPos <= topAngle) {
      armUp = true;
    }
  }
}

// Adds new number to array and maintains it's size
void updateArr(long arr[], long value) {
  // Serial.println(arrPos);
  arr[arrPos] = value;
}

// Returns distance of sensor
float distance(int trigger, int echo) {
  digitalWrite(trigger, LOW);
  delayMicroseconds(5);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  pinMode(echo, INPUT);

  long duration = pulseIn(echo, HIGH);

  // Convert the time into a distance
  long cm = (duration / 2) / 29.1;  // Divide by 29.1 or multiply by 0.0343

  // Serial.print(duration);
  // Serial.print(",");
  // Serial.print(cm);
  // Serial.println("");
  return cm;
}

//Returns the average of all the values in the array
long average(long* array, int len) {
  long sum = 0L;  // sum will be larger than an item, long for safety.
  for (int i = 0; i < len; i++)
    sum += array[i];
  return ((long)sum) / len;  // average will be fractional, so float may be appropriate.
}

// Prints out an array (Used for testing/debugging)
long show(long* array) {
  for (int i = 0; i < ARRAY_SIZE; i++) {
    Serial.print(array[i]);
    Serial.print(",");
  }
}


/*
 * created by Rui Santos, https://randomnerdtutorials.com
 * 
 * Complete Guide for Ultrasonic Sensor HC-SR04
 *
    Ultrasonic sensor Pins:
        VCC: +5VDC
        Trig : Trigger (INPUT) - Pin11
        Echo: Echo (OUTPUT) - Pin 12
        GND: GND


PCA9685 Servo Driver library downloaded and tutorial found: https://www.electroniclinic.com/pca9685-servo-driver-arduino-circuit-diagram-and-code/#Adding_the_library_to_the_Arduino_IDE
 */
