/*
  DIY Camera Slider with Pan and Tilt Head
  by Dejan Nedelkovski
  www.HowToMechatronics.com

  Library - AccelStepper by Mike McCauley:
  http://www.airspayce.com/mikem/arduino/AccelStepper/index.html

*/

#include <AccelStepper.h>
#include <MultiStepper.h>

#define JoyX A2       // Joystick X pin
#define JoyY A3       // Joystick Y pin
#define slider A0     // Slider potentiometer
#define inOutPot A1   // In and Out speed potentiometer
#define JoySwitch 10  // Joystick switch connected
#define InOutSet 9   // Set Button
#define limitSwitch 12
#define inLED A4
#define outLED A5
#define ind 11

// Define the stepper motors and the pins the will use
AccelStepper stepper1(1, 5, 2); // (Type:driver, STEP, DIR)
AccelStepper stepper3(1, 6, 3);
AccelStepper stepper2(1, 7, 4);

MultiStepper StepperControl;  // Create instance of MultiStepper

long gotoposition[3]; // An array to store the In or Out position for each stepper motor

int JoyXPos = 0;
int JoyYPos = 0;
int sliderPos = 0;
int currentSpeed = 200;
int inOutSpeed = 100;

int XInPoint = 0;
int YInPoint = 0;
int ZInPoint = 0;
int XOutPoint = 0;
int YOutPoint = 0;
int ZOutPoint = 0;
int InandOut = 0;

void setup() {
  // Set initial seed values for the steppers
  stepper1.setMaxSpeed(3000);
  stepper1.setSpeed(200);
  stepper2.setMaxSpeed(3000);
  stepper2.setSpeed(200);
  stepper3.setMaxSpeed(3000);
  stepper3.setSpeed(200);
  pinMode(JoySwitch, INPUT_PULLUP);
  pinMode(InOutSet, INPUT_PULLUP);
  pinMode(limitSwitch, INPUT_PULLUP);
  pinMode(inLED, OUTPUT);
  pinMode(outLED, OUTPUT);

  // Create instances for MultiStepper - Adding the 3 steppers to the StepperControl instance for multi control
  StepperControl.addStepper(stepper1);
  StepperControl.addStepper(stepper2);
  StepperControl.addStepper(stepper3);

  // Move the slider to the initial position - homing
  while (digitalRead(limitSwitch) != 0) {
    digitalWrite(ind,HIGH);
    stepper1.setSpeed(3000);
    stepper1.runSpeed();
    stepper1.setCurrentPosition(0); // When limit switch pressed set position to 0 steps
  }
  delay(20);
  // Move 200 steps back from the limit switch
  while (stepper1.currentPosition() != -200) {
    stepper1.setSpeed(-3000);
    stepper1.run();
         }
         delay(20);
         digitalWrite(ind,LOW);

}

void loop() {
  // Limiting the movement - Do nothing if limit switch pressed or distance traveled in other direction greater then 80cm
  while (digitalRead(limitSwitch) == 0 || stepper1.currentPosition() < -64800) {}

  // If Joystick pressed increase the Pan and Tilt speeds
  if (digitalRead(JoySwitch) == 0) {
    currentSpeed = currentSpeed + 50;
    delay(200);
  }
  // If Set button is pressed - toggle between the switch cases
  if (digitalRead(InOutSet) == 0) {
    delay(500);
    // If we hold set button pressed longer then half a second, reset the in and out positions
    if (digitalRead(InOutSet) == 0) {
      InandOut = 4;
    }
    switch (InandOut) { 
      case 0:   // Set IN position
        InandOut = 1;
        XInPoint = stepper1.currentPosition(); // Set the IN position for steppers 1
        YInPoint = stepper2.currentPosition(); // Set the IN position for steppers 2
        ZInPoint = stepper3.currentPosition(); // Set the IN position for steppers 3
        digitalWrite(inLED, HIGH); // Light up inLed
        break;

      case 1: // Set OUT position
        InandOut = 2;
        XOutPoint = stepper1.currentPosition(); //  Set the OUT Points for both steppers
        YOutPoint = stepper2.currentPosition();
        ZOutPoint = stepper3.currentPosition();
        digitalWrite(outLED, HIGH);
        break;

      case 2: // Move to IN position / go to case 3
        InandOut = 3;
        inOutSpeed = analogRead(inOutPot); // Auto speed potentiometer
        // Place the IN position into the Array
        gotoposition[0] = XInPoint;
        gotoposition[1] = YInPoint;
        gotoposition[2] = ZInPoint;
        stepper1.setMaxSpeed(inOutSpeed*3);
        stepper2.setMaxSpeed(inOutSpeed*3);
        stepper3.setMaxSpeed(inOutSpeed*3);
        StepperControl.moveTo(gotoposition); // Calculates the required speed for all motors
        StepperControl.runSpeedToPosition(); // Blocks until all steppers are in position
        delay(200);
        break;

      case 3: // Move to OUT position / go back to case 2
        InandOut = 2;
        inOutSpeed = analogRead(inOutPot);
        // Place the OUT position into the Array
        gotoposition[0] = XOutPoint;
        gotoposition[1] = YOutPoint;
        gotoposition[2] = ZOutPoint;
        stepper1.setMaxSpeed(inOutSpeed*3);
        stepper2.setMaxSpeed(inOutSpeed*3);
        stepper3.setMaxSpeed(inOutSpeed*3);
        StepperControl.moveTo(gotoposition); // Calculates the required speed for all motors
        StepperControl.runSpeedToPosition(); // Blocks until all are in position
        delay(200);
        break;

      case 4: // If Set button is held longer then half a second go back to case 0
        InandOut = 0;
        digitalWrite(inLED, LOW);
        digitalWrite(outLED, LOW);
        delay(1000);
        break;
    }
  }

  // Joystick X - Pan movement
  JoyXPos = analogRead(JoyX);
  // if Joystick is moved left, move stepper 2 or pan to left
  if (JoyXPos > 600) {
    stepper2.setSpeed(currentSpeed);
  }
  // if Joystick is moved right, move stepper 2 or pan to right
  else if (JoyXPos < 400) {
    stepper2.setSpeed(-currentSpeed);
  }
  // if Joystick stays in middle, no movement
  else {
    stepper2.setSpeed(0);
  }

  //Joystick Y - Tilt movement
  JoyYPos = analogRead(JoyY);
  if (JoyYPos > 600) {
    stepper3.setSpeed(currentSpeed);
  }
  else if (JoyYPos < 400) {
    stepper3.setSpeed(-currentSpeed);
  }
  else {
    stepper3.setSpeed(0);
  }

  // Slider potentiometer
  sliderPos = analogRead(slider);
  // If potentiometer is turned left, move slider left
  if (sliderPos > 600) {
    sliderPos = map(sliderPos, 600, 1024, 0, 4000);
    stepper1.setSpeed(sliderPos); // Increase speed as turning
  }
  // If potentiometer is turned right, move slider right
  else if (sliderPos < 400 ) {
    sliderPos = map(sliderPos, 400, 0, 0, 4000);
    stepper1.setSpeed(-sliderPos); // Increase speed as turning
  }
  // If potentiometer in middle, no movement
  else {
    stepper1.setSpeed(0);
  }
  // Execute the above commands - run the stepper motors
  stepper1.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
}
