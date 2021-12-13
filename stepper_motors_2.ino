//This program is the best for control of the stepper motors because it uses the AccelStepper library!
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>
#include <AccelStepper.h>
#define STEPS_PER_REV_L_R 400
#define STEPS_PER_REV_C 3200
//
const int TX_pin = 2;
const int RX_pin = 3;
const int C_directionPin = 4;
const int C_stepPin = 5;
const int C_MS = 6;
const int L_directionPin = 7;
const int L_stepPin = 8;
const int L_MS = 9;
const int R_directionPin = 10;
const int R_stepPin = 11;
const int R_MS = 12;
const int delay4speed = 1000;
//
AccelStepper C_motor(1,C_stepPin,C_directionPin); //AccelStepper nameOfMotorObject(motorInterfaceType,stepPin,directionPin);
AccelStepper L_motor(1,L_stepPin,L_directionPin);
AccelStepper R_motor(1,R_stepPin,R_directionPin);

***************************************************************************************

void setup() {
  Serial.begin(9600);
  Dabble.begin(9600); 
  //
  C_motor.setMaxSpeed(STEPS_PER_REV_C); //set max speed in steps per second
  C_motor.setAcceleration(1000); //set acceleration in steps per second per second
  delay(500);
  L_motor.setMaxSpeed(STEPS_PER_REV_L_R); 
  L_motor.setAcceleration(1000);
  delay(500);
  R_motor.setMaxSpeed(STEPS_PER_REV_L_R);
  R_motor.setAcceleration(1000);
  delay(500);
  //
  pinMode(C_directionPin,OUTPUT);
  pinMode(C_stepPin,OUTPUT);
  pinMode(C_MS,OUTPUT);
  pinMode(L_directionPin,OUTPUT);
  pinMode(L_stepPin,OUTPUT);
  pinMode(L_MS,OUTPUT);
  pinMode(R_directionPin,OUTPUT);
  pinMode(R_stepPin,OUTPUT);
  pinMode(R_MS,OUTPUT);
  //
  digitalWrite(C_MS,HIGH);
  digitalWrite(L_MS,HIGH);
  digitalWrite(R_MS,HIGH);

}

***************************************************************************************

void loop() {
  int L_mode;
  int R_mode;
  Dabble.processInput();    
  //
  if (GamePad.isUpPressed()) { //while loop instead??
    Serial.println("UP");
    L_motor.setSpeed(STEPS_PER_REV_L_R/2);
    R_motor.setSpeed(-STEPS_PER_REV_L_R/2);
    L_motor.runSpeed();
    R_motor.runSpeed(); 
  }
  if (GamePad.isRightPressed()) {
    Serial.println("RIGHT");
    L_motor.setSpeed(STEPS_PER_REV_L_R/2);
    R_motor.setSpeed(STEPS_PER_REV_L_R/2);
    L_motor.runSpeed();
    R_motor.runSpeed(); 
  }
  if (GamePad.isDownPressed()) {
    Serial.println("DOWN");
    L_motor.setSpeed(-STEPS_PER_REV_L_R/2); 
    R_motor.setSpeed(STEPS_PER_REV_L_R/2);
    L_motor.runSpeed();
    R_motor.runSpeed(); 
  }
  if (GamePad.isLeftPressed()) {
    Serial.println("LEFT");
    L_motor.setSpeed(-STEPS_PER_REV_L_R/2);
    R_motor.setSpeed(-STEPS_PER_REV_L_R/2);
    L_motor.runSpeed();
    R_motor.runSpeed(); 
  }

  //if the below code appears only here and not in the if statements, I will have motors that are always turning. i want them to only turn when button pressed.
  //L_motor.runSpeed();
  //R_motor.runSpeed(); 
}
