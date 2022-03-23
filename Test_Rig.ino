#include <AccelStepper.h>
#include <SoftwareSerial.h>
#define STEPS_PER_REV_L_R 400
#define STEPS_PER_REV_C 3200

//Pins and motors declarations
//const int RX_blue = 0;
//const int TX_blue = 1;
const int RX_yellow = 2;
const int TX_yellow = 3;
const int C_directionPin = 4;
const int C_stepPin = 5;
const int C_MS = 6;
const int L_directionPin = 7;
const int L_stepPin = 8;
const int L_MS = 9;
const int R_directionPin = 10;
const int R_stepPin = 11;
const int R_MS = 12;
const byte LED_test = 13;
const int delay4speed = 1000;
AccelStepper C_motor(1,C_stepPin,C_directionPin); //AccelStepper nameOfMotorObject(motorInterfaceType,stepPin,directionPin);
AccelStepper L_motor(1,L_stepPin,L_directionPin);
AccelStepper R_motor(1,R_stepPin,R_directionPin);
//SoftwareSerial blueSerial(TX_blue,RX_blue);
SoftwareSerial yellowSerial(TX_yellow,RX_yellow);

//Communication declarations
const byte nobytes = 7; // number of bytes in packet: 2x pre-amble, 4x data, 1x checksum
unsigned int preamble_y = 23893; //Same preamble as used in TX
unsigned int preamble_b = 23999; //Same preamble as used in TX
byte data_y[2*nobytes-1]; byte datain_y[nobytes];
byte data_b[2*nobytes-1]; byte datain_b[nobytes];
byte y = 0; byte b = 0; bool preambletest_y = 1; bool preambletest_b = 1;
byte testval = 78; // Same value as used in TX for the flashing LED
byte navail_y; byte navail_b;
byte checksum_y; byte checksum_b;

//Odometry declarations
byte stage = 0;
//long P0_L; long P0_R;
byte rotCount = 0;
long S1_L; long S1_R; byte R1;
long S2_L; long S2_R; byte R2;
long Sm_L; long Sm_R; byte Rm;
long S3_L; long S3_R; byte R3;
long S4_L; long S4_R; byte R4;
long L_target; long R_target;
bool targetingEnabled = 1;
const long calib_d = 350.0; //350 millimeters between two sweeping points
const long wheel_diameter = 55.0; //55mm wheel diameter
const long calib_steps = 350*400/3.141593/52;

//Timer declarations
int T1 = 0; int T2 = 0;

//***************************************************************************************

void setup() {
  Serial.begin(9600); //blueSerial
  yellowSerial.begin(9600);
  //
  //because of trade-off between step resolution and torque, I think it is smart to 
  //NOT increase the number inside .setAcceleration() with the number of steps.
  //Hence, held constant at 500 for all motors.
  //
  C_motor.setMaxSpeed(STEPS_PER_REV_C/8); //set max speed in steps per second
  C_motor.setAcceleration(200); //set acceleration in steps per second per second
  delay(1);
  L_motor.setMaxSpeed(STEPS_PER_REV_L_R/2); 
  L_motor.setAcceleration(200);
  delay(1);
  R_motor.setMaxSpeed(STEPS_PER_REV_L_R/2);
  R_motor.setAcceleration(200);
  delay(1);
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
  pinMode(LED_test,OUTPUT);
  //
  digitalWrite(C_MS,HIGH);
  digitalWrite(L_MS,HIGH);
  digitalWrite(R_MS,HIGH);
  digitalWrite(LED_test,HIGH);
  delay(500);
  digitalWrite(LED_test,LOW);
  delay(1);
  //
  L_motor.setCurrentPosition(0);
  R_motor.setCurrentPosition(0);
}

/*Note:
 * L_motor +speed, R_motor -speed, move backwards
 * L_motor +speed, R_motor +speed, rotate counterclockwise
 * L_motor -speed, R_motor -speed, rotate clockwise
 * L_motor -speed, R_motor +speed, move forwards
 */

//***************************************************************************************

void loop() {
  //processing signal from blue transmitter
  while(Serial.available()>2*nobytes-1) {Serial.read();} preambletest_b = 0; checksum_b = 0; // trim serial buffer
  navail_b = Serial.available();
  if(navail_b > 0) { data_b[nobytes-1] = 0; // Avoid multiple reads of same data
  for (int i = 0; i<2*nobytes-1-navail_b; i++){data_b[i] = data_b[i+navail_b];} // shift existing data to make room for new data
  for (int i = 2*nobytes-1-navail_b; i<2*nobytes-1; i++){data_b[i] = Serial.read();} // insert new data
  for (int i = 0; i<nobytes; i++) { if(data_b[i] == lowByte(preamble_b) && data_b[i + 1] == highByte(preamble_b)) {b = i; preambletest_b = 1;}} // find preamble
  for (int i = 0; i<nobytes; i++) {datain_b[i] = data_b[i+b];}} // extract data, preamble at beginning
  for (int i = 2; i < nobytes-1; i++) {checksum_b = checksum_b + datain_b[i];} // form and apply checksum
  if (preambletest_b == 1 && checksum_b == datain_b[nobytes-1]) {preambletest_b = 1;} else {preambletest_b = 0;}

  //processing signal from yellow transmitter
  while(yellowSerial.available()>2*nobytes-1) {yellowSerial.read();} preambletest_y = 0; checksum_y = 0; // trim serial buffer
  navail_y = yellowSerial.available();
  if(navail_y > 0) { data_y[nobytes-1] = 0; // Avoid multiple reads of same data
  for (int i = 0; i<2*nobytes-1-navail_y; i++){data_y[i] = data_y[i+navail_y];} // shift existing data to make room for new data
  for (int i = 2*nobytes-1-navail_y; i<2*nobytes-1; i++){data_y[i] = yellowSerial.read();} // insert new data
  for (int i = 0; i<nobytes; i++) { if(data_y[i] == lowByte(preamble_y) && data_y[i + 1] == highByte(preamble_y)) {y = i; preambletest_y = 1;}} // find preamble
  for (int i = 0; i<nobytes; i++) {datain_y[i] = data_y[i+y];}} // extract data, preamble at beginning
  for (int i = 2; i < nobytes-1; i++) {checksum_y = checksum_y + datain_y[i];} // form and apply checksum
  if (preambletest_y == 1 && checksum_y == datain_y[nobytes-1]) {preambletest_y = 1;} else {preambletest_y = 0;}
  
  if(millis()>T1+300) {digitalWrite(LED_test,LOW);} //stop blinking LED

  if(stage == 0) { //first sweep. spin clockwise.
    L_target = -4000; R_target = -4000; //in reality -400 ok. -4000 better for prototyping.
    L_motor.moveTo(L_target); R_motor.moveTo(R_target); targetingEnabled=0;
    if(L_motor.distanceToGo() != 0) { L_motor.run(); } //turn motor
    if(R_motor.distanceToGo() != 0) { R_motor.run(); } //turn motor
    if(preambletest_b == 1) { //during rotation, if laser shines on marker b (which it will do first)
      S1_L = L_motor.currentPosition();
      S1_R = R_motor.currentPosition();
      //optimizing, S1 = currentPosition()-STEPS_PER_REV_L_R/2*td; 
      //where STEPS_PER_REV_L_R/2 is the number of steps per second, or angular velocity, 
      //and td is time delay constant that we will find iteratively.
      digitalWrite(LED_test,HIGH); //turn LED on but only for ~100ms and do not interrupt other function.
      T1=millis();
      //T2-T1 in place so that LED shines brighter when it lights up (PWM stuff, essentially)
    }
    if(preambletest_y == 1) { //during rotation, if laser shines on marker y (which it will do second)
      S2_L = L_motor.currentPosition();
      S2_R = R_motor.currentPosition();
      digitalWrite(LED_test,HIGH);
      T1=millis();
      stage=stage+1;
      delay(100); //let the motors fully stop
    }
  }
  
  if(stage == 1) { //rotate to half-angle
    L_target = (S1_L+S2_L)/2.0; R_target = (S1_R+S2_R)/2.0; //inbetween angle
    L_motor.moveTo(L_target); R_motor.moveTo(R_target);
    if(L_motor.distanceToGo() != 0) { L_motor.run(); } //turn motor
    if(R_motor.distanceToGo() != 0) { R_motor.run(); } //turn motor
    if(L_motor.distanceToGo() == 0 && R_motor.distanceToGo() == 0) {
      stage=stage+1;
      delay(100);
    }
  }

  if(stage == 2) { //move to second position
    if(targetingEnabled) { //targetingEnabled only in one loop because currentPosition must only be read at one loop   
      L_motor.moveTo(L_motor.currentPosition()-calib_steps); //moving 350mm forward
      R_motor.moveTo(R_motor.currentPosition()+calib_steps);
      targetingEnabled = 0;
    }
    if(L_motor.distanceToGo() != 0) { L_motor.run(); } //turn motor
    if(R_motor.distanceToGo() != 0) { R_motor.run(); } //turn motor
    if(L_motor.distanceToGo() == 0 && R_motor.distanceToGo() == 0) {
      stage=stage+1;
      targetingEnabled = 1;
      delay(100);
    }
  }
  
  if (stage == 3) { //second sweep
    if(targetingEnabled) { //targetingEnabled only in one loop because currentPosition must only be read at one loop
      L_motor.setCurrentPosition(0); R_motor.setCurrentPosition(0); //fresh frame of reference.
      L_target = -4000; R_target = -4000; //in reality -400 ok. -4000 better for prototyping.
      L_motor.moveTo(L_target); R_motor.moveTo(R_target);
      targetingEnabled = 0;
    }
    if(L_motor.distanceToGo() != 0) { L_motor.run(); } //turn motor
    if(R_motor.distanceToGo() != 0) { R_motor.run(); } //turn motor
    if(preambletest_b == 1) { //during rotation, if laser shines on marker b (which it will do first)
      S3_L = L_motor.currentPosition();
      S3_R = R_motor.currentPosition();
      //optimizing, S1 = currentPosition()-STEPS_PER_REV_L_R/2*td; 
      //where STEPS_PER_REV_L_R/2 is the number of steps per second, or angular velocity, 
      //and td is time delay constant that we will find iteratively.
      digitalWrite(LED_test,HIGH); //turn LED on but only for ~100ms and do not interrupt other function.
      T1=millis();
      //T2-T1 in place so that LED shines brighter when it lights up (PWM stuff, essentially)
    }
    if(preambletest_y == 1) { //during rotation, if laser shines on marker y (which it will do second)
      S4_L = L_motor.currentPosition();
      S4_R = R_motor.currentPosition();
      digitalWrite(LED_test,HIGH);
      T1=millis();
      stage=stage+1;
      delay(100); //let the motors fully stop
    }
  }

/////// Checking motors and rf modules: ////////////

//L_motor.setSpeed(STEPS_PER_REV_L_R/2);
//R_motor.setSpeed(STEPS_PER_REV_L_R/2);
//L_motor.runSpeed();
//R_motor.runSpeed();
//
//if(preambletest_b == 1)) {
//if(datain_b[2] == 4) {
//L_motor.setSpeed(STEPS_PER_REV_L_R/2);
//R_motor.setSpeed(STEPS_PER_REV_L_R/2);
//L_motor.runSpeed();
//R_motor.runSpeed();}}

//
//C_motor.setSpeed(STEPS_PER_REV_C/2);
//C_motor.runSpeed();
////////////////////////////////////////////////////

//first, if the reset button has been pressed, save current position as PO, set rotation counter to 0, and do sweep one. as you sweep, every time a rotation is done, increase rotation counter by 1.
//if yellowSerial gives some output, save current position as S1 and rotation counter as R1(for each of motor, L and R) and ignore any yellowSerial signals after this (for the timebeing).
//(ACTUALLY, MUST NOTE POSIITON FROM A FEW MILLISECONDS AGO. NOT SURE YET HOW MANY MILLISECONDS. ALSO NOT SURE HOW TO FIND POSITION AT GIVEN TIME IN MOTOR. IDEALLY WOULD BE LIKE READING A PAST VALUE ON A PLOT)
//(ACTUALLY, TO MAKE A MVP NOW WE JUST HAVE TO DO MATHS WITH ANGULAR SPEED, WHEEL RADIUS, AND SOME TIME DELAY CONSTANT WE WILL INCREMENTALLY FIND. HAVE THE LASER POINTING MIDWAY BETWEEN MARKERS AND MAKE SURE
//-------irrelevant: if Serial gives some output and it is not within 10ms of getting last signal (10ms delay because three photodiodes and light hitting any one of them will emit signal), 
//then save current position as S2 and ignore signals sent up to 10ms after this one. ---------
//if Serial gives some output, save current position as S2 and rotation counter as R2(for each of motor, L and R) and ignore any Serial signals after this (for the timebeing). also, now, stop the sweep. 
//turn backwards until you are halfway between S1 and S2 (just take Rm=(R1+R2)/2 and Sm=(S1+S2)/2. Hmm figure out exactly how you are going to do this. Fortunately, you dont' have to worry about the distance 
//the robot turns after S2 is taken. Either way, the robot is trying to get to halfway point between S1 and S2). Do this by turning backwards and subtracting 1 from rotation counter until 1 is reached,
//at which point you slow down.

//do up to this much, for now, and test.



}
