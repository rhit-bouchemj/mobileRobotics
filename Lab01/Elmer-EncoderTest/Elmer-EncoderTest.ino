
/*
  NOTE:
   THIS IS THE STANDARD FOR HOW TO PROPERLY COMMENT CODE
   Header comment has program, name, author name, date created
   Header comment has brief description of what program does
   Header comment has list of key functions and variables created with decription
   There are sufficient in line and block comments in the body of the program
   Variables and functions have logical, intuitive names
   Functions are used to improve modularity, clarity, and readability
***********************************
  RobotIntro.ino
  Carlotta Berry 11.21.16

  This program will introduce using the stepper motor library to create motion algorithms for the robot.
  The motions will be go to angle, go to goal, move in a circle, square, figure eight and teleoperation (stop, forward, spin, reverse, turn)
  It will also include wireless commmunication for remote control of the robot by using a game controller or serial monitor.
  The primary functions created are
  moveCircle - given the diameter in inches and direction of clockwise or counterclockwise, move the robot in a circle with that diameter
  moveFigure8 - given the diameter in inches, use the moveCircle() function with direction input to create a Figure 8
  forward, reverse - both wheels move with same velocity, same direction
  pivot- one wheel stationary, one wheel moves forward or back
  spin - both wheels move with same velocity opposite direction
  turn - both wheels move with same direction different velocity
  stop -both wheels stationary

  Interrupts
  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  https://www.arduino.cc/en/Tutorial/CurieTimer1Interrupt
  https://playground.arduino.cc/code/timer1
  https://playground.arduino.cc/Main/TimerPWMCheatsheet
  http://arduinoinfo.mywikis.net/wiki/HOME

  Hardware Connections:
  Arduino pin mappings: https://docs.arduino.cc/tutorials/giga-r1-wifi/cheat-sheet#pins
  A4988 Stepper Motor Driver Pinout: https://www.pololu.com/product/1182 

  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 50 - right stepper motor step pin
  digital pin 51 - right stepper motor direction pin
  digital pin 52 - left stepper motor step pin
  digital pin 53 - left stepper motor direction pin
  digital pin 13 - enable LED on microcontroller

  digital pin 5 - red LED in series with 220 ohm resistor
  digital pin 6 - green LED in series with 220 ohm resistor
  digital pin 7 - yellow LED in series with 220 ohm resistor

  digital pin 18 - left encoder pin
  digital pin 19 - right encoder pin

  INSTALL THE LIBRARY
  AccelStepper Library: https://www.airspayce.com/mikem/arduino/AccelStepper/
  
  Sketch->Include Library->Manage Libraries...->AccelStepper->Include
  OR
  Sketch->Include Library->Add .ZIP Library...->AccelStepper-1.53.zip
  See PlatformIO documentation for proper way to install libraries in Visual Studio
*/

//includew all necessary libraries
#include <Arduino.h>//include for PlatformIO Ide
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library

//state LEDs connections
#define redLED 5            //red LED for displaying states
#define grnLED 6            //green LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
#define enableLED 13        //stepper enabled LED
int leds[3] = {5,6,7};      //array of LED pin numbers

//define motor pin numbers
#define stepperEnable 48    //stepper enable pin on stepStick 
#define rtStepPin 50 //right stepper motor step pin 
#define rtDirPin 51  // right stepper motor direction pin 
#define ltStepPin 52 //left stepper motor step pin 
#define ltDirPin 53  //left stepper motor direction pin 

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor
#define max_speed 1500 //maximum stepper motor speed
#define max_accel 10000 //maximum motor acceleration

int init_time = 10;   //time before robot moves
int pauseTime = 2500;   //pause time
int stepTime = 4000;     //delay time between high and low on step pin
int wait_time = 5000;   //delay for printing data

//define encoder pins
#define LEFT 0        //left encoder
#define RIGHT 1       //right encoder
const int ltEncoder = 18;        //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
const int rtEncoder = 19;        //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long encoder[2] = {0, 0};  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = {0, 0};          //variable to hold encoder speed (left, right)
int accumTicks[2] = {0, 0};         //variable to hold accumulated ticks since last reset

//dimention of the robot
const float wheel_width = 8.5;  //diameter of the wheel (cm)
const float wheel_base = 22;  //length between the two wheel's center (cm)

// Helper Functions

//convert travel distance (cm) to steps
int dis_to_step(int dis){
  int step = dis * 360 / (3.14*wheel_width*0.45);
  return step;
}

//interrupt function to count left encoder tickes
void LwheelSpeed()
{
  encoder[LEFT] ++;  //count the left wheel encoder interrupts
}

//interrupt function to count right encoder ticks
void RwheelSpeed()
{
  encoder[RIGHT] ++; //count the right wheel encoder interrupts
}

void allOFF(){
  for (int i = 0;i<3;i++){
    digitalWrite(leds[i],LOW);
  }
}

//function to set all stepper motor variables, outputs and LEDs
void init_stepper(){
  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);//set enable LED as output
  digitalWrite(enableLED, LOW);//turn off enable LED
  pinMode(redLED, OUTPUT);//set red LED as output
  pinMode(grnLED, OUTPUT);//set green LED as output
  pinMode(ylwLED, OUTPUT);//set yellow LED as output
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  delay(pauseTime / 5); //wait 0.5 seconds
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(grnLED, LOW);//turn off green LED

  stepperRight.setMaxSpeed(max_speed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_speed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
}

//function prints encoder data to serial monitor
void print_encoder_data() {
  static unsigned long timer = 0;                           //print manager timer
  if (millis() - timer > 100) {                             //print encoder data every 100 ms or so
    lastSpeed[LEFT] = encoder[LEFT];                        //record the latest left speed value
    lastSpeed[RIGHT] = encoder[RIGHT];                      //record the latest right speed value
    accumTicks[LEFT] = accumTicks[LEFT] + encoder[LEFT];    //record accumulated left ticks
    accumTicks[RIGHT] = accumTicks[RIGHT] + encoder[RIGHT]; //record accumulated right ticks
    Serial.println("Encoder value:");
    Serial.print("\tLeft:\t");
    Serial.print(encoder[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(encoder[RIGHT]);
    Serial.println("Accumulated Ticks: ");
    Serial.print("\tLeft:\t");
    Serial.print(accumTicks[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(accumTicks[RIGHT]);
    encoder[LEFT] = 0;                          //clear the left encoder data buffer
    encoder[RIGHT] = 0;                         //clear the right encoder data buffer
    timer = millis();                           //record current time since program started
  }
}
  
/*function to run both wheels to a position at speed*/
void runAtSpeedToPosition() {
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
}

/*function to run both wheels continuously at a speed*/
void runAtSpeed ( void ) {
  while (stepperRight.runSpeed() || stepperLeft.runSpeed()) {
  }
}

/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it
*/
void runToStop ( void ) {
  int runNow = 1;
  int rightStopped = 0;
  int leftStopped = 0;

  while (runNow) {
    if (!stepperRight.run()) {
      rightStopped = 1;
      stepperRight.stop();//stop right motor
    }
    if (!stepperLeft.run()) {
      leftStopped = 1;
      stepperLeft.stop();//stop ledt motor
    }
    if (rightStopped && leftStopped) {
      runNow = 0;
    }
  }
}

/*
  Robot pivot on one wheel. left wheel is 0 and right wheel is 1. degree can be both positive and negative
*/
void pivot(bool wheel, int degree, int speed) {
  int offset = 30*degree/90;
  int dis = degree * 3.14 * wheel_base / 180;
  int signOfDegree = constrain(degree, -1, 1);
  int steps = dis_to_step(dis) + offset;
  Serial.println(steps);
  if (!wheel){
	  stepperRight.moveTo(steps);
	  stepperRight.setSpeed(constrain(degree, -1, 1) * speed);
	  stepperRight.runSpeedToPosition();
    runToStop();
  }else{
    stepperLeft.moveTo(steps);
	  stepperLeft.setSpeed(constrain(degree, -1, 1) * speed);
	  stepperLeft.runSpeedToPosition();
    runToStop();
  }
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
}

/*
  spin in space for certain degree, recommand 400 speed
*/
void spin(int degree, int speed) {
  int offset = 0;
  if (degree < 0){
    offset = 15*degree/90;
  }else{
    offset = 15*degree/90;
  }
  int signOfDegree = constrain(degree, -1, 1);
  int dis = degree * 3.14 * wheel_base / (2*180);
  int steps = dis_to_step(dis) + offset;
  Serial.println(steps);
  stepperRight.moveTo(steps);
  stepperLeft.moveTo(-steps);
  stepperRight.setSpeed(signOfDegree * speed);
  stepperLeft.setSpeed(-signOfDegree * speed);
  steppers.runSpeedToPosition();
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
}

/*
  turn in certain degree with certain radius (cm) left of robot is negative
*/
void turn(int degree, int radius, int speed) {
  int dis_l = degree * 3.14 * (abs(radius) + constrain(radius, -1, 1)*wheel_base/2)/180;
  int dis_r = degree * 3.14 * (abs(radius) - constrain(radius, -1, 1)*wheel_base/2)/180;
  int dis_m = degree * 3.14 * (abs(radius))/180;
  float t = float(dis_m) / float(speed);
  int step_l = dis_to_step(dis_l);
  int step_r = dis_to_step(dis_r);
  int speed_l = dis_l / t;
  int speed_r = dis_r / t;
  /*
  Serial.println(t);
  Serial.println(speed_l);
  Serial.println(speed_r);
  */
  stepperRight.moveTo(step_r);
  stepperLeft.moveTo(step_l);
  stepperRight.setSpeed(constrain(dis_r, -1, 1) * speed_r);
  stepperLeft.setSpeed(constrain(dis_l, -1, 1) * speed_l);
  steppers.runSpeedToPosition();
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);

}
/*
  go foward for certain distance (cm) with certain speed
*/
void forward(int distance, int speed) {
  int steps = dis_to_step(distance);
  stepperRight.moveTo(steps);
  stepperLeft.moveTo(steps);
  stepperRight.setSpeed(speed);
  stepperLeft.setSpeed(speed);
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
  runToStop();
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
}
/*
  go backward for certain distance (cm) with certain speed (0-1000, 300 ideal)
*/
void reverse(int distance, int speed) {
  int steps = dis_to_step(distance);
  stepperRight.moveTo(-steps);
  stepperLeft.moveTo(-steps);
  stepperRight.setSpeed(-speed);
  stepperLeft.setSpeed(-speed);
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
  runToStop();
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
}
/*
  stop any movement
*/
void stop() {
  stepperRight.stop();
  stepperLeft.stop();
}

/* 
  go to certain angle
*/
void GoToAngle(int degree){
  spin(degree, 350);
}

/*
  go to certain location (x,y) in cm
*/
void GoToGoal(long x, long y){
  float radian = atan2(y, x);
  int degree = radian * 57.3;
  long dis = sqrt(sq(x) + sq(y));
  spin(degree, 350);
  delay(250);
  forward(dis, 400);
  delay(250);
}

/*
  move square with certain length
  */
void moveSquare(int length){
  for(int i = 0; i < 4; i++){
    forward(length, 300);
    delay(250);
    spin(90, 350);
    delay(250);
  }
}

/*
  run in a circle. diam (cm): left of robot is negative, right is positive
*/
void moveCircle(int diam, int speed) {
  turn(360, diam/2, speed);
}

/*
  The moveFigure8() function takes the diameter in cm as the input. It uses the moveCircle() function
  twice with 2 different direcitons to create a figure 8 with circles of the given diameter.
*/
void moveFigure8(int diam) {
  turn(360, diam/2, 800);
  turn(360, -diam/2, 800);
}

/*
  Move the robot wheels a part of a rotation by steps (0-800) w/ speed (0-1000, ideal 300, negative to reverse)
*/
void moveRotation(int steps, int speed) {
  stepperRight.moveTo(steps);
  stepperLeft.moveTo(steps);
  stepperRight.setSpeed(speed);
  stepperLeft.setSpeed(speed);
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
  runToStop();
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
}


//// MAIN
void setup()
{
  int baudrate = 9600; //serial monitor baud rate'
  init_stepper(); //set up stepper motor

  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);    //init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);   //init the interrupt mode for the right encoder


  Serial.begin(baudrate);     //start serial monitor communication
  Serial.println("Robot starting...Put ON TEST STAND");
  delay(pauseTime); //always wait 2.5 seconds before the robot moves
}

void loop()
{
  moveRotation(200, 300);
  //Uncomment to read Encoder Data (uncomment to read on serial monitor)
  print_encoder_data();   //prints encoder data
  delay(wait_time*2);               //wait to move robot or read data

  moveRotation(400, 300);
  //Uncomment to read Encoder Data (uncomment to read on serial monitor)
  print_encoder_data();   //prints encoder data
  delay(wait_time*2);               //wait to move robot or read data
  
  moveRotation(800, 300);
  //Uncomment to read Encoder Data (uncomment to read on serial monitor)
  print_encoder_data();   //prints encoder data
  delay(wait_time*2);               //wait to move robot or read data

  moveRotation(1600, 300);
  //Uncomment to read Encoder Data (uncomment to read on serial monitor)
  print_encoder_data();   //prints encoder data
  delay(wait_time*2);               //wait to move robot or read data
}