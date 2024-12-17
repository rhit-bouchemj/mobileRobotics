
/***********************************
  .ino
  Mitch Boucher & Everest Zang 12.17.24

  This program introduces the use of feedback controls and sensor pulling to create better motion algorithms for the robot.
  The robot controls will be: random wander, collide, run away, and follow
  These motion controls define the state of the robot, and will be used with the feedback to control the current state and actions.
  The primary functions created are:
  ---

  moveFigure8 - given the diameter in inches, use the moveCircle() function with direction input to create a Figure 8
  
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

  digital pin ## - left sonor pin
  digital pin ## - right sonor pin

  digital pin ## - front infared pin
  digital pin ## - back infared pin
  digital pin ## - left infared pin
  digital pin ## - right infared pin
  ^^ LIDAR????


  VV Keep?
  INSTALL THE LIBRARY
  AccelStepper Library: https://www.airspayce.com/mikem/arduino/AccelStepper/
  
  Sketch->Include Library->Manage Libraries...->AccelStepper->Include
  OR
  Sketch->Include Library->Add .ZIP Library...->AccelStepper-1.53.zip
  See PlatformIO documentation for proper way to install libraries in Visual Studio
*/

//includew all necessary libraries
#include <Arduino.h>       //include for PlatformIO Ide
#include <AccelStepper.h>  //include the stepper motor library
#include <MultiStepper.h>  //include multiple stepper motor library

//state LEDs connections
#define redLED 5            //red LED for displaying states
#define grnLED 6            //green LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
#define enableLED 13        //stepper enabled LED
int leds[3] = { 5, 6, 7 };  //array of LED pin numbers

//define motor pin numbers
#define stepperEnable 48  //stepper enable pin on stepStick
#define rtStepPin 50      //right stepper motor step pin
#define rtDirPin 51       // right stepper motor direction pin
#define ltStepPin 52      //left stepper motor step pin
#define ltDirPin 53       //left stepper motor direction pin

//define sonar pin numbers


//define infared and lidar sensors

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);  //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);   //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;                                                 //create instance to control multiple steppers at the same time

#define stepperEnTrue false  //variable for enabling stepper motor
#define stepperEnFalse true  //variable for disabling stepper motor
#define max_speed 1500       //maximum stepper motor speed
#define max_accel 10000      //maximum motor acceleration

int init_time = 10;    //time before robot moves
int pauseTime = 2500;  //pause time
int stepTime = 4000;   //delay time between high and low on step pin
int wait_time = 5000;  //delay for printing data

//define encoder pins
#define LEFT 0                        //left encoder
#define RIGHT 1                       //right encoder
const int ltEncoder = 18;             //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
const int rtEncoder = 19;             //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long encoder[2] = { 0, 0 };  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = { 0, 0 };          //variable to hold encoder speed (left, right)
int accumTicks[2] = { 0, 0 };         //variable to hold accumulated ticks since last reset

//dimention of the robot
const float wheel_width = 8.5;  //diameter of the wheel (cm)
const float wheel_base = 22;    //length between the two wheel's center (cm)

// Helper Functions

//convert travel distance (cm) to steps
int dis_to_step(int dis) {
  int step = dis * 360 / (3.14 * wheel_width * 0.45);
  return step;
}

//interrupt function to count left encoder tickes
void LwheelSpeed() {
  encoder[LEFT]++;  //count the left wheel encoder interrupts
}

//interrupt function to count right encoder ticks
void RwheelSpeed() {
  encoder[RIGHT]++;  //count the right wheel encoder interrupts
}

void allOFF() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(leds[i], LOW);
  }
}

void redOn() {
  digitalWrite(redLED, HIGH);
}
void grnOn() {
  digitalWrite(grnLED, HIGH);
}
void ylwOn() {
  digitalWrite(ylwLED, HIGH);
}


//function to set all stepper motor variables, outputs and LEDs
void init_stepper() {
  pinMode(rtStepPin, OUTPUT);                   //sets pin as output
  pinMode(rtDirPin, OUTPUT);                    //sets pin as output
  pinMode(ltStepPin, OUTPUT);                   //sets pin as output
  pinMode(ltDirPin, OUTPUT);                    //sets pin as output
  pinMode(stepperEnable, OUTPUT);               //sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);  //turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);                   //set enable LED as output
  digitalWrite(enableLED, LOW);                 //turn off enable LED
  pinMode(redLED, OUTPUT);                      //set red LED as output
  pinMode(grnLED, OUTPUT);                      //set green LED as output
  pinMode(ylwLED, OUTPUT);                      //set yellow LED as output
  digitalWrite(redLED, HIGH);                   //turn on red LED
  digitalWrite(ylwLED, HIGH);                   //turn on yellow LED
  digitalWrite(grnLED, HIGH);                   //turn on green LED
  delay(pauseTime / 5);                         //wait 0.5 seconds
  digitalWrite(redLED, LOW);                    //turn off red LED
  digitalWrite(ylwLED, LOW);                    //turn off yellow LED
  digitalWrite(grnLED, LOW);                    //turn off green LED

  stepperRight.setMaxSpeed(max_speed);         //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);     //set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_speed);          //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);      //set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);           //add right motor to MultiStepper
  steppers.addStepper(stepperLeft);            //add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);  //turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);               //turn on enable LED
}

//function prints encoder data to serial monitor
void print_encoder_data() {
  static unsigned long timer = 0;                            //print manager timer
  if (millis() - timer > 100) {                              //print encoder data every 100 ms or so
    lastSpeed[LEFT] = encoder[LEFT];                         //record the latest left speed value
    lastSpeed[RIGHT] = encoder[RIGHT];                       //record the latest right speed value
    accumTicks[LEFT] = accumTicks[LEFT] + encoder[LEFT];     //record accumulated left ticks
    accumTicks[RIGHT] = accumTicks[RIGHT] + encoder[RIGHT];  //record accumulated right ticks
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
    encoder[LEFT] = 0;   //clear the left encoder data buffer
    encoder[RIGHT] = 0;  //clear the right encoder data buffer
    timer = millis();    //record current time since program started
  }
}

/*function to run both wheels to a position at speed*/
void runAtSpeedToPosition() {
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
}

/*function to run both wheels continuously at a speed*/
void runAtSpeed(void) {
  while (stepperRight.runSpeed() || stepperLeft.runSpeed()) {
  }
}

/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it
*/
void runToStop(void) {
  int runNow = 1;
  int rightStopped = 0;
  int leftStopped = 0;

  while (runNow) {
    if (!stepperRight.run()) {
      rightStopped = 1;
      stepperRight.stop();  //stop right motor
    }
    if (!stepperLeft.run()) {
      leftStopped = 1;
      stepperLeft.stop();  //stop left motor
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
  int offset = 30 * degree / 90;
  int dis = degree * 3.14 * wheel_base / 180;
  int signOfDegree = constrain(degree, -1, 1);
  int steps = dis_to_step(dis) + offset;
  Serial.println(steps);
  if (!wheel) {
    stepperRight.moveTo(steps);
    stepperRight.setSpeed(constrain(degree, -1, 1) * speed);
    stepperRight.runSpeedToPosition();
    runToStop();
  } else {
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
  if (degree < 0) {
    offset = 15 * degree / 90;
  } else {
    offset = 15 * degree / 90;
  }
  int signOfDegree = constrain(degree, -1, 1);
  int dis = degree * 3.14 * wheel_base / (2 * 180);
  int steps = dis_to_step(dis); //+ offset;
  Serial.println("Degrees:");
  Serial.println(degree);
  Serial.println(steps);
  stepperRight.moveTo(steps);
  stepperLeft.moveTo(-steps);
  stepperRight.setMaxSpeed(speed);
  stepperLeft.setMaxSpeed(speed);
  steppers.runSpeedToPosition();
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
}

/*
  turn in certain degree with certain radius (cm) left of robot is negative
*/
void turn(int degree, int radius, int speed) {
  int dis_l = degree * 3.14 * (abs(radius) + constrain(radius, -1, 1) * wheel_base / 2) / 180;
  int dis_r = degree * 3.14 * (abs(radius) - constrain(radius, -1, 1) * wheel_base / 2) / 180;
  int dis_m = degree * 3.14 * (abs(radius)) / 180;
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
  go backward for certain distance (cm) with certain speed
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
void GoToAngle(int degree) {
  spin(degree, 350);
}

/*
  go to certain location (x,y) in cm
*/
void GoToGoal(long x, long y) {
  float radian = atan2(y, x);
  int degree = radian * 57.3;
  long dis = sqrt(sq(x) + sq(y));
  spin(degree, 350);
  delay(250);
  forward(dis, 400);
  delay(250);
}


//Lab 2 commands

void prepForward(int distance, int speed) {
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  int steps = dis_to_step(distance);
  stepperRight.moveTo(steps);
  stepperLeft.moveTo(steps);
  Serial.println("Steps: ");
  Serial.println(steps);
  stepperRight.setSpeed(speed);
  stepperLeft.setSpeed(speed);
}


void randomMovement(int randomNumber) {
  GoToAngle(randomNumber % 360 - 180);
  Serial.println("Past Angle");
  prepForward(randomNumber % 40 + 1, randomNumber % 400 + 100);
}

/*
  The robot will move randomly repeatedly, changing its angle and driving forward a random distance (unless it detects an object too close)
*/
void randomWander() {
  allOFF();                   //turn off all LEDs
  digitalWrite(grnLED, HIGH);  //turn on green LED

  int maxVal = 1000;                  //maximum value for random number
  randomSeed(analogRead(0));    //generate a new random number each time called
  int randomNumber;  //generate a random number up to the maximum value
  
  while(1) //Bigger loop that continues to randomize the movement // TODO: add "!close" (continue random movement till wall)
  {
    if(stepperLeft.distanceToGo() == 0 && stepperRight.distanceToGo() == 0) 
    {
      randomNumber = random(maxVal);  //generate a random number up to the maximum value
      Serial.println("Randomize: ");
      Serial.println(randomNumber);
      delay(1000);
      randomMovement(randomNumber);
    }
    stepperRight.run();  //Robot begins moving
    stepperLeft.run(); 
    Serial.print("running");
  }
  stepperLeft.stop(); //Stop robot after completing distance
  stepperRight.stop();
  allOFF();  //turn off all LEDs
}

void randomWanderNoSpin(){
  allOFF();
  digitalWrite(grnLED, HIGH);  //turn on green LED

  int maxVal = 6000;                  //maximum value for random number
  randomSeed(analogRead(0));    //generate a new random number each time called
  while(1){
    if(stepperLeft.distanceToGo() == 0){      
      int randomSteps = random(maxVal) + 5;
      int randomMaxSpeed = random(maxVal) % 400 + 250;
      int randomAcc = random(maxVal) % 200 + 150;
      Serial.println("steps:");
      Serial.println(randomSteps);
      Serial.println("speed:");
      Serial.println(randomMaxSpeed);
      stepperLeft.moveTo(randomSteps);
      stepperLeft.setMaxSpeed(randomMaxSpeed);
      stepperLeft.setAcceleration(randomAcc);
    }
    if(stepperRight.distanceToGo() == 0){      
      int randomSteps = random(maxVal) + 5;
      int randomMaxSpeed = random(maxVal) % 400 + 250;
      int randomAcc = random(maxVal) % 200 + 150;
      stepperRight.moveTo(randomSteps);
      stepperRight.setMaxSpeed(randomMaxSpeed);
      stepperRight.setAcceleration(randomAcc);
    }
    stepperLeft.run();
    stepperRight.run();
  }
}

/*

*/
void runAway() {

}

/*
*/
void collide() {
  allOFF(); //Turn off all LEDs
  redOn();  //Turn on red LED
  // while(sensor != close) {

  // }

  allOFF(); //Turn off all LEDs
}



//// MAIN
void setup() {
  delay(2000);
  int baudrate = 9600;  //serial monitor baud rate'
  init_stepper();       //set up stepper motor

  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);  //init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);  //init the interrupt mode for the right encoder


  Serial.begin(baudrate);  //start serial monitor communication
  Serial.println("Robot starting...Put ON TEST STAND");
  delay(init_time);  //always wait 5 seconds before the robot moves
  //randomWander();
  randomWanderNoSpin();
  //GoToAngle(450);
  
}

void loop() {


  //Uncomment to read Encoder Data (uncomment to read on serial monitor)
  //print_encoder_data();   //prints encoder data



  //delay(wait_time);  //wait to move robot or read data
}