/* 
  Author: Mitch Boucher and Everes Zhang
  Created: 12.23.2025

  Description: 
  This program has multiple functions to move the robot based on various lidar sensor data
  smart movement functions are ones that use a state machine to switch between different kinds of movement control of the robot

  Lidar data is measured and transfered from the M4 processor core
*/

// read about multicore: https://docs.arduino.cc/tutorials/giga-r1-wifi/giga-dual-core
#include "Arduino.h"
#include "RPC.h"
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
#define lf_sonar 4
#define rt_sonar 3

//define infared and lidar sensors
#define ft_lidar 8
#define bk_lidar 9
#define lf_lidar 10
#define rt_lidar 11

// a struct to hold lidar data
struct lidar {
  // this can easily be extended to contain sonar data as well
  int front;
  int back;
  int left;
  int right;
  int closeObj;
  // this defines some helper functions that allow RPC to send our struct (I found this on a random forum)
  MSGPACK_DEFINE_ARRAY(front, back, left, right);  //https://stackoverflow.com/questions/37322145/msgpack-to-pack-structures https://www.appsloveworld.com/cplus/100/391/msgpack-to-pack-structures
} dist;

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);  //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);   //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;

#define stepperEnTrue false  //variable for enabling stepper motor
#define stepperEnFalse true  //variable for disabling stepper motor
#define max_speed 1500       //maximum stepper motor speed
#define max_accel 10000      //maximum motor acceleration

bool object = 0;
//dimention of the robot
const float wheel_width = 8.5;  //diameter of the wheel (cm)
const float wheel_base = 22;    //length between the two wheel's center (cm)
lidar data;



// a struct to hold lidar data
struct sonar {
  // this can easily be extended to contain sonar data as well
  int left;
  int right;
  // this defines some helper functions that allow RPC to send our struct (I found this on a random forum)
  MSGPACK_DEFINE_ARRAY(left, right);  //https://stackoverflow.com/questions/37322145/msgpack-to-pack-structures https://www.appsloveworld.com/cplus/100/391/msgpack-to-pack-structures
} dist2;

// read_lidars is the function used to get lidar data to the M7
struct lidar read_lidars() {
  return dist;
}

// read_lidars is the function used to get lidar data to the M7
struct sonar read_sonars() {
  return dist2;
}

// reads a lidar given a pin
int read_lidar(int pin) {
  int d;
  int16_t t = pulseIn(pin, HIGH);
  d = (t - 1000) * 3 / 40;
  if (t == 0 || t > 1850 || d < 0) { d = 0; }
  return d;
}

// reads a sonar given a pin
int read_sonar(int pin) {
  float velocity((331.5 + 0.6 * (float)(20)) * 100 / 1000000.0);
  uint16_t distance, pulseWidthUs;

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  digitalWrite(pin, HIGH);            //Set the trig pin High
  delayMicroseconds(10);              //Delay of 10 microseconds
  digitalWrite(pin, LOW);             //Set the trig pin Low
  pinMode(pin, INPUT);                //Set the pin to input mode
  pulseWidthUs = pulseIn(pin, HIGH);  //Detect the high level time on the echo pin, the output high level time represents the ultrasonic flight time (unit: us)
  distance = pulseWidthUs * velocity / 2.0;
  if (distance < 0 || distance > 50) { distance = 0; }
  return distance;
}
//read sensor data from M4 and write to M7
void read_sensors() {
  // read lidar data from struct
  data = RPC.call("read_lidars").as<struct lidar>();
  // struct sonar data2 = RPC.call("read_sonars").as<struct sonar>();
  // print lidar data
  Serial.print("lidar: ");
  Serial.print(data.front);
  Serial.print(", ");
  Serial.print(data.back);
  Serial.print(", ");
  Serial.print(data.left);
  Serial.print(", ");
  Serial.println(data.right);
  Serial.println(" ");
}

void allOff() {
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
void redOff() {
  digitalWrite(redLED, LOW);
}
void grnOff() {
  digitalWrite(grnLED, LOW);
}
void ylwOff() {
  digitalWrite(ylwLED, LOW);
}

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
  delay(2500 / 5);                              //wait 0.5 seconds
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

//convert travel distance (cm) to steps
int dis_to_step(int dis) {
  int step = dis * 360 / (3.14 * wheel_width * 0.45);
  return step;
}

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

void spin(int degree, int speed) {
  int offset = 0;
  if (degree < 0) {
    offset = 15 * degree / 90;
  } else {
    offset = 15 * degree / 90;
  }
  int signOfDegree = constrain(degree, -1, 1);
  int dis = degree * 3.14 * wheel_base / (2 * 180);
  int steps = dis_to_step(dis);  //+ offset;
  stepperRight.moveTo(steps);
  stepperLeft.moveTo(-steps);
  /*
  stepperRight.setMaxSpeed(speed);
  stepperLeft.setMaxSpeed(speed);
  stepperLeft.setAcceleration(200);
  stepperLeft.setAcceleration(200);
  */
  steppers.runSpeedToPosition();
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
}

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

void GoToGoal(long x, long y) {
  float radian = atan2(y, x);
  int degree = radian * 57.3;
  long dis = sqrt(sq(x) + sq(y));
  if (degree <= 90 && degree >= -90) {
    spin(degree, 500);
    delay(50);
    forward(dis, 300);
  } else {
    spin(constrain(degree, -1, 1) * (-180) + degree, 500);
    delay(50);
    reverse(dis, 300);
  }
}

void prepMovement(int maxVal) {
  if (stepperLeft.distanceToGo() == 0) {
    stepperLeft.setCurrentPosition(2000);
    int randomSteps = random(maxVal) + 5;
    int randomMaxSpeed = random(maxVal) % 400 + 250;
    int randomAcc = random(maxVal) % 200 + 150;
    stepperLeft.moveTo(randomSteps);
    stepperLeft.setMaxSpeed(randomMaxSpeed);
    stepperLeft.setAcceleration(randomAcc);
  }
  if (stepperRight.distanceToGo() == 0) {
    stepperRight.setCurrentPosition(2000);
    int randomSteps = random(maxVal) + 5;
    int randomMaxSpeed = random(maxVal) % 400 + 250;
    int randomAcc = random(maxVal) % 200 + 150;
    stepperRight.moveTo(randomSteps);
    stepperRight.setMaxSpeed(randomMaxSpeed);
    stepperRight.setAcceleration(randomAcc);
  }
}

/*
  Prepare movement step (such as numberof steps, max speed, and acceleration)
  the movement parameters will be equal for both wheels resulting in forward movement
*/
void prepForward(int maxVal) {
  if(stepperLeft.distanceToGo() == 0){      
      int randomSteps = random(maxVal) + 5;
      int randomMaxSpeed = random(maxVal) % 400 + 250;
      int randomAcc = random(maxVal) % 200 + 150;
      stepperLeft.moveTo(randomSteps);
      stepperLeft.setMaxSpeed(randomMaxSpeed);
      stepperLeft.setAcceleration(randomAcc);
      stepperRight.moveTo(randomSteps);
      stepperRight.setMaxSpeed(randomMaxSpeed);
      stepperRight.setAcceleration(randomAcc);
  }
}

/*
  Change the 
*/
void randomWanderNoSpin() {
  allOff();
  grnOn();  //turnx on green LED

  int maxVal = 6000;  //maximum value for random number
  // while(1){
  prepMovement(maxVal);
  // }
  stepperLeft.run();
  stepperRight.run();
}

void collide() {
  allOff();              //Turn off all LEDs
  data = RPC.call("read_lidars").as<struct lidar>();
  int distThresh = 20;
  while (!object) {      //sensor != close)  //Move forward while not sensing wall
    grnOn();             //Turn on green LED
    prepMovement(6000);  //prepare to move forward
    stepperLeft.run();   //increment left motor
    stepperRight.run();  //increment right motor
    data = RPC.call("read_lidars").as<struct lidar>();
    if (data.right != 0 || data.back != 0 || data.front != 0 || data.left != 0) {
      if (data.right < distThresh || data.back < distThresh || data.front < distThresh || data.left < distThresh) {
        object = 1;
        read_sensors();
        grnOff();
      }
    }
  }
  redOn();  //turn on red LED
}

void runAway() {
  int p = 2;
  int distThresh = 20;
  allOff();
  ylwOn();
  data = RPC.call("read_lidars").as<struct lidar>();

  // if(data.closeObj <= distThresh) {
  if (data.right != 0 || data.back != 0 || data.front != 0 || data.left != 0) {  // object present
    delay(100);
    data = RPC.call("read_lidars").as<struct lidar>();  // read again incase of missing data
    bool FB = 0;                                                     //not on both front and back
    bool LR = 0;                                                     //not on both left and right

    if (data.right > distThresh) {  // if further then 30 cm, ignore
      data.right = 0;
    }
    if (data.back > distThresh) {
      data.back = 0;
    }
    if (data.left > distThresh) {
      data.left = 0;
    }
    if (data.front > distThresh) {
      data.front = 0;
    }
    if (data.front <= distThresh && data.back <= distThresh && data.front && data.back) {  // both front and back
      data.front = 0;   //ignore front and back sensor data
      data.back = 0;
      FB = 1;
      if (!data.left && !data.right) {  // turn to right sensor and move if free space
        data.left = 4;    // fake object to left <-- force to right
      }
    }
    if (data.left <= distThresh && data.right <= distThresh && data.left && data.right) {  // both left and right
      data.left = 0;    //ignore left and right sensor data
      data.right = 0;
      LR = 1;
      if (!data.front && !data.back) {  // turn to front sensor and move if free space
        data.back = 4;    // fake object to back
      }
    }

    if (LR && FB) {  //in a box
      //do nothing
    } else {
      int xSens = data.front - data.back;  //total X sensor = front - back
      int ySens = data.left - data.right;  //total Y sensor = left - right <-- Based on directions listed in lab 1

      read_sensors();
      Serial.println(xSens);
      Serial.println(ySens);
      Serial.println(constrain(xSens, -1, 1) * ((-30) + xSens));
      Serial.println(constrain(ySens, -1, 1) * ((-30) + ySens));
      Serial.println("-----");

      stepperLeft.setCurrentPosition(0);    //reset position from wander
      stepperRight.setCurrentPosition(0);
      GoToGoal((constrain(xSens, -1, 1) * ((-distThresh) + xSens)) * p, (constrain(ySens, -1, 1) * ((-distThresh) + ySens)) * p);
      //Test sense data
      delay(500);
    }
  }
  object = 0;
}

void follow(int distance) {
  allOff();
  ylwOn();
  grnOn();
  struct lidar data = RPC.call("read_lidars").as<struct lidar>();
  if (data.front){  //object in front of robot
    stepperLeft.setCurrentPosition(0);    //reset world position
    stepperRight.setCurrentPosition(0);
    int delta = data.front - distance;        // difference between intended distance and actual
    int sign = constrain(delta, -1, 1);
    stepperLeft.moveTo(sign*5000);   // determine if moving forward or backword based on delta
    stepperRight.moveTo(sign*5000);
    stepperLeft.setSpeed(constrain(sign*sq(delta)*25, -700, 700));
    stepperRight.setSpeed(constrain(sign*sq(delta)*25, -700, 700));
    stepperRight.run();
    stepperLeft.run();

    // Test Delta data
    Serial.print(constrain(delta, -1, 1)*5000);
    Serial.print("  ");
    Serial.print(delta);
    Serial.print("  ");
    Serial.println(data.front);
  }
  else
  {
    object = 0;
  }
}

/*
  Randomly wander until detecting an object, then proceed to run away from it
*/
void smartWander(int threshold) {
  allOff();
  data = RPC.call("read_lidars").as<struct lidar>();
  collide();      //wander until detects object
  delay(1000);    
  if(object)
  {
    Serial.print("in while");
    data = RPC.call("read_lidars").as<struct lidar>();
    runAway();    //move opposite of object while detecting

    // data = RPC.call("read_lidars").as<struct lidar>();  
    // if (data.closeObj > threshold)    //if an object is farther than the threshold
    // {
    //   object = 0;                     //no longer count an object as present
    // }
  }
  Serial.print("out while");
}

/*
  Randomly wander until detecting an object, stop, then begin following the object at a set distance
*/
void smartFollow() {
  allOff();              //Turn off all LEDs
  collide();             //randomly wanders until it detects 
  delay(100);
  while (!object) {      //sensor != close) { //Move forward while not sensing wall
    follow(15);
  }
  allOff();
}

//setup function w/infinite loops to send/recv data
void setup() {
  RPC.begin();
  init_stepper();

  Serial.begin(9600);
  delay(500);

  // Change these to suit your stepper if you want
  // stepperLeft.setMaxSpeed(100);
  // stepperLeft.setAcceleration(20);
  // stepperLeft.moveTo(500);
}

// loop() is never called as setup() never returns
// this may need to be modified to run the state machine.
// consider usingnamespace rtos Threads as seen in previous example
void loop() {
  smartWander(20);
  // runAway();
  // read_sensors();
  //struct lidar data = RPC.call("read_lidars").as<struct lidar>();
  // Serial.println(data.front);
}
