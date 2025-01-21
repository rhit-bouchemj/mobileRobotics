//Justin Dewitt 12.12.23
// read about multicore: https://docs.arduino.cc/tutorials/giga-r1-wifi/giga-dual-core
//This code will show how to run the same code on the M4 and M7 to server and client, respectively
//M4 will read sensor data and send to M7 to run the state machine

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

struct lidar data;

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
  Serial.println(", ");
  Serial.println(data.closeObj);
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
  // stepperLeft.setMaxSpeed(speed);
  // stepperLeft.setAcceleration(100.0);
  // stepperLeft.runToNewPosition(-steps);
  // stepperRight.setMaxSpeed(speed);
  // stepperRight.setAcceleration(100.0);
  // stepperRight.runToNewPosition(steps);
  // steppers.runSpeedToPosition();

  stepperLeft.setMaxSpeed(speed);
  stepperRight.setMaxSpeed(speed);
  stepperLeft.setAcceleration(100.0);
  stepperRight.setAcceleration(100.0);

  stepperRight.moveTo(steps);
  stepperLeft.moveTo(-steps);

  while(stepperLeft.distanceToGo() != 0 && stepperRight.distanceToGo() != 0)
  {
    stepperLeft.run();
    stepperRight.run();
  }  
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
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);

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

/*
  When sensing an object in any direction the robot will stop.
  The distance threshold that determines how close to an object counts is a parameter in the command.
*/
void collide(int distThresh) {
  allOff();              //Turn off all LEDs
  // int distThresh = 15;
  while (!object) {      //sensor != close)  //Move forward while not sensing wall
    grnOn();             //Turn on green LED
    prepMovement(6000);  //prepare to move forward
    stepperLeft.run();   //increment left motor
    stepperRight.run();  //increment right motor
    data = RPC.call("read_lidars").as<struct lidar>();
    if (data.right != 0 || data.back != 0 || data.front != 0 || data.left != 0) {
      if (data.right < distThresh || data.back < distThresh || data.front < distThresh || data.left < distThresh) {
        object = 1;
        grnOff();
      }
    }
  }
  redOn();  //turn on red LED
  data = RPC.call("read_lidars").as<struct lidar>();
  if (data.right == 0 & data.back == 0 & data.front == 0 & data.left == 0) {
    object = 0;
    redOff();
  } else if (data.right >= distThresh & data.back >= distThresh & data.front >= distThresh & data.left >= distThresh) {
    object = 0;
    redOff();
  }
}

/*
  When sensing an object in any direction the robot will try to get away from the object at a distance that is proportional to how close to the object that the robot is.
  The robot will try to run away in a direction opposite of where the objects were detected.
  It uses a deadzone of 10% of the distance threshold.
  If the robot detects an object in 2 polar directions it will negate them.
  If the robot detects an object in 4 directions it will do nothing
  The distance threshold that determines how far to run away from is a parameter in the command.
*/
void runAway(int distThresh) {
  int p = 2;
  int tolerance = distThresh/10;
  allOff();
  ylwOn();
  data = RPC.call("read_lidars").as<struct lidar>();

  if (data.closeObj < distThresh - tolerance) {  // object present
    delay(100);
    data = RPC.call("read_lidars").as<struct lidar>();  // read again incase of missing data
    bool FB = 0;                                                     //not on both front and back
    bool LR = 0;                                                     //not on both left and right
    // int distThresh = 20;


    if (data.front <= distThresh && data.back <= distThresh && data.back && data.front) {  // both front and back
      data.front = 0;   //ignore front and back sensor data
      data.back = 0;
      FB = 1;
      if (!data.left && !data.right) {  // turn to right sensor and move if free space
        data.left = 4;    // fake object to left <-- force to right
      }
    }
    else
    {
      if (data.front > distThresh - tolerance) {  // if further then 30 cm, ignore
        data.front = 0;
      }
      if (data.back > distThresh - tolerance) {
        data.back = 0;
      }
    }

    //clear data above threshold
    if (data.left <= distThresh && data.right <= distThresh && data.left && data.right) {  // both left and right
      data.left = 0;    //ignore left and right sensor data
      data.right = 0;
      LR = 1;
      if (!data.front && !data.back) {  // turn to front sensor and move if free space
        data.back = 4;    // fake object to back
      }
    }
    else
    {
      if (data.left > distThresh - tolerance) {
        data.left = 0;
      }
      if (data.right > distThresh - tolerance) {
        data.front = 0;
    }
    }

    if (LR && FB) {  //in a box
      //do nothing
    } else {
      int xSens = data.front - data.back;  //total X sensor = front - back
      int ySens = data.left - data.right;  //total Y sensor = left - right <-- Based on directions listed in lab 1
      GoToGoal((constrain(xSens, -1, 1) * (-distThresh) + xSens) * p, (constrain(ySens, -1, 1) * (-distThresh) + ySens) * p);
      
      //Test sense data
      // Serial.println(xSens);
      // Serial.println(ySens);
      // Serial.println(constrain(xSens, -1, 1) * (-30) + xSens);
      // Serial.println(constrain(ySens, -1, 1) * (-30) + ySens);
      // Serial.println("-----");
      // delay(500);
  
      if(data.closeObj >= distThresh - tolerance)    //check if object in threshold to leave loops
      {
        object = 0;
      }
    }
  }
}



/*
  The command should maintain a distance from an object that it detects to the front via a proportional controller.
  It uses a deadzone of 2 to determine when to stop in order to avoid stuttering.
  The distance that it should follow from is a parameter in the command.
*/
void follow(int distance) {
  allOff();
  ylwOn();
  grnOn();
  data = RPC.call("read_lidars").as<struct lidar>();
  while(data.front){  //object in front of robot
    data = RPC.call("read_lidars").as<struct lidar>();

    stepperLeft.setCurrentPosition(0);    //reset world position
    stepperRight.setCurrentPosition(0);

    int delta = data.front - distance;        // difference between intended distance and actual
    if(delta < 2 && delta > -2)   //deadzone creation
    {
      delta = 0;
    }
    int sign = constrain(delta, -1, 1);
    stepperLeft.moveTo(sign*5000);   // determine if moving forward or backword based on delta
    stepperRight.moveTo(sign*5000);
    stepperLeft.setSpeed(constrain(sign*sq(delta)*25, -700, 700));
    stepperRight.setSpeed(constrain(sign*sq(delta)*25, -700, 700));
    stepperRight.run();
    stepperLeft.run();

    // Test Delta data
    // Serial.print(constrain(delta, -1, 1)*5000);
    // Serial.print("  ");
    // Serial.print(delta);
    // Serial.print("  ");
    // Serial.println(data.front);
  }
}

void smartWander(int collideDist, int runDist) {
  allOff();
  collide(collideDist);
  delay(100);
  while(object)
  {
    runAway(runDist);
  }
}

void smartFollow(int collideDist, int followDist) {
  allOff();               //Turn off all LEDs
  collide(collideDist);
  delay(100);
  follow(followDist);     //
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
  // follow(20);
  smartFollow(10, 15);
  // read_sensors();
  //data = RPC.call("read_lidars").as<struct lidar>();
  // Serial.println(data.front);
}
