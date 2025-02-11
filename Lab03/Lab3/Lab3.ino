/*
  Made by: Everest Zhang and Mitch Boucher
  Created: 1/21/2025
  Modified last: 1/24/2025

  This program is made to complete lab 3 of ECE425
  This includes:
  Following a wall at a set distance
  Updating the robot's global positioning
  Avoiding obstacles
  Pairing random wandering with the previous abilities

*/
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

//Object detection variable (typically based on lidar)
bool object = 0;

//Dimention of the robot
const float wheel_width = 8.5;  //diameter of the wheel (cm)
const float wheel_base = 23;    //length between the two wheel's center (cm)

//Global positioning of the object
float global_x = 0.00;
float global_y = 0.00;
int prev_l_step = 0;
int prev_r_step = 0;
int delta_l_step = 0;
int delta_r_step = 0;
float global_theta = 0;
float delta_theta = 0;


// a struct to hold lidar data
struct lidar {
  // this can easily be extended to contain sonar data as well
  float front;
  float back;
  float left;
  float right;
  float closestObj;
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
  float d;
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

/*
  The helper function will return the minimum (non-zero) lidar value, used for detecting if an object on anyside is within the threshold.
*/
float minLidar(float frontL, float backL, float leftL, float rightL) {
  Serial.print(data.front);
  Serial.print(data.back);
  Serial.print(data.left);
  Serial.println(data.right);
  float ldrs[4];
  ldrs[0] = frontL;
  ldrs[1] = backL;
  ldrs[2] = leftL;
  ldrs[3] = rightL;


  float minValue = 100.0;
  for (int i = 0; i < 4; i++) {
    if (ldrs[i] < minValue && ldrs[i] != 0) {
      minValue = ldrs[i];
    }
  }
  return minValue;
}

/*
  The helper function will read sensor data from M4 and print it using Serial
*/
void print_sensors() {
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
  Serial.print(data.right);
  Serial.println(", ");
  Serial.println(data.closestObj);
}

/*
  The following commands are used to control the LEDs turning on or off
*/
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

  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  steppers.addStepper(stepperRight);           //add right motor to MultiStepper
  steppers.addStepper(stepperLeft);            //add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);  //turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);               //turn on enable LED
}

//Positioning Commands

/*
  convert travel distance (cm) to steps
*/
int dis_to_step(int dis) {
  int step = dis * 360 / (3.14 * wheel_width * 0.45);
  return step;
}

/*
  Convert a number of steps into a value of distance (for global positioning)
  Distance is a value of cm
*/
float step_to_dis(int step) {
  float dis = step * 3.14 * wheel_width * 0.45 / 360;
  return dis;
}

/*
  Set the current position and step counter to zero
*/
void set_zero() {
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  prev_l_step = 0;
  prev_r_step = 0;
}

/*
  Keep track of an update the global positioning of the robot using the motor step counter
*/

unsigned long previousMillisGlobal = millis();
const unsigned long intervalGlobal = 25;

void updateGlobalPosition() {

  if (millis() - previousMillisGlobal >= intervalGlobal) {
    delta_l_step = stepperLeft.currentPosition() - prev_l_step;
    delta_r_step = stepperRight.currentPosition() - prev_r_step;

    delta_theta = step_to_dis(delta_r_step - delta_l_step) / wheel_base;
    global_x = cos(global_theta + delta_theta) * step_to_dis((delta_l_step + delta_r_step) / 2) + global_x;
    global_y = sin(global_theta + delta_theta) * step_to_dis((delta_l_step + delta_r_step) / 2) + global_y;
    global_theta = global_theta + delta_theta;
    prev_l_step = stepperLeft.currentPosition();
    prev_r_step = stepperRight.currentPosition();


    /*
    Serial.print("cos: ");
    Serial.print(cos(global_theta + delta_theta));
    Serial.print("  ");

    Serial.print("step to dis: ");
    Serial.print(step_to_dis((delta_l_step + delta_r_step) / 2));
    Serial.print("  ");

    Serial.print("product ");
    Serial.print(cos(global_theta + delta_theta) * step_to_dis((delta_l_step + delta_r_step) / 2));
    Serial.print("  ");
    
    Serial.print("delta_l: ");
    Serial.print(delta_l_step);
    Serial.print("   ");
    Serial.print("delta_r: ");
    Serial.print(delta_r_step);
    Serial.print("   ");

    Serial.print(global_x);
    Serial.print("  ");
    Serial.print(global_y);
    Serial.print("   ");
    Serial.println(global_theta);
    */
    previousMillisGlobal = millis();
  }
}

//Simple Helper Functions

void runAndUpdate(AccelStepper &stepperMotor) {
  stepperMotor.run();
  updateGlobalPosition();
}

void runSpeedAndUpdate(AccelStepper &stepperMotor) {
  stepperMotor.runSpeed();
  updateGlobalPosition();
}

// Function returns a reference to the selected member <-- Created w/ chatGPT
int& getLidarValue(const String &side) {
    if (side == "front") return data.front;
    else if (side == "back") return data.back;
    else if (side == "left") return data.left;
    else return data.right;  // Default to right if invalid input
}

void parallelDistance(String side) { //TODO: I'm not fully sure what this function actually does?
  int avgValue = 3;
  float preDistance = 0;
  for(int i = 0; i < avgValue; i++) {  
    delay(500);
    data = RPC.call("read_lidars").as<struct lidar>();
    preDistance += getLidarValue(side);
  }
  preDistance = preDistance / avgValue;
  int offset = 5;
  forward(offset, 500);
  float currDistance = 0;
  for(int i = 0; i < avgValue; i++) {  
    delay(500);
    data = RPC.call("read_lidars").as<struct lidar>();
    preDistance += getLidarValue(side);
  }
  currDistance = currDistance / avgValue;
  float a = float(offset) * curr / (pre - curr);                   //positive if going towards, negative if going away
  float theta = atan2(curr, abs(a)) * 57.3 * constrain(a, -1, 1);  //in degree
  float delta = (-1) * a * (goal_dis - curr) / curr;               //TODO: ?
  Serial.print(pre);    //debugging commands, remove in final
  Serial.print("  ");
  Serial.print(curr);
  Serial.print("  ");
  Serial.print(delta);
  Serial.print("  ");
  Serial.println(theta);
  if (abs(theta) > 5) {     //TODO: unsure of what this does?
    forward(delta, 300);
  }
  if (delta > 0) {
    pivot(1, -theta);
  } else {
    pivot(0, -theta);
  }
}

/*
  The robot will move to a set position, specified by the x and y value
  TODO: update to be non-blocking
*/
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
    forward(-dis, 300);
  }
}

/*
  The robot will move to a set goal.
  If an object is detected to be between the robot and the goal it will attempt to follow along the object until it has passed
  After the robot is past the obstacle it will begin to reroute towards the goal again.
  It will stop moving once the robot is within 5cm of the goal.
  The robot is constantly updating it's position as it follows the wall and moves to ensure the direction and distance to the goal.
  The inputs to the function are (xGoal) and (yGoal) which are the X and Y coordinates of the goal position respectively
*/

void smartGoToGoal(int xGoal, int yGoal) {
  object = 0;
  int wall = 0;
  unsigned long previousMillis = 0;
  const unsigned long interval = 50;
  previousMillis = millis();


  while (true) {
    while (object) {  //while it detects an object in the way of the robot
      if (data.front < 15 && data.front) {
        set_zero();   //set current position to new global 0
        int dis = 120 * 3.14 * wheel_base / 180;  //pivot to side
        int steps = dis_to_step(dis);
        stepperLeft.moveTo(-steps);
        stepperLeft.setMaxSpeed(300);
        stepperLeft.setAcceleration(200);
        while (!data.right || data.right > 20) {  //nothing on right or more than 20 cm

          runAndUpdate(stepperLeft);    //step + update position based on step
          if (millis() - previousMillis >= interval) {    //read sensors every once in a while
            data = RPC.call("read_lidars").as<struct lidar>();
            previousMillis = millis();
            Serial.print("Searching for right: ");    //TODO: Remove because slows down?
            Serial.println(data.right);
          }
          if (!stepperLeft.distanceToGo()) {    //leave while loop if reached distance
            break;
          }
        }
        if (data.right && data.right < 20) {    // if something is within range on right
        } else {
          pivot(0, -120);   //pivot to the left
          object = 0;       //clear object flag to begin going towards goal
          break;
        }
      }
      if (data.right < 25 && data.right) {  // if object within deadzone of right side
        stepperLeft.setMaxSpeed(2000);
        stepperRight.setMaxSpeed(2000);
        stepperLeft.setSpeed(200);
        stepperRight.setSpeed(200);
        int p = data.right;   //set previous position
        int d = data.right;   //set current position

        while (data.right && data.right < 25) {
          stepperLeft.setMaxSpeed(2000);
          stepperRight.setMaxSpeed(2000);
          stepperLeft.setSpeed(200);
          stepperRight.setSpeed(200);

          runSpeedAndUpdate(stepperLeft);
          runSpeedAndUpdate(stepperRight);
          if (millis() - previousMillis >= 500) {

            data = RPC.call("read_lidars").as<struct lidar>();
            d = data.right;            
            if(!d){
              break;  //if no wall on right leave if statement
            }
            if ((p-d) > 0.7) {    //if difference between position and previous is less than threshold, pivot
              pivotConst(0, 5, 100);  
            }
            data = RPC.call("read_lidars").as<struct lidar>();
            p = data.right;
            previousMillis = millis();
          }
        }  //no more wall on right
        forward(15, 600);
      }
      if (data.left < 25 && data.left) {  //same as right, but for left sensor
        stepperLeft.setMaxSpeed(2000);
        stepperRight.setMaxSpeed(2000);
        stepperLeft.setSpeed(200);
        stepperRight.setSpeed(200);
        int p = data.left;
        int d = data.left;

        while (data.left && data.left < 25) {
          stepperLeft.setMaxSpeed(2000);
          stepperRight.setMaxSpeed(2000);
          stepperLeft.setSpeed(200);
          stepperRight.setSpeed(200);

          runSpeedAndUpdate(stepperLeft);
          runSpeedAndUpdate(stepperRight);
          if (millis() - previousMillis >= 500) {

            data = RPC.call("read_lidars").as<struct lidar>();
            d = data.left;            
            if(!d){
              break;
            }
            if ((p-d) > 0.7) {
              pivotConst(1, -5, 100);
            }
            data = RPC.call("read_lidars").as<struct lidar>();
            p = data.left;
            previousMillis = millis();
          }
        }  //no more wall on left
        forward(15, 600);
      }
      object = 0;
      break;
    }

    float delta_x = xGoal - global_x;   //difference between current and goal X
    float delta_y = yGoal - global_y;   //difference between current and goal Y
    float radian = atan2(delta_y, delta_x) - global_theta;
    int degree = radian * 57.3;
    long dis = sqrt(sq(delta_x) + sq(delta_y));
    spin(degree, 400);    //spin towards goal

    set_zero();
    stepperLeft.moveTo(dis_to_step(dis));   //set to move towards goal
    stepperRight.moveTo(dis_to_step(dis));
    stepperLeft.setMaxSpeed(600);
    stepperRight.setMaxSpeed(600);
    stepperLeft.setSpeed(400);
    stepperRight.setSpeed(400);

    while (stepperLeft.distanceToGo()) {  //Still have to move

      runSpeedAndUpdate(stepperLeft);
      runSpeedAndUpdate(stepperRight);

      if (millis() - previousMillis >= interval) {  //update sensors every set time interval

        data = RPC.call("read_lidars").as<struct lidar>();
        Serial.print("going to goal: ");
        Serial.println(data.front);
        if ((data.front && data.front < 15) || (data.right && data.right < 25) || (data.left && data.left < 25)) {    //detect if object within threshold
          object = 1;
          break;
        }
        previousMillis = millis();
      }
    }
    set_zero(); //set current position to new global 0
    delta_x = xGoal - global_x;   // re-obtain distance values
    delta_y = yGoal - global_y;
    if (delta_x < 5 && delta_y < 5) { //if within 5cm of the goal, stop the loop
      break;
    }
  }
}


/*
  Turn the robot a number of degrees at a specified speed
  Produce a "tank turn" <-- moves both wheels opposite evenly
*/
void spin(int degree, int speed) {
  set_zero();
  int offset = 0;
  if (degree < 0) {
    offset = 15 * degree / 90;
  } else {
    offset = 15 * degree / 90;
  }
  int signOfDegree = constrain(degree, -1, 1);
  int dis = degree * 3.14 * wheel_base / (2 * 180);
  int steps = dis_to_step(dis);  //+ offset;
  stepperLeft.setMaxSpeed(speed);
  stepperRight.setMaxSpeed(speed);
  stepperLeft.setAcceleration(100.0);
  stepperRight.setAcceleration(100.0);

  stepperRight.moveTo(steps);
  stepperLeft.moveTo(-steps);

  while (stepperLeft.distanceToGo() != 0 && stepperRight.distanceToGo() != 0) {
    runAndUpdate(stepperLeft);
    runAndUpdate(stepperRight);
  }
  set_zero();
}
/*
  Turn in certain degree with certain radius (cm) left of robot is negative
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
  stepperRight.moveTo(step_r);
  stepperLeft.moveTo(step_l);
  stepperRight.setSpeed(constrain(dis_r, -1, 1) * speed_r);
  stepperLeft.setSpeed(constrain(dis_l, -1, 1) * speed_l);
  steppers.runSpeedToPosition();
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
}

/*
  Move the robot forward by a set distance (in cm) at a set speed
  A negative distance will result in a reverse movment
*/
void forward(int distance, int speed) {
  set_zero();
  stepperLeft.moveTo(dis_to_step(distance));
  stepperRight.moveTo(dis_to_step(distance));
  stepperLeft.setMaxSpeed(speed);
  stepperRight.setMaxSpeed(speed);
  stepperLeft.setAcceleration(int(speed / 3));
  stepperRight.setAcceleration(int(speed / 3));
  while (stepperLeft.distanceToGo()) {
    runAndUpdate(stepperLeft);
    runAndUpdate(stepperRight);
  }
  set_zero();
}


/*
  Robot pivot on one wheel. left wheel is 0 and right wheel is 1. degree can be both positive and negative
*/
void pivot(bool wheel, float degree) {
  set_zero();
  int dis = degree * 3.14 * wheel_base / 180;
  int steps = dis_to_step(dis);
  if (wheel == 0) {  //pivot on left wheel
    stepperRight.moveTo(steps);
    stepperRight.setMaxSpeed(700);
    stepperRight.setAcceleration(500);
    while (stepperRight.distanceToGo()) {
      runAndUpdate(stepperRight);
    }
  } else {
    stepperLeft.moveTo(-steps);
    stepperLeft.setMaxSpeed(700);
    stepperLeft.setAcceleration(500);
    while (stepperLeft.distanceToGo()) {
      runAndUpdate(stepperLeft);
    }
  }
  set_zero();
}

void pivotConst(bool wheel, float degree, int speed) {
  set_zero();
  int dis = degree * 3.14 * wheel_base / 180;
  int steps = dis_to_step(dis);
  if (wheel == 0) {  //pivot on left wheel
    stepperRight.moveTo(steps);
    stepperRight.setMaxSpeed(speed + 100);
    stepperRight.setSpeed(speed);
    while (stepperRight.distanceToGo()) {
      runSpeedAndUpdate(stepperRight);
    }
  } else {
    stepperLeft.moveTo(-steps);
    stepperLeft.setMaxSpeed(speed + 100);
    stepperLeft.setSpeed(speed);
    while (stepperLeft.distanceToGo()) {
      runSpeedAndUpdate(stepperLeft);
    }
  }
  set_zero();
}

/*
  Prepare to randomly move the robot <-- sets the distance it WILL move
  the robot will move a random amount, at a random speed, and random acceleration
  The randomness of the robot is restricted to the input of maxVal
  Non-blocking way of setting movment, 
*/
void prepMovement(int maxVal) {
  if (stepperLeft.distanceToGo() == 0) {
    stepperLeft.setCurrentPosition(2000);
    int randomSteps = random(maxVal) + 5;
    int randomMaxSpeed = random(maxVal) % 200 + 150;
    int randomAcc = random(maxVal) % 100 + 150;
    stepperLeft.moveTo(randomSteps);
    stepperLeft.setMaxSpeed(randomMaxSpeed);
    stepperLeft.setAcceleration(randomAcc);
  }
  if (stepperRight.distanceToGo() == 0) {
    stepperRight.setCurrentPosition(2000);
    int randomSteps = random(maxVal) + 5;
    int randomMaxSpeed = random(maxVal) % 200 + 150;
    int randomAcc = random(maxVal) % 100 + 150;
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
  if (stepperLeft.distanceToGo() == 0) {
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
  The robot will randomly wander around
  The type of random wandering is determined by each wheel independently getting a distance to travel, at a random speed and acceleration
  The distance, speed, and acceleration random value is limited by the constant maxValue
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

/*
  When sensing an object in any direction the robot will stop.
  The distance threshold that determines how close to an object counts is a parameter in the command.
*/
void collide(int distThresh) {
  allOff();  //Turn off all LEDs
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
  int tolerance = distThresh / 10;
  allOff();
  ylwOn();
  data = RPC.call("read_lidars").as<struct lidar>();

  if (data.closestObj < distThresh - tolerance) {  // object present
    delay(100);
    data = RPC.call("read_lidars").as<struct lidar>();  // read again incase of missing data
    bool FB = 0;                                        //not on both front and back
    bool LR = 0;                                        //not on both left and right
    // int distThresh = 20;


    if (data.front <= distThresh && data.back <= distThresh && data.back && data.front) {  // both front and back
      data.front = 0;                                                                      //ignore front and back sensor data
      data.back = 0;
      FB = 1;
      if (!data.left && !data.right) {  // turn to right sensor and move if free space
        data.left = 4;                  // fake object to left <-- force to right
      }
    } else {
      if (data.front > distThresh - tolerance) {  // if further then 30 cm, ignore
        data.front = 0;
      }
      if (data.back > distThresh - tolerance) {
        data.back = 0;
      }
    }

    //clear data above threshold
    if (data.left <= distThresh && data.right <= distThresh && data.left && data.right) {  // both left and right
      data.left = 0;                                                                       //ignore left and right sensor data
      data.right = 0;
      LR = 1;
      if (!data.front && !data.back) {  // turn to front sensor and move if free space
        data.back = 4;                  // fake object to back
      }
    } else {
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

      /*Test sense data
      Serial.println(xSens);
      Serial.println(ySens);
      Serial.println(constrain(xSens, -1, 1) * (-30) + xSens);
      Serial.println(constrain(ySens, -1, 1) * (-30) + ySens);
      Serial.println("-----");
      delay(500);
      */

      if (data.closestObj >= distThresh - tolerance)  //check if object in threshold to leave loops
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
  while (data.front) {  //object in front of robot
    data = RPC.call("read_lidars").as<struct lidar>();

    stepperLeft.setCurrentPosition(0);  //reset world position
    stepperRight.setCurrentPosition(0);

    int delta = data.front - distance;  // difference between intended distance and actual
    if (delta < 2 && delta > -2)        //deadzone creation
    {
      delta = 0;
    }
    int sign = constrain(delta, -1, 1);
    stepperLeft.moveTo(sign * 5000);  // determine if moving forward or backword based on delta
    stepperRight.moveTo(sign * 5000);
    stepperLeft.setSpeed(constrain(sign * sq(delta) * 25, -700, 700));
    stepperRight.setSpeed(constrain(sign * sq(delta) * 25, -700, 700));
    stepperRight.run();
    stepperLeft.run();
  }
}

/*
  Randomly wander the robot until an object is detected
  Once an object is detected stop the robot using the collide command
  While there is an object within the running threshold, run away from the object
  The inputs are the distance to measure for collision and the distance to measure for when to run away 
*/
void smartWander(int collideDist, int runDist) {
  allOff();
  collide(collideDist);
  delay(100);
  while (object) {
    runAway(runDist);
  }
}

/*
  Randomly wander the robot until an object is detected
  Once an object is detected stop the robot using the collide command
  Begin following the object that stopped the robot
  If the followed object is no longer detected, return to randomly wandering
  The inputs are the distance to measure for collision and the distance to measure for following
*/
void smartFollow(int collideDist, int followDist) {
  allOff();  //Turn off all LEDs
  collide(collideDist);
  delay(100);
  follow(followDist);  //
}

/*
  Follow a wall on either side of the robot
  If spotting a corner the robot will turn into the corner to follow the new wall to the front
  If losing a wall the robot will check for if it can round an outer corner and refind the wall
  If in a dead end the robot will turn around and try to leave the passageway following both walls.ABC
  If the robot is between 2 walls it will follow them equidistant from each other but prioritize the first wall it saw.
  Wall = 1 <-- Front, 2 <-- Back, 3 <-- left, 4 <-- right
*/
bool wallFollow(int distThresh) {
  data = RPC.call("read_lidars").as<struct lidar>();  //Update sensor
  unsigned long previousMillis = 0;
  const unsigned long interval = 500;
  float d = 0;      //Distance from wall
  float p = 0;      //Previous distance from wall
  if (data.left && data.left <= 15) { //TODO: does data.left still work if float? does 0.1 count as true?
    getParallel(3);
  } else if (data.right && data.right <= 15) {
    getParallel(4);
  } else if (data.front && data.front <= 15) {
    getParallel(1);
  } else {
    getParallel(1);
  }
  delay(500);
  data = RPC.call("read_lidars").as<struct lidar>();
  int wall = 0;  // 3 left, 4 right

  if (data.left && data.left < 15) {  //prioritizes left wall over right if both
    wall = 3; //exists wall to Left
  } else if (data.right && data.right < 15) {
    wall = 4; //exists wall to right
  }

  if (wall == 3) {  //Wall to left
    delay(50);
    int upperThresh = 15;
    int lowerThresh = 10;
    data = RPC.call("read_lidars").as<struct lidar>();
    d = data.left;
    p = data.left;
    previousMillis = millis();  // Timer for arduino (return current time)
    while (d) {
      if (data.right && data.right < 15){ //if wall to left and right
        upperThresh = (data.left + data.right)/2 + 2;   //follow center dist set
        lowerThresh = upperThresh - 5;
      } else {
        upperThresh = 15;
        lowerThresh = 10;
      }
      if (d > 15) {   // Too far from wall <-- red LED
        redOn();
        pivot(0, 30);   //pivot left
        pivot(1, -30);  //pivot right  (To get back to being parallel, but closer to wall)
        redOff();
        data = RPC.call("read_lidars").as<struct lidar>();  //FIXME: move update data 
        d = data.left;  
        p = data.left;
      } else if (d < 10) {    // Too close to wall <-- Yellow LED
        ylwOn();
        pivot(1, -30);    //pivot right
        pivot(0, 30);     //pivot left 
        ylwOff();
        data = RPC.call("read_lidars").as<struct lidar>();  //FIXME: move update data 
        d = data.left;
        p = data.left;
      } else {  //if in good deadzone, go forward at speed 
        stepperLeft.setMaxSpeed(2000);  // FIXME: can turn into function
        stepperRight.setMaxSpeed(2000);
        stepperLeft.setSpeed(200);
        stepperRight.setSpeed(200);
        runSpeedAndUpdate(stepperLeft);
        runSpeedAndUpdate(stepperRight);
      }
      if (millis() - previousMillis >= interval) {  // if elapsed time has reached set interval
        data = RPC.call("read_lidars").as<struct lidar>();
        if (data.front && data.front < 10) {  // wall front     if left wall + front wall <-- Red + Green LED
          redOn();
          grnOn();
          if (data.right && data.right < 15) {    // Dead end
            forward(-20, 700);
            spin(180, 300);
          } else {
            pivot(1, 90);   // pivot right CCW 
            pivot(0, -90);  // pivot left CW
            pivot(1, -90);  // pivot right CW
            forward(-30, 500);  //move backward into corner; All combined this will pivot the robot to the right and tuck into the corner
            redOff();
            grnOff();
            getParallel(3);
          }
          data = RPC.call("read_lidars").as<struct lidar>();  // FIXME: move update data
          d = data.left;
          p = data.left;
        }
        if (data.left == 0 || data.left > 25) {  //lost wall on left
          pivot(0, 90);       //pivot left CCW
          forward(30, 500);   // move forward (looking for wall because might be an outer corner)
          data = RPC.call("read_lidars").as<struct lidar>();
          if (data.left == 0) {   //if no wall on left still (lost wall)
            return false;
          }
          getParallel(3);   //get parallel if found wall
          data = RPC.call("read_lidars").as<struct lidar>();  // FIXME: move update data
          d = data.left;
          p = data.left;
        }
        d = data.left;
        int i = 0;

        while (i < 500) { 
          stepperLeft.setMaxSpeed(2000);
          stepperRight.setMaxSpeed(2000);
          stepperLeft.setSpeed(200 - (d - p) * 50);   //move @ speed relative to previoua position (PD control)
          stepperRight.setSpeed(200 + (d - p) * 50);
          runSpeedAndUpdate(stepperLeft);
          runSpeedAndUpdate(stepperRight);
          i = i + 1;
        }
        p = d;    // set previous to the current distance
        previousMillis = millis();    // previous timestamp
      }
    }
  } else if (wall == 4) {   //wall on Right
    delay(50);
    int upperThresh = 15;
    int lowerThresh = 10;
    data = RPC.call("read_lidars").as<struct lidar>();
    d = data.right;
    p = data.right;

    previousMillis = millis();
    while (d) { // wall on right
      if (data.left && data.left < 15) { // wall on left and right (follow center)
        upperThresh = (data.left + data.right) / 2 + 2;
        lowerThresh = upperThresh - 5;
      } else {
        upperThresh = 15;
        lowerThresh = 10;
      }
      if (d > 15) {   //too far from wall
        redOn();
        pivot(1, -30);
        pivot(0, 30);
        redOff();
        data = RPC.call("read_lidars").as<struct lidar>();
        d = data.right;
        p = data.right;
      } else if (d < 10) {    //too close to wall
        ylwOn();
        pivot(0, 30);
        pivot(1, -30);
        ylwOff();
        data = RPC.call("read_lidars").as<struct lidar>();
        d = data.right;
        p = data.right;
      } else {

        stepperLeft.setMaxSpeed(2000);  //just right (goldilocks)
        stepperRight.setMaxSpeed(2000);
        stepperLeft.setSpeed(200);
        stepperRight.setSpeed(200);
        runSpeedAndUpdate(stepperLeft);
        runSpeedAndUpdate(stepperRight);
      }
      if (millis() - previousMillis >= interval) {    //update sensor every once in a while
        data = RPC.call("read_lidars").as<struct lidar>();
        if (data.front && data.front < 10) {    //object to front
          redOn();
          grnOn();
          if (data.left && data.left < 25) {    //Dead end
            forward(-20, 700);
            spin(180, 300);
          } else {
            // TODO: Helper func <-- TurnInnerCorner
            pivot(0, -90);    //turn to left (at inner corner)
            pivot(1, 90);
            pivot(0, 90);
            forward(-30, 500);
            getParallel(4);
          }

          data = RPC.call("read_lidars").as<struct lidar>();
          d = data.right;
          p = data.right;
          allOff();
        }
        if (data.right == 0 || data.right > 25) {  //lost wall on right
          pivot(1, -90);
          forward(30, 300);
          data = RPC.call("read_lidars").as<struct lidar>();
          if (data.right == 0) {
            return false;
          }
          getParallel(4);   //if refound wall
          data = RPC.call("read_lidars").as<struct lidar>();
          d = data.right;
          p = data.right;
        }
        d = data.right;
        int i = 0;

        while (i < 500) {
          stepperLeft.setMaxSpeed(2000);
          stepperRight.setMaxSpeed(2000);
          stepperLeft.setSpeed(200 + (d - p) * 50);
          stepperRight.setSpeed(200 - (d - p) * 50);
          runSpeedAndUpdate(stepperLeft);
          runSpeedAndUpdate(stepperRight);
          i = i + 1;
        }
        p = d;
        previousMillis = millis();
      }
    }
  }
}

/*
  Try to position the robot by pivoting to be parallel to the observed wall
  input (location) = location of the wall observed  1-front; 2-back; 3-left; 4-right  can be multiple?
*/
void getParallel(int location, float goal_dis) {  //location = location of the wall
  unsigned long previousMillis = 0;
  const unsigned long interval = 50;
  data = RPC.call("read_lidars").as<struct lidar>();  //update sensor
  if (location == 1 || location == 2) {  //pivot 90 or till right sensor find the wall; then pivot the other way till left sensor find the wall
    set_zero();
      //TODO: how is the following different from pivoting??
    int dis = 90 * 3.14 * wheel_base / 180; //calculate distance that steppers must move? <-- don't we have a subroutine for this?
    int steps = dis_to_step(dis);
    stepperLeft.moveTo(-steps);
    stepperLeft.setMaxSpeed(200);
    stepperLeft.setAcceleration(200);

    while (!data.right || data.right > goal_dis) {  //TODO while: nothing closer than 15 cm on right <-- should set deadzone to a variable (changes in smart... why???)
      runAndUpdate(stepperLeft);    //step + update position
      if (millis() - previousMillis >= interval) {    //update sensors every once in a while
        data = RPC.call("read_lidars").as<struct lidar>();
        previousMillis = millis();
      }
      if (!stepperLeft.distanceToGo()) {    //if reached position, leave loop
        break;
      }
    }
    // finish
    if (data.right) {  //TODO: if there is anything on the right, location = 4 (doesn't do anything yet) does this matter since didn't even check range?
      location = 4;
    } else {
      pivot(1, -90);  // pivot back; (back to where? haven't pivoted yet?)
      set_zero();
      int dis = -90 * 3.14 * wheel_base / 180;    //TODO: re-using this block again but w/ negative sign?
      int steps = dis_to_step(dis);
      stepperRight.moveTo(steps);
      stepperRight.setMaxSpeed(200);
      stepperRight.setAcceleration(200);

      while (!data.left || data.left > goal_dis) {  //nothing in range of left
        runAndUpdate(stepperRight); //TODO: step + update (how is this ANY different than before?) <-- can make a helper command rather than copy + paste
        if (millis() - previousMillis >= interval) {  //update sensors every once in a while
          data = RPC.call("read_lidars").as<struct lidar>();
          previousMillis = millis();
        }
        if (!stepperRight.distanceToGo()) { //if reached position, leave loop
          break;
        }
      }

      if (data.left) {    //FIXME: no deadzone?
        location = 3;
      }
    }
  }
  if (location == 3) {  // wall on left
    parallelDistance("left");  //flatten to wall?
  }
  if (location == 4) {  // wall on right
    parallelDistance("right");  //flatten to wall?
  }
}


/*
  The robot will randomly wander around until it detects an object within its threshold.
  After detecting an object the robot will begin following the object around its perimeter
  The input (distThresh) is used to measure the deadzone/threshold to detect an object to follow
*/
void randomWanderWallFollow(int distThresh) {
  data = RPC.call("read_lidars").as<struct lidar>();
  unsigned long previousMillis = 0;
  const unsigned long interval = 50;
  while (!object) {
    grnOn();             //Turn on green LED
    prepMovement(6000);  //prepare to move forward
    stepperLeft.run();   //increment left motor
    stepperRight.run();  //increment right motor
    updateGlobalPosition();
    if (millis() - previousMillis >= interval) {
      data = RPC.call("read_lidars").as<struct lidar>();
      previousMillis = millis();
    }

    if (minLidar(data.front, data.back, data.left, data.right) < distThresh) {
      object = 1;
      grnOff();
    }
  }
  wallFollow(distThresh);
  object = 0;
}



void setup() {
  RPC.begin();
  init_stepper();

  Serial.begin(9600);
  delay(5000);

  smartGoToGoal(200, 0);
}

void loop() {

}
