
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
const float wheel_base = 23;    //length between the two wheel's center (cm)

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
  // this defines some helper functions that allow RPC to send our struct (I found this on a random forum)
  MSGPACK_DEFINE_ARRAY(front, back, left, right);  //https://stackoverflow.com/questions/37322145/msgpack-to-pack-structures https://www.appsloveworld.com/cplus/100/391/msgpack-to-pack-structures
} dist;


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
//read sensor data from M4 and write to M7
void read_sensors() {
  // read lidar data from struct
  struct lidar data = RPC.call("read_lidars").as<struct lidar>();
  struct sonar data2 = RPC.call("read_sonars").as<struct sonar>();
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

  //stepperRight.setMaxSpeed(max_speed);         //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  //stepperRight.setAcceleration(max_accel);     //set desired acceleration in steps/s^2
  //stepperLeft.setMaxSpeed(max_speed);          //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  //stepperLeft.setAcceleration(max_accel);      //set desired acceleration in steps/s^2
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
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

float step_to_dis(int step) {
  float dis = step * 3.14 * wheel_width * 0.45 / 360;
  return dis;
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

void forward(int distance, int speed) {
  set_zero();
  stepperLeft.moveTo(dis_to_step(distance));
  stepperRight.moveTo(dis_to_step(distance));
  stepperLeft.setMaxSpeed(speed);
  stepperRight.setMaxSpeed(speed);
  stepperLeft.setAcceleration(int(speed / 3));
  stepperRight.setAcceleration(int(speed / 3));
  while (stepperLeft.distanceToGo()) {
    stepperLeft.run();
    stepperRight.run();
    updateGlobalPosition();
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
      stepperRight.run();
      updateGlobalPosition();
    }
  } else {
    stepperLeft.moveTo(-steps);
    stepperLeft.setMaxSpeed(700);
    stepperLeft.setAcceleration(500);
    while (stepperLeft.distanceToGo()) {
      stepperLeft.run();
      updateGlobalPosition();
    }
  }
  set_zero();
}

void updateGlobalPosition() {
  delta_l_step = stepperLeft.currentPosition() - prev_l_step;
  delta_r_step = stepperRight.currentPosition() - prev_r_step;

  delta_theta = step_to_dis(delta_r_step - delta_l_step) / wheel_base / 2;
  global_x = cos(global_theta + delta_theta) * step_to_dis((delta_l_step + delta_r_step) / 2) + global_x;
  global_y = sin(global_theta + delta_theta) * step_to_dis((delta_l_step + delta_r_step) / 2) + global_y;

  delayMicroseconds(500);  // time for calculation to finish (500 is too much and 100 is too little);

  prev_l_step = stepperLeft.currentPosition();
  prev_r_step = stepperRight.currentPosition();
}

void wallFollow() {
  int a = 3;
  struct lidar data = RPC.call("read_lidars").as<struct lidar>();
  unsigned long previousMillis = 0;
  const unsigned long interval = 500;
  float d = 0;
  float p = 0;
  if (data.left) {
    getParallel(3);
  } else if (data.right) {
    getParallel(4);
  } else if (data.front) {
    getParallel(1);
  }
  delay(500);
  data = RPC.call("read_lidars").as<struct lidar>();
  int wall = 0;  // 3 left, 4 right
  if (data.left && data.left < 25) {
    wall = 3;
  } else if (data.right && data.right < 25) {
    wall = 4;
  }

  if (wall == 3) {
    delay(50);
    data = RPC.call("read_lidars").as<struct lidar>();
    d = data.left;
    p = data.left;
    previousMillis = millis();
    while (d) {
      if (d > 15) {
        redOn();
        pivot(0, 30);
        pivot(1, -30);
        redOff();
        data = RPC.call("read_lidars").as<struct lidar>();
        d = data.left;
        p = data.left;
      } else if (d < 10) {
        ylwOn();
        pivot(1, -30);
        pivot(0, 30);
        ylwOff();
        data = RPC.call("read_lidars").as<struct lidar>();
        d = data.left;
        p = data.left;
      } else {
        stepperLeft.setMaxSpeed(2000);
        stepperRight.setMaxSpeed(2000);
        stepperLeft.setSpeed(200);
        stepperRight.setSpeed(200);
        stepperLeft.runSpeed();
        stepperRight.runSpeed();
        updateGlobalPosition();
      }
      if (millis() - previousMillis >= interval) {
        data = RPC.call("read_lidars").as<struct lidar>();
        if (data.front && data.front < 10) {
          redOn();
          grnOn();
          pivot(1, 90);
          pivot(0, -90);
          pivot(1, -90);
          forward(-30, 500);
          redOff();
          grnOff();
          getParallel(3);
          data = RPC.call("read_lidars").as<struct lidar>();
          d = data.left;
          p = data.left;
        }
        d = data.left;
        int i = 0;

        while (i < 500) {

          stepperLeft.setMaxSpeed(2000);
          stepperRight.setMaxSpeed(2000);
          stepperLeft.setSpeed(200 - (d - p) * 50);
          stepperRight.setSpeed(200 + (d - p) * 50);
          stepperLeft.runSpeed();
          stepperRight.runSpeed();
          updateGlobalPosition();
          i = i + 1;
        }
        p = d;
        previousMillis = millis();
      }
    }
  } else if (wall == 4) {
    delay(50);
    data = RPC.call("read_lidars").as<struct lidar>();
    d = data.right;
    p = data.right;

    previousMillis = millis();
    while (d) {
      if (d > 15) {
        redOn();
        pivot(1, -30);
        pivot(0, 30);
        redOff();
        data = RPC.call("read_lidars").as<struct lidar>();
        d = data.right;
        p = data.right;
      } else if (d < 10) {
        ylwOn();
        pivot(0, 30);
        pivot(1, -30);
        ylwOff();
        data = RPC.call("read_lidars").as<struct lidar>();
        d = data.right;
        p = data.right;
      } else {

        stepperLeft.setMaxSpeed(2000);
        stepperRight.setMaxSpeed(2000);
        stepperLeft.setSpeed(200);
        stepperRight.setSpeed(200);
        stepperLeft.runSpeed();
        stepperRight.runSpeed();
        updateGlobalPosition();
      }
      if (millis() - previousMillis >= interval) {
        data = RPC.call("read_lidars").as<struct lidar>();
        if (data.front && data.front < 10) {
          redOn();
          grnOn();
          pivot(0, -90);
          pivot(1, 90);
          pivot(0, 90);
          forward(-30, 500);
          getParallel(4);
          data = RPC.call("read_lidars").as<struct lidar>();
          d = data.right;
          p = data.right;
          allOFF();
        }
        d = data.right;
        int i = 0;

        while (i < 500) {
          stepperLeft.setMaxSpeed(2000);
          stepperRight.setMaxSpeed(2000);
          stepperLeft.setSpeed(200 + (d - p) * 50);
          stepperRight.setSpeed(200 - (d - p) * 50);
          stepperLeft.runSpeed();
          stepperRight.runSpeed();
          updateGlobalPosition();
          i = i + 1;
        }
        p = d;
        previousMillis = millis();
      }
    }
  }
}

void set_zero() {
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  prev_l_step = 0;
  prev_r_step = 0;
}

/*
1-front; 2-back; 3-left; 4-right
*/
void getParallel(int location) {
  float goal_dis = 15.0;  //cm
  unsigned long previousMillis = 0;
  const unsigned long interval = 50;
  struct lidar data = RPC.call("read_lidars").as<struct lidar>();
  if (location == 1 || location == 2) {  //pivot 90 or till right sensor find the wall; then pivot the other way till left sensor find the wall
    set_zero();
    int dis = 90 * 3.14 * wheel_base / 180;
    int steps = dis_to_step(dis);
    stepperLeft.moveTo(-steps);
    stepperLeft.setMaxSpeed(200);
    stepperLeft.setAcceleration(200);
    while (!data.right || data.right > 15) {  //nothing on right or more than 15
      stepperLeft.run();
      updateGlobalPosition();
      if (millis() - previousMillis >= interval) {
        data = RPC.call("read_lidars").as<struct lidar>();
        previousMillis = millis();
      }
      if (!stepperLeft.distanceToGo()) {
        break;
      }
    }
    // finish
    if (data.right) {  //if there is a wall on the right, jump to location = 4
      location = 4;
    } else {
      pivot(1, -90);  // pivot back
      set_zero();
      int dis = -90 * 3.14 * wheel_base / 180;
      int steps = dis_to_step(dis);
      stepperRight.moveTo(steps);
      stepperRight.setMaxSpeed(200);
      stepperRight.setAcceleration(200);

      while (!data.left || data.left > 15) {  //nothing on left
        stepperRight.run();
        updateGlobalPosition();
        if (millis() - previousMillis >= interval) {
          data = RPC.call("read_lidars").as<struct lidar>();
          previousMillis = millis();
        }
        if (!stepperRight.distanceToGo()) {
          break;
        }
      }

      if (data.left) {
        location = 3;
      }
    }
  }
  if (location == 3) {  // wall on left
    delay(500);
    data = RPC.call("read_lidars").as<struct lidar>();
    float pre = data.left;
    delay(500);
    data = RPC.call("read_lidars").as<struct lidar>();
    pre = pre + data.left;
    delay(500);
    data = RPC.call("read_lidars").as<struct lidar>();
    pre = pre + data.left;
    pre = pre / 3;
    int offset = 5;
    forward(offset, 300);
    delay(500);
    data = RPC.call("read_lidars").as<struct lidar>();
    float curr = data.left;
    delay(500);
    data = RPC.call("read_lidars").as<struct lidar>();
    curr = curr + data.left;
    delay(500);
    data = RPC.call("read_lidars").as<struct lidar>();
    curr = curr + data.left;
    curr = curr / 3;
    float a = float(offset) * curr / (pre - curr);                   //positive if going towards, negative if going away
    float theta = atan2(curr, abs(a)) * 57.3 * constrain(a, -1, 1);  //in degree
    float delta = (-1) * a * (goal_dis - curr) / curr;
    Serial.print(pre);
    Serial.print("  ");
    Serial.print(curr);
    Serial.print("  ");
    Serial.print(delta);
    Serial.print("  ");
    Serial.println(theta);
    if (abs(theta) > 5) {
      forward(delta, 300);
    }
    if (delta > 0) {
      pivot(1, -theta);
    } else {
      pivot(0, -theta);
    }
  }
  if (location == 4) {
    delay(500);
    data = RPC.call("read_lidars").as<struct lidar>();
    float pre = data.right;
    delay(500);
    data = RPC.call("read_lidars").as<struct lidar>();
    pre = pre + data.right;
    delay(500);
    data = RPC.call("read_lidars").as<struct lidar>();
    pre = pre + data.right;
    pre = pre / 3;
    int offset = 5;
    forward(offset, 300);
    delay(500);
    data = RPC.call("read_lidars").as<struct lidar>();
    float curr = data.right;
    delay(500);
    data = RPC.call("read_lidars").as<struct lidar>();
    curr = curr + data.right;
    delay(500);
    data = RPC.call("read_lidars").as<struct lidar>();
    curr = curr + data.right;
    curr = curr / 3;
    float a = float(offset) * curr / (pre - curr);                          //positive if going towards, negative if going away
    float theta = atan2(curr, abs(a)) * 57.3 * constrain(a, -1, 1) * (-1);  //in degree, opposite of wall on left
    float delta = (-1) * a * (goal_dis - curr) / curr;
    Serial.print(pre);
    Serial.print("  ");
    Serial.print(curr);
    Serial.print("  ");
    Serial.print(delta);
    Serial.print("  ");
    Serial.println(theta);
    if (abs(theta) > 5) {  //only do this if the robot is really not parallel
      forward(delta, 200);
    }
    if (delta > 0) {
      pivot(1, -theta);
    } else {
      pivot(0, -theta);
    }
  }
}


void setup() {
  RPC.begin();
  init_stepper();

  Serial.begin(9600);
  delay(500);








  /*
  stepperLeft.setMaxSpeed(300);
  stepperLeft.setAcceleration(100);
  stepperLeft.moveTo(1000);
  stepperRight.setMaxSpeed(200);
  stepperRight.setAcceleration(100);
  stepperRight.moveTo(1000);
  while (stepperLeft.distanceToGo()) {

    stepperLeft.run();
    stepperRight.run();
    updateGlobalPosition();
  }
  Serial.print("done: ");
  Serial.print(global_x);
  Serial.print("  ");
  Serial.println(global_y);
  Serial.print("suppose:");
  Serial.println(step_to_dis(1000));
  */
  wallFollow();
  /*
  stepperLeft.setMaxSpeed(1000);
  stepperRight.setMaxSpeed(1000);
  stepperLeft.setSpeed(200);
  stepperRight.setSpeed(20);
  */
}

void loop() {

  /*
  stepperLeft.runSpeed();
  stepperRight.runSpeed();
  struct lidar data = RPC.call("read_lidars").as<struct lidar>();
  delay(10);
  Serial.println(data.left);
  */
}
