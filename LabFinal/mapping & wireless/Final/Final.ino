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
const float wheel_width = 8.4;  //diameter of the wheel (cm)
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

// WIFI connection
#include <ArduinoJson.h>

#include <ArduinoMqttClient.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_NICLA_VISION) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_GIGA) || defined(ARDUINO_OPTA)
#include <WiFi.h>
#elif defined(ARDUINO_PORTENTA_C33)
#include <WiFiC3.h>
#elif defined(ARDUINO_UNOR4_WIFI)
#include <WiFiS3.h>
#endif

#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = "zangs1";      // your network SSID (name)
char pass[] = "1234567890";  // your network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "test.mosquitto.org";
int port = 1883;
const char topic[] = "arduino/zang";

unsigned long previousMillis = 0;

int count = 0;

//Json
JsonDocument doc;
char output[256];
char input[256];
String premessage = "";
String command = "";

//mapping

int array[6][6];
const byte numChars = 32;
char receivedChars[numChars];  // an array to store the received data
int flag[2][5];
int flag_count = 0;
int traveled[20][2];
int current[2];

int x = 0;
int y = 1;
int tc = 0;

int gridx = 1;
int gridy = 1;
int facing = 0;


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
//read sensor data from M4 and write to M7
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
  sendSensor(500);
  stepperMotor.run();
  updateGlobalPosition();
}

void runSpeedAndUpdate(AccelStepper &stepperMotor) {
  sendSensor(500);
  stepperMotor.runSpeed();
  updateGlobalPosition();
}

void GoToGrid(int x, int y) {
  Serial.print("facing: ");
  Serial.println(facing);
  Serial.print("goal x: ");
  Serial.println(x);
  Serial.print("goal y: ");
  Serial.println(y);
  Serial.print("current x: ");
  Serial.println(gridx);
  Serial.print("current x: ");
  Serial.println(gridy);

  int dx = x - gridx;
  int dy = y - gridy;
  float radian = atan2(dy, dx);
  int degree = radian * 57.3;
  int dtheta = 0;
  if (facing == 0) {
    dtheta = -degree + 90;
  } else if (facing == 1) {
    dtheta = -degree + 180;
  } else if (facing == 2) {
    dtheta = -degree - 90;
  } else {
    dtheta = -degree;
  }
  Serial.print("sprin: ");
  Serial.println(dtheta);
  spin(dtheta, 500);
  if (dx >= 1) {
    facing = 3;
  } else if (dx <= -1) {
    facing = 1;
  } else if (dy >= 1) {
    facing = 0;
  } else if (dy <= -1) {
    facing = 2;
  }
  Serial.print("now facing: ");
  Serial.println(facing);

  forward(42, 500);
  gridx = x;
  gridy = y;
}

/*
  The robot will move to a set position, specified by the x and y value
  TODO: update to be non-blocking
*/
void GoToGoal(long x, long y) {

  float delta_x = x - global_x;
  float delta_y = y - global_y;

  float radian = (atan2(delta_y, delta_x) - global_theta);
  int degree = int(radian * 57.3) % 360;
  long dis = sqrt(sq(delta_x) + sq(delta_y));
  spin(degree, 400);

  set_zero();
  stepperLeft.moveTo(dis_to_step(dis));
  stepperRight.moveTo(dis_to_step(dis));
  stepperLeft.setMaxSpeed(400);
  stepperRight.setMaxSpeed(400);
  stepperLeft.setAcceleration(200);
  stepperRight.setAcceleration(200);

  if (degree <= 90 && degree >= -90) {
    spin(degree, 500);
    delay(50);
    while (stepperLeft.distanceToGo()) {
      runAndUpdate(stepperLeft);
      runAndUpdate(stepperRight);
    }
  } else {
    spin(constrain(degree, -1, 1) * (-180) + degree, 500);
    delay(50);
    while (stepperLeft.distanceToGo()) {
      runAndUpdate(stepperLeft);
      runAndUpdate(stepperRight);
    }
  }


  /*
  
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
  */
}

void smartGoToGoal(int xGoal, int yGoal) {
  object = 0;
  int wall = 0;
  unsigned long previousMillis = 0;
  const unsigned long interval = 50;
  previousMillis = millis();


  while (true) {
    while (object) {
      if (data.front < 15 && data.front) {
        set_zero();
        int dis = 120 * 3.14 * wheel_base / 180;
        int steps = dis_to_step(dis);
        stepperLeft.moveTo(-steps);
        stepperLeft.setMaxSpeed(300);
        stepperLeft.setAcceleration(200);
        while (!data.right || data.right > 20) {  //nothing on right or more than 15

          runAndUpdate(stepperLeft);
          if (millis() - previousMillis >= interval) {
            data = RPC.call("read_lidars").as<struct lidar>();
            previousMillis = millis();
            Serial.print("Searching for right: ");
            Serial.println(data.right);
          }
          if (!stepperLeft.distanceToGo()) {
            break;
          }
        }
        if (data.right && data.right < 20) {
        } else {
          pivot(0, -120);
          object = 0;
          break;
        }
      }
      if (data.right < 25 && data.right) {
        stepperLeft.setMaxSpeed(2000);
        stepperRight.setMaxSpeed(2000);
        stepperLeft.setSpeed(200);
        stepperRight.setSpeed(200);
        int p = data.right;
        int d = data.right;

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
            if (!d) {
              break;
            }
            if ((p - d) > 0.7) {
              pivotConst(0, 5, 100);
            }
            data = RPC.call("read_lidars").as<struct lidar>();
            p = data.right;
            previousMillis = millis();
          }
        }  //no more wall on right
        forward(15, 600);
      }
      if (data.left < 25 && data.left) {
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
            if (!d) {
              break;
            }
            if ((p - d) > 0.7) {
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

    float delta_x = xGoal - global_x;
    float delta_y = yGoal - global_y;
    float radian = atan2(delta_y, delta_x) - global_theta;
    int degree = radian * 57.3;
    long dis = sqrt(sq(delta_x) + sq(delta_y));
    spin(degree, 400);

    set_zero();
    stepperLeft.moveTo(dis_to_step(dis));
    stepperRight.moveTo(dis_to_step(dis));
    stepperLeft.setMaxSpeed(600);
    stepperRight.setMaxSpeed(600);
    stepperLeft.setSpeed(400);
    stepperRight.setSpeed(400);

    while (stepperLeft.distanceToGo()) {

      runSpeedAndUpdate(stepperLeft);
      runSpeedAndUpdate(stepperRight);

      if (millis() - previousMillis >= interval) {

        data = RPC.call("read_lidars").as<struct lidar>();
        Serial.print("going to goal: ");
        Serial.println(data.front);
        if ((data.front && data.front < 15) || (data.right && data.right < 25) || (data.left && data.left < 25)) {
          object = 1;
          break;
        }
        previousMillis = millis();
      }
    }
    set_zero();
    delta_x = xGoal - global_x;
    delta_y = yGoal - global_y;
    if (delta_x < 5 && delta_y < 5) {
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
  data = RPC.call("read_lidars").as<struct lidar>();
  float pre_l = data.left;
  float pre_r = data.right;
  unsigned long previousMillis = 0;
  previousMillis = millis();
  set_zero();
  stepperLeft.moveTo(dis_to_step(distance));
  stepperRight.moveTo(dis_to_step(distance));
  stepperLeft.setMaxSpeed(speed);
  stepperRight.setMaxSpeed(speed);
  stepperLeft.setAcceleration(int(speed - 50));
  stepperRight.setAcceleration(int(speed - 50));
  long a = stepperLeft.distanceToGo();
  while (a) {
    if (millis() - previousMillis >= 1500) {
      data = RPC.call("read_lidars").as<struct lidar>();
      Serial.print("left: ");
      Serial.println(data.left);
      Serial.print("right: ");
      Serial.println(data.right);

      if (pre_l - data.left >= 1 && data.left != 0) {
        set_zero();
        stepperLeft.moveTo(50);
        stepperLeft.setMaxSpeed(speed + 100);
        stepperLeft.setSpeed(speed);
        while (stepperLeft.distanceToGo()) {
          stepperLeft.runSpeed();
        }
        set_zero();
        stepperLeft.moveTo(a);
        stepperRight.moveTo(a);
        stepperLeft.setMaxSpeed(speed);
        stepperLeft.setAcceleration(int(speed - 50));
        pre_l = data.left;
      }
      else if (pre_r - data.right >= 1 && data.right != 0) {
        set_zero();
        stepperRight.moveTo(50);
        stepperRight.setMaxSpeed(speed + 100);
        stepperRight.setSpeed(speed);
        while (stepperRight.distanceToGo()) {
          stepperRight.runSpeed();
        }
        set_zero();
        stepperLeft.moveTo(a);
        stepperRight.moveTo(a);
        stepperRight.setMaxSpeed(speed);
        stepperRight.setAcceleration(int(speed - 50));
        pre_r = data.right;
      }
      else if (data.right - pre_r >= 1 && data.right != 0 && pre_r != 0) {
        set_zero();
        stepperLeft.moveTo(50);
        stepperLeft.setMaxSpeed(speed + 100);
        stepperLeft.setSpeed(speed);
        while (stepperLeft.distanceToGo()) {
          stepperLeft.runSpeed();
        }
        set_zero();
        stepperLeft.moveTo(a);
        stepperRight.moveTo(a);
        stepperLeft.setMaxSpeed(speed);
        stepperLeft.setAcceleration(int(speed - 50));
        pre_l = data.left;
      }
      else if (data.left - pre_l >= 1 && data.left != 0 && pre_l != 0) {
        set_zero();
        stepperRight.moveTo(50);
        stepperRight.setMaxSpeed(speed + 100);
        stepperRight.setSpeed(speed);
        while (stepperRight.distanceToGo()) {
          stepperRight.runSpeed();
        }
        set_zero();
        stepperLeft.moveTo(a);
        stepperRight.moveTo(a);
        stepperRight.setMaxSpeed(speed);
        stepperRight.setAcceleration(int(speed - 50));
        pre_r = data.right;
      }

      previousMillis = millis();
    }

    runAndUpdate(stepperLeft);
    runAndUpdate(stepperRight);
    a = stepperLeft.distanceToGo();
  }
  set_zero();
}

void forwardConst() {
  set_zero();
  stepperLeft.moveTo(dis_to_step(5));
  stepperRight.moveTo(dis_to_step(5));
  stepperLeft.setMaxSpeed(500);
  stepperRight.setMaxSpeed(500);
  stepperLeft.setSpeed(400);
  stepperRight.setSpeed(400);
  while (stepperLeft.distanceToGo()) {
    runSpeedAndUpdate(stepperLeft);
    runSpeedAndUpdate(stepperRight);
  }
  set_zero();
}

void backwardConst() {
  set_zero();
  stepperLeft.moveTo(dis_to_step(-5));
  stepperRight.moveTo(dis_to_step(-5));
  stepperLeft.setMaxSpeed(500);
  stepperRight.setMaxSpeed(500);
  stepperLeft.setAcceleration(500);
  stepperRight.setAcceleration(500);
  while (stepperLeft.distanceToGo()) {
    runAndUpdate(stepperLeft);
    runAndUpdate(stepperRight);
  }
  set_zero();
}

void stop() {
  stepperLeft.stop();
  stepperRight.stop();
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
  Follow a wall
  Wall = 1<-- Front, 2<-- Back, 3<-- left, 4<-- right
*/
bool wallFollow() {
  data = RPC.call("read_lidars").as<struct lidar>();
  unsigned long previousMillis = 0;
  const unsigned long interval = 500;
  float d = 0;
  float p = 0;

  if (data.left && data.left <= 15) {
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

  if (data.left && data.left < 15) {
    wall = 3;
  } else if (data.right && data.right < 15) {
    wall = 4;
  }

  if (wall == 3) {
    delay(50);
    int upperThresh = 15;
    int lowerThresh = 10;
    data = RPC.call("read_lidars").as<struct lidar>();
    d = data.left;
    p = data.left;
    previousMillis = millis();
    while (d) {
      if (data.right && data.right < 15) {
        upperThresh = (data.left + data.right) / 2 + 2;
        lowerThresh = upperThresh - 5;
      } else {
        upperThresh = 15;
        lowerThresh = 10;
      }
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
        runSpeedAndUpdate(stepperLeft);
        runSpeedAndUpdate(stepperRight);
      }
      if (millis() - previousMillis >= interval) {
        data = RPC.call("read_lidars").as<struct lidar>();
        if (data.front && data.front < 10) {  // wall front
          redOn();
          grnOn();
          if (data.right && data.right < 15) {
            forward(-20, 700);
            spin(180, 300);
          } else {


            pivot(1, 90);
            pivot(0, -90);
            pivot(1, -90);
            forward(-30, 500);
            redOff();
            grnOff();
            getParallel(3);
          }
          data = RPC.call("read_lidars").as<struct lidar>();
          d = data.left;
          p = data.left;
        }
        if (data.left == 0 || data.left > 25) {  //lost wall on left
          pivot(0, 90);
          forward(30, 500);
          data = RPC.call("read_lidars").as<struct lidar>();
          if (data.left == 0) {
            return false;
          }
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
          runSpeedAndUpdate(stepperLeft);
          runSpeedAndUpdate(stepperRight);
          i = i + 1;
        }
        p = d;
        previousMillis = millis();
      }
    }
  } else if (wall == 4) {
    delay(50);
    int upperThresh = 15;
    int lowerThresh = 10;
    data = RPC.call("read_lidars").as<struct lidar>();
    d = data.right;
    p = data.right;

    previousMillis = millis();
    while (d) {
      if (data.left && data.left < 15) {
        upperThresh = (data.left + data.right) / 2 + 2;
        lowerThresh = upperThresh - 5;
      } else {
        upperThresh = 15;
        lowerThresh = 10;
      }
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
        runSpeedAndUpdate(stepperLeft);
        runSpeedAndUpdate(stepperRight);
      }
      if (millis() - previousMillis >= interval) {
        data = RPC.call("read_lidars").as<struct lidar>();
        if (data.front && data.front < 10) {
          redOn();
          grnOn();
          if (data.left && data.left < 25) {
            forward(-20, 700);
            spin(180, 300);
          } else {
            // Helper func <-- TurnInnerCorner
            pivot(0, -90);
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
          getParallel(4);
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
      runAndUpdate(stepperLeft);
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
        runAndUpdate(stepperRight);
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
    forward(offset, 500);
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
  // if(minValue == 100) //starting value
  // {
  //   minValue = 0;
  // }
  return minValue;
}

void randomWanderWallFollow() {
  struct lidar data = RPC.call("read_lidars").as<struct lidar>();
  int distThresh = 15;
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
  wallFollow();
  object = 0;
}

void onMqttMessage(int messageSize) {
  String message = "";

  // use the Stream interface to print the contents
  while (mqttClient.available()) {
    message = message + (char)mqttClient.read();
  }
  Serial.println(message);
  JsonDocument rec;
  deserializeJson(rec, message);

  if (rec["command"]) {
    String buff = rec["command"];
    command = buff;
    Serial.println(command);
  }
}

void followCommand() {
  if (command != premessage) {
    if (command == "forward") {  // prep forward
      Serial.println("here");
      set_zero();
      stepperLeft.moveTo(9000);
      stepperRight.moveTo(9000);
      stepperLeft.setMaxSpeed(300);
      stepperRight.setMaxSpeed(300);
      stepperLeft.setAcceleration(200);
      stepperRight.setAcceleration(200);
    } else if (command == "backward") {
      set_zero();
      stepperLeft.moveTo(-9000);
      stepperRight.moveTo(-9000);
      stepperLeft.setMaxSpeed(300);
      stepperRight.setMaxSpeed(300);
      stepperLeft.setAcceleration(200);
      stepperRight.setAcceleration(200);
    } else if (command == "spin_right") {
      set_zero();
      stepperLeft.moveTo(9000);
      stepperRight.moveTo(-9000);
      stepperLeft.setMaxSpeed(300);
      stepperRight.setMaxSpeed(300);
      stepperLeft.setAcceleration(200);
      stepperRight.setAcceleration(200);
    } else if (command == "spin_left") {
      set_zero();
      stepperLeft.moveTo(-9000);
      stepperRight.moveTo(9000);
      stepperLeft.setMaxSpeed(300);
      stepperRight.setMaxSpeed(300);
      stepperLeft.setAcceleration(200);
      stepperRight.setAcceleration(200);
    }
    premessage = command;
    Serial.println("__________");
    Serial.println(premessage);
  }

  if (command == "forward") {
    stepperLeft.run();
    stepperRight.run();

  } else if (command == "backward") {
    stepperLeft.run();
    stepperRight.run();

  } else if (command == "spin_right") {
    stepperLeft.run();
    stepperRight.run();

  } else if (command == "spin_left") {
    stepperLeft.run();
    stepperRight.run();

  } else if (command == "stop") {
    stop();
  }
}


void sendSensor(int interval) {
  if (millis() - previousMillis >= interval) {

    data = RPC.call("read_lidars").as<struct lidar>();

    doc["front"] = data.front;
    doc["back"] = data.back;
    doc["left"] = data.left;
    doc["right"] = data.right;

    serializeJson(doc, output);

    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(topic);
    mqttClient.print(output);
    mqttClient.endMessage();

    Serial.println();

    previousMillis = millis();
  }
}

void sendMap() {
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      doc["map"][i][j] = array[i][j];
    }
  }
  serializeJson(doc, output);
  // send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage(topic);
  mqttClient.print(output);
  mqttClient.endMessage();
}




void mapping() {


  array[1][1] = 0;
  traveled[tc][x] = 1;
  traveled[tc][y] = 1;
  current[x] = 1;
  current[y] = 1;
  tc = tc + 1;
  sendMap();
  printArray();
  bool done = false;

  int surrand[4];
  int c[4];

  while (done == false) {
    //read sensors
    data = RPC.call("read_lidars").as<struct lidar>();
    if (data.front < 30 && data.front != 0) {
      c[0] = 1;
    } else {
      c[0] = 0;
    }
    if (data.right < 30 && data.right != 0) {
      c[1] = 1;
    } else {
      c[1] = 0;
    }
    if (data.back < 30 && data.back != 0) {
      c[2] = 1;
    } else {
      c[2] = 0;
    }
    if (data.left < 30 && data.left != 0) {
      c[3] = 1;
    } else {
      c[3] = 0;
    }
    //Serial.println(global_theta);
    //Serial.println(data.front);
    //Serial.println(data.right);
    //Serial.println(data.back);
    //Serial.println(data.left);
    if (facing == 0) {
      surrand[0] = c[0];
      surrand[1] = c[1];
      surrand[2] = c[2];
      surrand[3] = c[3];
    } else if (facing == 3) {
      surrand[0] = c[1];
      surrand[1] = c[2];
      surrand[2] = c[3];
      surrand[3] = c[0];
    } else if (facing == 2) {
      surrand[0] = c[2];
      surrand[1] = c[3];
      surrand[2] = c[0];
      surrand[3] = c[1];
    } else {
      surrand[0] = c[3];
      surrand[1] = c[0];
      surrand[2] = c[1];
      surrand[3] = c[2];
    }
    Serial.print("surrand: ");
    Serial.print(surrand[0]);
    Serial.print(surrand[1]);
    Serial.print(surrand[2]);
    Serial.println(surrand[3]);



    int direction[0];
    int j = 0;
    for (int i = 0; i < 4; i++) {  // update map
      if (surrand[i] == 0) {
        direction[j] = i;
        j = j + 1;
        if (i == 0) {
          array[current[y] + 1][current[x]] = 0;
        } else if (i == 1) {
          array[current[y]][current[x] - 1] = 0;
        } else if (i == 2) {
          array[current[y] - 1][current[x]] = 0;
        } else {
          array[current[y]][current[x] + 1] = 0;
        }

      } else {
        if (i == 0) {
          array[current[y] + 1][current[x]] = 9;
        } else if (i == 1) {
          array[current[y]][current[x] - 1] = 9;
        } else if (i == 2) {
          array[current[y] - 1][current[x]] = 9;
        } else {
          array[current[y]][current[x] + 1] = 9;
        }
      }
    }

    sendMap();
    printArray();


    if (j > 2) {  //flag if have more than one option
      flag[x][flag_count] = current[x];
      flag[y][flag_count] = current[y];
      Serial.println("flag!");
      //Serial.println(flag_count);
      //Serial.print("x: ");
      //Serial.println(flag[x][flag_count]);
      //Serial.print("y: ");
      //Serial.println(flag[y][flag_count]);
      flag_count = flag_count + 1;
    }

    bool moved = false;
    Serial.println(j);
    for (int a = 0; a < j; a++) {
      Serial.println(direction[0]);
      if (direction[a] == 0 && moved == false) {  // move base on direction
        //Serial.println("move down");
        current[y] = current[y] + 1;
        moved = true;
        for (int i = 0; i < tc; i++) {
          if (traveled[i][x] == current[x] && moved == true) {
            if (traveled[i][y] == current[y]) {
              current[y] = current[y] - 1;
              moved = false;
              //break;
            }
          }
        }
      }
      if (direction[a] == 1 && moved == false) {
        //Serial.println("move left");
        current[x] = current[x] - 1;
        moved = true;
        for (int i = 0; i <= tc; i++) {
          if (traveled[i][x] == current[x] && moved == true) {
            if (traveled[i][y] == current[y]) {
              current[x] = current[x] + 1;
              moved = false;
              //break;
            }
          }
        }
      }
      if (direction[a] == 2 && moved == false) {
        //Serial.println("move up");
        current[y] = current[y] - 1;
        Serial.println(current[y]);
        moved = true;
        for (int i = 0; i <= tc; i++) {
          if (traveled[i][x] == current[x] && moved == true) {
            if (traveled[i][y] == current[y]) {
              current[y] = current[y] + 1;
              Serial.println(current[y]);
              moved = false;
              //break;
            }
          }
        }
      }
      if (direction[a] == 3 && moved == false) {
        //Serial.println("move right");
        current[x] = current[x] + 1;
        moved = true;
        for (int i = 0; i <= tc; i++) {
          if (traveled[i][x] == current[x] && moved == true) {
            if (traveled[i][y] == current[y]) {
              current[x] = current[x] - 1;
              moved = false;
              //break;
            }
          }
        }
      }
    }
    if (moved == false) {
      Serial.println("stuck");
      bool unknow = false;
      for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
          if (array[i][j] == 1) {
            unknow = true;
          }
        }
      }

      if (unknow == false) {
        done = true;
      }

      if (done == false) {
        bool backed = false;
        int k = 0;
        while (backed != true) {

          current[x] = traveled[tc - 2 - k][x];
          current[y] = traveled[tc - 2 - k][y];

          //Serial.println(current[x]);
          //Serial.println(current[y]);
          if (current[x] == flag[x][flag_count - 1] && current[y] == flag[y][flag_count - 1]) {
            backed = true;
          } else {
            GoToGrid(current[x], current[y]);
          }
          k = k + 1;
          if (k > tc) {
            Serial.print("finished");
            done = true;
          }
        }
      }
    }


    if (done == false) {
      GoToGrid(current[x], current[y]);




      traveled[tc][x] = current[x];
      traveled[tc][y] = current[y];



      Serial.println(tc);
      Serial.print("current Position: ");
      Serial.print(current[x]);
      Serial.print(" ");
      Serial.println(current[y]);

      for (int i = 0; i <= tc; i++) {
        Serial.print(traveled[i][x]);
        Serial.print("  ");
        Serial.println(traveled[i][y]);
      }



      tc = tc + 1;
      delay(500);
    }
  }
}

void printArray() {
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      if (j == 5) {
        Serial.println(array[i][j]);
      } else {
        Serial.print(array[i][j]);
        Serial.print(" ");
      }
    }
  }
}




void setup() {
  RPC.begin();
  init_stepper();
  global_theta = -3.14;

  Serial.begin(9600);
  delay(1500);

  //forward(100, 400);


  //initialize map
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      if (i == 0 || j == 0 || i == 5 || j == 5) {
        array[i][j] = 9;
      } else {
        array[i][j] = 1;
      }
    }
  }


  // attempt to connect to WiFi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1)
      ;
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  grnOn();
  delay(3000);
  grnOff();

  // set the message receive callback
  mqttClient.onMessage(onMqttMessage);

  Serial.print("Subscribing to topic: ");
  Serial.println(topic);
  Serial.println();

  // subscribe to a topic
  mqttClient.subscribe(topic);

  // topics can be unsubscribed using:
  // mqttClient.unsubscribe(topic);

  Serial.print("Waiting for messages on topic: ");
  Serial.println(topic);
  Serial.println();




  mapping();
}

void loop() {

  //sendSensor(500);
  //followCommand();

  //mqttClient.poll();
}
