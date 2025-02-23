/*
  Made by: Mitch Boucher and Everest Zhang
  Created: 2/5/2025
  Modified last: 2/11/2025

  This program is made to complete the final of ECE425
  This includes:
  Mapping a Maze by occupancy grid
  Metric path planning and following on occupancy grid
  Localization and path following on occupancy grid
  Getting to a goal through the Maze given the position of the robot's start and end position


*/
#include "Arduino.h"
#include "RPC.h"
#include <AccelStepper.h> //include the stepper motor library
#include <MultiStepper.h> //include multiple stepper motor library

// state LEDs connections
#define redLED 5         // red LED for displaying states
#define grnLED 6         // green LED for displaying states
#define ylwLED 7         // yellow LED for displaying states
#define enableLED 13     // stepper enabled LED
int leds[3] = {5, 6, 7}; // array of LED pin numbers

// define motor pin numbers
#define stepperEnable 48 // stepper enable pin on stepStick
#define rtStepPin 50     // right stepper motor step pin
#define rtDirPin 51      // right stepper motor direction pin
#define ltStepPin 52     // left stepper motor step pin
#define ltDirPin 53      // left stepper motor direction pin

// define sonar pin numbers
#define lf_sonar 4
#define rt_sonar 3

// define infared and lidar sensors
#define ft_lidar 8
#define bk_lidar 9
#define lf_lidar 10
#define rt_lidar 11

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin); // create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);  // create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;

#define stepperEnTrue false // variable for enabling stepper motor
#define stepperEnFalse true // variable for disabling stepper motor
#define max_speed 1500      // maximum stepper motor speed
#define max_accel 10000     // maximum motor acceleration

// Object detection variable (typically based on lidar)
bool object = 0;

// Dimention of the robot
const float wheel_width = 8.5; // diameter of the wheel (cm)
const float wheel_base = 23;   // length between the two wheel's center (cm)

// Global positioning of the object
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
struct lidar
{
  // this can easily be extended to contain sonar data as well
  float front;
  float back;
  float left;
  float right;
  float closestObj;
  // this defines some helper functions that allow RPC to send our struct (I found this on a random forum)
  MSGPACK_DEFINE_ARRAY(front, back, left, right); // https://stackoverflow.com/questions/37322145/msgpack-to-pack-structures https://www.appsloveworld.com/cplus/100/391/msgpack-to-pack-structures
} dist;

struct lidar data;

// a struct to hold lidar data
struct sonar
{
  // this can easily be extended to contain sonar data as well
  int left;
  int right;
  // this defines some helper functions that allow RPC to send our struct (I found this on a random forum)
  MSGPACK_DEFINE_ARRAY(left, right); // https://stackoverflow.com/questions/37322145/msgpack-to-pack-structures https://www.appsloveworld.com/cplus/100/391/msgpack-to-pack-structures
} dist2;

// read_lidars is the function used to get lidar data to the M7
struct lidar read_lidars()
{
  return dist;
}

// read_lidars is the function used to get lidar data to the M7
struct sonar read_sonars()
{
  return dist2;
}

// reads a lidar given a pin
int read_lidar(int pin)
{
  float d;
  int16_t t = pulseIn(pin, HIGH);
  d = (t - 1000) * 3 / 40;
  if (t == 0 || t > 1850 || d < 0)
  {
    d = 0;
  }
  return d;
}

// reads a sonar given a pin
int read_sonar(int pin)
{
  float velocity((331.5 + 0.6 * (float)(20)) * 100 / 1000000.0);
  uint16_t distance, pulseWidthUs;

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  digitalWrite(pin, HIGH);           // Set the trig pin High
  delayMicroseconds(10);             // Delay of 10 microseconds
  digitalWrite(pin, LOW);            // Set the trig pin Low
  pinMode(pin, INPUT);               // Set the pin to input mode
  pulseWidthUs = pulseIn(pin, HIGH); // Detect the high level time on the echo pin, the output high level time represents the ultrasonic flight time (unit: us)
  distance = pulseWidthUs * velocity / 2.0;
  if (distance < 0 || distance > 50)
  {
    distance = 0;
  }
  return distance;
}

/*
  The helper function will return the minimum (non-zero) lidar value, used for detecting if an object on anyside is within the threshold.
*/
float minLidar(float frontL, float backL, float leftL, float rightL)
{
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
  for (int i = 0; i < 4; i++)
  {
    if (ldrs[i] < minValue && ldrs[i] != 0)
    {
      minValue = ldrs[i];
    }
  }
  return minValue;
}

/*
  The helper function will read sensor data from M4 and print it using Serial
*/
void print_sensors()
{
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
void allOff()
{
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(leds[i], LOW);
  }
}

void redOn()
{
  digitalWrite(redLED, HIGH);
}
void grnOn()
{
  digitalWrite(grnLED, HIGH);
}
void ylwOn()
{
  digitalWrite(ylwLED, HIGH);
}
void redOff()
{
  digitalWrite(redLED, LOW);
}
void grnOff()
{
  digitalWrite(grnLED, LOW);
}
void ylwOff()
{
  digitalWrite(ylwLED, LOW);
}

void init_stepper()
{
  pinMode(rtStepPin, OUTPUT);                  // sets pin as output
  pinMode(rtDirPin, OUTPUT);                   // sets pin as output
  pinMode(ltStepPin, OUTPUT);                  // sets pin as output
  pinMode(ltDirPin, OUTPUT);                   // sets pin as output
  pinMode(stepperEnable, OUTPUT);              // sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse); // turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);                  // set enable LED as output
  digitalWrite(enableLED, LOW);                // turn off enable LED
  pinMode(redLED, OUTPUT);                     // set red LED as output
  pinMode(grnLED, OUTPUT);                     // set green LED as output
  pinMode(ylwLED, OUTPUT);                     // set yellow LED as output
  digitalWrite(redLED, HIGH);                  // turn on red LED
  digitalWrite(ylwLED, HIGH);                  // turn on yellow LED
  digitalWrite(grnLED, HIGH);                  // turn on green LED
  delay(2500 / 5);                             // wait 0.5 seconds
  digitalWrite(redLED, LOW);                   // turn off red LED
  digitalWrite(ylwLED, LOW);                   // turn off yellow LED
  digitalWrite(grnLED, LOW);                   // turn off green LED

  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  steppers.addStepper(stepperRight);          // add right motor to MultiStepper
  steppers.addStepper(stepperLeft);           // add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue); // turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);              // turn on enable LED
}

// Positioning Commands

/*
  convert travel distance (cm) to steps
*/
int dis_to_step(float dis)
{
  int step = dis * 360 / (3.14 * wheel_width * 0.45);
  return step;
}

/*
  Convert a number of steps into a value of distance (for global positioning)
  Distance is a value of cm
*/
float step_to_dis(int step)
{
  float dis = step * 3.14 * wheel_width * 0.45 / 360;
  return dis;
}

/*
  Set the current position and step counter to zero
*/
void set_zero()
{
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

void updateGlobalPosition()
{

  if (millis() - previousMillisGlobal >= intervalGlobal)
  {
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

// Simple Helper Functions

void runAndUpdate(AccelStepper &stepperMotor)
{
  stepperMotor.run();
  updateGlobalPosition();
}

void runSpeedAndUpdate(AccelStepper &stepperMotor)
{
  stepperMotor.runSpeed();
  updateGlobalPosition();
}

// Function returns a reference to the selected member <-- Created w/ chatGPT
int getLidarValue(const String &side)
{
  if (side == "front")
    return data.front;
  else if (side == "back")
    return data.back;
  else if (side == "left")
    return data.left;
  else
    return data.right; // Default to right if invalid input
}

void getParallel(int location)
{
  float goal_dis = 15.0; // cm
  unsigned long previousMillis = 0;
  const unsigned long interval = 50;
  struct lidar data = RPC.call("read_lidars").as<struct lidar>();
  if (location == 1 || location == 2)
  { // pivot 90 or till right sensor find the wall; then pivot the other way till left sensor find the wall
    set_zero();
    int dis = 90 * 3.14 * wheel_base / 180;
    int steps = dis_to_step(dis);
    stepperLeft.moveTo(-steps);
    stepperLeft.setMaxSpeed(200);
    stepperLeft.setAcceleration(200);
    while (!data.right || data.right > 15)
    { // nothing on right or more than 15
      runAndUpdate(stepperLeft);
      if (millis() - previousMillis >= interval)
      {
        data = RPC.call("read_lidars").as<struct lidar>();
        previousMillis = millis();
      }
      if (!stepperLeft.distanceToGo())
      {
        break;
      }
    }
    // finish
    if (data.right)
    { // if there is a wall on the right, jump to location = 4
      location = 4;
    }
    else
    {
      pivot(1, -90); // pivot back
      set_zero();
      int dis = -90 * 3.14 * wheel_base / 180;
      int steps = dis_to_step(dis);
      stepperRight.moveTo(steps);
      stepperRight.setMaxSpeed(200);
      stepperRight.setAcceleration(200);

      while (!data.left || data.left > 15)
      { // nothing on left
        runAndUpdate(stepperRight);
        if (millis() - previousMillis >= interval)
        {
          data = RPC.call("read_lidars").as<struct lidar>();
          previousMillis = millis();
        }
        if (!stepperRight.distanceToGo())
        {
          break;
        }
      }

      if (data.left)
      {
        location = 3;
      }
    }
  }
  if (location == 3)
  { // wall on left
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
    float a = float(offset) * curr / (pre - curr);                  // positive if going towards, negative if going away
    float theta = atan2(curr, abs(a)) * 57.3 * constrain(a, -1, 1); // in degree
    float delta = (-1) * a * (goal_dis - curr) / curr;
    Serial.print(pre);
    Serial.print("  ");
    Serial.print(curr);
    Serial.print("  ");
    Serial.print(delta);
    Serial.print("  ");
    Serial.println(theta);
    if (abs(theta) > 5)
    {
      forward(delta, 300);
    }
    if (delta > 0)
    {
      pivot(1, -theta);
    }
    else
    {
      pivot(0, -theta);
    }
  }
  if (location == 4)
  {
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
    float a = float(offset) * curr / (pre - curr);                         // positive if going towards, negative if going away
    float theta = atan2(curr, abs(a)) * 57.3 * constrain(a, -1, 1) * (-1); // in degree, opposite of wall on left
    float delta = (-1) * a * (goal_dis - curr) / curr;
    Serial.print(pre);
    Serial.print("  ");
    Serial.print(curr);
    Serial.print("  ");
    Serial.print(delta);
    Serial.print("  ");
    Serial.println(theta);
    if (abs(theta) > 5)
    { // only do this if the robot is really not parallel
      forward(delta, 200);
    }
    if (delta > 0)
    {
      pivot(1, -theta);
    }
    else
    {
      pivot(0, -theta);
    }
  }
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
void goToGoal(long x, long y)
{
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);

  // float correctionFactor = 0.95;
  float radian = atan2(y, x);
  int degree = radian * 57.3;
  long dis = sqrt(sq(x) + sq(y));
  if (degree <= 90 && degree >= -90)
  {
    spin(degree, 500);
    delay(50);
    forward(dis, 300);
  }
  else
  {
    spin(constrain(degree, -1, 1) * (-180) + degree, 500);
    delay(50);
    forward(-dis, 300);
  }
}

/*
  The robot will move to a set position, specified by the x and y value
  TODO: update to be non-blocking
*/
void goToGoalWithReturn(long x, long y)
{
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);

  // float correctionFactor = 0.95;
  float radian = atan2(y, x);
  int degree = radian * 57.3;
  long dis = sqrt(sq(x) + sq(y));
  if (degree <= 90 && degree >= -90)
  {
    spin(degree, 500);
    delay(50);
    forward(dis, 300);
    spin(-degree, 500);
  }
  else
  {
    spin(constrain(degree, -1, 1) * (-180) + degree, 500);
    delay(50);
    forward(-dis, 300);
    spin(-(constrain(degree, -1, 1) * (-180) + degree), 500);
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

void smartgoToGoal(int xGoal, int yGoal)
{
  object = 0;
  int wall = 0;
  unsigned long previousMillis = 0;
  const unsigned long interval = 50;
  previousMillis = millis();

  while (true)
  {
    while (object)
    { // while it detects an object in the way of the robot
      if (data.front < 15 && data.front)
      {
        set_zero();                              // set current position to new global 0
        int dis = 120 * 3.14 * wheel_base / 180; // pivot to side
        int steps = dis_to_step(dis);
        stepperLeft.moveTo(-steps);
        stepperLeft.setMaxSpeed(300);
        stepperLeft.setAcceleration(200);
        while (!data.right || data.right > 20)
        { // nothing on right or more than 20 cm

          runAndUpdate(stepperLeft); // step + update position based on step
          if (millis() - previousMillis >= interval)
          { // read sensors every once in a while
            data = RPC.call("read_lidars").as<struct lidar>();
            previousMillis = millis();
            Serial.print("Searching for right: "); // TODO: Remove because slows down?
            Serial.println(data.right);
          }
          if (!stepperLeft.distanceToGo())
          { // leave while loop if reached distance
            break;
          }
        }
        if (data.right && data.right < 20)
        { // if something is within range on right
        }
        else
        {
          pivot(0, -120); // pivot to the left
          object = 0;     // clear object flag to begin going towards goal
          break;
        }
      }
      if (data.right < 25 && data.right)
      { // if object within deadzone of right side
        stepperLeft.setMaxSpeed(2000);
        stepperRight.setMaxSpeed(2000);
        stepperLeft.setSpeed(200);
        stepperRight.setSpeed(200);
        int p = data.right; // set previous position
        int d = data.right; // set current position

        while (data.right && data.right < 25)
        {
          stepperLeft.setMaxSpeed(2000);
          stepperRight.setMaxSpeed(2000);
          stepperLeft.setSpeed(200);
          stepperRight.setSpeed(200);

          runSpeedAndUpdate(stepperLeft);
          runSpeedAndUpdate(stepperRight);
          if (millis() - previousMillis >= 500)
          {

            data = RPC.call("read_lidars").as<struct lidar>();
            d = data.right;
            if (!d)
            {
              break; // if no wall on right leave if statement
            }
            if ((p - d) > 0.7)
            { // if difference between position and previous is less than threshold, pivot
              pivotConst(0, 5, 100);
            }
            data = RPC.call("read_lidars").as<struct lidar>();
            p = data.right;
            previousMillis = millis();
          }
        } // no more wall on right
        forward(15, 600);
      }
      if (data.left < 25 && data.left)
      { // same as right, but for left sensor
        stepperLeft.setMaxSpeed(2000);
        stepperRight.setMaxSpeed(2000);
        stepperLeft.setSpeed(200);
        stepperRight.setSpeed(200);
        int p = data.left;
        int d = data.left;

        while (data.left && data.left < 25)
        {
          stepperLeft.setMaxSpeed(2000);
          stepperRight.setMaxSpeed(2000);
          stepperLeft.setSpeed(200);
          stepperRight.setSpeed(200);

          runSpeedAndUpdate(stepperLeft);
          runSpeedAndUpdate(stepperRight);
          if (millis() - previousMillis >= 500)
          {

            data = RPC.call("read_lidars").as<struct lidar>();
            d = data.left;
            if (!d)
            {
              break;
            }
            if ((p - d) > 0.7)
            {
              pivotConst(1, -5, 100);
            }
            data = RPC.call("read_lidars").as<struct lidar>();
            p = data.left;
            previousMillis = millis();
          }
        } // no more wall on left
        forward(15, 600);
      }
      object = 0;
      break;
    }

    float delta_x = xGoal - global_x; // difference between current and goal X
    float delta_y = yGoal - global_y; // difference between current and goal Y
    float radian = atan2(delta_y, delta_x) - global_theta;
    int degree = radian * 57.3;
    long dis = sqrt(sq(delta_x) + sq(delta_y));
    spin(degree, 400); // spin towards goal

    set_zero();
    stepperLeft.moveTo(dis_to_step(dis)); // set to move towards goal
    stepperRight.moveTo(dis_to_step(dis));
    stepperLeft.setMaxSpeed(600);
    stepperRight.setMaxSpeed(600);
    stepperLeft.setSpeed(400);
    stepperRight.setSpeed(400);

    while (stepperLeft.distanceToGo())
    { // Still have to move

      runSpeedAndUpdate(stepperLeft);
      runSpeedAndUpdate(stepperRight);

      if (millis() - previousMillis >= interval)
      { // update sensors every set time interval

        data = RPC.call("read_lidars").as<struct lidar>();
        Serial.print("going to goal: ");
        Serial.println(data.front);
        if ((data.front && data.front < 15) || (data.right && data.right < 25) || (data.left && data.left < 25))
        { // detect if object within threshold
          object = 1;
          break;
        }
        previousMillis = millis();
      }
    }
    set_zero();                 // set current position to new global 0
    delta_x = xGoal - global_x; // re-obtain distance values
    delta_y = yGoal - global_y;
    if (delta_x < 5 && delta_y < 5)
    { // if within 5cm of the goal, stop the loop
      break;
    }
  }
}

/*
  Turn the robot a number of degrees at a specified speed
  Produce a "tank turn" <-- moves both wheels opposite evenly
*/
void spin(int degree, int speed)
{
  set_zero();
  float correctionFactor = 0.97;
  int signOfDegree = constrain(degree, -1, 1);
  int dis = degree * correctionFactor * 3.14 * wheel_base / (2 * 180);
  int steps = dis_to_step(dis); //+ offset;
  stepperLeft.setMaxSpeed(speed);
  stepperRight.setMaxSpeed(speed);
  stepperLeft.setAcceleration(100.0);
  stepperRight.setAcceleration(100.0);

  stepperRight.moveTo(steps);
  stepperLeft.moveTo(-steps);

  while (stepperLeft.distanceToGo() != 0 && stepperRight.distanceToGo() != 0)
  {
    runAndUpdate(stepperLeft);
    runAndUpdate(stepperRight);
  }
  set_zero();
}
/*
  Turn in certain degree with certain radius (cm) left of robot is negative
*/
void turn(int degree, int radius, int speed)
{
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
void forward(int distance, int speed)
{
  set_zero();
  stepperLeft.moveTo(dis_to_step(distance));
  stepperRight.moveTo(dis_to_step(distance));
  stepperLeft.setMaxSpeed(speed);
  stepperRight.setMaxSpeed(speed);
  stepperLeft.setAcceleration(int(speed / 3));
  stepperRight.setAcceleration(int(speed / 3));
  while (stepperLeft.distanceToGo())
  {
    runAndUpdate(stepperLeft);
    runAndUpdate(stepperRight);
  }
  set_zero();
}

/*
  Robot pivot on one wheel. left wheel is 0 and right wheel is 1. degree can be both positive and negative
*/
void pivot(bool wheel, float degree)
{
  set_zero();
  int dis = degree * 3.14 * wheel_base / 180;
  int steps = dis_to_step(dis);
  if (wheel == 0)
  { // pivot on left wheel
    stepperRight.moveTo(steps);
    stepperRight.setMaxSpeed(700);
    stepperRight.setAcceleration(500);
    while (stepperRight.distanceToGo())
    {
      runAndUpdate(stepperRight);
    }
  }
  else
  {
    stepperLeft.moveTo(-steps);
    stepperLeft.setMaxSpeed(700);
    stepperLeft.setAcceleration(500);
    while (stepperLeft.distanceToGo())
    {
      runAndUpdate(stepperLeft);
    }
  }
  set_zero();
}

void pivotConst(bool wheel, float degree, int speed)
{
  set_zero();
  int dis = degree * 3.14 * wheel_base / 180;
  int steps = dis_to_step(dis);
  if (wheel == 0)
  { // pivot on left wheel
    stepperRight.moveTo(steps);
    stepperRight.setMaxSpeed(speed + 100);
    stepperRight.setSpeed(speed);
    while (stepperRight.distanceToGo())
    {
      runSpeedAndUpdate(stepperRight);
    }
  }
  else
  {
    stepperLeft.moveTo(-steps);
    stepperLeft.setMaxSpeed(speed + 100);
    stepperLeft.setSpeed(speed);
    while (stepperLeft.distanceToGo())
    {
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
void prepRandomMovement(int maxVal)
{
  if (stepperLeft.distanceToGo() == 0)
  {
    stepperLeft.setCurrentPosition(2000);
    int randomSteps = random(maxVal) + 5;
    int randomMaxSpeed = random(maxVal) % 200 + 150;
    int randomAcc = random(maxVal) % 100 + 150;
    stepperLeft.moveTo(randomSteps);
    stepperLeft.setMaxSpeed(randomMaxSpeed);
    stepperLeft.setAcceleration(randomAcc);
  }
  if (stepperRight.distanceToGo() == 0)
  {
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
void prepForward(int unitStep, int speed)
{
  if (stepperLeft.distanceToGo() == 0 && stepperRight.distanceToGo() == 0)
  {
    set_zero();
    stepperLeft.moveTo(dis_to_step(unitStep));
    stepperRight.moveTo(dis_to_step(unitStep));
    stepperLeft.setMaxSpeed(speed);
    stepperRight.setMaxSpeed(speed);
    stepperLeft.setAcceleration(int(speed / 3));
    stepperRight.setAcceleration(int(speed / 3));
    
  }
}

/*
  The robot will randomly wander around
  The type of random wandering is determined by each wheel independently getting a distance to travel, at a random speed and acceleration
  The distance, speed, and acceleration random value is limited by the constant maxValue
*/
void randomWanderNoSpin()
{
  allOff();
  grnOn(); // turnx on green LED

  int maxVal = 6000; // maximum value for random number
  // while(1){
  prepRandomMovement(maxVal);
  // }
  stepperLeft.run();
  stepperRight.run();
}

/*
  When sensing an object in any direction the robot will stop.
  The distance threshold that determines how close to an object counts is a parameter in the command.
*/
void collide(int distThresh)
{
  allOff(); // Turn off all LEDs
  // int distThresh = 15;
  while (!object)
  {                     // sensor != close)  //Move forward while not sensing wall
    grnOn();            // Turn on green LED
    prepRandomMovement(6000); // prepare to move forward
    stepperLeft.run();  // increment left motor
    stepperRight.run(); // increment right motor
    data = RPC.call("read_lidars").as<struct lidar>();
    if (data.right != 0 || data.back != 0 || data.front != 0 || data.left != 0)
    {
      if (data.right < distThresh || data.back < distThresh || data.front < distThresh || data.left < distThresh)
      {
        object = 1;
        grnOff();
      }
    }
  }
  redOn(); // turn on red LED
  data = RPC.call("read_lidars").as<struct lidar>();
  if (data.right == 0 & data.back == 0 & data.front == 0 & data.left == 0)
  {
    object = 0;
    redOff();
  }
  else if (data.right >= distThresh & data.back >= distThresh & data.front >= distThresh & data.left >= distThresh)
  {
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
void runAway(int distThresh)
{
  int p = 2;
  int tolerance = distThresh / 10;
  allOff();
  ylwOn();
  data = RPC.call("read_lidars").as<struct lidar>();

  if (data.closestObj < distThresh - tolerance)
  { // object present
    delay(100);
    data = RPC.call("read_lidars").as<struct lidar>(); // read again incase of missing data
    bool FB = 0;                                       // not on both front and back
    bool LR = 0;                                       // not on both left and right
    // int distThresh = 20;

    if (data.front <= distThresh && data.back <= distThresh && data.back && data.front)
    {                 // both front and back
      data.front = 0; // ignore front and back sensor data
      data.back = 0;
      FB = 1;
      if (!data.left && !data.right)
      {                // turn to right sensor and move if free space
        data.left = 4; // fake object to left <-- force to right
      }
    }
    else
    {
      if (data.front > distThresh - tolerance)
      { // if further then 30 cm, ignore
        data.front = 0;
      }
      if (data.back > distThresh - tolerance)
      {
        data.back = 0;
      }
    }

    // clear data above threshold
    if (data.left <= distThresh && data.right <= distThresh && data.left && data.right)
    {                // both left and right
      data.left = 0; // ignore left and right sensor data
      data.right = 0;
      LR = 1;
      if (!data.front && !data.back)
      {                // turn to front sensor and move if free space
        data.back = 4; // fake object to back
      }
    }
    else
    {
      if (data.left > distThresh - tolerance)
      {
        data.left = 0;
      }
      if (data.right > distThresh - tolerance)
      {
        data.front = 0;
      }
    }

    if (LR && FB)
    { // in a box
      // do nothing
    }
    else
    {
      int xSens = data.front - data.back; // total X sensor = front - back
      int ySens = data.left - data.right; // total Y sensor = left - right <-- Based on directions listed in lab 1
      goToGoal((constrain(xSens, -1, 1) * (-distThresh) + xSens) * p, (constrain(ySens, -1, 1) * (-distThresh) + ySens) * p);

      /*Test sense data
      Serial.println(xSens);
      Serial.println(ySens);
      Serial.println(constrain(xSens, -1, 1) * (-30) + xSens);
      Serial.println(constrain(ySens, -1, 1) * (-30) + ySens);
      Serial.println("-----");
      delay(500);
      */

      if (data.closestObj >= distThresh - tolerance) // check if object in threshold to leave loops
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
void follow(int distance)
{
  allOff();
  ylwOn();
  grnOn();
  data = RPC.call("read_lidars").as<struct lidar>();
  while (data.front)
  { // object in front of robot
    data = RPC.call("read_lidars").as<struct lidar>();

    stepperLeft.setCurrentPosition(0); // reset world position
    stepperRight.setCurrentPosition(0);

    int delta = data.front - distance; // difference between intended distance and actual
    if (delta < 2 && delta > -2)       // deadzone creation
    {
      delta = 0;
    }
    int sign = constrain(delta, -1, 1);
    stepperLeft.moveTo(sign * 5000); // determine if moving forward or backword based on delta
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
void smartWander(int collideDist, int runDist)
{
  allOff();
  collide(collideDist);
  delay(100);
  while (object)
  {
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
void smartFollow(int collideDist, int followDist)
{
  allOff(); // Turn off all LEDs
  collide(collideDist);
  delay(100);
  follow(followDist); //
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


/*
  Using the pregenerated occupancy grid, calculate a route to make it to the goal
  The route to the goal will be stored as an array of moves that the robot should take
  Inputs:
  startX - the current X position of the robot
  startY - the current Y position of the robot
  goalX - the final X position of the robot
  goalY - the final Y position of the robot
  occGrid[6][6] - Occupancy grid, 0 = unoccupied, 99 = occupied
  *moveList - char array of the list of moves necessary
  currentMoveNumber - current number of moves made (call @ 0 always)
*/
// global flag for NavMaze
bool doneFlag = false;
void navMaze(int startX, int startY, int goalX, int goalY, int occGrid[][6], char *moveList, int currentMoveNumber)
{
  int diffX = goalX - startX;
  int diffY = goalY - startY;
  char previousMove = 'X';
  if (currentMoveNumber != 0) // get Previous move from last iteration successful <-- will jump if goes backmultiple moves
  {
    previousMove = moveList[currentMoveNumber - 1];
  }

  // if reached goal set flag to true <-- starts pulling out of all iterations
  if (diffX == 0 && diffY == 0)
  {
    doneFlag = true;
  }

  // pull-out if done
  if (doneFlag)
    return;

  // Move in desired X direction
  if (diffX < 0 && occGrid[startY][startX - 1] == 0 && previousMove != 'R')
  {
    pathSpace(-1, 0, currentMoveNumber, moveList);
    navMaze(startX - 1, startY, goalX, goalY, occGrid, moveList, currentMoveNumber + 1);
  }
  else if (diffX > 0 && occGrid[startY][startX + 1] == 0 && previousMove != 'L')
  {
    pathSpace(1, 0, currentMoveNumber, moveList);
    navMaze(startX + 1, startY, goalX, goalY, occGrid, moveList, currentMoveNumber + 1);
  }

  // pull-out if done
  if (doneFlag)
    return;

  // Move in desired Y direction
  if (diffY < 0 && occGrid[startY - 1][startX] == 0 && previousMove != 'D')
  {
    pathSpace(0, -1, currentMoveNumber, moveList);
    navMaze(startX, startY - 1, goalX, goalY, occGrid, moveList, currentMoveNumber + 1);
  }
  else if (diffY > 0 && occGrid[startY + 1][startX] == 0 && previousMove != 'U')
  {
    pathSpace(0, 1, currentMoveNumber, moveList);
    navMaze(startX, startY + 1, goalX, goalY, occGrid, moveList, currentMoveNumber + 1);
  }

  // Move pseudorandomly (you must leave the goal to reach the goal - Sun Tzu)
  // Try to Move right
  if (!doneFlag && occGrid[startY][startX + 1] == 0 && previousMove != 'L') // space must be free AND didn't move exact opposite last turn
  {
    pathSpace(1, 0, currentMoveNumber, moveList);
    navMaze(startX + 1, startY, goalX, goalY, occGrid, moveList, currentMoveNumber + 1);
  }
  // Try to Move left
  if (!doneFlag && occGrid[startY][startX - 1] == 0 && previousMove != 'R')
  {
    pathSpace(-1, 0, currentMoveNumber, moveList);
    navMaze(startX - 1, startY, goalX, goalY, occGrid, moveList, currentMoveNumber + 1);
  }
  // Try to Move up
  if (!doneFlag && occGrid[startY - 1][startX] == 0 && previousMove != 'D')
  {
    pathSpace(0, -1, currentMoveNumber, moveList);
    navMaze(startX, startY - 1, goalX, goalY, occGrid, moveList, currentMoveNumber + 1);
  }
  // Try to Move down
  if (!doneFlag && occGrid[startY + 1][startX] == 0 && previousMove != 'U')
  {
    pathSpace(0, 1, currentMoveNumber, moveList);
    navMaze(startX, startY + 1, goalX, goalY, occGrid, moveList, currentMoveNumber + 1);
  }
  return; // dead-end
}

/*
  Add the char related to a certain move to the moveList <-- tracks good moves
*/
void pathSpace(int xDir, int yDir, int currMove, char *moveList)
{
  if (xDir > 0)
    moveList[currMove] = 'R'; // Right
  else if (xDir < 0)
    moveList[currMove] = 'L'; // Left
  else if (yDir > 0)
    moveList[currMove] = 'D'; // Down
  else if (yDir < 0)
    moveList[currMove] = 'U'; // Up
  else
    moveList[currMove] = 'X'; // N/A
}

// Helper Functions of path finding
/*
  Print out the 2D array occupancy grid to serial terminal for debugging
*/
void printOccGrid(int occGrid[][6])
{
  Serial.println("Occupancy Grid:");
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      Serial.print(occGrid[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

/*
  Print out the list of moves as an array to serial terminal for debugging
*/
void printMoves(char *moveList, int length = 16)
{
  for (int i = 0; i < length; i++)
  {
    Serial.print(moveList[i]);
    Serial.print(" ");
  }
  Serial.println();
}

/*

*/
void moveThroughMaze(char *moveList, int gridSize = 16, int unitStep = 47)
{
  // move until no next move
  int currentMoveNumber = 0;
  while (moveList[currentMoveNumber] != 'X')
  {
    switch (moveList[currentMoveNumber])
    {
    case 'U':
      goToGoalWithReturn(unitStep, 0);
      break;

    case 'D':
      goToGoalWithReturn(-unitStep, 0);
      break;

    case 'R':
      goToGoalWithReturn(0, -unitStep);
      break;

    case 'L':
      goToGoalWithReturn(0, unitStep);
      break;
    }
    currentMoveNumber += 1;
  }
}

void hallwayCorrect(int distanceThreshold)
{
  if(data.left < distanceThreshold && data.left)    //too closefrom Left
  {
    Serial.println("Run from Left");
    pivot(1, -30);
    pivot(0, 30);
    forward(5, 300);
  }
  if(data.right < distanceThreshold && data.right)  //too close to Right
  {
    Serial.println("Run from Right");
    pivot(0, 30);
    pivot(1, -30);
    forward(5, 300);
  }
}

/*
  Check the side Lidars for if there is an OPENING
  returns int: 0 = no opening, 1 = right opening, 2 = left opening, 3 = both open
*/
int checkSides(int distanceThreshold)
{
  int result = 0;
  // Serial.println("pre-data pull");
  data = RPC.call("read_lidars").as<struct lidar>();
  // Serial.println("post-data pull");

  if (!data.right || data.right > distanceThreshold)
  {
    result += 2;
  }
  if (!data.left || data.left > distanceThreshold)
  {
    result += 1;
  }
  return result;
}

bool checkFront(int distanceThreshold)
{
  data = RPC.call("read_lidars").as<struct lidar>();
  return (data.front && data.front < distanceThreshold);
}

void topologicalPathFollow(char *moveList, int unitStep = 47)
{
  //variables
  bool leftPassage = false;
  bool rightPassage = false;
  int distThreshold = 18;
  int currentMoveNumber = 0;
  int sideData = 0;
  int interval = 500;
  int previousMillis = millis();
  int moveSpeed = 300;
  // Serial.println(moveList);
  while (moveList[currentMoveNumber] != 'T')  //while not last move
  {
    while (!leftPassage && moveList[currentMoveNumber] == 'L' || !rightPassage && moveList[currentMoveNumber] == 'R') // no opening matching move option
    {
      // Serial.println(millis());
      // Check lidar
      if (millis() - previousMillis >= interval)
      {
        // Serial.println("checkingData");
        sideData = checkSides(distThreshold);
        // Serial.println("post-sideData");
        leftPassage = (sideData == 1 || sideData == 3);
        rightPassage = (sideData == 2 || sideData == 3);
        // Serial.println("passed passage checks");
        hallwayCorrect(distThreshold-5);
        previousMillis = millis();
        // Serial.print("Side Data: ");
        // Serial.println(sideData);
        // Serial.print("left passage");
        // Serial.println(leftPassage);
        // Serial.print("right passage");
        // Serial.println(rightPassage);
        print_sensors();
      }
      // Prepare to move forward(set moveTo if reached distance)
      prepForward(unitStep, moveSpeed);
      // run stepper motors
      stepperRight.run();
      stepperLeft.run();
    }
    if (moveList[currentMoveNumber] == 'L') // Want left move + possible
    {
      Serial.println("Moved Left");
      forward(5, 300);
      goToGoal(0, unitStep);
    }
    else // Want right move + possible
    {
      Serial.println("Moved Right");
      forward(5, 300);
      goToGoal(0, -unitStep);
    }
    leftPassage = false;
    rightPassage = false;
    currentMoveNumber += 1;
  }

  bool frontBlock = false;
  while(!frontBlock)
  {
    if (millis() - previousMillis >= interval)
      {
      frontBlock = checkFront(distThreshold);
      previousMillis = millis();
    }
    
    //prep forward movement
    prepForward(unitStep, moveSpeed);
    stepperLeft.run();
    stepperRight.run();

  }
  Serial.println("Terminated");
}

void setup()
{
  RPC.begin();
  init_stepper();

  Serial.begin(9600);
  delay(5000);

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



  char moves[16] = {'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
  int occGrid[6][6] = {99, 99, 99, 99, 99, 99,
                       99, 0, 99, 99, 0, 99,
                       99, 0, 0, 0, 0, 99,
                       99, 0, 99, 99, 0, 99,
                       99, 0, 99, 0, 0, 99,
                       99, 99, 99, 99, 99, 99};
  int startX = 1; // changes to current X per loop (1-4)
  int startY = 4; // changes to current Y per loop (1-4)
  int goalX = 3;  // Never changes (1-4)
  int goalY = 4;  // Never changes (1-4)

  // navMaze(startX, startY, goalX, goalY, occGrid, moves, 0);
  // printMoves(moves);
  // moveThroughMaze(moves);
  delay(5000);
  char topMove[4] = {'R', 'R', 'R', 'T'};
  topologicalPathFollow(topMove);

}

void loop()
{
  grnOn();
  delay(500);
  grnOff();
  delay(500);
}
