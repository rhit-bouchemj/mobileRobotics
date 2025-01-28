/*MultiCore_Sensors_M4server.ino
based off work of Justin Dewitt 12.12.23
CAB 12.26.23
This code will show how to run the same code on the M4 and M7 to server and client, respectively
M4 will read sensor data and send to M7 to run the state machine
upload M4 first and then turn on M7 serial monitor
    Read more here: Guide to GIGA R1 Dual Cores | Arduino Documentation
  https://docs.arduino.cc/tutorials/giga-r1-wifi/giga-dual-core
  https://docs.arduino.cc/tutorials/giga-r1-wifi/cheat-sheet
  To upload to the main core click Tools->Flash Split: "1.5MB M7 + 0.5MB M4""
  Then click Tools->Target Core" "Main Core"
  Click Check to Verify and Right Arrow to Upload
  Then Click Tools->Target Core: "M4 Co-processor"
  Click Check to Verify and Right Arrow to Upload
  M4 is the server and M7 is the client */

/*
  Modified by: Mitch Boucher and Evereset Zhang
  Modified last: 1/24/2025
*/

#include "Arduino.h"
#include "RPC.h"

#define frontLdr 8
#define backLdr 9
#define leftLdr 10
#define rightLdr 11
#define leftSnr 7
#define rightSnr 6
#define led 12

/*Read about RTOS here
https://os.mbed.com/docs/mbed-os/v6.16/mbed-os-api-doxy/namespacertos_1_1_kernel.html#details 
https://dumblebots.com/2020/04/02/using-mbed-with-arduino-arm-boards/   */

using namespace rtos;  //use real time OS methods 
Thread sensorThread;    //rtos Thread instance

//Array that holds the lidar values (to stop declaration)
int ldrs[4];


// a struct to hold lidar data
struct lidar {
  // this can easily be extended to contain sonar data as well
  float front;
  float back;
  float left;
  float right;
  float closeObj;
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

// read_sonars is the function used to get lidar data to the M7
struct sonar read_sonars() {
  return dist2;
}

// reads a lidar given a pin
float read_lidar(int pin) {
  float d;
  int16_t t = pulseIn(pin, HIGH, 100000);
  d = (float(t) - 1000.0) * 3.0 / 40.0;
  if (t == 0 || t > 1850 || d < 0) { d = 0; }
  return d;
}


/*
  get the minimum value for the lidar detectors to see how close to a wall they are (0 = max value too so don't include, 100 = newMax)
*/
float minLidar(int frontL, int backL, int leftL, int rightL) {
  // ldrs[0] = frontL;
  // ldrs[1] = backL;
  // ldrs[2] = leftL;
  // ldrs[3] = rightL;

  float minValue = 100.0;

  if (frontL != 0 && frontL < minValue) {
    minValue = frontL;
  }
  if (frontL != 0 && backL < minValue) {
    minValue = backL;
  }
  if (frontL != 0 && leftL < minValue) {
    minValue = leftL;
  }
  if (frontL != 0 && rightL < minValue) {
    minValue = rightL;
  }

  // for(int i = 0; i < sizeof(ldrs); i++)
  // {
  //   if(ldrs[i] < minValue && ldrs[i] != 0)
  //   {
  //     minValue = ldrs[i];
  //   }
  // }
  // if(minValue == 100) //starting value
  // {
  //   minValue = 0;
  // }
  return minValue;
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

/**
 * Returns the CPU that's currently running the sketch (M7 or M4)
 * Note that the sketch has to be uploaded to both cores. 
 **/
String currentCPU() {
  if (HAL_GetCurrentCPUID() == CM7_CPUID) {
    return "M7";
  } else {
    return "M4";
  }
}

void callSensorReadFromM7() {
  while (true) {
    delay(50);
    dist.front = read_lidar(frontLdr);
    dist.back = read_lidar(backLdr);
    dist.left = read_lidar(leftLdr);
    dist.right = read_lidar(rightLdr);
    dist2.right = read_sonar(rightSnr);
    dist2.left = read_sonar(leftSnr);

    RPC.println(currentCPU() + ": front: " 
      + String(dist.front) + ", back: " + 
      String(dist.back) + ", left: " + String(dist.left) + 
      ", right: " + String(dist.right));
    
    RPC.println(currentCPU() + ": front left: " + String(dist2.left) + 
    ", front right: " + String(dist2.right));

    struct lidar data = RPC.call("read_lidars").as<struct lidar>();
    struct sonar data2 = RPC.call("read_sonars").as<struct sonar>();
    RPC.println(currentCPU() + ": front: " + String(data.front) + 
      ", back: " +     String(data.back) + ", left: " + 
      String(data.left) + ", right: " + String(data.right));
    RPC.println(currentCPU() + ": front left: " + String(data2.left) + 
    ", front right: " + String(data2.right));
  }
}

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(12, OUTPUT);

  // Initialize RPC library; this also boots the M4 core
  RPC.begin();
  Serial.begin(115200);
  delay(100);
  Serial.println("starting...");

  if (currentCPU() == "M7") {
    sensorThread.start(callSensorReadFromM7);
  }

  if (currentCPU() == "M4") {
    RPC.bind("read_lidars", read_lidars);
    RPC.bind("read_sonars", read_sonars);
  }
}

void loop() {
  if (currentCPU() == "M4") {
    delayMicroseconds(50);
    dist.front = read_lidar(frontLdr);
    dist.back = read_lidar(backLdr);
    dist.left = read_lidar(leftLdr);
    dist.right = read_lidar(rightLdr);
    dist.closeObj = minLidar(dist.front, dist.back, dist.left, dist.right);
    // dist2.right = read_sonar(rightSnr);    //Slows down readings A LOT (Avoid if possible)
    // dist2.left = read_sonar(leftSnr);
   }
}

