// This example will allow you to perform different experiments to characterize Rocky the
// Robot.  You will have to tweak the code in here to run your particular experiment.

#include <Wire.h>
#include <Balboa32U4.h>
#include <LSM6.h>
#include <LIS3MDL.h>

// - STATES THE PROGRAM CAN BE IN

#define STATE_WAITING 0     // waiting to start the race
#define STATE_RUNNING 1     // running the race
#define STATE_TERMINATING 2 // done with the race

// we start in the waiting state
int state = STATE_WAITING;

// - VARIABLES FOR TRACKING MOTOR COMMANDS

// these global variables will be used to set the wheel PWMs when state == STATE_TESTING
float leftMotorPWM = 0;
float rightMotorPWM = 0;

// - VARIABLES FOR TRACKING SENSORS AND EXPERIMENT EXECUTION PROGRESS

// 80mm diameter wheels, 12 clicks per revolution, Gear ratio of 162.5
#define METERS_PER_CLICK 3.141592*80.0*(1/1000.0)/12.0/(162.5)

Balboa32U4Encoders encoders;
Balboa32U4ButtonA buttonA;
Balboa32U4Motors motors;

// the previous time stamp in milliseconds (needed for computing wheel velocities)
float prev_time = -1.0;

// keep track of total distances travelled by each wheel
float distanceLeft = 0;
float distanceRight = 0;

// variables to keep track of the encoder values at the last timestamp so we can calculate wheel velocities
int lastCountsLeft = 0;
int lastCountsRight = 0;

// time to start running the race (1000ms after the A button is pressed)
float startRunning = 0;

// controls whether the current sensor values should be sent over the serial port (only true when the experiment is running)
bool shouldOutput = false;

void updatePWMs(float timeMS, float totalDistanceLeft, float totalDistanceRight, float vL, float vR) {
  /* You will fill this function in with your code to run the race.  The inputs to the function are:
   *    timeMS: the time in milliseconds since the race started
   *    totalDistanceLeft: the total distance travelled by the left wheel (meters) as computed by the encoders
   *    totalDistanceRight: the total distance travelled by the right wheel (meters) as computed by the encoders
   *    vL: the velocity of the left wheel (m/s) measured over the last 10ms
   *    vR: the velocity of the right wheel (m/s) measured over the last 10ms
   */
  // by default don't do anything (TODO: this applies to the default code which should just have the robot do nothing)
  float p = 5;
  float k = 20;
  float t = timeMS/1000;
  leftMotorPWM = leftMotorPWM + k*(p-totalDistanceLeft);
  rightMotorPWM = rightMotorPWM + k*(p-totalDistanceRight);
  if (leftMotorPWM > 200){
    leftMotorPWM = 200;
  }
  if (rightMotorPWM > 200){
    rightMotorPWM = 200;
  }
  if (leftMotorPWM < 0){
    leftMotorPWM = 0;
  }
  if (rightMotorPWM < 0){
    rightMotorPWM = 0;
  }
  if (t>10) {
    leftMotorPWM = 0;
    rightMotorPWM = 0;
  }
  
}

void setup()
{
  Wire.begin();
}

void loop()
{
  int currentCountsLeft, currentCountsRight;
  float vL, vR;
  
  if (buttonA.isPressed() && (state == STATE_WAITING || state == STATE_TERMINATING)) {
    buttonA.waitForRelease();
    state = STATE_RUNNING;
    startRunning = millis() + 1000.0;     // start after 1000ms
    shouldOutput = true;
  }

  if (buttonA.isPressed() && state == STATE_RUNNING) {
    buttonA.waitForRelease();
    state = STATE_TERMINATING;
    shouldOutput = false;
  }

  float curr_time = millis();
  float t = curr_time - startRunning;  // t is your time in ms

  float delta_t = (curr_time - prev_time)/1000.0;
  prev_time = curr_time;
  
  // may consider resetting everytime to avoid overflow
  currentCountsLeft = encoders.getCountsLeft();
  currentCountsRight = encoders.getCountsRight();
  
  vL = (currentCountsLeft - lastCountsLeft)/delta_t*METERS_PER_CLICK;
  vR = (currentCountsRight - lastCountsRight)/delta_t*METERS_PER_CLICK;

  distanceLeft += (currentCountsLeft - lastCountsLeft)*METERS_PER_CLICK;
  distanceRight += (currentCountsRight - lastCountsRight)*METERS_PER_CLICK;
  
  lastCountsLeft = currentCountsLeft;
  lastCountsRight = currentCountsRight;

  if (state == STATE_RUNNING) {
    if (t >= 0 && t < 10000) {
      updatePWMs(t, distanceLeft, distanceRight, vL, vR);
    } else {
      // don't jump the gun
      leftMotorPWM = 0;
      rightMotorPWM = 0;
    }
    motors.setLeftSpeed(leftMotorPWM);
    motors.setRightSpeed(rightMotorPWM);
  }
  if (state == STATE_TERMINATING) {
     motors.setLeftSpeed(0);
     motors.setRightSpeed(0);
     Serial.println("-1");
  }
  if (shouldOutput) {
    Serial.print(t/1000.0);
    Serial.print(" ");
    Serial.print(leftMotorPWM);
    Serial.print(" ");
    Serial.print(rightMotorPWM);
    Serial.print(" ");
    Serial.print(distanceLeft);
    Serial.print(" ");
    Serial.print(distanceRight);
    Serial.print(" ");
    Serial.print(vL);
    Serial.print(" ");
    Serial.println(vR);
  }
  delay(10);
}
