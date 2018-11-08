// This code should help you get started with your balancing robot.
// The code performs the following steps
// Calibration phase:
//  In this phase the robot should be stationary lying on the ground.
//  The code will record the gyro data for a couple of seconds to zero
//  out any gyro drift.
//
//  The robot has a hardcoded angle offset between the lying down and the
//    standing up configuration.  This offset can be modified in Balance.cpp around the lines below:
//
//      // this is based on coarse measurement of what I think the angle would be resting on the flat surface.
//      // this corresponds to 94.8 degrees
//      angle = 94827-6000;
//
// Waiting phase:
//  The robot will now start to integrate the gyro over time to estimate
//  the angle.  Once the angle gets within +/- 3 degrees of vertical,
//  we transition into the armed phase.  A buzzer will sound to indicate
//  this transition.
//
// Armed phase:
//  The robot is ready to go, however, it will not start executing its control
//  loop until the angle leaves the region of [-3 degrees, 3 degrees].  This
//  allows you to let go of your robot, and it won't start moving until it's started
//  to fall just a little bit.  Once it leaves the region around vertical, it enters
//  the controlled phase.
//
// Controlled phase:
//  Here you can implement your control logic to do your balancing (or any of the
//  other Olympic events.


#include <Balboa32U4.h>
#include "Balance.h"

#define METERS_PER_CLICK 3.141592*80.0*(1/1000.0)/12.0/(162.5)
#define MOTOR_MAX 300
#define MAX_SPEED 0.75  // m/s
#define FORTY_FIVE_DEGREES_IN_RADIANS 0.78
#define DEBUG false
#define CMD_TIME 562.3

float desiredDistanceLeft = .3;
float desiredDistanceRight = .3;

extern int32_t angle_accum;
extern int32_t speedLeft;
extern int32_t driveLeft;
extern int32_t distanceRight;
extern int32_t speedRight;
extern int32_t distanceLeft;
extern int32_t distanceRight;
float vLsetpoint, vRsetpoint;

float vL, vR, totalDistanceLeft, totalDistanceRight;
int cmd_index = -1;
float leftMotorPWM = 0;
float rightMotorPWM = 0;
float eangle_rad_accum = 0;

float time_since_reset = 0;

static float pLarray[59] = {
  0,
  0.0427663193364274,
  0.0910215750001118,
  0.143506701809385,
  0.198951125773988,
  0.256070778405974,
  0.313583166905269,
  0.370231786244584,
  0.424816942693353,
  0.476233831147925,
  0.52352422247361,
  0.565959131877499,
  0.603189208601391,
  0.63551082228834,
  0.66421456199213,
  0.691714881387355,
  0.72103670294765,
  0.754842714468197,
  0.794797764069619,
  0.841590765920162,
  0.895243715280906,
  0.955371195003213,
  1.02132966905478,
  1.09229911988164,
  1.16733497636652,
  1.24540936693911,
  1.32544970222929,
  1.40637780211172,
  1.48715093728866,
  1.5668054880413,
  1.64450358801556,
  1.71958252779959,
  1.79160506234255,
  1.86040490575564,
  1.92611458598618,
  1.98915475673215,
  2.05016674554312,
  2.10990103991722,
  2.16911864978668,
  2.22854979969344,
  2.28887389062486,
  2.35065791732311,
  2.41426146014401,
  2.47976512323387,
  2.54695225225501,
  2.61533428916478,
  2.68419833185381,
  2.75266129083228,
  2.81972300721578,
  2.88431566273341,
  2.94534912571524,
  3.00175288869085,
  3.05251612102625,
  3.09672957217146,
  3.13364129990678,
  3.16277763375694,
  3.18447340013945,
  3.20741003393739,
  3.24491487550203
};

static float pRarray[59] = {
  0,
  0.0501866418449803,
  0.104482469810305,
  0.162256142656533,
  0.222658238179021,
  0.284726079666941,
  0.347461392485997,
  0.40989803739377,
  0.471169328842761,
  0.530580880951311,
  0.587691427810643,
  0.642395091086795,
  0.694975427453988,
  0.746064651911924,
  0.796450884963055,
  0.846848701791673,
  0.8979109945354,
  0.950448257566672,
  1.0054546633555,
  1.0638546605688,
  1.12625624745588,
  1.19286520154186,
  1.26351336244624,
  1.33772701533489,
  1.41479849301061,
  1.49385030816242,
  1.57389127488617,
  1.65386665072039,
  1.73270446902051,
  1.80935995121114,
  1.88285988706588,
  1.95234946622907,
  2.01714549270917,
  2.07680222536977,
  2.13119777005407,
  2.18064366675366,
  2.22599399421472,
  2.2686761395769,
  2.31053335663344,
  2.3534610493201,
  2.39900171361321,
  2.44810911169934,
  2.50113251910341,
  2.55792419470913,
  2.6179678286072,
  2.68048416982374,
  2.74450974366643,
  2.80895645383935,
  2.87265949357968,
  2.93441815912989,
  2.99303181872974,
  3.04733165835004,
  3.09620733974128,
  3.13862507257062,
  3.17362525313052,
  3.20024901959985,
  3.21705493345786,
  3.21469042767403,
  3.1870292678098
};

static float vRarray[59] = {
  0.0853758004844342,
  0.0931200776533707,
  0.0999907011436212,
  0.105489524048318,
  0.10933905752828,
  0.11141398477978,
  0.111713013565246,
  0.110351730181021,
  0.107568269439734,
  0.103737304619235,
  0.099384419366098,
  0.0951768833007014,
  0.0918325073979931,
  0.0898735332485744,
  0.08933221919643,
  0.0899147325888524,
  0.0916955216133386,
  0.0951606726841435,
  0.100477376476593,
  0.107230459764631,
  0.114709595180789,
  0.122194555293023,
  0.129075604571441,
  0.134875730986956,
  0.139239861955071,
  0.141919080846783,
  0.142757919190086,
  0.14168579758668,
  0.138712035195501,
  0.133924011064743,
  0.127488889084802,
  0.119660611223944,
  0.110795606231899,
  0.101381987080256,
  0.0920834483078965,
  0.0837778651261938,
  0.0775169780116005,
  0.0742880987409043,
  0.074583004011675,
  0.0780953964719545,
  0.0838764064848055,
  0.0907809888340879,
  0.0978042465694536,
  0.104183366777379,
  0.109370295416746,
  0.112977899504878,
  0.114738094357143,
  0.114475730328336,
  0.112093125456443,
  0.107560288973115,
  0.100907467715288,
  0.0922175790990415,
  0.0816156830758092,
  0.06924897316826,
  0.0552341110619777,
  0.0394570736950621,
  0.0203155319568554,
  -0.0287252306481735,
  -0.0696555901990871
};

static float vLarray[59] = {
  0.0706130203468907,
  0.0814914330304957,
  0.0901351974394038,
  0.09653556702284,
  0.100660355628765,
  0.102493754322757,
  0.10205717619564,
  0.0994216373400078,
  0.0947181790618944,
  0.0881532448552737,
  0.0800417093564888,
  0.0708840378003287,
  0.0615299870012449,
  0.0534263959283693,
  0.0486625073318409,
  0.049146261676515,
  0.0551409369802811,
  0.0650949164433391,
  0.0770108591667231,
  0.0894150591446856,
  0.101409231119857,
  0.112442640587309,
  0.122147985795492,
  0.130264887044824,
  0.136610731699742,
  0.141071862572524,
  0.143602891681713,
  0.144229322698315,
  0.143051737825436,
  0.140250919247814,
  0.136093344378814,
  0.130935506202587,
  0.12522280757073,
  0.119473550151238,
  0.114232205877077,
  0.10997906273135,
  0.107018690443432,
  0.105434762054563,
  0.105181029046212,
  0.106194247349317,
  0.108356899884829,
  0.111386715048768,
  0.114828265892937,
  0.11814476108472,
  0.120815750184998,
  0.122394619713091,
  0.122530071798747,
  0.120968108862998,
  0.117546354652537,
  0.112186545259058,
  0.104887583836568,
  0.0957203634675223,
  0.0848262401647772,
  0.0724251418273454,
  0.0588566298451004,
  0.0447708555794794,
  0.0323932010290774,
  0.0491841757544692,
  0.08420708900036
};

//initializing controller constants
//float Kp = 6.78;
//float Ki = 50;
//float Jp = 630;
//float Ji = 800;
//float Zp = 0.86;
//float Yp = 0.06; // Heading //0.06

/*
   Notes:
   For survivor like events, having aggressive constants works well, but agressive constants tend to make robot prone to falling over
   For battlebots, having more forgiving constants makes robot very stable, but introduces more drift (can be combated with teleoperated control) - position block probably unnecessary

*/
float Kp = 6.78;
float Ki = 50;
float Jp = 316;
float Ji = 800;
float Zp = .50;//.86;
float Yp = 0;



//Works, still pretty touchy
//float Kp = 6.78;
//float Ki = 50;
//float Jp = 630;
//float Ji = 800;
//float Zp = 0.86;

// THIS WORKS REALLY WELL
//float Kp = 6.78;
//float Ki = 50;
//float Jp = 316;
//float Ji = 800;
//float Zp = 0.86;//2;//0.86;
//float Yp = 0;//0.06; // THIS WORKS

//Works, but really touchy and still drifts. Checked output, position based angle adjustment works well but robot doesn't respond
//float Kp = 6.78;
//float Ki = 50;
//float Jp = 316;
//float Ji = 800;
//float Zp = 0.55;
//float Yp = 0.0;

// Works, but really aggressive
//float Kp = 6.78;
//float Ki = 50;
//float Jp = 720;
//float Ji = 800;
//float Zp = 0;

//Working, somewhat aggressive
//float Kp = 6.78;
//float Ki = 50;
//float Jp = 426;
//float Ji = 800;
//float Zp = 0;


//float Kp = 5;
//float Ki = 70;
//float Jp = 88;
//float Ji = 0;
// R2
//float Kp = 370;
//float Ki = 1340;
//float Jp = 88;


float angleDesired = 0;


void balanceDoDriveTicks();

extern int32_t displacement;
int32_t prev_displacement = 0;

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
Balboa32U4Buzzer buzzer;
Balboa32U4ButtonA buttonA;

//void updatePWMs(float leftMotorPWM, float rightMotorPWM) {
//  /* You will fill this function in with your code to run the race.  The inputs to the function are:
//        totalDistanceLeft: the total distance travelled by the left wheel (meters) as computed by the encoders
//        totalDistanceRight: the total distance travelled by the right wheel (meters) as computed by the encoders
//        vL: the velocity of the left wheel (m/s) measured over the last 10ms
//        vR: the velocity of the right wheel (m/s) measured over the last 10ms
//        angleRad: the angle in radians relative to vertical (note: not the same as error)
//        angleRadAccum: the angle integrated over time (note: not the same as error)
//  */
//
//  rightMotorPWM =
//
//  if (DEBUG) {
//    Serial.print("Final Left PWM: "); Serial.println(leftMotorPWM);
//    Serial.print("Final Right PWM: "); Serial.println(rightMotorPWM);
//  }
//
//  if (leftMotorPWM > 300) {
//    leftMotorPWM = 300;
//  }
//  else if (leftMotorPWM < -300) {
//    leftMotorPWM = -300;
//  }
//
//  if (rightMotorPWM > 300) {
//    rightMotorPWM = 300;
//  }
//  else if (rightMotorPWM < -300) {
//    rightMotorPWM = -300;
//  }
//}

uint32_t prev_time;

void setup()
{
  // I know this should be somwhere between 90825 and 90750
  angle = 90900;//90750;
  Serial.begin(9600);
  prev_time = 0;
  ledYellow(0);
  ledRed(1);
  balanceSetup();
  ledRed(0);
  angle_accum = 0;
  ledGreen(0);
  ledYellow(0);
}

extern int16_t angle_prev;
int16_t start_flag = 0;
int16_t armed_flag = 0;
int16_t start_counter = 0;
void lyingDown();
extern bool isBalancingStatus;
extern bool balanceUpdateDelayedStatus;

void newBalanceUpdate()
{
  static uint32_t lastMillis;
  uint32_t ms = millis();

  if ((uint32_t)(ms - lastMillis) < UPDATE_TIME_MS) {
    return;
  }
  balanceUpdateDelayedStatus = ms - lastMillis > UPDATE_TIME_MS + 1;
  lastMillis = ms;

  // call functions to integrate encoders and gyros
  balanceUpdateSensors();

  //  if (imu.a.x < 0)
  //  {
  //    lyingDown();
  //    isBalancingStatus = false;
  //  }
  //  else
  //  {
  //    isBalancingStatus = true;
  //  }
}


void loop()
{
  uint32_t cur_time = 0;
  static uint32_t prev_print_time = 0;   // this variable is to control how often we print on the serial monitor
  static float angle_rad;                // this is the angle in radians
  static float angle_rad_accum = 0;      // this is the accumulated angle in radians
  static float error_ = 0;      // this is the accumulated velocity error in m/s
  static float error_left_accum = 0;      // this is the accumulated velocity error in m/s
  static float error_right_accum = 0;      // this is the accumulated velocity error in m/s

  cur_time = millis();                   // get the current time in miliseconds

  newBalanceUpdate();                    // run the sensor updates. this function checks if it has been 10 ms since the previous

  if (angle > 3000 || angle < -3000)     // If angle is not within +- 3 degrees, reset counter that waits for start
  {
    start_counter = 0;
  }

  bool shouldPrint = cur_time - prev_print_time > 105;
  shouldPrint = false; // ELEPHANT - I just set this to false so that it wouldn't be printing all this extra stuff for troubleshooting
  if (shouldPrint)  // do the printing every 105 ms. Don't want to do it for an integer multiple of 10ms to not hog the processor
  {
    Serial.print(angle_rad);
    Serial.print("\t");
    Serial.print(angle_rad_accum);
    Serial.print("\t");
    Serial.print(leftMotorPWM);
    Serial.print("\t");
    Serial.print(rightMotorPWM);
    Serial.print("\t");
    Serial.print(vL);
    Serial.print("\t");
    Serial.print(vR);
    Serial.print("\t");
    Serial.print(totalDistanceLeft);
    Serial.print("\t");
    Serial.println(totalDistanceRight);

    prev_print_time = cur_time;
    /* Uncomment this and comment the above if doing wireless
            Serial1.print(angle_rad);
            Serial1.print("\t");
            Serial1.print(angle_rad_accum);
            Serial1.print("\t");
            Serial1.print(PWM_left);
            Serial1.print("\t");
            Serial1.print(PWM_right);
            Serial1.print("\t");
            Serial1.print(vL);
            Serial1.print("\t");
            Serial1.println(vR);
    */
  }

  float delta_t = (cur_time - prev_time) / 1000.0;

  // handle the case where this is the first time through the loop
  if (prev_time == 0) {
    delta_t = 0.01;
  }

  // every UPDATE_TIME_MS, check if angle is within +- 3 degrees and we haven't set the start flag yet
  if (cur_time - prev_time > UPDATE_TIME_MS && angle > -3000 && angle < 3000 && !armed_flag && !start_flag)
  {
    // increment the start counter
    start_counter++;
    // If the start counter is greater than 30, this means that the angle has been within +- 3 degrees for 0.3 seconds, then set the start_flag
    if (start_counter > 30)
    {
      armed_flag = 1;
      buzzer.playFrequency(DIV_BY_10 | 445, 1000, 15);
    }
  }

  // angle is in millidegrees, convert it to radians and subtract the desired theta
  angle_rad = ((float)angle) / 1000 / 180 * 3.14159;

  // only start when the angle falls outside of the 3.0 degree band around 0.  This allows you to let go of the
  // robot before it starts balancing
  if (cur_time - prev_time > UPDATE_TIME_MS && (angle < -3000 || angle > 3000) && armed_flag)
  {
    start_flag = 1;
    armed_flag = 0;
    angle_rad_accum = 0.0;
    error_left_accum = 0.0;
    error_right_accum = 0.0;
  }

  // every UPDATE_TIME_MS, if the start_flag has been set, do the balancing
  if (cur_time - prev_time > UPDATE_TIME_MS && start_flag)
  {
    // set the previous time to the current time for the next run through the loop
    prev_time = cur_time;

    // speedLeft and speedRight are just the change in the encoder readings
    // wee need to do some math to get them into m/s
    vL = METERS_PER_CLICK * speedLeft / delta_t;
    vR = METERS_PER_CLICK * speedRight / delta_t;

    totalDistanceLeft = METERS_PER_CLICK * distanceLeft;
    totalDistanceRight = METERS_PER_CLICK * distanceRight;
    angle_rad_accum += angle_rad * delta_t;


    // CONTROLLER STARTS HERE ---------------------------------------------------------------------------------
    desiredDistanceRight = pRarray[cmd_index];
    desiredDistanceLeft = pLarray[cmd_index];
    float setpoint = (totalDistanceRight + desiredDistanceRight + totalDistanceLeft + desiredDistanceLeft) / 2;
    if (setpoint > .2) {
      setpoint = .2;
    }
    if (setpoint < -.2) {
      setpoint = -.2;
    }

    if (cur_time - time_since_reset > CMD_TIME)
    {
      cmd_index++;
      time_since_reset = cur_time;
    }

    float E_angle = angle_rad - (angleDesired - (Zp * setpoint)); // Throwing in a squared term just because (but actually because its slow to respond when it drifts)

    eangle_rad_accum += E_angle * delta_t;
    vLsetpoint = vLarray[cmd_index];
    vRsetpoint = vRarray[cmd_index];
    Serial.print("vLsetpoint"); Serial.println(vLsetpoint);
    Serial.print("vRsetpoint"); Serial.println(vRsetpoint);

    float vLDesired = Kp * E_angle + Ki * eangle_rad_accum + vLsetpoint;//- Yp;
    //Serial.print("vLDesired"); Serial.println(vLDesired);
    float vRDesired = Kp * E_angle + Ki * eangle_rad_accum + vRsetpoint;//+ Yp;
    //Serial.print("vRDesired"); Serial.println(vRDesired);
    float E_vL = vLDesired - vL;
    float E_vR = vRDesired - vR;

    //Serial.print("Angle"); Serial.println(angle_rad);
    //Serial.print("Angle Desired"); Serial.println(angleDesired);
    //Serial.print("Setpoint"); Serial.println(setpoint);
    //Serial.print("Setpoint contribution"); Serial.println(Zp * setpoint / 2);
    //Serial.print("Error Angle: "); Serial.println(E_angle);
    //Serial.print("Angle adjustment: ");Serial.println((angleDesired - (Zp * (totalDistanceLeft + totalDistanceRight) / 2)));

    error_left_accum += (vL - vLDesired) * delta_t;
    error_right_accum += (vR - vRDesired) * delta_t;

    leftMotorPWM = Jp * E_vL - Ji * error_left_accum;
    rightMotorPWM = Jp * E_vR - Ji * error_right_accum;

    if (leftMotorPWM > 300) {
      leftMotorPWM = 300;
    }
    else if (leftMotorPWM < -300) {
      leftMotorPWM = -300;
    }

    if (rightMotorPWM > 300) {
      rightMotorPWM = 300;
    }
    else if (rightMotorPWM < -300) {
      rightMotorPWM = -300;
    }

    if (DEBUG) {
      Serial.print("Error Angle: "); Serial.println(E_angle);
      Serial.print("Velocity Desired: "); Serial.println(vLDesired);
      Serial.print("Error Velocity: "); Serial.println(E_vL);
    }


    // CONTROLLER ENDS HERE ---------------------------------------------------------------------------------


    //updatePWMs(error_left_accum,error_right_accum);

    // if the robot is more than 45 degrees, shut down the motor
    if (start_flag && fabs(angle_rad) > FORTY_FIVE_DEGREES_IN_RADIANS)
    {
      // reset the accumulated errors here
      start_flag = 0;   /// wait for restart
      prev_time = 0;
      angle_rad_accum = 0;
      // trying to make it so that Rocky has no brakes
      //motors.setSpeeds(0, 0);
      motors.setSpeeds((int)leftMotorPWM, (int)rightMotorPWM);
    } else if (start_flag) {
      motors.setSpeeds((int)leftMotorPWM, (int)rightMotorPWM);
    }
  }

  // kill switch
  if (buttonA.getSingleDebouncedPress())
  {
    motors.setSpeeds(0, 0);
    while (!buttonA.getSingleDebouncedPress());
  }
}
