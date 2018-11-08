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
#define CMD_TIME 1832

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

static float vRarray[179] = {0.0853758004844342,
0.087951889278429,
0.0904874162263094,
0.092953234102695,
0.0953250313840014,
0.0975824816659994,
0.0997085793317635,
0.101689123890844,
0.103512321369396,
0.10516847731376,
0.106649761453565,
0.107950028606021,
0.109064684022239,
0.10999058420909,
0.110725966454449,
0.111270401973724,
0.111624768891081,
0.111791242253302,
0.111773299007084,
0.111575736388585,
0.111204702492877,
0.110667737904608,
0.109973827150542,
0.109133458323627,
0.108158688439306,
0.107063210793949,
0.105862418638868,
0.104573456661466,
0.103215247859966,
0.101808478222749,
0.100375515123833,
0.0989402277942792,
0.0975276705457983,
0.0961635836619796,
0.0948736668370446,
0.0936825918307681,
0.0926127529679416,
0.09168281516548,
0.0909062140689657,
0.090289883295167,
0.0898335967711485,
0.0895303554391456,
0.0893681340755176,
0.089332979540365,
0.089412958212874,
0.08960197287685,
0.0899022825272716,
0.0903248552706298,
0.0908873979866334,
0.091610702270063,
0.0925144329601112,
0.0936134756939027,
0.0949155578617379,
0.0964203318083118,
0.0981196976245648,
0.0999989447339531,
0.102038280969104,
0.104214414002668,
0.106501976465048,
0.108874696458164,
0.111306292144893,
0.11377111336861,
0.116244572994967,
0.118703414968557,
0.12112586218142,
0.123491679965745,
0.125782183118059,
0.127980207199203,
0.130070058947854,
0.13203745605603,
0.133869463136471,
0.135554428247571,
0.137081922614926,
0.138442685011295,
0.13962857147763,
0.14063251057127,
0.141448464027689,
0.142071392557838,
0.142497226430445,
0.14272284047788,
0.142746033194954,
0.142565509659403,
0.142180868082755,
0.141592589896385,
0.140802033388358,
0.139811431032128,
0.138623890790287,
0.137243401837661,
0.135674845331479,
0.133924011064743,
0.131997621074074,
0.129903361534097,
0.127649924550301,
0.125247061745911,
0.122705651794988,
0.120037784230292,
0.117256861862091,
0.114377723845785,
0.111416790628801,
0.108392230405691,
0.105324143941138,
0.102234760235918,
0.0991486290496514,
0.0960927874179997,
0.0930968660430204,
0.0901930885723915,
0.0874161043120455,
0.0848025864707072,
0.082390528953007,
0.0802181913134288,
0.0783226790882651,
0.0767382064940338,
0.0754941638608104,
0.0746131870071287,
0.074109476563496,
0.0739876187226023,
0.0742421027575622,
0.0748576222438321,
0.0758101135979171,
0.0770683649090766,
0.0785959525498389,
0.0803532474517558,
0.0822992709735136,
0.0843932510464951,
0.0865958080014933,
0.088869766757803,
0.0911806379819782,
0.0934968342767278,
0.0957896928346436,
0.0980333696763259,
0.100204658516749,
0.102282773797095,
0.10424912505496,
0.106087099747953,
0.107781864135327,
0.109320186603595,
0.110690284440475,
0.111881693039123,
0.112885155435131,
0.113692529615694,
0.11429671095548,
0.114691567262806,
0.114871884155053,
0.114833318756559,
0.114572359985652,
0.114086293948324,
0.113373173172976,
0.112431788599249,
0.111261643372078,
0.109862927588853,
0.108236493200858,
0.106383828275419,
0.104307029773669,
0.102008773875147,
0.0994922826587536,
0.0967612855878386,
0.0938199736764934,
0.0906729433217529,
0.0873251253840961,
0.0837816928670735,
0.0800479369311259,
0.076129094987052,
0.0720301044334597,
0.0677552377790479,
0.0633075426255571,
0.0586879493105778,
0.0538937898138563,
0.0489162252757662,
0.0437355398664215,
0.0383119908754889,
0.0325666644510442,
0.0263375994417504,
0.0192667342838711,
0.0104589833048832,
-0.00280445339539941,
-0.0319486664672118,
-0.162823321847778,
-0.431212681713236,
-0.0696555901990871
};

static float vLarray[179] = {0.0706130203468907,
0.0744055059426458,
0.0779574611574964,
0.0812715991719374,
0.0843491659394479,
0.0871903521099785,
0.0897946297442422,
0.0921610236236219,
0.0942883284144181,
0.0961752821806799,
0.0978207052234562,
0.099223611609736,
0.100383299305325,
0.101299423625803,
0.101972057773972,
0.102401743517332,
0.102589534545548,
0.102537034709939,
0.102246433167482,
0.101720538423667,
0.100962813395949,
0.0999774139191731,
0.0987692336162066,
0.0973439588059825,
0.0957081381771955,
0.0938692733926353,
0.0918359386867979,
0.0896179399476308,
0.0872265267541071,
0.0846746742781299,
0.0819774555095055,
0.079152527126958,
0.0762207529389376,
0.0732069843681424,
0.0701410034964894,
0.0670586043788058,
0.0640027350803222,
0.0610245399188996,
0.058184029588366,
0.0555499840173657,
0.0531986048742212,
0.0512104592287694,
0.0496654829559802,
0.0486362884899749,
0.0481806705991908,
0.0483347847605209,
0.0491086529900066,
0.0504852117103912,
0.0524231508942558,
0.054862717756575,
0.0577329565712311,
0.0609587765909635,
0.0644666812613543,
0.0681886421449534,
0.072064164260931,
0.0760409203766511,
0.0800744334104308,
0.0841272376874287,
0.0881678373948393,
0.0921696633331673,
0.0961101351495334,
0.0999698723283302,
0.103732059244143,
0.107381950205904,
0.110906492753065,
0.114294046355116,
0.117534175748178,
0.120617501432259,
0.123535593295521,
0.126280896438345,
0.12884668086685,
0.131227008799615,
0.133416714940279,
0.135411396292912,
0.137207409016692,
0.138801870501493,
0.140192665353646,
0.141378454355921,
0.142358685741559,
0.143133608323869,
0.143704286168474,
0.144072614597039,
0.144241337377408,
0.144214064989638,
0.143995293862053,
0.143590426444363,
0.143005791922115,
0.142248667271207,
0.141327298193081,
0.140250919247814,
0.139029772197915,
0.137675121172124,
0.136199262736645,
0.13461552830283,
0.132938275493107,
0.131182864131719,
0.129365611449523,
0.127503719960718,
0.125615170417698,
0.123718571506157,
0.121832957857847,
0.119977529028734,
0.118171324944827,
0.116432838648255,
0.114779575559874,
0.113227580058677,
0.11179096421674,
0.110481487864259,
0.10930824988449,
0.108277552337723,
0.107392986041979,
0.106655755217298,
0.106065211769805,
0.105619516435415,
0.105316300782673,
0.105153188958131,
0.105128062139597,
0.105239008961002,
0.105483983629729,
0.105860264010255,
0.106363842202357,
0.106988880872241,
0.10772733589155,
0.108568795811389,
0.109500538991167,
0.110507772270725,
0.111573995709966,
0.112681434525124,
0.113811486928929,
0.11494514955632,
0.11606339608745,
0.117147496837622,
0.118179276283752,
0.119141311616855,
0.120017078901988,
0.120791054950787,
0.121448783215068,
0.121976911426181,
0.122363207725153,
0.122596560914266,
0.12266696937,
0.122565522174967,
0.122284375189051,
0.121816724094254,
0.12115677590472,
0.120299720016486,
0.119241699562225,
0.117979783617244,
0.116511940659711,
0.114837013609722,
0.112954696752012,
0.110865514884583,
0.108570805135068,
0.106072702061484,
0.10337412692863,
0.100478782467879,
0.0973911550562049,
0.0941165272051559,
0.0906610047241008,
0.0870315652407007,
0.0832361384911241,
0.079283734941659,
0.0751846497226481,
0.0709507870549251,
0.0665961832368361,
0.0621378680181001,
0.057597325348892,
0.053003064503715,
0.0483953597687885,
0.0438355010450072,
0.0394251759496709,
0.0353508871325658,
0.0319982911775037,
0.0302965127174832,
0.0330154920792581,
0.0516934596951345,
0.172682996643839,
0.437342275956387,
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
 * Notes:
 * For survivor like events, having aggressive constants works well, but agressive constants tend to make robot prone to falling over
 * For battlebots, having more forgiving constants makes robot very stable, but introduces more drift (can be combated with teleoperated control) - position block probably unnecessary
 * 
 */
float Kp = 6.78;
float Ki = 50;
float Jp = 316;
float Ji = 800;
float Zp = 0;
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

  if (cur_time - prev_time > CMD_TIME)
  {
      cmd_index++;
  }

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
    float setpoint = totalDistanceLeft-desiredDistanceLeft + totalDistanceRight-desiredDistanceRight;
    if (setpoint > .2){
      setpoint = .2;
    }
    if (setpoint < -.2){
      setpoint = -.2;
    }
    
    float E_angle = angle_rad - (angleDesired - (Zp * setpoint / 2)); // Throwing in a squared term just because (but actually because its slow to respond when it drifts)
    
    eangle_rad_accum += E_angle*delta_t;
    vLsetpoint = vLarray[cmd_index];
    vRsetpoint = vRarray[cmd_index];
    Serial.print("vLsetpoint"); Serial.println(vLsetpoint);
    
    float vLDesired = Kp * E_angle + Ki * eangle_rad_accum + .1;//vLsetpoint;//- Yp;
    Serial.print("vLDesired"); Serial.println(vLDesired);
    float vRDesired = Kp * E_angle + Ki * eangle_rad_accum + .1;//vRsetpoint;//+ Yp;
    Serial.print("vRDesired"); Serial.println(vRDesired);
    float E_vL = vLDesired - vL;
    float E_vR = vRDesired - vR;

    //Serial.print("Angle"); Serial.println(angle_rad);
    //Serial.print("Angle Desired"); Serial.println(angleDesired);
    //Serial.print("Setpoint"); Serial.println(setpoint);
    //Serial.print("Setpoint contribution"); Serial.println(Zp * setpoint / 2);
    //Serial.print("Error Angle: "); Serial.println(E_angle);
    //Serial.print("Angle adjustment: ");Serial.println((angleDesired - (Zp * (totalDistanceLeft + totalDistanceRight) / 2)));

    error_left_accum += (vL-vLDesired)*delta_t;
    error_right_accum += (vR-vRDesired)*delta_t;
    
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
