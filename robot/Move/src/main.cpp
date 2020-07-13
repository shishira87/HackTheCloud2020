/*
█▀▄▀█   █▀▀█   ▀█░█▀   █▀▀
█░▀░█   █░░█   ░█▄█░   █▀▀
▀░░░▀   ▀▀▀▀   ░░▀░░   ▀▀▀
Project by: Jay Katyan, Christian So, Shishir Sudhaman, Rohan Minocha
	-This project was submitted to Hack the Cloud on July 12th, 2020

Program Description:
	-The following file contains all necessary robot control code, as well
	device communication and camera detection code. The program uses Purdue's VEX
	Robotics API for general motion, along with fully custom pursuit and control
	algorithms, built using PROS Lib and OKAPI Lib in C++.
*/

//DEVICE CONFIGURATION

	//MOTORS (Make Negative to reverse a port)
	#define _LEFT_FRONT_PORT_ 2
	#define _LEFT_BACK_PORT_ 1
	#define _RIGHT_FRONT_PORT -9
	#define _RIGHT_BACK_PORT_ -10

	#define _STAGE_1_LEFT_PORT_ 4
	#define _STAGE_2_LEFT_PORT_ 3
	#define _STAGE_1_RIGHT_PORT_ -7
	#define _STAGE_2_RIGHT_PORT_ -8
	#define _CLAW_PORT_ 5
	#define _TABLE_PORT_ 6

	//VISION
	#define NUMBER_OBJECTS 2
	#define _VISION_PORT_ 11
	#define SIG 1
	#define COORDINATE_COUNT 5

	//ESP
	#define _ESP_IN_PORT_ 3
	#define _ESP_OUT_PORT_ 4

	//INERTIAL MEASUREMENT UNIT
	#define _IMU_PORT_ 21
	#define IMU_INIT 2000

	//PROPORTIONAL INTEGRAL DERIVATIVE CONSTANTS
	#define TABLE_KP 0.05

	#define DRIVE_KP .0019
	#define DRIVE_KD .00015

	#define TURN_KP .00365
  #define TURN_KD .0

  #define ANGLE_KP .0001

	#define LIFT_A_KP 0.1
	#define LIFT_A_KI 0.00001
	#define LIFT_A_KD 0

	#define LIFT_B_KP 0.1
	#define LIFT_B_KI 0.00001
	#define LIFT_B_KD 0

	//ROBOT SPECIFICS

		//Drive specifics
		#define WHEEL_DIAM 4_in
	  #define WHEEL_TRACK 15_in

		#define MAX_VEL 0.5
		#define MAX_ACC 2
		#define MAX_JERK 2

		//Positions
			//Start Position
			#define STAGE_1_START 0
			#define STAGE_2_START 0

			//About to Pick Up
			#define STAGE_1_A 800
			#define STAGE_2_A 1350

			//Pick Up
			#define STAGE_1_B 60
			#define STAGE_2_B 1350

			//About to Drop Off
			#define STAGE_1_C 2970
			#define STAGE_2_C -990

			//Picking up in the robot
			#define STAGE_1_D 2200
			#define STAGE_2_D -1000

			//Table Positions
			#define TABLE_VAL_1 225
			#define TABLE_VAL_2 450
			#define TABLE_VAL_3 675
			#define TABLE_VAL_4 900

			//Claw Positions
			#define CLAW_CLOSED 0
			#define CLAW_OPEN -1200

		//MISC
		#define MAX_PATH_SIZE 5
		#define MAX_PATHS 1

//PROGRAM INITIALIZATION
#include "main.h"

pros::Motor lFront (_LEFT_FRONT_PORT_, signbit(_LEFT_FRONT_PORT_));
pros::Motor lBack (_LEFT_BACK_PORT_, signbit(_LEFT_BACK_PORT_));
pros::Motor rFront (_RIGHT_FRONT_PORT, signbit(_RIGHT_FRONT_PORT));
pros::Motor rBack (_RIGHT_BACK_PORT_, signbit(_RIGHT_BACK_PORT_));
pros::Motor lStageOne (_STAGE_1_LEFT_PORT_, signbit(_STAGE_1_LEFT_PORT_));
pros::Motor lStageTwo (_STAGE_2_LEFT_PORT_, signbit(_STAGE_2_LEFT_PORT_));
pros::Motor rStageOne (_STAGE_1_RIGHT_PORT_, signbit(_STAGE_1_RIGHT_PORT_));
pros::Motor rStageTwo (_STAGE_2_RIGHT_PORT_, signbit(_STAGE_2_RIGHT_PORT_));
pros::Motor claw (_CLAW_PORT_, signbit(_CLAW_PORT_));
pros::Motor table (_TABLE_PORT_, signbit(_TABLE_PORT_));

pros::ADIDigitalIn espIn (_ESP_IN_PORT_);
pros::ADIDigitalOut espOut (_ESP_OUT_PORT_);
pros::ADIAnalogIn espUART (6);

pros::Vision vision_sensor (_VISION_PORT_);
pros::Imu imu (_IMU_PORT_);

void ReadArrayUART(int i, double* arr){
	int k;
	for(k=0; k<i; k++){
		//arr[k] = pros::c::espUART.read_byte();
	}
}

void initialize() {
	//Program Initialization Code
	pros::lcd::initialize();
	imu.reset();
	pros::delay(IMU_INIT);
}

void disabled() {
	//Disabled Mode Code
}
void competition_initialize() {
	//Initialize once connected to competition software
}
void autonomous() {
	//Autonomous Control Code
}

//PID CONTROL CODE
typedef struct{
	float kP;
	float kI;
	float kD;
	float EpsilonInner;
	float EpsilonOuter;
	float dInner;
	float dOuter;
	float sigma;
	float lastValue;
	float lastTarget;
	unsigned long lastTime;
	float LastSetPoint;
} PID;

PID pidInit(float fKP, float fKI, float fKD, float fEpsilonInner, float fEpsilonOuter,float dInner, float dOuter) {
	//DECLARE OBJECT
	PID pid;
	//INITIALIZE VARIABLES
	pid.kP = fKP;
	pid.kI = fKI;
	pid.kD = fKD;
	//BOUNDARY CONSTRAINTS
	pid.EpsilonInner = fEpsilonInner;
	pid.EpsilonOuter = fEpsilonOuter;
	//INTEGRAL
	pid.sigma = 0;
	//DERIVATIVE
	pid.lastValue = 0;
	pid.lastTime = pros::millis();
	pid.dInner = dInner;
	pid.dOuter = dOuter;

	//RETURN CONTRUCTED OBJECT
	return pid;
}

float pidCalculate(PID pid, float fSetPoint, float fProcessVariable){
	//DEFINE VARIABLES
  float fError;

	fError = fSetPoint - fProcessVariable;
	//UPDATE OLD ERROR VALUE
	pid.lastValue = fProcessVariable;

	//UPDATE PREVIOUS SET POINT
	if(fSetPoint != pid.lastTarget) {
		pid.sigma = 0;
		pid.lastTarget = fSetPoint;
	}

	//UPDATE TIME
  float fDeltaTime = (float)(pros::millis() - pid.lastTime) / 1000;
	pid.lastTime = pros::millis();

 //DERIVATIVE CALCULATION
	float fDeltaPV = 0;
	if(fDeltaTime > 0)
	//FIND DERIVATIVE
		fDeltaPV = (fProcessVariable - pid.lastValue) / fDeltaTime;

  //INTEGRAL BOUNDS
	if(fabs(fError) > pid.EpsilonInner && fabs(fError) < pid.EpsilonOuter)
		pid.sigma += fError * fDeltaTime;

	if (fabs (fError) > pid.EpsilonOuter)
		pid.sigma = 0;

	//APPLY OUTPUT POWER
	float fOutput = fError * pid.kP + pid.sigma * pid.kI + pid.kD * fDeltaPV;

	//CHECK IF IT'S GREATER OR LESS THAN THE LEGAL MAX
	fOutput = fabs(fOutput) > 127 ? 127 * fOutput/fabs(fOutput) : fOutput;

	return fOutput;
}

PID drivePID = pidInit (DRIVE_KP, 0, DRIVE_KD, 0, 100.0, 5, 15);
PID turnPID = pidInit (TURN_KP, 0, TURN_KD, 0, 100.0, 5, 15);
PID accelerationPID = pidInit (10, 0, 0, 0, 0, 0, 0);
PID curvaturePID = pidInit (10, 0, 0, 0, 0, 0, 0);
PID liftAPID = pidInit (LIFT_A_KP, LIFT_A_KI, LIFT_A_KD, 0, 0, 0, 0);
PID liftBPID = pidInit (LIFT_B_KP, LIFT_B_KI, LIFT_B_KD, 0, 0, 0, 0);
PID tablePID = pidInit (TABLE_KP, 0, 0, 0, 0, 0, 0);

//SLEW CONTROL CODE
float lastSlewTime;
float slewRateCalculate (float desiredRate){
	     //DEFINE SLEW VARIABLES
	      float maxAccel = 0.14, lastSlewRate = 0;
	      float deltaTime = pros::millis() - lastSlewTime;
	      float desiredAccel = (desiredRate - lastSlewRate)/deltaTime, addedRate, newRate, returnVal;

	      if(fabs(desiredAccel) < maxAccel || (desiredAccel < 0 && desiredRate > 0) || (desiredAccel > 0 && desiredRate < 0)){
		           addedRate = desiredAccel*deltaTime;
		           newRate = addedRate+lastSlewRate;
		      }else{
		          addedRate = ((desiredAccel>0)? 1: -1)*maxAccel*deltaTime;
              newRate = addedRate+lastSlewRate;
		       }

		    lastSlewTime = lastSlewTime+deltaTime;
	      lastSlewRate = newRate;

		    returnVal = newRate;
	    	return returnVal;
    }

//ODOMETRY CODE
float x = 0, y = 0, angle = 0;
float lastLeftPos = 0, lastRightPos = 0;
float deltaTheta = 0, thetaNew = 0;
float curLeft = 0, curRight = 0;
float leftAtReset = 0, rightAtReset = 0;
float deltaLeft = 0, deltaRight = 0;
float deltaLr = 0, deltaRr = 0;
float deltaX = 0, deltaY = 0;
float theta = 0, radius = 0;

float getX() {
  return x;
}

float getY() {
  return y;
}

float getAngleDegrees() {
  return angle*180/M_PI;
}
float getAngle() {
  return angle;
}

float modulo(float a, float b) {
  while (a>b) {
    a-=b;
  }
  return a;
}

void updatePosition() {
  curLeft = lFront.get_position();
  curRight = rFront.get_position();

  deltaLeft = (curLeft - lastLeftPos)*(M_PI/180)*2;
  deltaRight = (curRight - lastRightPos)*(M_PI/180)*2;

  lastLeftPos = curLeft;
  lastRightPos = curRight;

  deltaLr = (curLeft - leftAtReset)*(M_PI/180)*2;
  deltaRr = (curRight - rightAtReset)*(M_PI/180)*2;
  deltaTheta = thetaNew - angle;

  if (deltaX == 0) {
     if (deltaY > 0) {
       theta = M_PI/2;
     }
     else if (deltaY < 0) {
       theta = 3*M_PI/2;
     }
     else {
       theta = 0;
     }
   }
   else {
     theta = atan(deltaY/deltaX);
   }

  theta = atan2f(deltaY, deltaX);
  radius = sqrt(deltaX*deltaX + deltaY*deltaY);
  deltaX = radius*cos(theta);
  deltaY = radius*sin(theta);

  thetaNew+=M_PI;
  while (thetaNew <= 0) {
    thetaNew+=2*M_PI;
  }
  thetaNew = modulo(thetaNew, 2*M_PI);
  thetaNew-=M_PI;

  angle = thetaNew;
  x = x - deltaX; //step 11
  y = y + deltaY;
}

//GENERAL MOTION
void setDrive (int left, int right){
	lFront.move(left);
	lBack.move(left);
	rFront.move(right);
	rBack.move(right);
}

float globalEndX;
void moveToPoint (float targetX, float targetY, int minVol){

    const float maxAccel = .14;
	  bool atPoint = false;
    float targetAngle = 0, turnPower = 0, currentPosition, errorMargin = 1;
    float linearPower, constantAccelPower;
    float legY, legX;
    int sign1, sign2;
    float prevTime, deltaTime, currentTime;
    float position, prevPosition, deltaPosition, currentVelocity, preVelocity, acceleration;

	   while (!atPoint) {
     //lastSlewTime = pros::millis();
		 updatePosition();
     legY = pow(targetY-getY(),2);
     legX = pow(targetX-getX(),2);
     position = sqrt(legY + legX);
     deltaPosition = position - prevPosition;
     prevPosition = position;

     //CALCULATE MAX VELOCITY AT GIVEN TIME
     currentTime = pros::millis()/1000.0;
     deltaTime = currentTime - prevTime;
     prevTime = currentTime;

     currentVelocity = deltaPosition/deltaTime;
     preVelocity = currentVelocity;
     acceleration = (currentVelocity-preVelocity)/deltaTime;


	 	//CONVERT ANGLE BASED ON "ERROR" (in radians)
	  targetAngle = atan2f((targetY-getY()),(targetX-getX()));

    //PYTHAGOREAN THEOREM TO FIND LINEAR DISTANCE (LAST PARAMETER IS SET POINT FOR TRACKING CENTER)
    //CALCULATE HOW FAST YOU NEED TO GO
    linearPower = -pidCalculate(drivePID, 0 ,position);
    sign1 = linearPower/fabs(linearPower);
    if(fabs(linearPower) > 127)
    linearPower = 127*sign1;

    //CALCULATE WHETHER OR NOT YOU'RE AT THE TARGET ACCELERATION
    //(CONSTRAINS POWER OUTPUT)(THINK OF THIS LIKE CURRENT IN ELECTRICITY)
    constantAccelPower = pidCalculate(accelerationPID, maxAccel, acceleration);
    sign2 = constantAccelPower/fabs(constantAccelPower);
    if(fabs(constantAccelPower) > 127)
    constantAccelPower = 127*sign2;

    //CALCULATE HOW FAR OFF ANGLE YOU ARE, THIS OVER POWERS
    //THE OTHER POWER FILTERS BASED ON THE ORDER (NO SIGN CHECK NEEDED)
    turnPower = pidCalculate(turnPID, targetAngle, getAngle());
    /*
    //FILTER OUTPUT POWER TO CONSTRAIN ACCELERATION
    //power = slewRateCalculate(power);
		//FIND CORRECTION POWER BASED ON WHICH ANGLE TO TURN (FIRST MULTIPLIER)
		//SET AMOUNT OF POWER BASED ON PID CALCULATION USING TURNPID PARAMETERS
		//turnPower = ((fabs(targetAngle-getAngle())>0)? -1: 1)*pidCalculate(turnPID, targetAngle, getAngle());
    */
    //MINIMUM POWER FOR CURVED PATHS (FOR BETTER SMOOTHNESS)
    //(ONLY ALLOW THE ROBOT TO SLOW DOWN AT END POINT)
    if(fabs(getX()) < fabs(globalEndX) + 3 && fabs(linearPower) < abs(minVol))
      linearPower = minVol*sign1;

		setDrive(((linearPower+constantAccelPower) + turnPower),((linearPower+constantAccelPower) - turnPower));

		//PYTHAGOREAN THEOREM
		currentPosition = sqrt(pow(targetY-getY(),2) + pow(targetX-getX(),2));

		//IF AT TARGET, EXIT LOOP
		if (currentPosition < errorMargin)
			atPoint = true;

		pros::delay(10);
	}
}

//Okapi Controllers
using namespace okapi;
MotorGroup leftMotors = {{_LEFT_FRONT_PORT_, _LEFT_BACK_PORT_}};
MotorGroup rightMotors = {{_RIGHT_FRONT_PORT, _RIGHT_BACK_PORT_}};

std::shared_ptr<OdomChassisController> chassis = ChassisControllerBuilder()
    .withMotors(leftMotors, rightMotors)
    .withGains(
        {DRIVE_KP, 0, DRIVE_KD}, // Distance controller gains
        {TURN_KP, 0, TURN_KD}, // Turn controller gains
        {ANGLE_KP, 0, 0}  // Angle controller gains (helps drive straight)
    )
    .withDimensions(AbstractMotor::gearset::green, {{WHEEL_DIAM, WHEEL_TRACK}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis

auto profileController = AsyncMotionProfileControllerBuilder()
    .withLimits({
      MAX_VEL,  // Maximum linear velocity of the Chassis in m/s
      MAX_ACC,  // Maximum linear acceleration of the Chassis in m/s/s
      MAX_JERK // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(chassis)
    .buildMotionProfileController();

void waitUntilSettled(){
  chassis->waitUntilSettled();
}

void resetPos(){
  chassis->setState({0_ft,0_ft,0_deg});
}

void setPos(QLength x, QLength y, QAngle deg){
  chassis->setState({x,y,deg});
}

void drive(QLength dist, int rpm){
  chassis->setMaxVelocity(rpm);
  chassis->moveDistance(dist);
}

void driveAsync(QLength dist, int rpm){
  chassis->setMaxVelocity(rpm);
  chassis->moveDistanceAsync(dist);
}

void driveToPoint(QLength x, QLength y, int rpm){
  chassis->setMaxVelocity(rpm);
  chassis->driveToPoint({y, x});
}

void driveToPoint(QLength x, QLength y, QAngle angle, int rpm){
  chassis->setMaxVelocity(rpm);
  chassis->driveToPoint({y, x});
  chassis->turnToAngle(angle);
}

void turn(QAngle deg, int rpm){
  chassis->setMaxVelocity(rpm);
  chassis->turnAngle(deg);
}

void turnToAngle(QAngle deg, int rpm){
  chassis->setMaxVelocity(rpm);
  chassis->turnToAngle(deg);
}

void arc(QLength x, QLength y, QAngle deg, bool reversed, int rpm){
  /*
  chassis->setMaxVelocity(rpm);
  OdomState start = chassis->getState();
  */
  profileController->generatePath({/*{start.x, start.y, start.theta}*/{0_ft,0_ft,0_deg}, {y, x, deg}}, "A");
  (reversed)?profileController->setTarget("A", true, true):
             profileController->setTarget("A");
             profileController->waitUntilSettled();
             profileController->removePath("A"); //remove path once motion is complete.
}

//PATH CALCULATIONS
// To represent a data point corresponding to x and y = f(x)
struct Arc{
   float x, y;
};

struct Point{
   float x1, y1;
};

int findPathSize(Arc path[]){
  int size = 0;
  bool end = false;
  while(!end){
      size++;
  }
  return size;
}

float curvature(Arc path[], Arc prevPoint, Arc point, Arc nextPoint){
   float k1,k2,b1,b2,b,a,r;
   //POINTS
   float x1,x2,x3,y1,y2,y3;


   x1 = prevPoint.x, y1 = prevPoint.y;
   x2 = point.x, y2 = point.y;
   x3 = nextPoint.x, y2 = nextPoint.y;

   k1 = .5*((x1*x1)+(y1*y1)-(x2*x2)-(y2*y2))/(x1-x2);
   k2 = (y1-y2)/(x1-x2);

   b1 = .5*((x2*x2)-(2*x2*k1)+(y2*y2)-(x3*x3)+(2*x3*k1)-(y3*y3));
   b2 = (x3*k2)-y3+y2-(x2*k2);
   b = b1/b2;

   a = k1-k2*b;
   r = sqrt(pow(x1-a,2) + pow(y1-b,2));

   return 1/r;
 }

 // function to interpolate the given data points using Lagrange's formula
 // xi corresponds to the new data point whose value is to be obtained
 // n represents the number of known data points
 double interpolate(Arc p[], int xi, int n){
    double result = 0; // Initialize result

    for (int i=0; i<n; i++)
    {
        // Compute individual terms of above formula
        double term = p[i].y;
        for (int j=0;j<n;j++)
        {
            if (j!=i)
                term = term*(xi - p[j].x)/double(p[i].x - p[j].x);
        }

        // Add current term to result
        result += term;
    }

    return result;
 }

 void pursuePoint(Arc path[], float targetX, float targetY, Arc prevPoint1, Arc point1, Arc nextPoint1, int pathSize, int minVol){
       const float maxAccel = .14;

   	  bool atPoint = false;
       float targetAngle = 0, turnPower = 0, errorMargin = 1;
       float legY, legX;
       int sign1, sign2;

       float linearPower, constantAccelPower;
       float prevTime, deltaTime, currentTime;
       float positionAbsolute, positionRelative, prevPosition, deltaPosition, endY;
       float currentVelocity, preVelocity, acceleration;
       float curvatureExpected, currentCurvature, curvaturePower;

       Arc odom, odomPrev;
       int counter = 0;

   	while (!atPoint) {

        //FIND POSITION INFORMATION
        odomPrev = {getX(), getY()};
   		 updatePosition();
        odom = {getX(), getY()};

        legY = pow(targetY-getY(),2);
        legX = pow(targetX-getX(),2);
        positionRelative = sqrt(legY + legX);
        deltaPosition = positionRelative - prevPosition;
        prevPosition = positionRelative;

        endY = interpolate(path, globalEndX, pathSize);
        //DISTANCE FROM CURRENT POINT TO FINAL POINT WITH FINAL VECTOR
        positionAbsolute = sqrt(pow(globalEndX - getX(),2) + pow(endY - getY(),2));

        //CALCULATE MAX VELOCITY AT GIVEN TIME
        currentTime = pros::millis()/1000.0;
        deltaTime = currentTime - prevTime;
        prevTime = currentTime;

        currentVelocity = deltaPosition/deltaTime;
        preVelocity = currentVelocity;
        acceleration = (currentVelocity-preVelocity)/deltaTime;


   	 	//CONVERT ANGLE BASED ON "ERROR" (in radians)
   	  targetAngle = atan2f((targetY-getY()),(targetX-getX()));


       //PYTHAGOREAN THEOREM TO FIND VELOCITY FROM CURRENT POINT TO GLOBAL END POINT
       //(LAST PARAMETER IS SET POINT FOR TRACKING CENTER)
       //CALCULATE HOW FAST YOU NEED TO GO FOR THE DISPLACEMENT VECTOR
       //linearPower = -pidCalculate(drivePID, 0 , positionRelative);
       //CONSTANTLY SETS POWER TO THE VALUE IT WOULD BE AT IF THE ROBOT WERE AT THE COORDINATE
       //IN A TANGENT PATH TO ENDGLOBALX VARIABLE SO THIS POWER IS ESSENTIALLY AS if
       //THE ROBOT WER DRIVING IN A STRAIGHT LINE ACROSS THE WHOLE DISTANCE OF THE PATH TO
       //FINAL POINT ON THE PATH
       linearPower = -pidCalculate(drivePID, 0, positionAbsolute);//REVERSES THE SIGN TO THE RIGHT DIRECTION (ERROR IS ALREADY CALCULATED ABOVE)
       sign1 = linearPower/fabs(linearPower);
       if(fabs(linearPower) > 127)
       linearPower = 127*sign1;

       //CALCULATE WHETHER OR NOT YOU'RE AT THE TARGET ACCELERATION
       //(CONSTRAINS POWER OUTPUT)(THINK OF THIS LIKE CURRENT IN ELECTRICITY)
       //ACTS AS BOTH A SUBTRACTIVE AND ADDATIVE VALUE
       constantAccelPower = pidCalculate(accelerationPID, maxAccel, acceleration);
       sign2 = constantAccelPower/fabs(constantAccelPower);
       if(fabs(constantAccelPower) > 127)
       constantAccelPower = 127*sign2;

       //CALCULATE HOW FAR OFF ANGLE YOU ARE, THIS OVER POWERS
       //THE OTHER POWER FILTERS BASED ON THE ORDER (NO SIGN CHECK NEEDED)
       turnPower = pidCalculate(turnPID, targetAngle, getAngle());
       //THIS IS CURRENTLY IN RADIANS, CONVERT TO DEGREES

       //IF THE ROBOT NEEDS TO MOVE BACKWARDS BECAUSE POINT WAS OVERSHOT
       //CHANGE POWER TO NEGATIVE AND
       if(fabs(targetAngle - getAngle()) > 90)
       linearPower *= -1;

       //CURVATURE ADJUSTMENT BASED ON LOOKAHEAD POINT AND CURRENT PATH
       curvatureExpected = curvature(path, prevPoint1, point1, nextPoint1);
       currentCurvature = curvature(path, odomPrev, odom, nextPoint1);
       curvaturePower = pidCalculate(curvaturePID, curvatureExpected, currentCurvature);

       //MINIMUM POWER FOR CURVED PATHS (FOR BETTER SMOOTHNESS)
       //(ONLY ALLOW THE ROBOT TO SLOW DOWN AT END POINT)
       if(fabs(getX()) < fabs(globalEndX) + 3 && fabs(linearPower) < abs(minVol))
         linearPower = minVol*sign1;

   		setDrive(((linearPower+constantAccelPower) + (turnPower+curvaturePower)),((linearPower+constantAccelPower) - (turnPower+curvaturePower)));

   		//IF AT TARGET, EXIT LOOP
   		if (positionRelative < errorMargin)
   			atPoint = true;

   		pros::delay(10);
   	}
     if(getX() == globalEndX)
       setDrive(0,0);
 }

 float f(float x){
         //CUSTOM FUNCTION
         return pow(x,3);
 }

 //chassis.curvedPath(curveToCubes,2, 50, 100);
 Arc prevPointExpect, currentPointExpect, nextPoint;

 void curvedPath(Arc path[],float resolution, float endX, int pathSize, int min){
       float y,prevY, nextY, increment = 1/resolution;
       int currentArrayElement = 0;
       globalEndX = endX;
     if(endX > getX()){
       for(float x = 0; x < endX;x += increment){
         //y = f(i - getX()) + getY(); //translate the function to current x,y values
         y = interpolate(path, x, pathSize);
         nextY = interpolate(path,x+increment, pathSize);

         //END AND BEGINNING CASE TESTING
         currentPointExpect = {x,y};
         if(x != 0){
           prevPointExpect = {(x-increment),(y-prevY)};
           nextPoint = {x+increment, y};
         }else{
           prevPointExpect = {x,y};
           nextPoint = {x+increment, y};
         }

         //BOUNDARY CHECKS
         if(currentArrayElement != 0)
         pursuePoint(path,x-getX(),y+getY(), prevPointExpect, currentPointExpect, nextPoint, pathSize, min);
         else
         pursuePoint(path,x-getX(),y+getY(), prevPointExpect, currentPointExpect,nextPoint, pathSize, min);

         currentArrayElement++;
         pros::delay(10);
       }
       //FOR MOVING BACKWARDS ON THE X AXIS
     }else{
       for(float i = 0; fabs(i) < endX;i -= increment){
         //y = f(i - getX()) + getY(); //translate the function to current x,y values
         y = interpolate(path, i, pathSize);

         //BOUNDARY CHECKS
         if(currentArrayElement != 0)
         pursuePoint(path,i-getX(),y+getY(),path[currentArrayElement-1],path[currentArrayElement],path[currentArrayElement+1], min, 0);
         pros::delay(10);
       }
     }
   }

 //MOVES RELATIVE WITH INCHES FOR INPUT
 void moveDistance(float target){
   float power;
   int prevValue = (lFront.get_position()+rFront.get_position())/2, currentValue;
   bool atTarget = false;
   while(!atTarget){
     //CONVERT TO RELATIVE DISTANCE
     currentValue = ((lFront.get_position()+rFront.get_position())/2) - prevValue;
     power = pidCalculate(drivePID, target, currentValue);
     setDrive(power, power);

     //1 INCH FROM TARGET?
     if(abs(currentValue) < 1){
       atTarget = true;
     }
     pros::delay(20);
   }
   setDrive(0,0);
 }

 //MOVES TO ABSOLUTE ANGLE (INPUT IN DEGREES)
 void moveAngle(float targetAngle){
   targetAngle *= M_PI/180;
   float power;
   while(!targetAngle){
     power = pidCalculate(drivePID, targetAngle, getAngle());
     setDrive(power, -power);
     pros::delay(20);
   }
   setDrive(0,0);
}

void setBrakes(pros::motor_brake_mode_e_t mode){
  lFront.set_brake_mode(mode);
  lBack.set_brake_mode(mode);
  rFront.set_brake_mode(mode);
  rBack.set_brake_mode(mode);
	lStageOne.set_brake_mode(mode);
	lStageTwo.set_brake_mode(mode);
	rStageOne.set_brake_mode(mode);
	rStageTwo.set_brake_mode(mode);
	claw.set_brake_mode(mode);
	table.set_brake_mode(mode);
}

//TABLE CONTROL CODE
int currTable = 0;
void moveTable(int state, int velocity){
	switch(state){
		case 1:
		table.move_absolute(TABLE_VAL_1, velocity);
		pros::delay(20);
		break;
		case 2:
		table.move_absolute(TABLE_VAL_2, velocity);
		pros::delay(20);
		break;
		case 3:
		table.move_absolute(TABLE_VAL_3, velocity);
		pros::delay(20);
		break;
		case 4:
		table.move_absolute(TABLE_VAL_4, velocity);
		pros::delay(20);
		break;
	}
}

//LIFT AND CLAW CONTROL CODE
void moveStageOne(PID pid, int target, int time, float speed, bool slew){
  int atTarget = 0;
  int encoder = 0;
  int startTime = pros::millis();
	while ((atTarget != 1) && (pros::millis()-startTime) < time) {
	  encoder = lStageOne.get_position();
	  float val = pidCalculate(pid, target, encoder)*speed;
	  val = (slew)? slewRateCalculate(val): val;
	  lStageOne.move(val);
		rStageOne.move(val);
	  if(encoder == target){
	    atTarget = 1;
	  }
  pros::delay(20);
  }
	lStageOne.move(0);
	rStageOne.move(0);
}

void moveStageTwo(PID pid, int target, int time, float speed, bool slew){
  int atTarget = 0;
  int encoder = 0;
  int startTime = pros::millis();
	while ((atTarget != 1) && (pros::millis()-startTime) < time) {
	  encoder = lStageTwo.get_position();
	  float val = pidCalculate(pid, target, encoder)*speed;
	  val = (slew)? slewRateCalculate(val): val;
	  lStageTwo.move(val);
  	rStageTwo.move(val);
	  if(encoder == target){
	    atTarget = 1;
	  }
  pros::delay(20);
  }
	lStageTwo.move(0);
	rStageTwo.move(0);
}

void moveClaw(int position, int velocity){
	claw.move_absolute(position, velocity);
	pros::delay(20);
}

void setLiftState(int state){
	switch(state){
		case 1: //Init
		moveClaw(CLAW_CLOSED, 50);
		moveStageOne(liftAPID, STAGE_1_START, 2000, 0.5, true);
		moveStageTwo(liftBPID, STAGE_2_START, 2000, 0.5, true);
		break;
		case 2: //About to Pick Up
		moveClaw(CLAW_OPEN, 50);
		moveStageOne(liftAPID, STAGE_1_A, 2000, 0.5, true);
		moveStageTwo(liftBPID, STAGE_2_A, 2000, 0.5, true);
		break;
		case 3: //Pick Up
		moveStageOne(liftAPID, STAGE_1_B, 2000, 0.5, true);
		moveStageTwo(liftBPID, STAGE_2_B, 2000, 0.5, true);
		moveClaw(CLAW_CLOSED, 50);
		break;
		case 4: //Drop Off
		moveStageOne(liftAPID, STAGE_1_C, 2000, 0.5, true);
		moveStageTwo(liftBPID, STAGE_2_C, 2000, 0.5, true);
		moveClaw(CLAW_OPEN, 50);
		currTable++;
		if (currTable > 4) {currTable = 1;}
		moveTable(currTable, 50);
		break;
		case 5: //Picking Up in the Robot
		moveStageOne(liftAPID, STAGE_1_D, 2000, 0.5, true);
		moveStageTwo(liftBPID, STAGE_2_D, 2000, 0.5, true);
		moveClaw(CLAW_CLOSED, 50);
		moveStageOne(liftAPID, STAGE_1_B, 2000, 0.5, true);
		moveStageTwo(liftBPID, STAGE_2_B, 2000, 0.5, true);
		moveClaw(CLAW_OPEN, 50);
		currTable++;
		if (currTable > 4) {currTable = 1;}
		moveTable(currTable, 50);
		break;
	}
}

//MAIN CONTROL CODE
void opcontrol() {
setBrakes(MOTOR_BRAKE_HOLD);

bool espValue;
//EXAMPLE CODE:
//Arc backOfGarage[] = {{0,2}, {1,3}, {2,12}, {5,147}};
//curvedPath(backOfGarage,2, 50, 100, 0);

while(true){
	espValue = !espIn.get_value(); // flips the variable due to glitch in internal circuits

	double x_path_coord[COORDINATE_COUNT];
	double y_path_coord[COORDINATE_COUNT];

	if(espValue == HIGH){
		pros::lcd::print(1, "READING COORDINATES...");
		ReadArrayUART(COORDINATE_COUNT, x_path_coord);
		pros::delay(2);
		ReadArrayUART(COORDINATE_COUNT,y_path_coord);
	}
	else{
		pros::lcd::print(1, "INPUT: LOW");
	}

	pros::vision_object_s_t obj[NUMBER_OBJECTS];
	pros::c::vision_read_by_sig(_VISION_PORT_, 0, SIG, NUMBER_OBJECTS, obj);

	pros::lcd::print(3,"Object 1 Position: (%d,%d)", obj[0].x_middle_coord,obj[0].y_middle_coord);
	pros::lcd::print(4,"Object 2 Position: (%d,%d)", obj[1].x_middle_coord,obj[1].y_middle_coord);

	espOut.set_value(!espValue);
	}

	bool atEnd;
	Arc pathA;
	Arc pathB;

	while (!atEnd){
		for (int i = 0; i < 5; i++){
				Arc pathA[i] = x_path_coord[i];
				Arc pathB[i] = y_path_coord[i];
		}
		curvedPath(pathA, 2, 50, 100, 0);
		while(obj.number_objects() == 1){
			arc(obj[0].x_middle_coord, obj[0].y_middle_coord, 0, false, 50);
			for (int i = 0; i < 5; i++){
				setLiftState(i);
				pros::delay(500);
			}
		}
		curvedPath(pathB, 2, 50, 100, 0);
		atEnd = true;
	}

pros::delay(20);
}
