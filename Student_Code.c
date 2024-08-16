/**
 * @file Student_Code.c
 * @author Frazer and Yuan 
 * @brief C code file for VEX Project Part 2
 * @version 0.1
 * @date 09/05/2023
 *
 * @copyright Copyright (c) 2023
 *
 */

/* Libraries. DO NOT REMOVE */
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Student_Code.h"

// ---------------------- Defining physical robot parameters --------------------------
// Update these numbers to match the physical robot (information found in the lab manual)
int drivingWheelDiameter = 103;	    // diameter of the driving wheels [mm]
int robotWidth = 250;					// width of the robot including the wheel thickness [mm]
int wheelWidth = 22;					// width of the driving wheel [mm]
double pivotToWheel = 114.0;           //distance from pivot point to centre of the driving wheel [mm]
double drivingWheelRatio = 1.0;	    // ratio of wheel shaft rotations to wheel motor shaft rotations
double armRatio = 7.0;				// ratio of arm shaft rotations to arm motor shaft rotations
double encCountPerRev = 900.0;	    // number of encoder ticks per 1 revolution of the motor shaft
// ------------------------------------------------------------------------------------

/* Write your code in the function below. You may add helper functions below the studentCode function. */
void student_Main()
{

    // Demonstration Script
    
    //initialising variables for use in our functions
    int sonarDist;
    double distKp = 2.2, distKi = 1/4.9, armKp = 3.0;


    //Calibrating arm for rotateArm function
    armUp(4000);
    delay(300);
    resetEncoder(ArmEncoder);
    
    //Driving to the can
    driveDistStraightPI((673 + 94 + 46), distKp, distKi); //get pivot point to align with the black line's vertical center(using map dimensions)
    delay(300);
    sonarDist = driveToPayload(320); 
    delay(300);

    //Picking up the can
    rotateArm(-12, armKp);
    delay(300);
    driveDistStraightPI(60, distKp, distKi);
    delay(300);
    armUp(4000);
    delay(300);

    //Driving back so that pivot point is in line with the Black lines vertical centre
    driveDistStraightPI(-(sonarDist - 220), distKp, distKi); 
    delay(300);

    //Driving to the Black line to start line following
    turnRobotBody(-90);
    delay(300);
    driveUntilBlackSTR(40);
    delay(1000);

    //Drive off black line onto brown line
    driveUntilBrown(30); 
    
    LineFollowing();
    delay(300);

    //Dropping off payload
    driveDistStraightPI((714-280-44), distKp, distKi);//inputted distance = 390 and uses a combination of map dimmensions and offsets
    delay(300);
    rotateArm(-12, armKp);
    delay(300);

    //Returning to finishing area
    driveDistStraightPI(-102, distKp, distKi);
    delay(300);
    armUp(4000);
    delay(300);
    turnRobotBody(-90);
    delay(300);
    driveDistStraightPI(1290, distKp, distKi);//1308 - and offset
    delay(300);
    turnRobotBody(90);
    delay(300);
    driveDistStraightPI((714-194-50-114), distKp, distKi);//inputted distance = 356 and uses a combination of map dimmensions and offsets
    

}

// ----------------------------------------------- Function defintions go here  -----------------------------------------------//
// Don't forget to add your function prototypes to Student_Code.h


int convertPower(double inputPercPower) { 
    //This function converts the inputted % power to mV
    int voltage;
    double saturatedPercPower = saturate(inputPercPower, -100, 100); //guaranteeing input percentage power values will be between -100 and 100
    
    voltage = (saturatedPercPower * 4000) / 100; //finding the voltage in milivolts

    return voltage;
}


double getPosition(){
    //This function gets the current position of the robot reletive to the last time the encoders were reset

    //Initialising  variables
    double LCounts, RCounts, countsPerRev = 900.0;

    //Reading the encoders into their respective variables
    LCounts = readSensor(LeftEncoder);
    RCounts = readSensor(RightEncoder); 

    //Averaging encoder counts and then converting to distance in mm
    double avgCounts = ((LCounts + RCounts) / 2.0);

    //Converting encoder counts to distance in mm
    double distance = ((avgCounts/countsPerRev) * ((double) drivingWheelDiameter * PI));

    return distance;
}


double getAnglePos(){
    //This function returns the angle at which the robot arm is currently at
    //when the arm is at 0 degrees, it is horizontal

    //Initialising  variables
    double ArmAngle, ArmCounts, degreesPerRev = 360.0, countsPerRev = 900.0, GearRatio = 7.0;

    ArmCounts = (double) readSensor(ArmEncoder);

    //Converting to an angle in DEGREES
    ArmAngle = ((ArmCounts * degreesPerRev) / (countsPerRev * GearRatio));
    
    return ArmAngle;
}


void driveDistStraightPI(double targetDist, double distKp, double distKi){
    // This function drives a specified distance in a straight line and uses a PI-Controller to drive
    //the specified distance without any steady state error

    //initialise variables
    double currentPos, distError, distU = 0.0, straightKp = 1, rampingUpK = 1.5, errorIntegral = 0, loopDelay = 50;  
    int power = 0, exitCondition = false, straightError, straightU, elapsedTime, rampingUpPower, rampingUpU, straightPower;
    int prevPower = 0, secondPrevPower, currPower = 0, direction, exitTime, inToleranceRange = false;
    
    resetEncoder(LeftEncoder); // zero'ing (resetting) encoders
    resetEncoder(RightEncoder);
    resetTimer(T_1); //resetting timer to use for ramping up speed

    direction = targetDist/fabs(targetDist); // initialise variable that holds the direction of motion.


    do{
        currentPos = getPosition(); // getting current position from wheel encoders.

        //Finding our current error
        distError = targetDist - currentPos; 

        distU = (distKp * distError) + (distKi * errorIntegral); // calculating u: control effort in power percentage                   
        power = convertPower(distU); //saturate and convert from % power to mv 

        //anti-windup scheme
        if (fabs(distU) <= 100) { // checking if previous actuator effect was saturated. If its not saturated, we integrate
            errorIntegral = errorIntegral + distError; //accumulating error areas over time from limits 0 to k (integration)           
        }
        
        

        //If the direction of the robot changes, we want it to ramp up power again
        //for smooth movement

        //These variables keep track of the power values from previous loops
        secondPrevPower = prevPower;
        prevPower = currPower;
        currPower = distU;

        //This conditional statement will reset the timer if there is a change in direction
        if (((currPower > 0) && (secondPrevPower < 0)) || ((currPower < 0) && (secondPrevPower > 0))){
            resetTimer(T_1); //if the direction changes, reset timer so that ramping u works
            
        }

        
        //Implementing ramping up 
        elapsedTime = readTimer(T_1);
        direction = distU/(fabs(distU));
        rampingUpU = (int) rampingUpK * elapsedTime * direction; //rampingUpK is linear coefficient to calculate rampingUpPower in %
        rampingUpPower = convertPower(rampingUpU); // converting from % to mv


        //Using a controller to drive in a straight line
        straightError = readSensor(RightEncoder) - readSensor(LeftEncoder); // calculating encoder error = thetaR - thetaL
        straightU = (int) straightKp * straightError; // calculating control effort in encoder correction in %
        straightPower = convertPower(straightU); // converting to mv


        //Applying power to the motors
        //If we have jujst started moving, we want the robot to ramp up its power to the wheels
        //so there are no inaccuracies(wheelies etc.). This is accomplished by the following conditional statement
        if (abs(rampingUpPower) < power){ 
            motorPower(LeftMotor, rampingUpPower + straightPower); // adjusting motor power to correct for differences through both wheels.
            motorPower(RightMotor, rampingUpPower - straightPower); // If straightU is +ve, add more power to left motor and less power to right motor. And vice versa

           
        } else{
            motorPower(LeftMotor, power + straightPower); 
            motorPower(RightMotor, power - straightPower);
        }

       


        // timer based exit condition below
        // if the error is less than absolute 2.5 mm for 1+ second --> stop the wheels
        if (fabs(distError) < 2.5){ 
            if(inToleranceRange == true){ // checking if the robot has entered the tolerance range from the previous loop(or else it would constantlky reset the timer)
                exitTime = readTimer(T_2); // continuously reading the timer when the robot is in the tolerance range since the loop it entered the range.
                if(exitTime >= 1000){
                    exitCondition = true; // prepare exit condition
                    motorPower(LeftMotor, 0); // stopping wheels
                    motorPower(RightMotor, 0);
                }
            } else {
                resetTimer(T_2); // when robot has JUST entered the tolerance range (we know this if inToleranceRange is false from previous loop), reset the timer
                inToleranceRange = true; // set inToleranceRange state to true, so timer is not reset every loop.
            }
        } else {
            inToleranceRange = false; // changed to false when robot leaves the tolerance range, and stays false anytime outside of the tolerance range 
        }


        delay(50); //minimum delay between continuous sensor readings


    } while(!exitCondition);
}


void rotateArm(double targetAngle, double Kp){
    //This function will move the robot's arm to the desired input angle, with a desired input Kp
    // OffsetAngle = Angle from horizontal when arm is at highest position
    // Angle 0 degrees is parallel to ground
    
    // initialise variables
    double currentAngle, error, u = 0, anglePosFromTop;
    int power = 0, exitCondition = false;
    double OffsetAngle = 54; // measured offset angle
    
    do{
        anglePosFromTop = getAnglePos(); // getting current arm angular position
        currentAngle = (double) OffsetAngle + anglePosFromTop; // calculate current angle position in degrees.
        error = targetAngle - currentAngle; // updating error: degrees from desired location (E = R - Y)
        u = (Kp*error); // calculating u: control effort in % power                  
        power = convertPower(u); //saturate and convert from % power to mV 
        
        motorPower(ArmMotor, power); //send control voltage to actuator (motor)
        
        // tolerance based conditions to terminate loop. Tolerance range set to 2 degrees 
        if (fabs(error) <=2) {
            motorPower(ArmMotor,0);
            exitCondition = true;
        }
            
        delay(50); //minimum delay between continuous sensor readings
        
    } while(!exitCondition);

}


void turnRobotBody(double turningAngle){
    //This function turns the robot about its pivot point at a specified angle in the input.
    // robot rotates about its central pivot point
    // positive input = ccw. negative input = cw.

    //Initialising  variables
    int exitCondition = false, direction;
    int  elapsedTime, rampingUpPower, power, equalPower;
    double rampingUpK = 0.006;
    double equalPowerError, equalPowerU, equalPowerKp = 5/40; //placeholder value for equalPowerKp
    double arcLengthError, arcLengthU, arcLengthKp = 5;
    double arcLengthTarget, turnedArcLength, rampingUpU;
    double leftReading, rightReading, DegToRad = PI/180;;
    arcLengthTarget = (pivotToWheel * (fabs(turningAngle)) * DegToRad); // 114mm is radius from pivot point to back wheel center. arcLength = radius * angle (in rad)

    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);// zero'ing (resetting) encoders
    resetTimer(T_1);
    direction = turningAngle/(fabs(turningAngle)); // +ve --> ccw

    do{
        leftReading = (double) readSensor(LeftEncoder);
        rightReading = (double) readSensor(RightEncoder);//reading encoder counts into respective variables

        turnedArcLength = ((fabs(leftReading) + fabs(rightReading))/(2*encCountPerRev)) * drivingWheelDiameter * PI; // converting encoder counts to average mm, and getting current position
        arcLengthError = arcLengthTarget - turnedArcLength; //updating error 
        arcLengthU = arcLengthKp * arcLengthError * direction; // calculating control effort in PERCENTAGE
        power = convertPower(arcLengthU); // saturate and convert from % power to mV.
        
        equalPowerError = abs(readSensor(RightEncoder)) - abs(readSensor(LeftEncoder)); // calculating difference in counts. If +ve, add more to leftMotor & strip some off rightMotor. 
        equalPowerU = equalPowerKp * equalPowerError; // calculating control effort in encoder correction IN %
        equalPower = convertPower(equalPowerU);// saturate and convert from % power to mV.

        //initialising ramping up
        elapsedTime = readTimer(T_1);
        rampingUpU = direction * ((rampingUpK * ((double)elapsedTime)) + 50); //linear model for ramping up, with the base power of 50% when time = 0.
        rampingUpPower = convertPower(rampingUpU); //converting to mv.
        
        //if we're just starting to turn, ramp up the power slowly, the control is handed over to the normal power controller once ramping up power exceeds the normal power.
        if(abs(rampingUpPower) < abs(power)){
            motorPower(LeftMotor, -1 * (rampingUpPower + equalPower)); // if direction is +ve, turn ccw so send negative power to left wheel.
            motorPower(RightMotor, (rampingUpPower - equalPower));
        } else{
            motorPower(LeftMotor, -1 * (power + equalPower)); // if direction is +ve, turn ccw so send negative power to left wheel.
            motorPower(RightMotor, (power - equalPower));
        } 

        // tolerance based conditions to terminate loop. Tolerance range set to 4 mm (equivalent to 2 degrees)
        if(fabs(arcLengthError) <= 4) {
            motorPower(LeftMotor,0);
            motorPower(RightMotor,0);
            exitCondition = true;
        }
        delay(50);
        
    } while(!exitCondition);
    return;
}


void driveUntilBlackSTR(double drivePowerPerc){
    // this function drives in a straight line until it detects a black line, where it stops. 
    
    //Initialising  variables
    int readingLeft, readingMid, readingRight;
    double currentLeftWheelPower, currentRightWheelPower;
    int exitCondition = false; // initialising while loop condition 
    int error;
    double straightKp = 0.5, changeU; 

    // range for black light values from testing
    int BLACK_UPPER = 2900;
    int BLACK_LOWER = 2400;
    
    // zero'ing (resetting) wheel encoders
    resetEncoder(LeftEncoder); 
    resetEncoder(RightEncoder);

    do{
        readingLeft = readSensor(LeftLight);
        readingMid = readSensor(MidLight);
        readingRight = readSensor(RightLight); // appropriate sensors taking readings

        currentLeftWheelPower = drivePowerPerc;
        currentRightWheelPower= drivePowerPerc; 

        // P closed loop control to correct errors in wheel motor power differences to drive straight.
        error = readSensor(LeftEncoder) - readSensor(RightEncoder);
        changeU = (straightKp*error); // +ve control effect in percentage--> add more power to left wheel, strip off right wheel. For -ve, vice versa
        currentLeftWheelPower -= changeU; // calculate control effects of actuators in percentage
        currentRightWheelPower += changeU; 
        motorPower(LeftMotor, convertPower(currentLeftWheelPower)); // sending saturated power to motors.
        motorPower(RightMotor, convertPower(currentRightWheelPower));

        // if any of the sensors has detected black
        if (((readingLeft >= BLACK_LOWER) && (readingLeft <= BLACK_UPPER)) || ((readingMid >= BLACK_LOWER) && (readingMid <= BLACK_UPPER)) || ((readingRight >= BLACK_LOWER) && (readingRight <= BLACK_UPPER))) {
            exitCondition = true; // prepare exiting condition
            motorPower(LeftMotor, 0);
            motorPower(RightMotor, 0); //stop motors 
        }  

        delay(50); // minimum delay between continuous sensor readings

    } while (!exitCondition);

    return;
}


int driveToPayload(int distFromPayload){
    //This function drives a specified distance from an object that is detected using the ultrasonic sensor
    //It drives the distance using a PI controller
    int sensDist;
    double kp = 2.2, ki = 1.0/4.9; //setting Kp and Ki constants found from testing

    sensDist = readSensor(SonarSensor); //reads distance from payload 
    delay(500);

    //drive the relevant distance
    driveDistStraightPI((double)(sensDist - distFromPayload), kp, ki);
    return sensDist; 
}


void LineFollowing(){
    //this function follows a brown line until it sees a black line

    int exitCondition = false, leftReading, midReading, rightReading, lastTurnMemory = 0;
    //initialising our bounds for light values
    int BLACK_LOWER = 2400, BLACK_UPPER = 2900, BROWN_LOWER = 1700, BROWN_UPPER = 2200;

    while(!exitCondition){

        leftReading = readSensor(LeftLight);
        midReading = readSensor(MidLight);
        rightReading = readSensor(RightLight); //taking appropriate light sensor readings

        if (!((BROWN_LOWER <= leftReading) && (leftReading <= BROWN_UPPER)) && ((BROWN_LOWER <= midReading) && (midReading <= BROWN_UPPER)) && !((BROWN_LOWER <= rightReading) && (rightReading <= BROWN_UPPER))){ 
            //if only mid sensor sees brown
            motorPower(LeftMotor, 2000);
            motorPower(RightMotor, 2000);
            
        } else if (((BROWN_LOWER <= leftReading) && (leftReading <= BROWN_UPPER)) && !((BROWN_LOWER <= midReading) && (midReading <= BROWN_UPPER)) && !((BROWN_LOWER <= rightReading) && (rightReading <= BROWN_UPPER))){ 
            //if only left sees brown
            motorPower(LeftMotor, -2000);
            motorPower(RightMotor, 2000);
            lastTurnMemory = 1;
            
        } else if (!((BROWN_LOWER <= leftReading) && (leftReading <= BROWN_UPPER)) && !((BROWN_LOWER <= midReading) && (midReading <= BROWN_UPPER)) && ((BROWN_LOWER <= rightReading) && (rightReading <= BROWN_UPPER))){ 
            //if only right sees brown
            motorPower(LeftMotor, 2000);
            motorPower(RightMotor, -2000);
            lastTurnMemory = -1;
            
        } else if (((BROWN_LOWER <= leftReading) && (leftReading <= BROWN_UPPER)) && ((BROWN_LOWER <= midReading) && (midReading <= BROWN_UPPER)) && !((BROWN_LOWER <= rightReading) && (rightReading <= BROWN_UPPER))){ 
            //if mid and left sees brown
            motorPower(LeftMotor, 2000);
            motorPower(RightMotor, 2000);
            lastTurnMemory = 1;
            
        } else if (!((BROWN_LOWER <= leftReading) && (leftReading <= BROWN_UPPER)) && ((BROWN_LOWER <= midReading) && (midReading <= BROWN_UPPER)) && ((BROWN_LOWER <= rightReading) && (rightReading <= BROWN_UPPER))){ 
            //if mid and right sees brown
            motorPower(LeftMotor, 2000);
            motorPower(RightMotor, 2000);
            lastTurnMemory = -1;
            
        } else if (((BROWN_LOWER <= leftReading) && (leftReading <= BROWN_UPPER)) && ((BROWN_LOWER <= midReading) && (midReading <= BROWN_UPPER)) && ((BROWN_LOWER <= rightReading) && (rightReading <= BROWN_UPPER))){ 
            //if all sensors sees brown
            motorPower(LeftMotor, 2000);
            motorPower(RightMotor, 2000);
            
        } else { //if no sensors see, or both side sensors see, turn in the direction of the sensor where brown was last seen. 
            motorPower(LeftMotor, 2000*-lastTurnMemory);
            motorPower(RightMotor, 2000*lastTurnMemory);
            
        }
        
        if (((BLACK_LOWER <= leftReading) && (leftReading <= BLACK_UPPER)) || ((BLACK_LOWER <= midReading) && (midReading <= BLACK_UPPER)) || ((BLACK_LOWER <= rightReading) && (rightReading <= BLACK_UPPER))){ // if any of the sensors see black
            exitCondition = true; // prepare exiting condition
            motorPower(LeftMotor, 0);
            motorPower(RightMotor, 0); //stop motors
        }
        delay(50);
    }

    return;
}


void driveUntilBrown(double drivePowerPerc){
    //This function will drive from when the light sensors are seeing black, to when at least one of them sees brown
    //so that line following can comence

    int readingLeft, readingMid, readingRight;
    int exitCondition = false; // initialising while loop condition 

    //initialising our bounds for light values
    int BROWN_UPPER = 2200, BROWN_LOWER = 1700, BLACK_UPPER = 2900, BLACK_LOWER = 2400;
    int seeBrown = false, seeBlack = true;

    motorPower(LeftMotor, convertPower(drivePowerPerc)); // initalise motor power. 
    motorPower(RightMotor, convertPower(drivePowerPerc)); 

    while(!exitCondition){
        readingLeft = readSensor(LeftLight);
        readingMid = readSensor(MidLight);
        readingRight = readSensor(RightLight); // appropriate sensors taking readings


        //Setting up exit conditions
        if (((readingLeft <= BROWN_UPPER) && (readingLeft >= BROWN_LOWER)) || ((readingMid <= BROWN_UPPER) && (readingMid >= BROWN_LOWER)) || ((readingRight <= BROWN_UPPER) && (readingRight >= BROWN_LOWER))){
           // if any of the light sensors sees brown
           seeBrown = true; 
        }

        if (!((readingLeft <= BLACK_UPPER) && (readingLeft >= BLACK_LOWER)) && !((readingMid <= BLACK_UPPER) && (readingMid >= BLACK_LOWER)) && !((readingRight <= BLACK_UPPER) && (readingRight >= BLACK_LOWER))){
           //if none of the light sensors see black
            seeBlack = false;
        }
        
        if ((seeBrown == true) && (seeBlack == false)){
           //exit conditions met, where the robot is touching the brown line and completely off the black line. 
            exitCondition = true;
            motorPower(LeftMotor, 0);
            motorPower(RightMotor, 0);//stop motors
        }
        delay(50); //minimum delay between loops
    }

    return;
}


