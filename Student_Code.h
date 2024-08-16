#ifndef STUDENT_CODE_H
#define STUDENT_CODE_H

void student_Main();    // The main entry point to the student code

// Add your function prototypes below
int convertPower(double inputPercPower);
double getPosition();
double getAnglePos();
void rotateArm(double targetAngle, double Kp);
void driveUntilBlackSTR(double drivePowerPerc);
void turnRobotBody(double turningAngle);
void driveDistStraightPI(double targetDist, double distKp, double distKi);
int driveToPayload(int distFromPayload);
void driveUntilBrown(double drivePowerPerc);
void LineFollowing();



// DO NOT ADD ANY PROTOTYPES AFTER THIS LINE
#endif  // STUDENT_CODE_H