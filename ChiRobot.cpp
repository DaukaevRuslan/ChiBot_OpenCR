#include "ChiRobot.h"

ChiRobot ChiBot = ChiRobot();

void ChiRobot::init(int _FL_FE, int _FL_SE, int _FL_FM, int _FL_SM,
			 	    int _BL_FE, int _BL_SE, int _BL_FM, int _BL_SM,
			 	    int _BR_FE, int _BR_SE, int _BR_FM, int _BR_SM,
			 	    int _FR_FE, int _FR_SE, int _FR_FM, int _FR_SM) {

	ChiMotorFL.init(_FL_FE, _FL_SE, _FL_FM, _FL_SM);
	ChiMotorBL.init(_BL_FE, _BL_SE, _BL_FM, _BL_SM);
	ChiMotorBR.init(_BR_FE, _BR_SE, _BR_FM, _BR_SM);
	ChiMotorFR.init(_FR_FE, _FR_SE, _FR_FM, _FR_SM);
}

void ChiRobot::tick() {

	if (FlagNewGoalVelocity) {
		FlagNewGoalVelocity = false;
		IKSolve();
	}

	ChiMotorFL.tick();
	ChiMotorBL.tick();
	ChiMotorBR.tick();
	ChiMotorFR.tick();
}

void ChiRobot::setGoalVelocity(double _GoalVelocityX, double _GoalVelocityY, double _GoalVelocityW) {

	FlagNewGoalVelocity = true;

	GoalVelocityX = constrainVel(_GoalVelocityX, MIN_GOAL_VELOCITY_X, MAX_GOAL_VELOCITY_X);
	GoalVelocityY = constrainVel(_GoalVelocityY, MIN_GOAL_VELOCITY_Y, MAX_GOAL_VELOCITY_Y);
	GoalVelocityW = constrainVel(_GoalVelocityW, MIN_GOAL_VELOCITY_W, MAX_GOAL_VELOCITY_W);
}

void ChiRobot::IKSolve() {

	double tempVel1 = -(1 / R) * (GoalVelocityX + GoalVelocityY + (L1 + L2) * GoalVelocityW / 2);
	double tempVel2 = -(1 / R) * (GoalVelocityX - GoalVelocityY + (L1 + L2) * GoalVelocityW / 2);
	double tempVel3 = (1 / R) * (GoalVelocityX + GoalVelocityY - (L1 + L2) * GoalVelocityW / 2);
	double tempVel4 = (1 / R) * (GoalVelocityX - GoalVelocityY - (L1 + L2) * GoalVelocityW / 2);

	ChiMotorFL.setGoalVelocity(tempVel1);
	ChiMotorBL.setGoalVelocity(tempVel2);
	ChiMotorBR.setGoalVelocity(tempVel3);
	ChiMotorFR.setGoalVelocity(tempVel4);

}

void ChiRobot::FKSolve() {

	double tempVel1 = ChiMotorFL.getRealRadianVelocity();
	double tempVel2 = ChiMotorBL.getRealRadianVelocity();
	double tempVel3 = ChiMotorBR.getRealRadianVelocity();
	double tempVel4 = ChiMotorFR.getRealRadianVelocity();

	RealVelocityX = ( tempVel1 + tempVel2 + tempVel3 + tempVel4) * (R / 4);
	RealVelocityY = (-tempVel1 - tempVel2 + tempVel3 + tempVel4) * (R / 4);
	RealVelocityW = (-tempVel1 + tempVel2 - tempVel3 + tempVel4) * (R / (2 * (L1 + L2)));

}

void ChiRobot::getRealVelocity(double &_RealVelocityX, double &_RealVelocityY, double &_RealVelocityW){

	FKSolve();

	_RealVelocityX = RealVelocityX;
	_RealVelocityY = RealVelocityY;
	_RealVelocityW = RealVelocityW;
}

void ChiRobot::getGoalVelocity(double &_GoalVelocityX, double &_GoalVelocityY, double &_GoalVelocityW) {

	_GoalVelocityX = GoalVelocityX;
	_GoalVelocityY = GoalVelocityY;
	_GoalVelocityW = GoalVelocityW;

}

double ChiRobot::constrainVel(double value, double min, double max) {

	if (value < min)
		return min;
	else if (value > max)
		return max;
	else return value;
}