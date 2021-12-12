#include "ChiRobot.h"

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
	GoalVelocityX = _GoalVelocityX;
	GoalVelocityY = _GoalVelocityY;
	GoalVelocityW = _GoalVelocityW;
}

void ChiRobot::IKSolve() {

	double tempVel1 = (1 / R) * (GoalVelocityX + GoalVelocityY + (L1 + L2) * GoalVelocityW / 2);
	double tempVel2 = (1 / R) * (GoalVelocityX + GoalVelocityY - (L1 + L2) * GoalVelocityW / 2);
	double tempVel3 = (1 / R) * (GoalVelocityX - GoalVelocityY - (L1 + L2) * GoalVelocityW / 2);
	double tempVel4 = (1 / R) * (GoalVelocityX - GoalVelocityY + (L1 + L2) * GoalVelocityW / 2);

	ChiMotorFL.setGoalVelocity(tempVel1);
	ChiMotorBL.setGoalVelocity(tempVel2);
	ChiMotorBR.setGoalVelocity(tempVel3);
	ChiMotorFR.setGoalVelocity(tempVel4);

}