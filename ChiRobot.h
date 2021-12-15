#ifndef CHI_ROBOT_H
#define CHI_ROBOT_H

#include "ChiMotor.h"
#include "ChiRobotConfig.h"

class ChiRobot {

	private:

		ChiMotor ChiMotorFL;
		ChiMotor ChiMotorBL;
		ChiMotor ChiMotorBR;
		ChiMotor ChiMotorFR;

		double GoalVelocityX;
		double GoalVelocityY;
		double GoalVelocityW;

		double RealVelocityX;
		double RealVelocityY;
		double RealVelocityW;

		long Timer;
		bool FlagNewGoalVelocity;

		void IKSolve();
		void FKSolve();

		double constrainVel(double value, double min, double max);

	public:

		void init(int _FL_FE, int _FL_SE, int _FL_FM, int _FL_SM,
			 	  int _BL_FE, int _BL_SE, int _BL_FM, int _BL_SM,
			 	  int _BR_FE, int _BR_SE, int _BR_FM, int _BR_SM,
			 	  int _FR_FE, int _FR_SE, int _FR_FM, int _FR_SM);

		void tick();

		void setGoalVelocity(double _GoalVelocityX, double _GoalVelocityY, double _GoalVelocityW);
		void getGoalVelocity(double &_GoalVelocityX, double &_GoalVelocityY, double &_GoalVelocityW);
		void getRealVelocity(double &_RealVelocityX, double &_RealVelocityY, double &_RealVelocityW);



};

extern ChiRobot ChiBot;

#endif
