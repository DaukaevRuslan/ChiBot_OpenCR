#ifndef CHI_MOTOR_H
#define CHI_MOTOR_H

#include "ChiMotorConfig.h"
#include <Arduino.h>

class ChiMotor {

	private:

		byte F;
		byte S;

		byte FE, SE, FM, SM;	// First encoder, Second encoder, First motor, Second motor(PWM)

		bool FlagWheelMode;
		bool FlagJointMode;
		bool FlagInterrupt;
		bool FlagNewGoalVelocity; // Проверить необходимость
		bool FlagSavePosition;

		int PWM;

		long Timer;
		int  HardPositionPrev;
		int  HardPosition;
		int  HardVelocity;
		int  GoalPosition;

		double GoalRadianVelocity;
		double RealRadianVelocity;
		double integralVel;
		double integralVelPrevErr;	
		double integralPos;
		double integralPosPrevErr;	
		
		void calcRealVelocity();
		void controlDriver();
		void velocityPID();
		void positionPID();
		double constrainPWM(double value);

		

	public:

		void init(int FE, int SE, int FM, int SM);
		void interruptListener();
		void wheelMode();
		void jointMode();
		void tick();

		double getRealRadianVelocity();
		double getGoalRadianVelocity();

		void setGoalVelocity(double _GoalRadianVelocity);




};

#endif
