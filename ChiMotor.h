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
		bool FlagSavePosition;

		long Timer;
   
		int  HardPositionPrev;
		int  HardPosition;
		int  HardVelocity;
		int  GoalPosition;
	
		double integralVel;
		double integralVelPrevErr;	
		double integralPos;
		double integralPosPrevErr;	

    double constrainPWM(double value);
		void calcRealVelocity();
		void controlDriver();
		void velocityPID();
		void positionPID();	
    void wheelMode();
    void jointMode();

		int PWM;
    double GoalRadianVelocity;
    double RealRadianVelocity;

	public:

		void init(int FE, int SE, int FM, int SM);
		void interruptListener();
		void tick();

		double getRealRadianVelocity();
		double getGoalRadianVelocity();

		void setGoalVelocity(double _GoalRadianVelocity);
};

#endif
