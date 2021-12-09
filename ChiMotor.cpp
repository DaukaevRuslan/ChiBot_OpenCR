#include "ChiMotor.h"

void ChiMotor::init(int _FE, int _SE, int _FM, int _SM) {

	Timer = millis();

	FE = _FE;
	SE = _SE;
	FM = _FM;
	SM = _SM;

	F = 0;
	S = 0;

	HardPositionPrev = 0;
	HardPosition 	 = 0;
	HardVelocity	 = 0;

	FlagInterrupt = false;

	pinMode(FE, INPUT);
	pinMode(SE, INPUT);
	pinMode(FM, OUTPUT);
	pinMode(SM, OUTPUT);
}

void ChiMotor::jointMode() {
	FlagWheelMode = false;
	FlagJointMode = true;
}

void ChiMotor::wheelMode() {
	FlagJointMode = false;
	FlagWheelMode = true;
}

void ChiMotor::setGoalVelocity(double _GoalRadianVelocity) {
	wheelMode();
	GoalRadianVelocity = _GoalRadianVelocity;
	FlagNewGoalVelocity = true;	// Проверить необходимость
}

void ChiMotor::calcRealVelocity() {
	if (millis() - Timer > ENCODER_POLLING_RATE) {
		Timer = millis();
		HardVelocity = HardPosition - HardPositionPrev;
		HardPositionPrev = HardPosition;
		RealRadianVelocity = HardVelocity * SPEED_RATIO * (1000 / ENCODER_POLLING_RATE); 
		
		//if (FlagNewGoalVelocity and FlagWheelMode) {
			if (GoalRadianVelocity != 0){

				
				velocityPID();
				controlDriver();
				FlagSavePosition = true;
			}
			else {
				if (FlagSavePosition){
					GoalPosition = HardPosition;
					FlagSavePosition = false;	
				}
				jointMode();
				positionPID();
				controlDriver();
			}

			
		
			//FlagNewGoalVelocity = true;
		//}
	}
}

double ChiMotor::getRealRadianVelocity() {
	return RealRadianVelocity;
}

double ChiMotor::getGoalRadianVelocity() {
	return GoalRadianVelocity;
}

void ChiMotor::tick() {

	if (FlagInterrupt) {		

		F = digitalRead(FE);
		S = digitalRead(SE);

		if (F == S)
			HardPosition++;
		else
			HardPosition--;

		FlagInterrupt = false;
	}

	calcRealVelocity();



}

void ChiMotor::interruptListener() {
	FlagInterrupt = true;
}

void ChiMotor::velocityPID() {
	
  double err = GoalRadianVelocity - RealRadianVelocity;
  integralVel = constrainPWM(integralVel + (double)err * V_KI * ((double)ENCODER_POLLING_RATE / 1000));
  double D = (err - integralVelPrevErr) / ((double)ENCODER_POLLING_RATE / 1000);
  integralVelPrevErr = err;
  PWM =  constrainPWM(err * V_KP + integralVel + D * V_KD);
}

void ChiMotor::positionPID() {
  double err = GoalPosition - HardPosition;
  integralPos = constrainPWM(integralPos + (double)err * P_KI * ((double)ENCODER_POLLING_RATE / 1000));
  double D = (err - integralPosPrevErr) / ((double)ENCODER_POLLING_RATE / 1000);
  integralPosPrevErr = err;
  PWM =  constrainPWM(err * P_KP + integralPos + D * P_KD);
}

void ChiMotor::controlDriver() {
	if (PWM < 0) {
    	analogWrite(SM, 255 + PWM);
    	digitalWrite(FM, HIGH);
  	}
  	if (PWM >= 0) {
    	analogWrite(SM, PWM);
    	digitalWrite(FM, LOW);
  	}
}

double ChiMotor::constrainPWM(double value) {
	if (value < MIN_PWM)
		return MIN_PWM;
	else if (value > MAX_PWM)
		return MAX_PWM;
	else return value;
}