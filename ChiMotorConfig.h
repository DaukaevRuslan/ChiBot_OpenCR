#ifndef CHI_MOTOR_CONFIG_H
#define CHI_MOTOR_CONFIG_H

	#define SPEED_RATIO				0.0285
	#define ENCODER_POLLING_RATE	20

	#define V_KP	10
	#define V_KI	110
	#define V_KD	0

	#define P_KP	7	//7		//7
	#define P_KI	2	//2     //15
	#define P_KD	0 //0     //0.2

	#define MIN_PWM	-255
	#define MAX_PWM	255

#endif