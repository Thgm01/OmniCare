#include "robot_data.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "pin_mapping.h"

extern MotorData motors_data[3];

void Init_Motors_Data()
{
	init_encoders();
	motors_data[0].set_point_velocity = 0;
	motors_data[1].set_point_velocity = 0;
	motors_data[2].set_point_velocity = 0;

}

void set_motors_velocity_string(char *char_velocity_list)
{
	int16_t pwm_motors[3];
	char *token = strtok(char_velocity_list, " ");
	char _ = token[0];

	// Pegamos os próximos números
	for (int i = 0; i < 3; i++) {
	   token = strtok(NULL, " ");
	   pwm_motors[i] = (int16_t)atoi(token);
	}

	set_motors_velocity(pwm_motors);
}

void set_motors_velocity(int16_t *velocity_list)
{
	motors_data[0].set_point_velocity = velocity_list[0];
	motors_data[1].set_point_velocity = velocity_list[1];
	motors_data[2].set_point_velocity = velocity_list[2];


	int pwm_positive = velocity_list[0];
	int pwm_negative = 0;
	if(pwm_positive < 0)
	{
		pwm_positive = 0;
//		pwm_negative = abs(velocity_list[0]);
		pwm_negative = 254;
	}
	else if (pwm_positive > 0){
		pwm_positive = 254;
		pwm_negative = 0;

	}

	MOTOR0_PMW_POSITIVE = pwm_positive;
	MOTOR0_PMW_NEGATIVE = pwm_negative;

	pwm_positive = velocity_list[1];
	pwm_negative = 0;
	if(pwm_positive < 0)
	{
		pwm_positive = 0;
//		pwm_negative = abs(velocity_list[1]);
		pwm_negative = 254;
	}
	else if (pwm_positive > 0){
		pwm_positive = 254;
		pwm_negative = 0;

	}

	MOTOR1_PMW_POSITIVE = pwm_positive;
	MOTOR1_PMW_NEGATIVE = pwm_negative;

	pwm_positive = velocity_list[2];
	pwm_negative = 0;
	if(pwm_positive < 0)
	{
		pwm_positive = 0;
//		pwm_negative = abs(velocity_list[2]);
		pwm_negative = 254;
	}
	else if (pwm_positive > 0){
		pwm_positive = 254;
		pwm_negative = 0;

	}

	MOTOR2_PMW_POSITIVE = pwm_positive;
	MOTOR2_PMW_NEGATIVE = pwm_negative;
}


const int ENCODERS_DIRECTION[] = {-1, -1, 1};

void init_encoders()
{
	motors_data[0].encoder = 0;
	motors_data[1].encoder = 0;
	motors_data[2].encoder = 0;
}


