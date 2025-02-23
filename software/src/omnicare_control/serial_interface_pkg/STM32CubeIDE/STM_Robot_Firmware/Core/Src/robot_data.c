#include "robot_data.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "defines.h"
#include "pin_mapping.h"

extern MotorData motors_data[3];

void Init_Motors_Data()
{
	init_encoders();
	motors_data[0].set_point_velocity = 0;
	motors_data[1].set_point_velocity = 0;
	motors_data[2].set_point_velocity = 0;

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
		pwm_negative = abs(velocity_list[0]);
	}

	MOTOR0_PMW_POSITIVE = pwm_positive;
	MOTOR0_PMW_NEGATIVE = pwm_negative;

	pwm_positive = velocity_list[1];
	pwm_negative = 0;
	if(pwm_positive < 0)
	{
		pwm_positive = 0;
		pwm_negative = abs(velocity_list[1]);
	}

	MOTOR1_PMW_POSITIVE = pwm_positive;
	MOTOR1_PMW_NEGATIVE = pwm_negative;

	pwm_positive = velocity_list[2];
	pwm_negative = 0;
	if(pwm_positive < 0)
	{
		pwm_positive = 0;
		pwm_negative = abs(velocity_list[2]);
	}

	MOTOR2_PMW_POSITIVE = pwm_positive;
	MOTOR2_PMW_NEGATIVE = pwm_negative;
}


void init_encoders()
{
	motors_data[0].encoder = 0;
	motors_data[1].encoder = 0;
	motors_data[2].encoder = 0;
}


