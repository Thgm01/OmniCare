#include "robot_data.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "tx_api.h"
#include "pin_mapping.h"

extern MotorData motors_data[3];

void Init_Motors_Data()
{
	init_encoders();
	motors_data[1].set_point_velocity = 0;
	motors_data[2].set_point_velocity = 0;

	for(int i=0; i<3; i++){
		motors_data[i].set_point_velocity = 0;
		motors_data[i].atual_velocity = 0;

		motors_data[i].PID[0] = 0.005;
		motors_data[i].PID[1] = 0;
		motors_data[i].PID[2] = 0;
	}
}

void actual_velocity(int Motor_Number,uint32_t dt)
{
	int32_t diff_count = motors_data[Motor_Number].encoder_actual_count - motors_data[Motor_Number].encoder_last_count;

	// verificacao para tratar estouro do buffer
 	if(abs(diff_count) > 2e16)
	{
		diff_count = (2e32 - abs(diff_count)) * (-diff_count/(abs(diff_count)));
	}

 	// pulse/second = (pulse/(ms * 10 ^-3))
 	int32_t encoder_per_second = (int32_t) (diff_count/(dt*1e-3));

 	motors_data[Motor_Number].atual_velocity = encoder_per_second;

	// Salvando o valor do count para proxima atualizacao de dados
 	motors_data[Motor_Number].encoder_last_count = motors_data[Motor_Number].encoder_actual_count;


}

void update_all_motors_data(uint32_t dt)
{
	actual_velocity(0,dt);
	actual_velocity(1,dt);
	actual_velocity(2,dt);
}



void set_motors_velocity_string(char *char_velocity_list)
{
	int32_t count_per_seconds_motors[3];
	char *token = strtok(char_velocity_list, " ");
	char _ = token[0];

	// Pegamos os próximos números
	for (int i = 0; i < 3; i++) {
	   token = strtok(NULL, " ");
	   count_per_seconds_motors[i] = (int32_t)atoi(token);
	}

	set_motors_velocity(count_per_seconds_motors);
}

void set_motors_velocity(int32_t *velocity_list)
{
	motors_data[0].set_point_velocity = velocity_list[0];
	motors_data[1].set_point_velocity = velocity_list[1];
	motors_data[2].set_point_velocity = velocity_list[2];
}

void update_velocity(int *velocitys_pwm){


	int pwm_positive = velocitys_pwm[0];
	int pwm_negative = 0;
	if(pwm_positive < 0)
	{
		pwm_positive = 0;
		pwm_negative = abs(velocitys_pwm[0]);
	}

	if(pwm_positive > 255) pwm_positive = 255;
	if(pwm_negative > 255) pwm_negative = 255;

	MOTOR0_PMW_POSITIVE = pwm_positive;
	MOTOR0_PMW_NEGATIVE = pwm_negative;



	pwm_positive = velocitys_pwm[1];
	pwm_negative = 0;
	if(pwm_positive < 0)
	{
		pwm_positive = 0;
		pwm_negative = abs(velocitys_pwm[1]);
	}

	if(pwm_positive > 255) pwm_positive = 255;
	if(pwm_negative > 255) pwm_negative = 255;

	MOTOR1_PMW_POSITIVE = pwm_positive;
	MOTOR1_PMW_NEGATIVE = pwm_negative;



	pwm_positive = velocitys_pwm[2];
	pwm_negative = 0;
	if(pwm_positive < 0)
	{
		pwm_positive = 0;
		pwm_negative = abs(velocitys_pwm[2]);
	}

	if(pwm_positive > 255) pwm_positive = 255;
	if(pwm_negative > 255) pwm_negative = 255;

	MOTOR2_PMW_POSITIVE = pwm_positive;
	MOTOR2_PMW_NEGATIVE = pwm_negative;

}

void motor_pid_control_thread_entry(unsigned long thread_input)
{
	int32_t last_error[3] = {0, 0 ,0};
	int32_t error[3] = {0, 0 ,0};
	double integral[3] = {0, 0 ,0};
	double derivate[3] = {0, 0 ,0};
	unsigned long atual_ms = (tx_time_get() * 1000) / TX_TIMER_TICKS_PER_SECOND;
	uint32_t last_time_ms = atual_ms;
	int first_time = 1;


	while(1){
//		if(first_time){
//			first_time = 0;
//		}
//		else{
//			int motor_vel_pwm[3] = {0};
//			atual_ms = (tx_time_get() * 1000) / TX_TIMER_TICKS_PER_SECOND;
//			uint32_t dt = atual_ms - last_time_ms;
//
//
//			// update all motors
//			update_all_motors_data(dt);
//
//
//			for(int i=0; i<3; i++){
//				error[i] 		= motors_data[i].set_point_velocity - motors_data[i].atual_velocity;
//				atual_ms = (tx_time_get() * 1000) / TX_TIMER_TICKS_PER_SECOND;
//				dt 				= atual_ms -last_time_ms;
//				integral[i] 	= error[i]*dt;
//				derivate[i] 	= (error[i] - last_error[i])/dt;
//
//				last_error[i] = error[i];
//				motor_vel_pwm[i] = (int)(motors_data[i].PID[0] * error[i] +
//										  motors_data[i].PID[1] * integral[i] +
//										  motors_data[i].PID[2] * derivate[i]);
//			}
////			update_velocity(motor_vel_pwm);
//		}
//		atual_ms = (tx_time_get() * 1000) / TX_TIMER_TICKS_PER_SECOND;
//		last_time_ms = atual_ms;
		tx_thread_sleep(10);
	}
}



const int ENCODERS_DIRECTION[] = {-1, -1, 1};

void init_encoders()
{
	motors_data[0].encoder_actual_count= 0;
	motors_data[1].encoder_actual_count = 0;
	motors_data[2].encoder_actual_count = 0;

	motors_data[0].encoder_last_count= 0;
	motors_data[1].encoder_last_count = 0;
	motors_data[2].encoder_last_count = 0;

}


