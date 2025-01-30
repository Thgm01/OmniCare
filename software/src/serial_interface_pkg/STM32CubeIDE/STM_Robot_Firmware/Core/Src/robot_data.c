#include "robot_data.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "encoder.h"
#include "defines.h"
#include "pin_mapping.h"

extern Motors motors_data[3];

void Init_Motors_Data()
{
	for(int i=0; i<3; i++)
	{
		motors_data[i].position_m = 0;
		motors_data[i].velocity_rpm = 0;
		init_encoder(&motors_data[i].encoder);
	}
}

void Update_Motors_Data(int Motor_Number)
{
	int32_t diff_count = motors_data[Motor_Number].encoder.last_count - motors_data[Motor_Number].encoder.old_count;

	// verificacao para tratar estouro do buffer
 	if(abs(diff_count) > 2e16)
	{
		diff_count = (2e32 - abs(diff_count)) * (-diff_count/(abs(diff_count)));
	}

 	// dividindo diff por 4 pois a cada pulso do encoder o cout incrementa 4
 	diff_count /= 4;

 	/*
	 * CALCULO DA DISTANCIA PERCORRIDA
	 */
	float angle_rad = 2 * M_PI * diff_count / (MOTOR_COUNTS_PER_ROTATION);

	// como a distancia esta em mm, dividimos por 1000 paraficar em m
	float distance_m = angle_rad *  WHEEL_RADIUS_MM/1e3;

	motors_data[Motor_Number].position_m += distance_m * ENCODERS_DIRECTION[Motor_Number];


 	/*
	 * CALCULO DA VELOCIDADE
	 *
	 * 		Rotacoes    		 			 Counts      							 Rotacao     			60 Segundos
	 * 	    -------- =  				   --------- 					 * 			--------- 			 * -------------
	 *  	 Minuto      		 			Segundo     							 Counts	      		       Minuto
	 */
	float velocidade_rpm = (diff_count / SPEED_READ_INTERVAL_MS * 1e2) * (1/(MOTOR_COUNTS_PER_ROTATION)) * 60 ;
	//por algum motivo para dar o speed_read_interval certo na funcao a conversao de ms pra  tem que ser /10

	motors_data[Motor_Number].velocity_rpm = velocidade_rpm * ENCODERS_DIRECTION[Motor_Number];

	// Salvando o valor do count para proxima atualizacao de dados
	motors_data[Motor_Number].encoder.old_count = motors_data[Motor_Number].encoder.last_count;
}

void update_all_motors_data()
{
	Update_Motors_Data(0);
	Update_Motors_Data(1);
	Update_Motors_Data(2);
}


void set_motors_velocity(int16_t *velocity_list)
{
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

