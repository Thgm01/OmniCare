#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "main.h"


#include "robot_data.h"
#include "defines.h"

/* PINOS ENCODERS 1
 * PRETO  : GND
 * CINZA  : CHANEL A (A0)
 * BRANCO : 3V3
 * MARROM :	CHANEL B (A1)
 *
 * PINOS ENCODERS 2
 * PRETO  : GND
 * CINZA  : CHANEL A (D12)
 * BRANCO : 3V3
 * MARROM :	CHANEL B (D11)
 *
 * PINOS ENCODERS 3
 * PRETO  : GND
 * CINZA  : CHANEL A (D15)
 * BRANCO : 3V3
 * MARROM :	CHANEL B (D14)
 *
 */

Motors motors_data[4];

extern TIM_HandleTypeDef *encoders[4];

void Init_Motors_Data()
{
	for(int i=0; i<3; i++)
	{
		motors_data[i].position_m = 0;
		motors_data[i].velocity_rpm = 0;
		motors_data[i].old_count = 0;
	}
}

void Update_Motors_Data(int Motor_Number, int32_t count)
{
	int32_t diff_count = count - motors_data[Motor_Number].old_count;

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

	motors_data[Motor_Number].position_m += distance_m;


 	/*
	 * CALCULO DA VELOCIDADE
	 *
	 * 		Rotacoes    		 			 Counts      							 Rotacao     			60 Segundos
	 * 	    -------- =  				   --------- 					 * 			--------- 			 * -------------
	 *  	 Minuto      		 			Segundo     							 Counts	      		       Minuto
	 */
	float velocidade_rpm = (diff_count / SPEED_READ_INTERVAL_MS * 100) * (1/(MOTOR_COUNTS_PER_ROTATION)) * 60;

	motors_data[Motor_Number].velocity_rpm = velocidade_rpm;



	// Salvando o valor do count para proxima atualizacao de dados
	motors_data[Motor_Number].old_count = count;

//	DEBUG PRINTS
//	printf("MOTOR %d \n", Motor_Number);
//	printf("Posicao: %d | Velocidade: %d \n", (int)(motors_data[Motor_Number-1].position_m*1000) , (int) (motors_data[Motor_Number-1].velocity_rpm*1000));
//	printf("Posicao: %f | Velocidade: %d \n", motors_data[Motor_Number-1].position_m , (int) (motors_data[Motor_Number-1].velocity_rpm*1000));
//	printf("\n");
}

