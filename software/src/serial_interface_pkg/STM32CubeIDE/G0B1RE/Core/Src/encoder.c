/*
 * encoder.c
 *
 *  Created on: Dec 8, 2024
 *      Author: llagoeiro
 */
#include "encoder.h"


int16_t getEncoder(TIM_HandleTypeDef *htim){
	return (int16_t)__HAL_TIM_GET_COUNTER(htim);
}

//uint16_t updateEncoder(struct Motor motor, TIM_HandleTypeDef *htim){
//	static uint8_t first_time = 1;
//
//	if(first_time){
//		motor.encoder = 0;
//	}
//
//}





