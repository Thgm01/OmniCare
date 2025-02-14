/*
 * pin_mapping.h
 *
 *  Created on: Jan 29, 2025
 *      Author: thiago
 */

#ifndef INC_PIN_MAPPING_H_
#define INC_PIN_MAPPING_H_

#include "stm32u5xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern TIM_HandleTypeDef *ENCODER_0;
extern TIM_HandleTypeDef *ENCODER_1;
extern TIM_HandleTypeDef *ENCODER_2;


#define MOTOR0_PMW_CW  		(*((volatile uint32_t *)&TIM5->CCR1))
#define MOTOR0_PMW_ACW 		(*((volatile uint32_t *)&TIM5->CCR2))

#define MOTOR1_PMW_CW (*((volatile uint32_t *)&TIM5->CCR3))
#define MOTOR1_PMW_ACW (*((volatile uint32_t *)&TIM5->CCR4))

#define MOTOR2_PMW_CW (*((volatile uint32_t *)&TIM15->CCR1))
#define MOTOR2_PMW_ACW (*((volatile uint32_t *)&TIM5->CCR2))

#endif /* INC_PIN_MAPPING_H_ */
