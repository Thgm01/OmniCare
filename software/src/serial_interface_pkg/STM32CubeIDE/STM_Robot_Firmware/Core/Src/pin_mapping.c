/*
 * pin_mapping.c
 *
 *  Created on: Jan 29, 2025
 *      Author: thiago
 */

#include "pin_mapping.h"

/*	ENCODER 0 PINOUT
 * 	CH1: PA15 (CN7  Pin 17 de 38) | Arduino Pin XXX
 * 	CH2: PB3  (CN10 Pin 31 de 38) | Arduino Pin D3
 */
TIM_HandleTypeDef *ENCODER_0 = &htim2;

/*	ENCODER 1 PINOUT
 * 	CH1: PA6  (CN10 Pin 13 de 38) | Arduino Pin D12
 * 	CH2: PA7  (CN10 Pin 15 de 38) | Arduino Pin D11
 */
TIM_HandleTypeDef *ENCODER_1 = &htim3;

/*	ENCODER 2 PINOUT
 * 	CH1: PB6  (CN10 Pin 3  de 38) | Arduino Pin D15
 * 	CH2: PB7  (CN10 Pin 5  de 38) | Arduino Pin D14
 */
TIM_HandleTypeDef *ENCODER_2 = &htim4;

//#define MOTOR0_PMW_POSITIVE (TIM5->CCR1);
//#define MOTOR0_PMW_NEGATIVE (TIM5->CCR2);
//
//#define MOTOR1_PMW_POSITIVE (TIM5->CCR3);
//#define MOTOR1_PMW_NEGATIVE (TIM5->CCR4);
//
//#define MOTOR2_PMW_POSITIVE (TIM15->CCR1);
//#define MOTOR2_PMW_NEGATIVE (TIM15->CCR2);


