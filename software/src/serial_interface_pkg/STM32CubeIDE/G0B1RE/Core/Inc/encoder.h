/*
 * encoder.h
 *
 *  Created on: Dec 8, 2024
 *      Author: llagoeiro
 */

#ifndef __ENCODER_H_
#define __ENCODER_H_

#include "main.h"


struct Motor {
  int16_t encoder;
  int16_t last_encoder;
  int16_t velocity;
};

int16_t getEncoder(TIM_HandleTypeDef *htim);
//uint16_t updateEncoder(uint16_t encoder, TIM_HandleTypeDef *htim);
//
//uint16_t getVelocity(uint16_t encoder);



#endif /* SRC_ENCODER_H_ */
