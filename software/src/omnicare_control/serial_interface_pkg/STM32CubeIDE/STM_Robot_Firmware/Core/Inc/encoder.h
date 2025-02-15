/*
 * encoders.h
 *
 *  Created on: Jan 29, 2025
 *      Author: thiago
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>

#include "pin_mapping.h"

typedef struct
{
	int32_t old_count;
	int32_t last_count;
}Encoder;

void init_encoder(Encoder *encoder);

#endif /* INC_ENCODER_H_ */
