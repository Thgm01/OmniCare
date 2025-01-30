/*
 * encoder.c
 *
 *  Created on: Jan 29, 2025
 *      Author: thiago
 */

#include "encoder.h"

void init_encoder(Encoder *encoder)
{
	encoder->old_count  = 0;
	encoder->last_count = 0;
}


