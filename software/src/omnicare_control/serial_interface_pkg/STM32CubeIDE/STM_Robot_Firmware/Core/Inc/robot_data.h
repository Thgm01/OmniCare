#ifndef INC_ROBOT_DATA_H_
#define INC_ROBOT_DATA_H_

#include <stdint.h>

#include "pin_mapping.h"
#include "robot_data.h"

typedef struct
{
	int16_t set_point_velocity;
	uint32_t encoder;
} MotorData;

extern MotorData motors_data[3];

void Init_Motors_Data();

void set_motors_velocity(int16_t *velocity_list);


extern const int ENCODERS_DIRECTION[];
void init_encoders();


#endif /* INC_ROBOT_DATA_H_ */
