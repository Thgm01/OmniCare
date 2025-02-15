#ifndef INC_ROBOT_DATA_H_
#define INC_ROBOT_DATA_H_

#include <stdint.h>

#include "encoder.h"

typedef struct
{
	float position_m;
	float velocity_rpm;
	Encoder encoder;
} Motors;

extern Motors motors_data[3];

void Init_Motors_Data();
void Update_Motors_Data(int Motor_Number);
void update_all_motors_data();

void set_motors_velocity(int16_t *velocity_list);



#endif /* INC_ROBOT_DATA_H_ */
