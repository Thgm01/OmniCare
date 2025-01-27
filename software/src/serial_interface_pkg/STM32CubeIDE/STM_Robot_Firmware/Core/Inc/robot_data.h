#ifndef INC_ROBOT_DATA_H_
#define INC_ROBOT_DATA_H_

#include <stdint.h>

typedef struct
{
	float position_m;
	float velocity_rpm;
	int32_t old_count;
} Motors;

void Init_Motors_Data();
void Update_Motors_Data(int Motor_Number, int32_t count );

#endif /* INC_ROBOT_DATA_H_ */
