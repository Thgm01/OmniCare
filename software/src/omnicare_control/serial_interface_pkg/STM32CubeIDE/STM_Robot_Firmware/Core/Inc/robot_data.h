#ifndef INC_ROBOT_DATA_H_
#define INC_ROBOT_DATA_H_

#include <stdint.h>

#include "pin_mapping.h"
#include "robot_data.h"

typedef struct
{
	int32_t set_point_velocity; // definir qual a unidade de medida dessa variavel no ros2 control
	int32_t atual_velocity;  	// definir qual a unidade de medida dessa variavel no ros2 control
	uint32_t encoder_last_count;
	uint32_t encoder_actual_count;
	int32_t error;
	uint32_t dt;
	uint32_t PWM;
	float PID[3];
} MotorData;

extern MotorData motors_data[3];

void Init_Motors_Data();

void actual_velocity(int Motor_Number,uint32_t dt);
void update_all_motors_data(uint32_t dt);
void set_motors_velocity(int32_t *velocity_list);
void set_motors_velocity_string(char *velocity_list);
void update_velocity(int *velocitys_pwm);

void motor_pid_control_thread_entry(unsigned long thread_input);

extern const int ENCODERS_DIRECTION[];
void init_encoders();


#endif /* INC_ROBOT_DATA_H_ */
