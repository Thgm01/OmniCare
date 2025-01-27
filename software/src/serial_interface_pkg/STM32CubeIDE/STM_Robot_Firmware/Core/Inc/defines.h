/*
 * defines.h
 *
 *  Created on: Jan 23, 2025
 *      Author: thi-m
 */

#ifndef INC_DEFINES_H_
#define INC_DEFINES_H_

#define GEAR_RATIO 127.7
#define ENCODER_COUNTS_PER_ROTATION 100
#define MOTOR_COUNTS_PER_ROTATION (GEAR_RATIO*ENCODER_COUNTS_PER_ROTATION)

#define SPEED_READ_INTERVAL_MS 10

#define WHEEL_RADIUS_MM 60

#endif /* INC_DEFINES_H_ */
