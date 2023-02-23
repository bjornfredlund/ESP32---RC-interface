#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <freeRTOS/freertos.h>
#include <freeRTOS/task.h>
#include <freeRTOS/semphr.h>

#define MOTOR_READY_BIT  	(1 << 0)
#define GPS_READY_BIT 		(1 << 1)
#define STEERING_READY_BIT 	(1 << 2)
#define UPDATE_STEERING_BIT (1 << 3)

typedef struct telemetry_t{ 
	double desired_speed;
	double actual_speed;
	double steering_angle;
	double orientation;
	SemaphoreHandle_t mutex;
	TaskHandle_t* motor_task;
	TaskHandle_t* gps_task;
	TaskHandle_t car_task;
} telemetry_t;

#endif
