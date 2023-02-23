#ifndef CAR_H
#define CAR_H
#include "steering.h"
#include <motor.h>
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include "telemetry.h"


#ifdef __cplusplus
class Car
{ 
	public:
		Car(telemetry_t*, Motor);
		void start();
	private:
		telemetry_t* telemetry;
		Motor motor;
};
#else
typedef struct Car Car;
#endif

#endif
