#ifndef MOTOR_H
#define MOTOR_H
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>
#include "telemetry.h"



#ifdef __cplusplus
class Motor
{ 
	public:
		Motor(telemetry_t*);
		Motor() = delete;
		void set_reference_speed(double);
	private:
		telemetry_t* telemetry;
};
#else
	static inline uint32_t signal_to_duty_us(int);
	static void hall_interrupt_handle(void*);
	static uint32_t moving_average(int64_t,int64_t);
	static void set_duty_cycle(uint32_t);
	static void speed_update_task(void*);
#endif


#ifdef __cplusplus
extern "C" { 
#endif
	void init_motor_pwm();
	void init_vel_interrupt(telemetry_t*);
	void motor_controller(void*);

#ifdef __cplusplus
} 
#endif
#endif


