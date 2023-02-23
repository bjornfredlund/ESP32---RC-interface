#ifndef STEERING_H
#define STEERING_H
#include <inttypes.h>
#ifdef __cplusplus
extern "C" { 
#endif

	void set_angle(double);		
	void init_servo();

#ifdef __cplusplus
} 
#endif

void start_up_sequence(void*);
inline uint32_t convert_servo_angle_to_duty_us(double angle);
#endif
