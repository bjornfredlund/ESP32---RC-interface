#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm.h"

#include "steering.h"

#define SERVO_MIN_PULSEWIDTH_US 	 (1000.0) // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 	 (2000.0) // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 			 (90.0)   // Maximum angle in degree upto which servo can rotate
#define SERVO_MAX_DEGREE_CONSTRAINED (60.0)   // Maximum angle in degree upto which servo is able to rotate

#define SERVO_PULSE_GPIO             (33.0)   // GPIO connects to the PWM signal line


static const char *TAG = "Steering";

void init_servo()
{ 
	ESP_LOGI(TAG, "Starting up");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PULSE_GPIO); // To drive a RC servo, one MCPWM generator is enough

    mcpwm_config_t pwm_config = {
        .frequency = 50, // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,     // duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

	xTaskCreate(start_up_sequence,"Startup",10024,NULL,tskIDLE_PRIORITY,NULL);
} 

void start_up_sequence(void* args)
{ 
	for (double angle = 3.14*SERVO_MAX_DEGREE_CONSTRAINED/180.0; angle > -3.14*SERVO_MAX_DEGREE_CONSTRAINED/180.0; angle-=3.14*0.5/180)
	{
		set_angle(angle);
		vTaskDelay(pdMS_TO_TICKS(5)); //Add delay, since it takes time for servo to rotate
	}
	set_angle(0);
	vTaskDelete(NULL); /* Delete this task */ 
	ESP_LOGI(TAG, "Done");
} 
void set_angle(double angle)
{
	uint32_t duty = convert_servo_angle_to_duty_us(angle);
	ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,(uint32_t) duty));
}

inline uint32_t convert_servo_angle_to_duty_us(double angle)
{
	angle = 180*angle/3.14;

	return (angle + SERVO_MAX_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (2 * SERVO_MAX_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}
