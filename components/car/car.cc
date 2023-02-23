#include "car.h"
#include "include/telemetry.h"
#include "motor.h"
#include <freeRTOS/freertos.h>
#include <freeRTOS/task.h>
#include <esp_log.h>

Car::Car(telemetry_t* tel, Motor m): telemetry(tel), motor(m)
{ 
	init_servo();
	uint32_t ulNotifiedValue;
	/* Wait til all setups have completed */
	while(xTaskNotifyWait(pdFALSE, 0x0, &ulNotifiedValue, portMAX_DELAY))
	{
		if(ulNotifiedValue & MOTOR_READY_BIT)
		{ 
			break;
		} 
	}
	ESP_LOGI(pcTaskGetName(NULL),"Ready to go");

} 
void Car::start() 
{
	for(;;)
	{ 
		uint32_t ulNotifiedValue;
		while(xTaskNotifyWait(ULONG_MAX, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY))
		{ 
			if(ulNotifiedValue & UPDATE_STEERING_BIT)
			{ 
				set_angle(telemetry->steering_angle);
			} 
		} 
	} 
}
