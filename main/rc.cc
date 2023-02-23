#include <stdio.h>
#include "car.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "communication.h"
#include "esp_log.h"
#include "gnss.h"
#include "motor.h"

#include "telemetry.h"


extern "C" telemetry_t* new_telemetry()
{ 
	telemetry_t* temp = (telemetry_t*)calloc(1,sizeof(telemetry_t));
	temp->mutex = xSemaphoreCreateMutex();
	temp->car_task = xTaskGetHandle(pcTaskGetName(NULL));
	return temp;
} 
extern "C" void app_main(void)
{
	vTaskDelay(300 / portTICK_PERIOD_MS);

	telemetry_t* tel = new_telemetry();

	TaskHandle_t receiver_handle{NULL};
	xTaskCreate(receiver_task, "Gnss", 8192, tel, tskIDLE_PRIORITY, &receiver_handle);

	TaskHandle_t gnss_task_handle{NULL};
	xTaskCreate(gnss_task, "Gnss", 8192, tel, tskIDLE_PRIORITY, &gnss_task_handle);

	TaskHandle_t motor_controller_handle{NULL};
	xTaskCreate(motor_controller, "Motor Controller", 16384, tel,tskIDLE_PRIORITY, &motor_controller_handle);
	Motor m{tel};
	Car car{tel, m};
	car.start();
}
