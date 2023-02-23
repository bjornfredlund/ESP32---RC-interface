#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "motor.h"
#include "esp_log.h"
#include <cstddef>

static const char *TAG = "Motor";

Motor::Motor(telemetry_t* tel): telemetry(tel)
{ 
} 

void Motor::set_reference_speed(double velocity)
{ 
	xSemaphoreTake(telemetry->mutex,50/portTICK_PERIOD_MS);
	telemetry->desired_speed = velocity;
	ESP_LOGI(pcTaskGetName(NULL),"Desired speed = %lf",telemetry->desired_speed*3.6);
	xSemaphoreGive(telemetry->mutex);
} 
