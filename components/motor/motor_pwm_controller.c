#include "motor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "driver/mcpwm.h"
#include <inttypes.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_log.h"

#define MOTOR_PWM_GPIO 32
#define MOTOR_UNIT MCPWM_UNIT_1
#define MOTOR_TIMER MCPWM_TIMER_1
#define MOTOR_PWM_IO_SIGNAL MCPWM1A

#define MOTOR_MIN_PULSEWIDTH_US 	 (1000) // Minimum pulse width in microsecond
#define MOTOR_MAX_PULSEWIDTH_US 	 (2000) // Maximum pulse width in microsecond
#define MOTOR_MAX 			 (2000)   // Maximum angle in degree upto which servo can rotate

#define HALL_PIN 26
#define ESP_INTR_FLAG_DEFAULT 0
#define BUFSIZE 4
#define WHEEL_RADIUS 0.05 /* m */
#define pi 3.14
#define FRACTIONAL_BITS 21

#define K 35.0
#define Ibin 6.0
#define CONTROLLER_PERIOD 50 /* ms */

#define COMMUNICATION_TIMEOUT 1000.0

static inline uint32_t signal_to_duty_us(int duty)
{ 
	return (duty + MOTOR_MAX) * (MOTOR_MAX_PULSEWIDTH_US - MOTOR_MIN_PULSEWIDTH_US) / (2 * MOTOR_MAX ) + MOTOR_MIN_PULSEWIDTH_US;
} 
static void set_duty_cycle(uint32_t duty)
{
	ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, signal_to_duty_us(duty)));
}
void init_motor_pwm()
{
	mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, MOTOR_PWM_GPIO); // To drive a BLDC motor, one MCPWM generator is enough

	mcpwm_config_t pwm_config = {
		.frequency = 50, // frequency = 50Hz -> period 20ms. Needed for ESC
		.cmpr_a = 0,	 // duty cycle of PWMxA = 0
		.counter_mode = MCPWM_UP_COUNTER,
		.duty_mode = MCPWM_DUTY_MODE_0,
	};
	mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);

}
void motor_controller(void* arg)
{ 
	telemetry_t* telemetry = (telemetry_t*) arg;
	init_motor_pwm();
	init_vel_interrupt(telemetry);
	telemetry->motor_task = xTaskGetHandle(pcTaskGetName(NULL));
	ESP_LOGI(pcTaskGetName(NULL), "Starting up...");

	/* Arming ESC requires neutral pulse for atleast 2s */
	ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, signal_to_duty_us(0)));
	ESP_LOGI(pcTaskGetName(NULL), "Waiting...");
	vTaskDelay(2500 / portTICK_PERIOD_MS);
	ESP_LOGI(pcTaskGetName(NULL), "Done...");

	/* Resume parent task -> we're ready to run */ 
	xTaskNotify(telemetry->car_task, MOTOR_READY_BIT, eSetBits);

	volatile static double I = 0.0;
	for(;;){ 
		int64_t time = esp_timer_get_time();

		if(xSemaphoreTake(telemetry->mutex, 40 / portTICK_PERIOD_MS) != pdTRUE){ 
				continue;
		} 

		/* stop vehicle if no msg has been received for COMMUNICATION_TIMEOUT */
		if((time - telemetry->last_time_updated) / 1000 > COMMUNICATION_TIMEOUT)
		{ 
			telemetry->desired_speed = 0.0;
		} 


		double duty = K*(telemetry->desired_speed - telemetry->actual_speed) + I;

		set_duty_cycle((uint32_t)duty);

		I += Ibin*(telemetry->desired_speed - telemetry->actual_speed);

		xSemaphoreGive(telemetry->mutex);

		int64_t duration = CONTROLLER_PERIOD - ((esp_timer_get_time()-time) / 1000);
		if(duration > 0)
		{ 
			vTaskDelay(duration / portTICK_PERIOD_MS);
		} 
		else
		{ 
			ESP_LOGI(pcTaskGetName(NULL),"Lagging behind...");
		} 

	} 
} 
static uint32_t moving_average(int64_t curr_time, int64_t last_read)
{ 
	static double vel_array[BUFSIZE] = {0.0};
	static uint8_t pos = 0; /* Position in array */
	static double speed_avg = 0.0;

	double time_diff = (double)(curr_time - last_read); /* time_diff in microseconds */

	/* Scale to seconds */
	time_diff /= 1000000.0;

	double speed_measurement = (double)pi*WHEEL_RADIUS / (double)time_diff;


	/* calculate moving average */
	speed_avg += (speed_measurement - vel_array[pos]) / BUFSIZE;

	vel_array[pos] = speed_measurement;

	++pos;

	pos %= BUFSIZE;

	/* Only able to send uint32_t from xTaskNotifyFromISR -> convert to fixed point first */
	return (uint32_t)(speed_avg * (1 << FRACTIONAL_BITS));
} 
static void speed_update_task(void* arg)
{ 
	telemetry_t* tel = (telemetry_t*) arg;
	uint32_t speed;

	for(;;)
	{ 
		/* if nothing has been received for 200ms -> car at standstill -> reset speed */
		if(xTaskNotifyWait(0x00, ULONG_MAX ,&speed,pdMS_TO_TICKS(200)))
		{ 
			// testa med 50ms
			if(xSemaphoreTake(tel->mutex, 50/portTICK_PERIOD_MS) != pdTRUE)
			{ 
				continue;
			} 
			double tmp = (double)(speed / (double)(1 << FRACTIONAL_BITS));
			tel->actual_speed = tmp;
			xSemaphoreGive(tel->mutex);
		} 
		else
		{
			if(xSemaphoreTake(tel->mutex, 50/portTICK_PERIOD_MS) != pdTRUE)
			{ 
				continue;
			} 
			tel->actual_speed = 0.0;
			xSemaphoreGive(tel->mutex);
		}
	} 
} 

static void hall_interrupt_handle(void* arg)
{ 
	TaskHandle_t speed_update_handle = (TaskHandle_t)arg;
	// behöver reseta last_time_read om diff är för stor
	static int64_t last_time_read = 0;

	int64_t curr_time = esp_timer_get_time();

	uint32_t speed = moving_average(curr_time, last_time_read);

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(speed_update_handle, speed,eSetValueWithOverwrite, &xHigherPriorityTaskWoken);

	last_time_read = curr_time;
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
} 
void init_vel_interrupt(telemetry_t* tel)
{ 
	gpio_config_t config = { 
		.intr_type = GPIO_INTR_NEGEDGE,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = 0,
		.pull_down_en = 0,
		.pin_bit_mask = (1 << HALL_PIN),
	};
	gpio_config(&config);

	TaskHandle_t speed_update_handle = NULL;
	xTaskCreate(speed_update_task, "Speed update",4096, tel, tskIDLE_PRIORITY + 1, &speed_update_handle);

	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	gpio_isr_handler_add(HALL_PIN, hall_interrupt_handle, speed_update_handle);


} 
