#include "gnss.h"
#include "driver/uart.h"
#include <esp_log.h>
#include <string.h>

#include "minmea.h"


#define UART_RX_GPIO 34
#define UART_TX_GPIO 27

static const int RX_BUF_SIZE = 1024;


static char *readLine(uart_port_t uart) {
	static char line[256];
	int size;
	char *ptr = line;
	while(1) {
		size = uart_read_bytes(uart, (unsigned char *)ptr, 1, portMAX_DELAY);
		if (size == 1) {
			if (*ptr == '\n') {
				ptr++;
				*ptr = 0;
				return line;
			}
			ptr++;
		} // End of read a character
	} // End of loop
} // End of readLine
static void listen(telemetry_t* tel)
{ 
	for(;;)
	{ 
		char* line = readLine(UART_NUM_1);
		switch(minmea_sentence_id(line, false)) {
			case MINMEA_SENTENCE_RMC: {
										  struct minmea_sentence_rmc frame;
										  if (minmea_parse_rmc(&frame, line)) {
											  xSemaphoreTake(tel->mutex, portMAX_DELAY);
											  tel->latitude = minmea_tocoord(&frame.latitude);
											  tel->longitude = minmea_tocoord(&frame.longitude);
											  xSemaphoreGive(tel->mutex);
										  }
										  else {
											  ESP_LOGI(pcTaskGetTaskName(NULL), "$xxRMC sentence is not parsed\n");
										  }

										  break;
									  }
			case MINMEA_SENTENCE_VTG: {
										  struct minmea_sentence_vtg frame;
										  if (minmea_parse_vtg(&frame, line)) {
											  xSemaphoreTake(tel->mutex, portMAX_DELAY);
											  tel->actual_speed = minmea_tofloat(&frame.speed_kph);
											  tel->orientation = minmea_tofloat(&frame.true_track_degrees);
											  xSemaphoreGive(tel->mutex);
										  }
										  else {
											  ESP_LOGI("SAD", "$xxVTG sentence is not parsed\n");
										  }
										  break;
									  } 

			default:
									  break;
		} 
	}
} 

static void init_gnss()
{ 
	ESP_LOGI(pcTaskGetTaskName(NULL), "Initializing...");
	uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	};

	uart_param_config(UART_NUM_1, &uart_config);

	uart_set_pin(UART_NUM_1, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	/* uart init: No TX buffer, no uart event queue */
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, RX_BUF_SIZE*2,0,0,NULL,0));
} 
void gnss_task(void* arg)
{ 
	telemetry_t* tel = (telemetry_t*)arg;
	init_gnss();
	listen(tel);
} 
