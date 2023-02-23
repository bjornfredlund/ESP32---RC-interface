#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "protocol.h"
#include "telemetry.h"
#include <freeRTOS/freertos.h>
#include <freeRTOS/task.h>
#include "mirf.h"

#define WIFI_CONNECTED_BIT 		(1 << 0)
#define WIFI_DISCONNECTED_BIT 	(1 << 1)
#define RADIO_CONNECTED_BIT 	(1 << 2)
#define RADIO_DISCONNECTED_BIT 	(1 << 3)

#define BUFSIZE 1024 
#define CHANNEL 90
#define MAX_RECIEVE_TRIES 500

typedef union states_u{ 

	struct states_t { 
			double longitude, latitude, velocity, orientation;
	} states_t;
	double states_a[4];
} states_u;

#ifdef __cplusplus
extern "C" {
	void receiver_task(void*);
	 }
#endif
	typedef struct Car Car;

	void init_radio(NRF24_t* t);
	void add_code(unsigned char*, enum Protocol);
	void append_double(unsigned char*, double);
	int get_latitude(states_u*);
	int get_states(unsigned char*,telemetry_t*);
	int get_longitude(states_u*);
	int handle_receive(unsigned char*, unsigned char*, telemetry_t*);
	int get_pos(states_u*);
	void listen(NRF24_t*, telemetry_t*);
	int transmitt(NRF24_t*, unsigned char*,int);
	int receive_radio(NRF24_t*, char*);
	int reply_needed(unsigned char*);
#endif
