#include "communication.h"
#include <esp_log.h>
#include "include/communication.h"
#include "include/mirf.h"
#include "mirf.h"
#include "protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <sys/types.h>
#include "telemetry.h"
#include "string.h"

#define PASS(input) (input ? 0 : input) 
#define PAYLOAD_SIZE_MAX 32

static const char* TAG = "Receiver";

void receiver_task(void* arg)
{ 
	telemetry_t* tel = (telemetry_t*)arg;
	tel->gps_task = xTaskGetHandle(pcTaskGetName(NULL));

	NRF24_t receiver;
	init_radio(&receiver);


	listen(&receiver,tel);

} 
int should_reply(unsigned char* buf)
{ 
	enum Protocol p = buf[0];

	int loops;
	switch (p) {
		case COM_GET_STATES:
			loops = 2;
			break;
		case COM_WRITEOUTPUT:
			loops = -1;
			break;
		default:
			loops = -1;
			break;
	}
	return loops;
} 
void listen(NRF24_t* receiver, telemetry_t* tel)
{ 
	ESP_LOGI(TAG, "Listening to radio");
	unsigned char recvbuf[BUFSIZE];
	unsigned char sendbuf[BUFSIZE];

	// Clear RX FiFo
	while(Nrf24_dataReady(receiver)) {
		Nrf24_getData(receiver,(u_int8_t*)recvbuf);
	}

	for(;;)
	{ 
		if(Nrf24_dataReady(receiver))
		{ 
			bzero(recvbuf,strlen((char*)recvbuf));
			bzero(sendbuf,strlen((char*)sendbuf));
			Nrf24_getData(receiver,(u_int8_t*)recvbuf);
			ESP_LOGI(TAG, "Got: %s",recvbuf);
			int payload_size = should_reply(recvbuf);
			if(handle_receive(recvbuf,sendbuf, tel) < 0)
			{ 

			} 
			if(payload_size < 0)
			{ 
				ESP_LOGI(TAG,"Continuing");
				continue;
			} 
			if(transmitt(receiver, sendbuf,payload_size) < 0)
			{ 
				ESP_LOGW(TAG, "transmitt error");
			} 
		} 
		vTaskDelay(1);
	} 
} 
void init_radio(NRF24_t *t)
{ 
	Nrf24_init(t);
	Nrf24_config(t, CHANNEL, (uint8_t)32);

	//Set own address using 5 characters
	esp_err_t ret = Nrf24_setRADDR(t, (uint8_t *)"FGHIJ");
	if (ret != ESP_OK) {
		ESP_LOGE(pcTaskGetName(0), "nrf24l01 not installed");
		while(1) { vTaskDelay(1); }
	}

	//Set the receiver address using 5 characters
	ret = Nrf24_setTADDR(t, (uint8_t *)"ABCDE");
	if (ret != ESP_OK) {
		ESP_LOGE(pcTaskGetName(0), "nrf24l01 not installed");
		while(1) { vTaskDelay(1); }
	}
	/* set faster data rate */
	Nrf24_SetSpeedDataRates(t,1);

} 
int get_latitude(states_u* s)
{


	return -1;
}
int receive_radio(NRF24_t* t,char* sendbuf)
{ 
	int waited = 0;
	while(!Nrf24_dataReady(t) && waited < MAX_RECIEVE_TRIES)
	{ 
		++waited;
		vTaskDelay(1/portTICK_PERIOD_MS);
	} 
	if(waited == MAX_RECIEVE_TRIES)
	{ 
		ESP_LOGW(TAG, "receive timeout");
		return -1;
	} 

	Nrf24_getData(t, (u_int8_t*)sendbuf);
	return 0;
} 
int send(NRF24_t* t, unsigned char* buf)
{ 
	Nrf24_send(t,buf);

	if(!Nrf24_isSend(t,1000))
	{ 
		ESP_LOGW(TAG, "transmission error");
		return -1;
	} 
	return 0;
} 
int transmitt(NRF24_t* t,unsigned char* msg, int loops)
{ 
	unsigned char temp[PAYLOAD_SIZE_MAX];
	memset(temp, '\0', PAYLOAD_SIZE_MAX);

	ESP_LOGI(TAG, "msg: %s",msg);
	int l = strlen((char*)msg); // how long is the message to be sent 
	int half = l / 2;  			// if the division > 1 -> message larger than 32 bytes 

	/* Message smaller than 32 bytes -> send */
	if(half <= 1)
	{ 
		strcpy((char*)temp, (char*)msg);
		if (send(t,temp) < 0) return -1;

	} 
	/* else, split it up, assumption is that no message is larger than 64 bytes -> split into half */
	else
	{ 
		strncpy((char*)temp, (char*)msg, half);
		if(send(t,temp) < 0 ) return -1;
		memset(temp, '\0', PAYLOAD_SIZE_MAX);
		strncpy((char*)temp, (char*)msg + half, l - half);
		if(send(t,temp) < 0 ) return -1;

	} 

	return 0;
} 
void add_code(unsigned char* buf, enum Protocol p)
{ 
	sprintf((char*)buf + strlen((char*)buf), "%c", p);
} 
void append_double(unsigned char* buf, double num)
{ 
	sprintf((char*)buf + strlen((char*)buf), "%f", num);

} 
int get_states(unsigned char* buf, telemetry_t* tel) 
{ 
	states_u s;
	static double example = 100.44;
	s.states_t.longitude = example + 31.2;
	s.states_t.latitude = example-90.4;
	s.states_t.orientation = 30.31;
	example -= 5.6754;

	xSemaphoreTake(tel->mutex, portMAX_DELAY);
	s.states_t.velocity = tel->actual_speed;
	xSemaphoreGive(tel->mutex);

	for(int i = 0; i< sizeof(s.states_a)/sizeof(s.states_a[0]); ++i)
	{ 
		append_double(buf,s.states_a[i]);
		if(i != 3)
		{ 
			add_code(buf, DELIM);

		} 
	} 
	add_code(buf, ANS_END);
	//ESP_LOGI(TAG, "Returned longitude %lf\nLatitude %lf\nspeed %lf",s.states_t.longitude,s.states_t.latitude,s.states_t.velocity);

	return 0;
} 
int write_output(unsigned char* recvbuf, telemetry_t* tel)
{ 
	/* remove first char */
	memmove(recvbuf, recvbuf+1,strlen((char*)recvbuf+1)+1);
	/* remove last char */
	recvbuf[strlen((char*)recvbuf)-1] = '\0';

	double angle;
	double velocity;
	char dummy;

	sscanf((char*)recvbuf,"%lf" "%c" "%lf",&angle, &dummy, &velocity);

	/* update desired states */
	xSemaphoreTake(tel->mutex, portMAX_DELAY);
	tel->steering_angle = angle;
	tel->desired_speed = velocity;
	xSemaphoreGive(tel->mutex);

	/* notify car task */
	xTaskNotify(tel->car_task, UPDATE_STEERING_BIT, eSetBits);

	return 0;
} 
int handle_receive(unsigned char* recvbuf,unsigned char* sendbuf, telemetry_t* tel)
{ 
	enum Protocol commmand = recvbuf[0];
	switch (commmand) {
		case COM_GET_STATES:
			//check_end(connfd);
			add_code(sendbuf,ANS_GET_STATES);
			return get_states(sendbuf, tel);
		case COM_WRITEOUTPUT:
			add_code(sendbuf,ANS_WRITE_OUTPUT);
			sprintf((char*)sendbuf + strlen((char*)sendbuf), "%c", ANS_END);
			return write_output(recvbuf, tel);
		default:
			return ERR_UNKNOWN_CODE;
	}

	return 0;
} 
