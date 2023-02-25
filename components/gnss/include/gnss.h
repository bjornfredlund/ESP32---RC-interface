#ifndef GNSS_H
#define GNSS_H

#include "telemetry.h"
#include "minmea.h"

#if __cplusplus
extern "C" {
#else
	static void init_gnss();
	static void listen(telemetry_t*);
	static char* readLine(uart_port_t);
#endif
	void gnss_task(void*);

#if __cplusplus
}
#endif


#endif
