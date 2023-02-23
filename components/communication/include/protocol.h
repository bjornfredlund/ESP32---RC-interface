#ifndef PROTOCOL_H
#define PROTOCOL_H


enum Protocol{ 
	/* error codes */
	ERR_CODE = -1,
	ERR_RADIO_LOST = -1,
	ERR_WIFI_LOST = -1,
	ERR_UNKNOWN_CODE = -1,


	/* Requests */
	COM_GET_STATES = 'a',
	COM_GET_POS,
	COM_GET_VELOCITY,
	COM_SET_SPEED, /* m/s */
	COM_SET_STEERING_ANGLE, /* radians */
	COM_WRITEOUTPUT, /* set steering angle & speed */

	/* Replies */
	ANS_ACK,
	ANS_NACK,
	ANS_GET_STATES,
	ANS_GET_POS,
	ANS_GET_VELOCITY,
	ANS_SET_SPEED, 
	ANS_SET_STEERING_ANGLE,
	ANS_WRITE_OUTPUT,

	COM_END = '\n',
	ANS_END = '\n',
	DELIM 	= '$',

};
#endif
