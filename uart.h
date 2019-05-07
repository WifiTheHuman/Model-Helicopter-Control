/*
 * uart.h
 *
 *  Created on: May 7, 2019
 *      Author: joscraw
 */
#define BAUD_RATE 9600


void
initUART(void);

void
UARTSendString(char *message);
