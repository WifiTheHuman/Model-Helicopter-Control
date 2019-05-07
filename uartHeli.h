// *******************************************************
//
// uartHeli.h
//
// Header for the uartHeli.c file, also contains UART
// congfig data (baud rate).
//
// Joshua Hulbert, Josiah Craw, Yifei Ma
//
// *******************************************************


#define BAUD_RATE 9600

void
UARTSendString(char *message);

void
UARTSendData(uint16_t landedADCVal, uint16_t meanADCVal, int yawSlotCount);
