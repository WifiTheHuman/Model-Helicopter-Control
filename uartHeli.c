// *******************************************************
//
// uartHeli.c
//
// Gets the current helicopter info and puts the output to
// UART in a human readable format
//
// Joshua Hulbert, Josiah Craw, Yifei Ma
//
// *******************************************************


#include <stdint.h>
#include <stdbool.h>
#include "driverlib/uart.h"
#include "uartHeli.h"
#include "inc/hw_memmap.h"
#include "display.h"
#include "control.h"
#include "utils/ustdlib.h"


//*****************************************************************************
//
// Sends a given string over UART (Based off of code given in the lectures).
//
//*****************************************************************************
void
UARTSendString(char *message)
{
    while (*message)
    {
        UARTCharPut(UART_USB_BASE, *message);
        message++;
    }
}


//*****************************************************************************
//
// Uses current helicopter info to generate then send human readable data
// over UART.
//
//*****************************************************************************
void UARTSendData(uint16_t landedADCVal, uint16_t meanADCVal, int yawSlotCount)
{
    char UARTOut[60];

    // Gets data from the calc functions in display.c then creates a string from the data.
    usnprintf(UARTOut, sizeof(UARTOut),"PWMMain=%2d | PWMTail=%2d | Yaw=%2d [%2d] | Height=%2d [%2d]\n",
              getOutputMain(), getOutputTail(), yawSlotCount, getDistanceYaw(),
              calcPercentAltitude(landedADCVal, meanADCVal), getDistanceHeight());

    UARTSendString(UARTOut);

}
