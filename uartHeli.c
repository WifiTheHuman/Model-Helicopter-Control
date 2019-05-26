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
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "uartHeli.h"
#include "inc/hw_memmap.h"
#include "display.h"
#include "control.h"
#include "altitude.h"
#include "yaw.h"
#include "utils/ustdlib.h"


//*****************************************************************************
//
// Initialisation for UART - 8 bits, 1 stop bit, no parity.
// Taken from uartDemo.c by Phil Bones.
//
//*****************************************************************************
void initialiseUSB_UART(void) {
    //
    // Enable GPIO port A which is used for UART0 pins.
    //
    SysCtlPeripheralEnable(UART_USB_PERIPH_UART);
    SysCtlPeripheralEnable(UART_USB_PERIPH_GPIO);
    //
    // Select the alternate (UART) function for these pins.
    //
    GPIOPinTypeUART(UART_USB_GPIO_BASE, UART_USB_GPIO_PINS);
    GPIOPinConfigure (GPIO_PA0_U0RX);
    GPIOPinConfigure (GPIO_PA1_U0TX);

    UARTConfigSetExpClk(UART_USB_BASE, SysCtlClockGet(), BAUD_RATE,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART_USB_BASE);
    UARTEnable(UART_USB_BASE);
}


//*****************************************************************************
//
// Sends a given string over UART (Based off of code given in the lectures).
//
//*****************************************************************************
void UARTSendString(char *message) {
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
void UARTSendData(uint16_t landedADCVal, uint16_t meanADCVal, int yawSlotCount) {
    char UARTOut[100];

    // Gets data from the calc functions in display.c then creates a string from the data.
    usnprintf(UARTOut, sizeof(UARTOut),"Mode = %s | PWMMain=%2d | PWMTail=%2d | Yaw=%2d [%2d] | Height=%2d [%2d]\n",
              getMode(),
              getOutputMain(), getOutputTail(),
              calcYawDegrees(yawSlotCount), calcYawDegrees(getReferenceYaw()),
              calcPercentAltitude(landedADCVal, meanADCVal), getReferenceHeight());

    UARTSendString(UARTOut);

}
