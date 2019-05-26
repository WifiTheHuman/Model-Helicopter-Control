// *******************************************************
//
// userInput.c
//
// Checks the buttons on the Tiva and Orbit boards. Carries
// out the appropriate actions based on button pushes.
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
#include "buttons4.h"
#include "userInput.h"
#include "control.h"


//*****************************************************************************
//
// Sets the mean ADC sample value to the number corresponding to the helicopter
// landed altitude.
//
//*****************************************************************************
void setLandedAltitude(uint16_t* landedADCVal, uint16_t meanADCVal) {
    *landedADCVal = meanADCVal;
}


//*****************************************************************************
//
// Polls the buttons and carries out the desired action.
//
//*****************************************************************************
void checkButtons(void) {
    int butState;

    updateButtons();

    butState = checkButton(LEFT);
    // If LEFT button has been pushed, decrement the reference yaw by count 19 (15 deg)
    switch (butState) {
        case PUSHED:
            setReferenceCCW();
            break;
        case RELEASED:
            break;
    }

    butState = checkButton(RIGHT);
    // If RIGHT button has been pushed, increment the reference yaw count by 19 (15 deg)
    switch (butState) {
        case PUSHED:
            setReferenceCW();
            break;
        case RELEASED:
            break;
    }

    butState = checkButton(UP);
    // If UP button has been pushed, increment the reference altitude by 10%
    switch (butState) {
        case PUSHED:
            setReferenceUp();
            break;
        case RELEASED:
            break;
    }

    butState = checkButton(DOWN);
    // If DOWN button has been pushed, decrement the reference altitude by 10%
    switch (butState) {
        case PUSHED:
            setReferenceDown();
            break;
        case RELEASED:
            break;
    }

    butState = checkButton(RESET);
    // If RESET button has been pushed, perform a software reset
    switch (butState) {
        case PUSHED:
            SysCtlReset();
            break;
        case RELEASED:
            break;
    }
}

