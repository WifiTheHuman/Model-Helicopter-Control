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
#include "buttons4.h"
#include "userInput.h"
#include "control.h"


//*****************************************************************************
//
// Sets the mean ADC sample value to the number corresponding to the helicopter
// landed altitude.
//
//*****************************************************************************
void
setLandedAltitude(uint16_t* landedADCVal, uint16_t meanADCVal)
{
    *landedADCVal = meanADCVal;
}


//*****************************************************************************
//
// Polls the buttons and carries out the desired action.
//
//*****************************************************************************
void
checkButtons(uint16_t* landedADCVal, uint16_t meanADCVal, uint8_t* currentState)
{
    int butState;

    updateButtons();

    butState = checkButton(LEFT);
    // If LEFT button has been pushed, then set the landed altitude to the mean sample value
    switch (butState) {
    case PUSHED:
        setReferenceCCW();
        break;
    case RELEASED:
        break;
    }

    butState = checkButton(RIGHT);
        // If LEFT button has been pushed, then set the landed altitude to the mean sample value
        switch (butState) {
        case PUSHED:
            setReferenceCW();
            break;
        case RELEASED:
            break;
        }

    butState = checkButton(UP);
    // If UP button has been pushed, then cycle the display to the next state
    switch (butState) {
    case PUSHED:
        setReferenceUp();
        break;
    case RELEASED:
        break;
    }

    butState = checkButton(DOWN);
        // If UP button has been pushed, then cycle the display to the next state
        switch (butState) {
        case PUSHED:
            setReferenceDown();
            break;
        case RELEASED:
            break;
        }
}
