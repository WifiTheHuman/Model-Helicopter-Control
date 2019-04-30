#ifndef USERINPUT_H_
#define USERINPUT_H_


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

#define DISPLAY_PERCENT 0
#define DISPLAY_OFF 2

//*****************************************************************************
//
// Sets the mean ADC sample value to the number corresponding to the helicopter
// landed altitude.
//
//*****************************************************************************
void
setLandedAltitude(uint16_t* landedADCVal, uint16_t meanADCVal);


//*****************************************************************************
//
// Polls the buttons and carries out the desired action.
//
//*****************************************************************************
void
checkButtons(uint16_t* landedADCVal, uint16_t meanADCVal, uint8_t* currentState);

#endif /*DISPLAY_H_*/
