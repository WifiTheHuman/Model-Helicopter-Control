#ifndef DISPLAY_H_
#define DISPLAY_H_

// *******************************************************
//
// display.h
//
// Another layer of abstraction for the OLED display.
// Displays helicopter altitude information on Orbit OLED.
// Altitude information displayed as a percentage of maximum
// altidue or as the mean ADC value. Also clears the display.
//
// Joshua Hulbert, Josiah Craw, Yifei Ma
//
// *******************************************************

#include <stdint.h>

enum displayStates {PERCENT=0, MEAN, OFF};

// 993 ADC bits corresponds to 0.8V - the difference between landed and fully up
#define MAX_ALTITUDE_BITS 993

#define OLED_COL_ZERO 0
#define OLED_ROW_ZERO 0
#define OLED_ROW_ONE 1
#define OLED_ROW_TWO 2
#define OLED_ROW_THREE 3
#define OLED_STRING_BITS 17


//*****************************************************************************
//
// Displays the main and tail rotor PWM values.
//
//*****************************************************************************
void displayPWM(void);


//*****************************************************************************
//
// Clears the display.
//
//*****************************************************************************
void clearDisplay(void);


//*****************************************************************************
//
// Updates the display based on the FSM state.
// Displays all data after collecting from the calculator functions.
// Row 1 of orbit LED is the altitude in %.
// Row 2 is the yaw in degrees.
// Row 3 is the main rotor PWM.
// Row 4 is the tail rotor PWM.
//
//*****************************************************************************
void updateDisplay(uint8_t displayState,  uint16_t landedADCVal, uint16_t meanADCVal, int yawSlotCount);

#endif /*DISPLAY_H_*/
