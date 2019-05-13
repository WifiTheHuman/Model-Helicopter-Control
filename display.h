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
#define PERCENT_CONVERSION 100
#define MAX_DEGREES 360
#define HALF_DEGREES 180


//*****************************************************************************
//
// Returns the helicopters altitude as a percentage.
//
//*****************************************************************************
int16_t
calcPercentAltitude(uint16_t landedADCVal, uint16_t meanADCVal);


//*****************************************************************************
//
// Returns the yaw in degrees of the helicopter
//
//*****************************************************************************
int16_t
calcYawDegrees(int yawSlotCount);


void
displayPWM(uint32_t frequency, uint32_t duty_cycle);


//*****************************************************************************
//
// Clears the display.
//
//*****************************************************************************
void
clearDisplay(void);


//*****************************************************************************
//
// Updates the display based on the FSM state.
// Displays all data after collecting from the calculator functions
//
//*****************************************************************************
void
updateDisplay(uint8_t displayState,  uint16_t landedADCVal, uint16_t meanADCVal, int yawSlotCount,
              uint16_t tailDuty, uint16_t mainDuty);

#endif /*DISPLAY_H_*/
