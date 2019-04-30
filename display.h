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
#define MAX_DEGREES 180


//*****************************************************************************
//
// Displays the mean ADC value (12-bit value) and sample count.
//
//*****************************************************************************
void
displayMeanADC(uint32_t meanADCVal);


//*****************************************************************************
//
// Returns the helicopters altitude as a percentage.
//
//*****************************************************************************
int16_t
calcPercentAltitude(uint16_t landedADCVal, uint16_t meanADCVal);


//*****************************************************************************
//
// Displays the altitude of the helicopter as a percentage of the
// maximum altitude.
//
//*****************************************************************************
void
displayPercentAltitude(uint16_t landedADCVal, uint16_t meanADCVal);


//*****************************************************************************
//
// Displays the yaw angle of the helicopter in degrees.
//
//*****************************************************************************
void
displayYawDegrees(int yawSlotCount);


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
// Displays the yaw angle.
//
//*****************************************************************************
void
updateDisplay(uint8_t displayState,  uint16_t landedADCVal, uint16_t meanADCVal, int yawSlotCount);

#endif /*DISPLAY_H_*/
