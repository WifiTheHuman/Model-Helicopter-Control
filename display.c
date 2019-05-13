// *******************************************************
//
// display.c
//
// Displays helicopter altitude information on Orbit OLED.
// Altitude information displayed as a percentage of maximum
// altidue or as the mean ADC value. Also clears the display.
//
// Joshua Hulbert, Josiah Craw, Yifei Ma
//
// *******************************************************

#include <stdint.h>
#include <stdbool.h>
#include "utils/ustdlib.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "display.h"
#include "quadrature.h"
#include <stdio.h>
#include <stdlib.h>


//*****************************************************************************
//
// Returns the helicopters altitude as a percentage.
//
//*****************************************************************************
int16_t
calcPercentAltitude(uint16_t landedADCVal, uint16_t meanADCVal)
{
    // Calculate the number of ADC bits above the zero altitude point
    int16_t ADCAltitudeBits = landedADCVal - meanADCVal;

    // Calculate the altitude as a percentage of maximum altitude
    int16_t percentAltitude = (ADCAltitudeBits * PERCENT_CONVERSION) / MAX_ALTITUDE_BITS;
    return percentAltitude;
}


//*****************************************************************************
//
// Returns the yaw angle in degrees
//
//*****************************************************************************
int16_t
calcYawDegrees(int yawSlotCount)
{
    uint16_t yawDegrees;
    if (yawSlotCount >= 0) {
        yawDegrees = (((yawSlotCount % TOTAL_SLOTS)*MAX_DEGREES)/TOTAL_SLOTS) - HALF_DEGREES;
    } else {
        yawDegrees = ((((yawSlotCount % TOTAL_SLOTS)*MAX_DEGREES)/TOTAL_SLOTS) + MAX_DEGREES) - HALF_DEGREES;
    }

    // Copy the slot count into string
    return yawDegrees;
}



void
displayPWM(uint32_t frequency, uint32_t duty_cycle)
{
    // Buffers for strings displayed
    char freqString[17];
    char dutyString[17];


    // Draw blank lines (clear display)
    OLEDStringDraw ("                ", 0, 0);
    OLEDStringDraw ("                ", 1, 1);


    // Copy PWM info into buffers
    usnprintf (freqString, sizeof(freqString), "FREQ: - %d   Hz", frequency);
    usnprintf (dutyString, sizeof(dutyString), "DUTY: - %d   %", duty_cycle);

    // Show PWM info on display
    OLEDStringDraw(freqString, OLED_COL_ZERO, OLED_ROW_ONE);
    OLEDStringDraw(dutyString, OLED_COL_ZERO, OLED_ROW_TWO);
}


//*****************************************************************************
//
// Clears the display.
//
//*****************************************************************************
void
clearDisplay(void)
{
    char* blankLine = "                ";
    OLEDStringDraw(blankLine, OLED_COL_ZERO, OLED_ROW_ZERO);
    OLEDStringDraw(blankLine, OLED_COL_ZERO, OLED_ROW_ONE);
    OLEDStringDraw(blankLine, OLED_COL_ZERO, OLED_ROW_TWO);
    OLEDStringDraw(blankLine, OLED_COL_ZERO, OLED_ROW_THREE);
}


//*****************************************************************************
//
// Updates the display based on the FSM state.
// Displays all data after collecting from the calculator functions
//
//*****************************************************************************
void
updateDisplay(uint8_t displayState,  uint16_t landedADCVal, uint16_t meanADCVal, int yawSlotCount,
              uint16_t tailDuty, uint16_t mainDuty)
{
    char string[17];
    switch (displayState) {
        case (PERCENT):
            usnprintf(string, sizeof(string), "Altitude = %3d%%", calcPercentAltitude(landedADCVal, meanADCVal));
            OLEDStringDraw(string, OLED_COL_ZERO, OLED_ROW_ZERO);
            break;
        case (MEAN):
            usnprintf(string, sizeof(string), "Mean ADC = %4d", meanADCVal);
            OLEDStringDraw(string, OLED_COL_ZERO, OLED_ROW_ZERO);
            break;
        case (OFF):
            clearDisplay();
            break;
    }
    usnprintf(string, sizeof(string), "Yaw = %5d ", calcYawDegrees(yawSlotCount));
    OLEDStringDraw(string, OLED_COL_ZERO, OLED_ROW_THREE);
}
