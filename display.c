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

//*****************************************************************************
//
// Displays the mean ADC value (12-bit value) and sample count.
//
//*****************************************************************************
void
displayMeanADC(uint32_t meanADCVal)
{
    char string[17];

    // Copy the mean ADC value into string
    usnprintf(string, sizeof(string), "Mean ADC = %4d", meanADCVal);

    // Show the mean ADC on the display
    OLEDStringDraw(string, OLED_COL_ZERO, OLED_ROW_ZERO);
}


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
// Displays the altitude of the helicopter as a percentage of the
// maximum altitude.
//
//*****************************************************************************
void
displayPercentAltitude(uint16_t landedADCVal, uint16_t meanADCVal)
{
    char string[17];
    int16_t percentAltitude = calcPercentAltitude(landedADCVal, meanADCVal);

    // Copy the percent altitude into string
    usnprintf(string, sizeof(string), "Altitude = %3d%%", percentAltitude);

    // Show altitude on the display
    OLEDStringDraw(string, OLED_COL_ZERO, OLED_ROW_ZERO);
}


//*****************************************************************************
//
// Displays the yaw angle of the helicopter in degrees.
//
//*****************************************************************************
void
displayYawDegrees(int yawSlotCount)
{
    char string[17];
    int yawDegrees;

    yawDegrees = (((yawSlotCount % TOTAL_SLOTS) - MAX_SLOTS ) * MAX_DEGREES) / MAX_SLOTS; // Calculate yaw in degrees

    // Copy the slot count into string
    usnprintf(string, sizeof(string), "Yaw = %5d ", yawDegrees);

    // Show yaw on the display
    OLEDStringDraw(string, OLED_COL_ZERO, OLED_ROW_THREE);
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
// Displays the yaw angle.
//
//*****************************************************************************
void
updateDisplay(uint8_t displayState,  uint16_t landedADCVal, uint16_t meanADCVal, int yawSlotCount)
{
    switch (displayState) {
        case (PERCENT):
            displayPercentAltitude(landedADCVal, meanADCVal);
            break;
        case (MEAN):
            displayMeanADC(meanADCVal);
            break;
        case (OFF):
            clearDisplay();
            break;
    }

    displayYawDegrees(yawSlotCount);
}
