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
#include <stdio.h>
#include <stdlib.h>
#include "utils/ustdlib.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "display.h"
#include "yaw.h"
#include "control.h"
#include "altitude.h"


//*****************************************************************************
//
// Displays the main and tail rotor PWM values.
//
//*****************************************************************************
void displayPWM(void) {
    // Buffers for strings displayed
    char mainDutyString[OLED_STRING_BITS];
    char tailDutyString[OLED_STRING_BITS];

    clearDisplay();

    // Copy PWM info into buffers
    usnprintf (mainDutyString, sizeof(mainDutyString), "Main Duty: - %d", getOutputMain());
    usnprintf (tailDutyString, sizeof(tailDutyString), "Tail Duty: - %d", getOutputTail());

    // Show PWM info on display
    OLEDStringDraw(mainDutyString, OLED_COL_ZERO, OLED_ROW_TWO);
    OLEDStringDraw(tailDutyString, OLED_COL_ZERO, OLED_ROW_THREE);
}


//*****************************************************************************
//
// Clears the display.
//
//*****************************************************************************
void clearDisplay(void) {
    char* blankLine = "                ";
    OLEDStringDraw(blankLine, OLED_COL_ZERO, OLED_ROW_ZERO);
    OLEDStringDraw(blankLine, OLED_COL_ZERO, OLED_ROW_ONE);
    OLEDStringDraw(blankLine, OLED_COL_ZERO, OLED_ROW_TWO);
    OLEDStringDraw(blankLine, OLED_COL_ZERO, OLED_ROW_THREE);
}


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
void updateDisplay(uint8_t displayState,  uint16_t landedADCVal, uint16_t meanADCVal, int yawSlotCount) {
    char string[OLED_STRING_BITS];

    // Display the altitude or clear display based on FSM state
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

    // Display the yaw in degrees
    usnprintf(string, sizeof(string), "Yaw = %5d ", calcYawDegrees(yawSlotCount));
    OLEDStringDraw(string, OLED_COL_ZERO, OLED_ROW_ONE);

    // Display the main and tail rotor PWM
    displayPWM();
}
