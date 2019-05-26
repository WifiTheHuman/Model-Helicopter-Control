// *******************************************************
//
// altitude.c
//
// Functions and data for calculating the altitude of the
// helicopter.
//
// Joshua Hulbert, Josiah Craw, Yifei Ma
//
// *******************************************************

#include <stdint.h>
#include "altitude.h"


//*****************************************************************************
//
// Calculates and returns the helicopters altitude as a percentage.
//
//*****************************************************************************
int16_t calcPercentAltitude(uint16_t landedADCVal, uint16_t meanADCVal) {
    // Calculate the number of ADC bits above the zero altitude point
    int16_t ADCAltitudeBits = landedADCVal - meanADCVal;

    // Calculate the altitude as a percentage of maximum altitude
    int16_t percentAltitude = (ADCAltitudeBits * PERCENT_CONVERSION) / MAX_ALTITUDE_BITS;
    return percentAltitude;
}
