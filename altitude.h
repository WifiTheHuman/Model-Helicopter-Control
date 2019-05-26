#ifndef ALTITUDE_H_
#define ALTITUDE_H_

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

// 993 ADC bits corresponds to 0.8V - the difference between 0% and 100% altitude
#define MAX_ALTITUDE_BITS 993

#define PERCENT_CONVERSION 100


//*****************************************************************************
//
// Returns the helicopters altitude as a percentage.
//
//*****************************************************************************
int16_t calcPercentAltitude(uint16_t landedADCVal, uint16_t meanADCVal);

#endif /*ALTITUDE_H_*/
