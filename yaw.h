#ifndef YAW_H_
#define YAW_H_


// Possible states of the yaw sensors
// The yaw signals are on pins 0 and 1. GPIOPinRead returns a bit packed
// byte where the zeroth bit is the state of pin 0, the first bit is the state 
// of pin 1 on the port, etc. Bits two to seven are not read by the quadrature decoder,
// and hence their bit in the returned byte is zero. So PB0 low and PB1 low returns 0x00
// when read, PB0 high and PB1 low returns 0x01 when read etc.
enum yawStates {B_LOW_A_LOW = 0, B_LOW_A_HIGH, B_HIGH_A_LOW, B_HIGH_A_HIGH};

#define TOTAL_SLOTS 448
#define YAW_INCREMENT 1
#define YAW_DECREMENT 1
#define MAX_DEGREES 360
#define HALF_DEGREES 180


//*****************************************************************************
//
// Determines the rotation direction of the disk and increments or decrements
// the slot count appropriately. Sets the slot count to zero if the maximum
// number is exceeded (i.e. 360 degrees rotation performed).
//
//*****************************************************************************
void quadratureDecode(int* yawSlotCount, int currentYawState, int previousYawState);


//*****************************************************************************
//
// Returns the yaw in degrees of the helicopter
//
//*****************************************************************************
int16_t calcYawDegrees(int yawSlotCount);

#endif /*YAW_H_*/
