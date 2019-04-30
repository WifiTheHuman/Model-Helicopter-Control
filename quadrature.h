#ifndef QUADRATURE_H_
#define QUADRATURE_H_


// Possible states of the yaw sensors
enum yawStates {B_LOW_A_LOW = 0, B_LOW_A_HIGH, B_HIGH_A_LOW, B_HIGH_A_HIGH};

#define MAX_SLOTS (2*112)
#define MIN_SLOTS (-2*112)
#define TOTAL_SLOTS 448


//*****************************************************************************
//
// Determines the rotation direction of the disk and increments or decrements
// the slot count appropriately. Sets the slot count to zero if the maximum
// number is exceeded (i.e. 360 degrees rotation performed).
//
//*****************************************************************************
void
quadratureDecode(int* yawSlotCount, int currentYawState, int previousYawState);

#endif /*QUADRATURE_H_*/
