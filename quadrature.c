#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "quadrature.h"


//*****************************************************************************
//
// Determines the rotation direction of the disk and increments or decrements
// the slot count appropriately. Sets the slot count to zero if the maximum
// number is exceeded (i.e. 360 degrees rotation performed).
// This function is scheduled to be called by the interrupt handler when
// quadDecodeFlag is set.
//
//*****************************************************************************
void
quadratureDecode(int* yawSlotCount, int currentYawState, int previousYawState)
{
    // FSM implementation for quadrature decoding.
    // States are changed by the interrupt handler.
    if (currentYawState == B_HIGH_A_LOW) {
        if (previousYawState == B_LOW_A_LOW) {
            *yawSlotCount = *yawSlotCount + 1; // Clockwise rotation
        } else {
            *yawSlotCount = *yawSlotCount - 1; // Anticlockwise rotation
        }
    } else if (currentYawState == B_HIGH_A_HIGH) {
        if (previousYawState == B_HIGH_A_LOW) {
            *yawSlotCount = *yawSlotCount + 1;
        } else {
            *yawSlotCount = *yawSlotCount - 1;
        }
    } else if (currentYawState == B_LOW_A_HIGH) {
        if (previousYawState == B_HIGH_A_HIGH) {
            *yawSlotCount = *yawSlotCount + 1;
        } else {
            *yawSlotCount = *yawSlotCount - 1;
        }
    } else {
        if (previousYawState == B_LOW_A_HIGH) {
            *yawSlotCount = *yawSlotCount + 1;
        } else {
            *yawSlotCount = *yawSlotCount - 1;
        }
    }
}
