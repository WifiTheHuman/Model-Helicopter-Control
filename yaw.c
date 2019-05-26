#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "yaw.h"


//*****************************************************************************
//
// Determines the rotation direction of the disk and increments or decrements
// the slot count appropriately. Sets the slot count to zero if the maximum
// number is exceeded (i.e. 360 degrees rotation performed).
// This function is called upon rising and falling edges on PB1, PB2.
//
//*****************************************************************************
void quadratureDecode(int* yawSlotCount, int currentYawState, int previousYawState) {
    // FSM implementation for quadrature decoding.
    // States are changed by the interrupt handler.
    if (currentYawState == B_HIGH_A_LOW) {
        if (previousYawState == B_LOW_A_LOW) {
            *yawSlotCount = *yawSlotCount + YAW_INCREMENT; // Clockwise rotation
        } else {
            *yawSlotCount = *yawSlotCount - YAW_DECREMENT; // Anticlockwise rotation
        }
    } else if (currentYawState == B_HIGH_A_HIGH) {
        if (previousYawState == B_HIGH_A_LOW) {
            *yawSlotCount = *yawSlotCount + YAW_INCREMENT;
        } else {
            *yawSlotCount = *yawSlotCount - YAW_DECREMENT;
        }
    } else if (currentYawState == B_LOW_A_HIGH) {
        if (previousYawState == B_HIGH_A_HIGH) {
            *yawSlotCount = *yawSlotCount + YAW_INCREMENT;
        } else {
            *yawSlotCount = *yawSlotCount - YAW_DECREMENT;
        }
    } else {
        if (previousYawState == B_LOW_A_HIGH) {
            *yawSlotCount = *yawSlotCount + YAW_INCREMENT;
        } else {
            *yawSlotCount = *yawSlotCount - YAW_DECREMENT;
        }
    }
}


//*****************************************************************************
//
// Returns the yaw angle in degrees.
//
//*****************************************************************************
int16_t calcYawDegrees(int yawSlotCount) {
    int16_t yawDegrees;

    // Calculate the yaw in degrees.
    // The yaw is within the range -180 to +180 degrees.
    if (yawSlotCount >= 0) {
        yawDegrees = (((yawSlotCount % TOTAL_SLOTS)*MAX_DEGREES)/TOTAL_SLOTS) - HALF_DEGREES;
    } else {
        yawDegrees = ((((yawSlotCount % TOTAL_SLOTS)*MAX_DEGREES)/TOTAL_SLOTS) + MAX_DEGREES) - HALF_DEGREES;
    }

    return yawDegrees;
}
