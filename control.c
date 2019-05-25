//*****************************************************************************
//
// control.c
//
// Implements PID controllers for the helicopter.
// The controllers take altitude as a percentage, and yaw in slot counts.
// This module uses getter functions so other modules can access control values,
// and setter functions so other modules can alter control values.
//
// Joshua Hulbert, Josiah Craw, Yifei Ma.
//
// Last Edited 25/05/19
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "control.h"
#include "pwm.h"
#include "quadrature.h"
#include "uartHeli.h"

static int referencePercentHeight;              // Altitude reference
static int currentPercentHeight;                // Current altitude
static int heightError;                         // Altitude error

static int referenceYaw;                        // Reference yaw
static int currentYaw;                          // Current yaw
static int yawError;                            // Yaw error

static uint8_t currentMode;                     // Current helicopter mode

static int closestRef;                          // For determining the fastest way to the reference

static double heightErrorDerivative;
static double heightErrorPrevious;
static double heightErrorIntegrated;

static double yawErrorDerivative;
static double yawErrorPrevious;
static double yawErrorIntegrated;

static int outputMain;                          // Output main rotor PWM
static int outputTail;                          // Output tail rotor PWM

static volatile int lastRefCrossing;            // Slot count of last crossing of the independent yaw reference
static int yawFind = REFERENCE_FIND_INCREMENT;  // For finding the independent reference


//*****************************************************************************
//
// Finds the independent yaw reference point.
// The helicopter is facing the reference when PC4 is low.
//
//*****************************************************************************
void findIndependentYawReference(void) {

    // Begin flying heli, rotate CCW slowly
    setReferenceHeight(TAKE_OFF_HEIGHT);

    // Begin rotating to find the reference
    setReferenceYaw(REFERENCE_FIND_INCREMENT);

    // Rotate more if the error is small enough
    // Helps with stability
    if (yawError < REFERENCE_FIND_TOLERANCE) {
        yawFind += REFERENCE_FIND_INCREMENT;
    }

    // If the reference has been crossed, set the slot count to zero and begin
    // flying while facing the reference
    if (lastRefCrossing != ZERO_YAW) {
        resetYawSlots();
        setReferenceYaw(ZERO_YAW);
        lastRefCrossing = ZERO_YAW;
        setMode(FLYING);
    }
}


//*****************************************************************************
//
// Sets the last reference crossing value to the current yaw slot count when
// the helicopter faces the independent yaw reference signal.
//
//*****************************************************************************
void setLastRefCrossing(int yawSlotCount) {
    lastRefCrossing = yawSlotCount;
}


//*****************************************************************************
//
// Sets the current height variable to the helicopters current altitude, in
// percent.
//
//*****************************************************************************
void setCurrentHeight(int height) {
    currentPercentHeight = height;
}


//*****************************************************************************
//
// Sets the current yaw variable to the helicopters current yaw, in slots.
//
//*****************************************************************************
void setCurrentYaw(int yaw) {
    currentYaw = yaw;
}


//*****************************************************************************
//
// Decrements the reference height by 1%. Called when the current yaw is within
// 10 slots of the independent reference yaw slot count. Ensures smooth landing
// of the helicopter facing the reference.
//
//*****************************************************************************
void setHeightManualLanding(int height) {
    referencePercentHeight -= height;
    if (referencePercentHeight < ZERO_HEIGHT) {
        referencePercentHeight = ZERO_HEIGHT;
    }
}


//*****************************************************************************
//
// Increments the reference altitude by 10%.
//
//*****************************************************************************
void setReferenceUp(void) {
    if (currentMode == FLYING) {
        referencePercentHeight += HEIGHT_STEP;
        if (referencePercentHeight > MAX_HEIGHT) {
            referencePercentHeight = MAX_HEIGHT;
        }
    }
}


//*****************************************************************************
//
// Decrements the reference altitude by 10%.
//
//*****************************************************************************
void setReferenceDown(void) {
    if (currentMode == FLYING) {
        referencePercentHeight -= HEIGHT_STEP;
        if (referencePercentHeight < MIN_HEIGHT) {
            referencePercentHeight = MIN_HEIGHT;
        }
    }
}


//*****************************************************************************
//
// Increments the reference yaw by 15 degrees (19 slots).
//
//*****************************************************************************
void setReferenceCW(void) {
    if (currentMode == FLYING) {
        referenceYaw += YAW_STEP;
    }
}


//*****************************************************************************
//
// Decrements the reference yaw by 15 degrees (19 slots).
//
//*****************************************************************************
void setReferenceCCW(void) {
    if (currentMode == FLYING) {
        referenceYaw -= YAW_STEP;
    }
}


//*****************************************************************************
//
// Sets the reference yaw to the passed value.
//
//*****************************************************************************
void setReferenceYaw(int yaw) {
    referenceYaw = yaw;
}


//*****************************************************************************
//
// Sets the reference altitude to the passed value.
//
//*****************************************************************************
void setReferenceHeight(int height) {
    referencePercentHeight = height;
}


//*****************************************************************************
//
// Gets the current mode of the helicopter - for UART.
//
//*****************************************************************************
char* getMode(void) {
    if (currentMode == LANDED) {
        return "Landed";
    } else if (currentMode == TAKINGOFF) {
        return "Taking off";
    } else if (currentMode == FLYING) {
        return "Flying";
    } else {
        return "Landing";
    }
}


//*****************************************************************************
//
// Sets the current mode of the helicopter.
//
//*****************************************************************************
void setMode(uint8_t mode) {
    currentMode = mode;
}


//*****************************************************************************
//
// Gets the yaw error.
//
//*****************************************************************************
int getErrorYaw(void) {
    return yawError;
}


//*****************************************************************************
//
// Gets the altitude error.
//
//*****************************************************************************
int getErrorHeight(void) {
    return heightError;
}


//*****************************************************************************
//
// Gets the reference height.
//
//*****************************************************************************
int getReferenceHeight(void) {
    return referencePercentHeight;
}


//*****************************************************************************
//
// Gets the reference yaw.
//
//*****************************************************************************
int getReferenceYaw(void) {
    return referenceYaw;
}


//*****************************************************************************
//
// Finds the closest way to get to the last independent yaw reference crossing.
//
//*****************************************************************************
int getClosestRef(void) {
    if ((currentYaw - lastRefCrossing) < ((lastRefCrossing + TOTAL_SLOTS) - currentYaw)) {
        return lastRefCrossing;
    } else {
        return lastRefCrossing + TOTAL_SLOTS;
    }
}


//*****************************************************************************
//
// Performs PID control on the yaw by altering the tail rotor duty cycle.
// The tail rotor duty cycle cannot exceed 98 percent, or go below 2 percent.
//
//*****************************************************************************
void updateYaw(void) {
    yawError = referenceYaw - currentYaw; // yaw error signal
    yawErrorIntegrated += yawError * DELTA_T; // integral of yaw error signal
    yawErrorDerivative = (yawError - yawErrorPrevious) / DELTA_T; // derivative of error signal

    yawErrorPrevious = yawError; // Store previous yaw error (for derivative control)

    // Compute the PID control PWM value for the tail rotor
    outputTail = (KpTail * yawError) + (KiTail * yawErrorIntegrated) + (KdTail * yawErrorDerivative);

    // PWM duty cycle can't exceed 98%
    if (outputTail > PWM_DUTY_MAX) {
        outputTail = PWM_DUTY_MAX;
    }

    // PWM duty cycle can't go below 2%
    if (outputTail < PWM_DUTY_MIN) {
        outputTail = PWM_DUTY_MIN;
    }
    setTailPWM(PWM_TAIL_START_RATE_HZ, outputTail);
}


//*****************************************************************************
//
// Checks if the helicopter can change modes from a switch movement.
// A switch movement cannot trigger a mode change if the helicopter is taking
// off or landing.
//
//*****************************************************************************
bool canChangeMode(void) {
    if (currentMode == LANDED || currentMode == FLYING) {
        return true;
    } else {
        return false;
    }
}


//*****************************************************************************
//
// Resets the PID controller. Called when the helicopter is reaches the landed
// mode after landing.
//
//*****************************************************************************
void controlReset(void) {
    resetYawSlots();

    referencePercentHeight = ZERO_HEIGHT;
    currentPercentHeight = ZERO_HEIGHT;
    heightError = ZERO_HEIGHT;
    yawError = ZERO_YAW;
    closestRef = ZERO_YAW;
    yawErrorPrevious = ZERO_YAW;
    heightErrorPrevious = ZERO_HEIGHT;
    heightErrorIntegrated = ZERO_HEIGHT;
    yawErrorIntegrated = ZERO_YAW;
    heightErrorDerivative = ZERO_HEIGHT;
    yawErrorDerivative = ZERO_YAW;
    outputMain = PWM_OFF;
    outputTail = PWM_OFF;
    referenceYaw = ZERO_YAW;
    currentYaw = ZERO_YAW;
    lastRefCrossing = ZERO_YAW;
    yawFind = REFERENCE_FIND_INCREMENT;
}


//*****************************************************************************
//
// Performs PID control on the altitude by altering the main rotor duty cycle.
// The main rotor duty cycle cannot exceed 98 percent, or go below 2 percent.
//
//*****************************************************************************
void updateHeight(void) {
    heightError = referencePercentHeight - currentPercentHeight; // height error signal
    heightErrorIntegrated += heightError * DELTA_T; // height integral of error signal
    heightErrorDerivative = (heightError - heightErrorPrevious) / DELTA_T; // derivative of error signal

    heightErrorPrevious = heightError; // Store previous height error (for derivative control)

    // Compute the PID control PWM value for the main rotor
    outputMain = (KpMain * heightError) + (KiMain * heightErrorIntegrated) + (KdMain * heightErrorDerivative);

    // PWM duty cycle can't exceed 98%
    if (outputMain > PWM_DUTY_MAX) {
        outputMain = PWM_DUTY_MAX;
    }

    // PWM duty cycle can't go below 2%
    if (outputMain < PWM_DUTY_MIN) {
        outputMain = PWM_DUTY_MIN;
    }

    setMainPWM(PWM_MAIN_START_RATE_HZ, outputMain);
}


//*****************************************************************************
//
// Gets the duty cycle of the main rotor.
//
//*****************************************************************************
int getOutputMain(void) {
    return outputMain;
}


//*****************************************************************************
//
// Gets the duty cycle of the tail rotor.
//
//*****************************************************************************
int getOutputTail(void) {
    return outputTail;
}


//*****************************************************************************
//
// Updates the controller based on the helicopters current mode.
//
//*****************************************************************************
void updateControl(void) {
    // Check the helicopters current mode
    switch (currentMode) {
        // If the mode is LANDING
        case (LANDING):
                closestRef = getClosestRef();
                setReferenceYaw(closestRef);
                updateYaw(); // Perform PID control on yaw

                // Decrement the reference height by 1% if the yaw is within +/-5 slots of the reference
                if ((closestRef < (currentYaw + LANDING_YAW_TOLERANCE)) && (closestRef > (currentYaw - LANDING_YAW_TOLERANCE))) {
                    setHeightManualLanding(LANDING_HEIGHT_DECREMENT);
                }

                // If the altitude is zero, reset control and set the mode to landed
                if (currentPercentHeight == ZERO_HEIGHT) {
                    controlReset();
                    setMode(LANDED);
                }
                updateHeight(); // Perform PID control on height
                break;

        // If the mode is taking off
        case (TAKINGOFF):
                findIndependentYawReference();
                updateYaw();
                updateHeight();
                break;

        // If the mode is flying
        case (FLYING):
                updateYaw();
                updateHeight();
                break;

        // If the mode is landed
        case (LANDED):
                lastRefCrossing = ZERO_YAW;
                setMainPWM(PWM_MAIN_START_RATE_HZ, PWM_OFF);
                setTailPWM(PWM_TAIL_START_RATE_HZ, PWM_OFF);
                break;
    }
}
