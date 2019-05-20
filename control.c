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

static int referencePercentHeight;
static int currentPercentHeight;

static uint8_t currentMode;

static int heightError;
static int yawError;

static int closestRef = 0;

static double yawErrorPrevious;
static double heightErrorPrevious;
static double heightErrorIntegrated;
static double yawErrorIntegrated;
static double heightErrorDerivative;
static double yawErrorDerivative;

static int16_t outputMain;
static int16_t outputTail;

static int referenceYaw;
static int currentYaw;
static volatile int lastRefCrossing;
static int yawFind = 15;

void findIndependentYawReference(void) {

    // Begin flying heli, rotate CCW slowly
    setReferenceHeight(TAKE_OFF_HEIGHT);
    setReferenceYaw(TOTAL_SLOTS);

    // If the reference has been crossed (while the independent reference isn't found)
    if (lastRefCrossing != 0) {
        resetYawSlots();
        setReferenceYaw(ZERO_YAW);
        lastRefCrossing = ZERO_YAW;
        setMode(FLYING);
    }
}


void setLastRefCrossing(int yawSlotCount) {
    lastRefCrossing = yawSlotCount;
}


void setCurrentHeight(int height) {
    currentPercentHeight = height;
}


void setHeightManualLanding(int height) {
    referencePercentHeight -= height;
    if (referencePercentHeight < 0) {
        referencePercentHeight = 0;
    }
}


void setCurrentYaw(int yaw) {
    currentYaw = yaw;
}


void setReferenceUp(void) {
    if (currentMode == FLYING) {
        referencePercentHeight += HEIGHTSTEP;
        if (referencePercentHeight > MAXHEIGHT) {
            referencePercentHeight = MAXHEIGHT;
        }
    }
}


void setReferenceDown(void) {
    if (currentMode == FLYING) {
        referencePercentHeight -= HEIGHTSTEP;
        if (referencePercentHeight < MINHEIGHT) {
            referencePercentHeight = MINHEIGHT;
        }
    }
}

void setReferenceCW(void) {
    if (currentMode == FLYING) {
        referenceYaw += YAWSTEP;
    }
}


void setReferenceYaw(int yaw) {
    referenceYaw = yaw;
}

void setReferenceHeight(int height) {
    referencePercentHeight = height;
}


void setReferenceCCW(void) {
    if (currentMode == FLYING) {
        referenceYaw -= YAWSTEP;
    }
}

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

void setMode(uint8_t mode) {
    currentMode = mode;
}


int getDistanceYaw(void) {
    return yawError;
}


int getDistanceHeight(void) {
    return heightError;
}

int getReferenceHeight(void) {
    return referencePercentHeight;
}

int getReferenceYaw(void) {
    return referenceYaw;
}

int getClosestRef(void) {
    if ((currentYaw - lastRefCrossing) < ((lastRefCrossing + TOTAL_SLOTS) - currentYaw)) {
        return lastRefCrossing;
    } else {
        return lastRefCrossing + TOTAL_SLOTS;
    }
}


void updateYaw(void) {
    yawError = referenceYaw - currentYaw; // yaw error signal
    yawErrorIntegrated += yawError * DELTA_T; // integral of yaw error signal
    yawErrorDerivative = (yawError - yawErrorPrevious) / DELTA_T; // derivative of error signal

    yawErrorPrevious = yawError;

    outputTail = (KpTail * yawError) + (KiTail * yawErrorIntegrated) + (KdTail * yawErrorDerivative);

    if (outputTail > 98) {
        outputTail = 98;
    }

    if (outputTail < 2) {
        outputTail = 2;
    }

    setTailPWM(PWM_TAIL_START_RATE_HZ, outputTail);
}


bool canChangeMode(void) {
    if (currentMode == LANDED || currentMode == FLYING) {
        return 1;
    } else {
        return 0;
    }
}


void updateHeight(void) {
    heightError = referencePercentHeight - currentPercentHeight; // height error signal
    heightErrorIntegrated += heightError * DELTA_T; // height integral of error signal
    heightErrorDerivative = (heightError - heightErrorPrevious) / DELTA_T; // derivative of error signal

    heightErrorPrevious = heightError;

    outputMain = (KpMain * heightError) + (KiMain * heightErrorIntegrated) + (KdMain * heightErrorDerivative);

    if (outputMain > 98) {
        outputMain = 98;
    }

    if (outputMain < 2) {
        outputMain = 2;
    }

    setMainPWM(PWM_MAIN_START_RATE_HZ, outputMain);
}

int16_t getOutputMain(void) {
    return outputMain;
}


int16_t getOutputTail(void) {
    return outputTail;
}


void updateControl(void) {
    switch (currentMode) {
        case (LANDING):
                closestRef = getClosestRef();
                setReferenceYaw(closestRef);
                updateYaw();
                if (closestRef < currentYaw + 5 && closestRef > currentYaw - 5) {
                    setHeightManualLanding(1);
                }
                if (currentPercentHeight == 0) {
                    setMode(LANDED);
                }
                updateHeight();
                break;
        case (TAKINGOFF):
                findIndependentYawReference();
                updateYaw();
                updateHeight();
                break;
        case (FLYING):
                updateYaw();
                updateHeight();
                break;
        case (LANDED):
                lastRefCrossing = 0;
                setMainPWM(PWM_MAIN_START_RATE_HZ, PWM_OFF);
                setTailPWM(PWM_TAIL_START_RATE_HZ, PWM_OFF);
                break;
    }
}
