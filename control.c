#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "control.h"
#include "pwm.h"

static int referencePercentHeight;
static int currentPercentHeight;

static uint8_t currentMode;

static int distanceHeight;
static int distanceYaw;

static double heightErrorIntegrated;
static double yawErrorIntegrated;

static uint8_t referenceFound;

static int16_t outputMain;
static int16_t outputTail;

static int referenceYaw;
static int currentYaw;
static volatile int lastRefCrossing;

void findIndependentYawReference(void) {
    // Begin flying heli, rotate CCW slowly
    setReferenceHeight(TAKE_OFF_HEIGHT);
    setTailPWM(PWM_TAIL_START_RATE_HZ, PWM_TAIL_START_DUTY);

    // Read PC4 while it is high (while the independent reference isn't found)
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


void setMode(uint8_t mode) {
    currentMode = mode;
}


int getDistanceYaw(void) {
    return distanceYaw;
}


int getDistanceHeight(void) {
    return distanceHeight;
}


void updateYaw(void) {
    distanceYaw = referenceYaw - currentYaw; // yaw error signal
    yawErrorIntegrated += distanceYaw * DELTA_T; // integral of yaw error signal

    outputTail = KpTail * distanceYaw;

    if (outputTail > 98) {
        outputTail = 98;
    }

    if (outputTail < 2) {
        outputTail = 2;
    }

    setTailPWM(PWM_TAIL_START_RATE_HZ, outputTail);
}


void updateHeight(void) {
    distanceHeight = referencePercentHeight - currentPercentHeight; // height error signal
    heightErrorIntegrated += distanceHeight * DELTA_T; // height integral of error signal

    outputMain = (KpMain * distanceHeight) + (KiMain * heightErrorIntegrated);

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
                setReferenceYaw(0);
                setReferenceHeight(0);
                updateYaw();
                updateHeight();
                break;
        case (TAKINGOFF):
                findIndependentYawReference();
                break;
        case (FLYING):
                updateYaw();
                updateHeight();
                break;
        case (LANDED):
                setMainPWM(PWM_MAIN_START_RATE_HZ, PWM_OFF);
                setTailPWM(PWM_TAIL_START_RATE_HZ, PWM_OFF);
                break;
    }
}
