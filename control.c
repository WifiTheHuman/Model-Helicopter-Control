#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "control.h"
#include "pwm.h"

static int referencePercentHeight;
static int currentPercentHeight;

static uint8_t currentMode;

static int distanceHeight;
static int distanceYaw;

static uint8_t referenceFound;

static int referenceYaw;
static int currentYaw;


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
    uint8_t output;

    distanceYaw = referenceYaw - currentYaw;

    output = KpTail * distanceYaw;

    setTailPWM(PWM_TAIL_START_RATE_HZ, output);
}


void updateHeight(void) {
    uint8_t output;

    distanceHeight = referencePercentHeight - currentPercentHeight;

    output = KpMain * distanceHeight;

    setMainPWM(PWM_MAIN_START_RATE_HZ, output);
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
             setReferenceHeight(10);
             if (referenceFound) {
                 setReferenceYaw(0);
                 updateYaw();
             } else {
                 setTailPWM(PWM_TAIL_START_RATE_HZ, PWM_FIND_REF);
             }
             updateHeight();
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
