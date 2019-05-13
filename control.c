#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "control.h"
#include "pwm.h"

static int referencePercentHeight;
static int currentPercentHeight;
static int landedHeight;

static uint8_t currentMode;

static int currentYaw;
static int currretHeight;

static int referenceYaw;
static int currentYaw;


void setCurrentHeight(int height) {
    currentHeight = height;
}


void setCurrentYaw(int yaw) {
    currentYaw = yaw;
}


void setReferenceUp(void) {
    referencePercentHeight += HEIGHTSTEP;
    if (referencePercentHeight > MAXHEIGHT) {
        referencePercentHeight = MAXHEIGHT;
    }
}


void setReferenceDown(void) {
    referencePercentHeight -= HEIGHTSTEP;
    if (referencePercentHeight < MINHEIGHT) {
        referencePercentHeight = MINHEIGHT;
    }
}

void setReferenceCW(void) {
    referenceYaw += YAWSTEP;
}


void setReferenceCCW(void) {
    referenceYaw -= YAWSTEP;
}


void setLandedHeight(uint16_t landed) {
    landedHeight = landed;
}


void setMode(uint8_t mode) {
    currentMode = mode;
}

void updateControl(void) {

}
