#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "control.h"
#include "pwm.h"

static int referencePercentHeight;
static int currentPercentHeight;
static int landedHeight;

static int referenceYawDegrees;
static int currentYawDegrees;



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
    referenceYawDegrees -= YAWSTEP;
    if (referenceYawDegrees < MINYAW) {
        referenceYawDegrees = MAXYAW - (referenceYawDegrees + MAXYAW);
    }
}

void setReferenceCCW(void) {
    referenceYawDegrees += YAWSTEP;
    if (referenceYawDegrees > MAXYAW) {
        referenceYawDegrees = MINYAW + (referenceYawDegrees + MINYAW);
    }
}

void setLandedHeight(uint16_t landed) {
    landedHeight = landed;
}

