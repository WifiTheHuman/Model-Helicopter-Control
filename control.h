
#define HEIGHTSTEP 10
#define YAWSTEP
#define MAXHEIGHT 100
#define MINHEIGHT 0
#define MINYAW -180
#define MAXYAW 180

enum controlStates {LANDING=0, TAKINGOFF, FLYING, LANDED};


void setCurrentHeight(int height);

void setCurrentYaw(int yaw);

void setReferenceUp(void);

void setReferenceDown(void);

void setReferenceCW(void);

void setReferenceCCW(void);

void setLandedHeight(uint16_t landed);

void setMode(uint8_t mode);

void updateControl(void);
