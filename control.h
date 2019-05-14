
#define HEIGHTSTEP 10
#define YAWSTEP 19
#define MAXHEIGHT 100
#define MINHEIGHT 0
#define MINYAW -180
#define MAXYAW 180
#define ERROR 3

#define KpMain 0.8
#define KiMain 0.8
#define KdMain 0.8

#define KpTail 0.2
#define KiTail 0.8
#define KdTail 0.8

enum controlStates {LANDING=0, TAKINGOFF, FLYING, LANDED};


void setCurrentHeight(int height);

void setCurrentYaw(int yaw);

void setReferenceUp(void);

void setReferenceDown(void);

void setReferenceCW(void);

void setReferenceYaw(int yaw);

void setReferenceHeight(int height);

void setReferenceCCW(void);

void setMode(uint8_t mode);

void updateYaw(void);

void updateHeight(void);

void updateControl(void);
