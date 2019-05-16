
#define HEIGHTSTEP 10
#define YAWSTEP 19
#define MAXHEIGHT 100
#define MINHEIGHT 0
#define MINYAW -180
#define MAXYAW 180
#define ERROR 3
#define ZERO_YAW 0
#define TAKE_OFF_HEIGHT 10

#define DELTA_T 0.01

#define KpMain 2.0
#define KiMain 0.003
#define KdMain 0.05

#define KpTail 1.2
#define KiTail 0.003
#define KdTail 0.05

enum controlStates {LANDING=0, TAKINGOFF, FLYING, LANDED};

void findIndependentYawReference(void);

void setLastRefCrossing(int yawSlotCount);

void setCurrentHeight(int height);

void setCurrentYaw(int yaw);

void setReferenceUp(void);

void setReferenceDown(void);

void setReferenceCW(void);

void setReferenceYaw(int yaw);

void setReferenceHeight(int height);

void setReferenceCCW(void);

void setMode(uint8_t mode);

int getDistanceYaw(void);

int getDistanceHeight(void);

void updateYaw(void);

void updateHeight(void);

void updateControl(void);
