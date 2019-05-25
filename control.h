#ifndef CONTROL_H_
#define CONTROL_H_

//*****************************************************************************
//
// control.c
//
// Implements PID controllers for the helicopter.
// The controllers take altitude as a percentage, and yaw in slot counts.
// This module uses getter functions so other modules can access control values,
// and setter functions so other modules can alter control values.
// Altitude control values are percentages of the maximum altitude.
// Yaw control values are in terms of "slots" (there are 448 slots in a revolution).
//
// Joshua Hulbert, Josiah Craw, Yifei Ma.
//
// Last Edited 25/05/19
//
//*****************************************************************************

#define HEIGHT_STEP 10                  // 10% altitude increments
#define YAW_STEP 19                     // Corresponds to 15 deg

#define MAX_HEIGHT 100
#define MIN_HEIGHT 0
#define ZERO_YAW 0
#define ZERO_HEIGHT 0
#define TAKE_OFF_HEIGHT 10

#define REFERENCE_FIND_TOLERANCE 10     // Error tolerance for finding the yaw reference
#define REFERENCE_FIND_INCREMENT 15     // Increment for when finding the yaw reference
#define LANDING_YAW_TOLERANCE 5         // Yaw error tolerance when landing
#define LANDING_HEIGHT_DECREMENT 1

#define DELTA_T 0.01                    // Period of control (100Hz)

// Main rotor gains
#define KpMain 1.0
#define KiMain 0.47
#define KdMain 0.25

// Tail rotor gains
#define KpTail 1.0
#define KiTail 0.18
#define KdTail 0.22

// States for the helicopter
enum controlStates {LANDING=0, TAKINGOFF, FLYING, LANDED};


//*****************************************************************************
//
// Finds the independent yaw reference point.
// The helicopter is facing the reference when PC4 is low.
//
//*****************************************************************************
void findIndependentYawReference(void);


//*****************************************************************************
//
// Sets the last reference crossing value to the current yaw slot count when
// the helicopter faces the independent yaw reference signal.
//
//*****************************************************************************
void setLastRefCrossing(int yawSlotCount);


//*****************************************************************************
//
// Sets the current height variable to the helicopters current altitude, in
// percent.
//
//*****************************************************************************
void setCurrentHeight(int height);


//*****************************************************************************
//
// Sets the current yaw variable to the helicopters current yaw, in slots.
//
//*****************************************************************************
void setCurrentYaw(int yaw);


//*****************************************************************************
//
// Decrements the reference height by 1%. Called when the current yaw is within
// 10 slots of the independent reference yaw slot count. Ensures smooth landing
// of the helicopter facing the reference.
//
//*****************************************************************************
void setHeightManualLanding(int height);


//*****************************************************************************
//
// Increments the reference altitude by 10%.
//
//*****************************************************************************
void setReferenceUp(void);


//*****************************************************************************
//
// Decrements the reference altitude by 10%.
//
//*****************************************************************************
void setReferenceDown(void);


//*****************************************************************************
//
// Increments the reference yaw by 15 degrees (19 slots).
//
//*****************************************************************************
void setReferenceCW(void);


//*****************************************************************************
//
// Decrements the reference yaw by 15 degrees (19 slots).
//
//*****************************************************************************
void setReferenceCCW(void);


//*****************************************************************************
//
// Sets the reference yaw to the passed value.
//
//*****************************************************************************
void setReferenceYaw(int yaw);


//*****************************************************************************
//
// Sets the reference altitude to the passed value.
//
//*****************************************************************************
void setReferenceHeight(int height);


//*****************************************************************************
//
// Gets the current mode of the helicopter - for UART.
//
//*****************************************************************************
char* getMode(void);


//*****************************************************************************
//
// Sets the current mode of the helicopter.
//
//*****************************************************************************
void setMode(uint8_t mode);


//*****************************************************************************
//
// Gets the yaw error.
//
//*****************************************************************************
int getErrorYaw(void);


//*****************************************************************************
//
// Gets the altitude error.
//
//*****************************************************************************
int getErrorHeight(void);


//*****************************************************************************
//
// Gets the reference height.
//
//*****************************************************************************
int getReferenceHeight(void);


//*****************************************************************************
//
// Gets the reference yaw.
//
//*****************************************************************************
int getReferenceYaw(void);


//*****************************************************************************
//
// Finds the closest way to get to the last independent yaw reference crossing.
//
//*****************************************************************************
int getClosestRef(void);


//*****************************************************************************
//
// Performs PID control on the yaw by altering the tail rotor duty cycle.
// The tail rotor duty cycle cannot exceed 98 percent, or go below 2 percent.
//
//*****************************************************************************
void updateYaw(void);


//*****************************************************************************
//
// Checks if the helicopter can change modes from a switch movement.
// A switch movement cannot trigger a mode change if the helicopter is taking
// off or landing.
//
//*****************************************************************************
bool canChangeMode(void);


//*****************************************************************************
//
// Resets the PID controller. Called when the helicopter is reaches the landed
// mode after landing.
//
//*****************************************************************************
void controlReset(void);


//*****************************************************************************
//
// Performs PID control on the altitude by altering the main rotor duty cycle.
// The main rotor duty cycle cannot exceed 98 percent, or go below 2 percent.
//
//*****************************************************************************
void updateHeight(void);


//*****************************************************************************
//
// Gets the duty cycle of the main rotor.
//
//*****************************************************************************
int getOutputMain(void);


//*****************************************************************************
//
// Gets the duty cycle of the tail rotor.
//
//*****************************************************************************
int getOutputTail(void);


//*****************************************************************************
//
// Updates the controller based on the helicopters current mode.
//
//*****************************************************************************
void updateControl(void);

#endif /*CONTROL_H_*/
