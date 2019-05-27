//*****************************************************************************
//
// main.c - Main program for the ENCE361 2019 Helicopter Project.
// Implements PID control on the altitude and yaw of an RC helicopter.
// Uses PWM on the main and tail rotors. Uses ADC to sense the altitude of the
// helicopter. Uses quadrature to track the yaw of the helicopter.
//
// Joshua Hulbert, Josiah Craw, Yifei Ma.
//
// Adapted from ADCDemo1.c by Phil Bones.
//
// Last Edited 26/05/19
//
//*****************************************************************************

// Standard Libs
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// TivaWare Libs
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "utils/ustdlib.h"

// Provided OrbitBoosterBoard Lib
#include "OrbitOLED/OrbitOLEDInterface.h"

// Libs created as part of the project
#include "buttons4.h"
#include "circBufT.h"
#include "display.h"
#include "userInput.h"
#include "yaw.h"
#include "uartHeli.h"
#include "pwm.h"
#include "control.h"
#include "altitude.h"

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUFFER_FILL_DELAY 6
#define BUF_SIZE 40

#define SAMPLE_RATE_HZ 100

#define UART_SEND_PERIOD 25
#define DISPLAY_PERIOD 25

#define CHANNEL_A GPIO_PIN_0
#define CHANNEL_B GPIO_PIN_1

#define FLAG_CLEAR 0
#define FLAG_SET 1
#define FLAG_COUNT_ZERO 0

#define ADC_SEQUENCE_THREE 3
#define ADC_CHANNEL_ZERO 0

//*****************************************************************************
// Global variables
//*****************************************************************************
static circBuf_t g_inBuffer;		 // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulDispCnt;	     // Counter for display interrupts
static uint32_t g_ulUARTCnt;         // Counter to trigger a UART send
static uint8_t displayFlag;          // Flag for refreshing display
static uint8_t controlUpdateFlag;    // Flag for refreshing control system
static uint8_t UARTFlag;             // Flag for UART sending
static uint8_t buttonFlag;           // Flag for button polling
static int currentYawState;          // The current state of the yaw sensors
static int previousYawState;         // The previous state of the yaw sensors
int yawSlotCount = 0;  // Init the yaw slot to the zero value


//*****************************************************************************
//
// The interrupt handler for the for SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void) {
    // Initiate a conversion
    ADCProcessorTrigger(ADC0_BASE, ADC_SEQUENCE_THREE);

    g_ulDispCnt++;
    g_ulUARTCnt++;

    // Set the flag for button polling
    buttonFlag = FLAG_SET;

    // Set the flag for the PID controller
    controlUpdateFlag = FLAG_SET;

    // Set the flag for display refreshing every 25 SysTick interrupts (250ms)
    if (g_ulDispCnt >= DISPLAY_PERIOD) {
        g_ulDispCnt = FLAG_COUNT_ZERO;
        displayFlag = FLAG_SET;
    }

    // Set the flag to send UART data every 25 SysTick interrupts (250ms)
    if (g_ulUARTCnt >= UART_SEND_PERIOD) {
        g_ulUARTCnt = FLAG_COUNT_ZERO;
        UARTFlag = FLAG_SET;
    }
}


//*****************************************************************************
//
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//
//*****************************************************************************
void
ADCIntHandler(void) {
	uint32_t ulValue;
	
	// Get the single sample from ADC0.  ADC_BASE is defined in
	// inc/hw_memmap.h
	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE_THREE, &ulValue);

	// Place it in the circular buffer (advancing write index)
	writeCircBuf(&g_inBuffer, ulValue);

	// Clean up, clearing the interrupt
	ADCIntClear(ADC0_BASE, ADC_SEQUENCE_THREE);
}


//*****************************************************************************
//
// The interrupt handler for the quadrature decoding module.
// The interrupt is triggered by pin changes (edges) on PB0 and PB1.
//
//*****************************************************************************
void
quadratureIntHandler(void) {
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1); // Clear the interrupt

    previousYawState = currentYawState; // Save the previous state

    // Read the values on PB0 and PB1
    // The yaw signals are on pins 0 and 1. GPIOPinRead returns a bit packed
    // byte where the zeroth bit is the state of pin 0, the first bit is the state 
    // of pin 1 on the port, etc. Bits two to seven are not read by the quadrature decoder,
    // and hence their bit in the returned byte is zero. So PB0 low and PB1 low returns 0x00
    // when read, PB0 high and PB1 low returns 0x01 when read etc. The returned value will 
    // therefore match the desired state as given in the yawStates enum (yaw.h)
    currentYawState = (int) GPIOPinRead(GPIO_PORTB_BASE, CHANNEL_A | CHANNEL_B);

    // Determine the direction of rotation. Increment or decrement yawSlotCount appropriately.
    quadratureDecode(&yawSlotCount, currentYawState, previousYawState);

    setCurrentYaw(yawSlotCount);
}


//*****************************************************************************
//
// The interrupt handler for the independent yaw reference signal.
// The interrupt is triggered by falling edges on PC4.
//
//*****************************************************************************
void yawRefSignalIntHandler(void) {
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_4); // Clear the interrupt
    
    // Set the last reference crossing value to the current yaw slot count
    setLastRefCrossing(yawSlotCount);
}


//*****************************************************************************
//
// The interrupt handler for the slider switch.
// The interrupt is triggered by edges on PA7.
//
//*****************************************************************************
void switchIntHandler(void) {
     GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_7); // Clear the interrupt

     // Check the helicopter isn't currently landing or taking off
     if (canChangeMode()) {
         // Check if the edge was rising or falling, set the helicopter mode.
         if (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7)) {
             setMode(TAKINGOFF);
         } else {
             setMode(LANDING);
         }
     }
}


//*****************************************************************************
//
// Initialisation for the clock (incl. SysTick).
//
//*****************************************************************************
void
initClock(void) {
    // Set the clock rate to 20 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);

    // Set the PWM clock rate (using the prescaler)
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);

    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);

    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);

    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}


//*****************************************************************************
//
// Initialisation for the ADC.
//
//*****************************************************************************
void 
initADC(void) {

    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    
    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCE_THREE, ADC_TRIGGER_PROCESSOR, ADC_CHANNEL_ZERO);
  

    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE_THREE, ADC_CHANNEL_ZERO, ADC_CTL_CH9 | ADC_CTL_IE |
                             ADC_CTL_END);    
                             
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCE_THREE);
  
    // Register the interrupt handler
    ADCIntRegister(ADC0_BASE, ADC_SEQUENCE_THREE, ADCIntHandler);
  
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, ADC_SEQUENCE_THREE);
}


//*****************************************************************************
//
// Initialisation for the quadrature peripherals (GPIOs PB0 and PB1).
//
//*****************************************************************************
void
quadratureInitialise(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enable Port B

    // Set PB0 and PB1 as inputs
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, CHANNEL_A | CHANNEL_B);

    // Enable interrupts on PB0 and PB1
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);

    // Set interrupts on PB0 and PB1 as pin change interrupts
    GPIOIntTypeSet(GPIO_PORTB_BASE, CHANNEL_A | CHANNEL_B, GPIO_BOTH_EDGES);

    // Register the interrupt handler
    GPIOIntRegister(GPIO_PORTB_BASE, quadratureIntHandler);

    // Read the values on PB0 and PB1
    currentYawState = GPIOPinRead(GPIO_PORTB_BASE, CHANNEL_A | CHANNEL_B);
}


//*****************************************************************************
//
// Initialisation for the independent yaw reference.
// A low value on PC4 indicates the reference is found.
//
//*****************************************************************************
void initYawReferenceSignal(void) {
    // Enable Port C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    // Set pin 4 as an input
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4);

    // Enable interrupts on PC4
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_INT_PIN_4);

    // Set interrupts on PC4 as a low level interrupt
    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);

    // Register the interrupt handler
    GPIOIntRegister(GPIO_PORTC_BASE, yawRefSignalIntHandler);
}


//*****************************************************************************
//
// Initialisation for SW1.
// SW1 switches the mode of the helicopter.
//
//*****************************************************************************
void initSliderSwitch(void) {
    // Enable Port A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Set PA7 as an input
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_7, GPIO_DIR_MODE_IN);

    // Enable interrupts on PA7
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_7);

    // Set interrupts on PA7 as pin change interrupts
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_BOTH_EDGES);

    // Register the interrupt handler
    GPIOIntRegister(GPIO_PORTA_BASE, switchIntHandler);
}


//*****************************************************************************
//
// Sets the yaw slot count to zero.
//
//*****************************************************************************
void resetYawSlots(void) {
    yawSlotCount = 0;
}


int main(void) {
    uint16_t landedADCVal;
    uint16_t meanADCVal;
    uint8_t currentDisplayState = PERCENT;

    // Initialise peripherals and variables
	initClock();
	initADC();
	initButtons();
	OLEDInitialise();
	initCircBuf(&g_inBuffer, BUF_SIZE);
	quadratureInitialise();
	initialisePWM();
	initialiseUSB_UART();
	initYawReferenceSignal();
	initSliderSwitch();

    // Initialisation is complete, so turn on the output.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);

    // Enable interrupts to the processor.
    IntMasterEnable();

    // Wait to ensure buffer fills
    SysCtlDelay(SysCtlClockGet() / BUFFER_FILL_DELAY);

    // Compute the mean ADC value, set the zero altitude value
    meanADCVal = calcMeanOfContents(&g_inBuffer, BUF_SIZE);
    landedADCVal = meanADCVal;

    // Update the display
    updateDisplay(currentDisplayState, landedADCVal, meanADCVal, yawSlotCount);

    // The helicopter starts in the LANDED mode
    setMode(LANDED);

	while (1)
	{
	    // Compute the mean ADC value.
	    // Disable interrupts for this part to avoid a race condition.
	    IntMasterDisable();
	    meanADCVal = calcMeanOfContents(&g_inBuffer, BUF_SIZE);
	    IntMasterEnable();

	    setCurrentHeight(calcPercentAltitude(landedADCVal, meanADCVal));
	    setCurrentYaw(yawSlotCount);

	    // Update the display at 4Hz. displayFlag is set every 25 SysTick interrupts (250ms).
	    if (displayFlag) {
	        displayFlag = FLAG_CLEAR;
	        updateDisplay(currentDisplayState, landedADCVal, meanADCVal, yawSlotCount);
	    }

	    // Send UART Data at 4Hz. UARTFlag is set every 25 SysTick interrupts (250ms).
	    if (UARTFlag) {
	        UARTFlag = FLAG_CLEAR;
	        UARTSendData(landedADCVal, meanADCVal, yawSlotCount);
	    }

	    // Poll the buttons at 100Hz. Update their states if necessary.
	    // buttonFlag is set every SysTick interrupt (10ms).
	    if (buttonFlag) {
	        buttonFlag = FLAG_CLEAR;
	        checkButtons();
	    }

	    // Update the PID controller. controlUpdateFlag is set every SysTick interrupt (10ms).
	    if (controlUpdateFlag) {
	        controlUpdateFlag = FLAG_CLEAR;
	        updateControl();
	    }
	}
}

