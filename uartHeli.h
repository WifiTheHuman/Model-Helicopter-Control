// *******************************************************
//
// uartHeli.h
//
// Header for the uartHeli.c file, also contains UART
// congfig data (baud rate).
//
// Joshua Hulbert, Josiah Craw, Yifei Ma
//
// *******************************************************


#define BAUD_RATE               9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX

void
UARTSendString(char *message);

void
UARTSendData(uint16_t landedADCVal, uint16_t meanADCVal, int yawSlotCount);
