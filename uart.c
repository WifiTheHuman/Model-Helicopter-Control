

void
initUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), BAUD_RATE,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);

    UARTFIFOEnable(UART0_BASE);

    UARTEnable(UART0_BASE);
}

void
UARTSendString(char *message)
{
    while (*message)
    {
        UARTCarPut(UART0_BASE, *message);
        message++;
    }
}
