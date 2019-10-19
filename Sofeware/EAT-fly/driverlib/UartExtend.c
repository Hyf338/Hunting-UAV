#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"

void UartSetBaudRate( uint32_t ui32Base, uint32_t ui32UARTClk,
                    uint32_t ui32Baud, uint32_t ui32Config )
{
		uint32_t ui32Div;
	
		ASSERT(_UARTBaseValid(ui32Base));
    ASSERT(ui32Baud != 0);
    ASSERT(ui32UARTClk >= (ui32Baud * UART_CLK_DIVIDER));
	
//
    // Is the required baud rate greater than the maximum rate supported
    // without the use of high speed mode?
    //
    if((ui32Baud * 16) > ui32UARTClk)
    {
        //
        // Enable high speed mode.
        //
        HWREG(ui32Base + UART_O_CTL) |= UART_CTL_HSE;

        //
        // Half the supplied baud rate to compensate for enabling high speed
        // mode.  This allows the following code to be common to both cases.
        //
        ui32Baud /= 2;
    }
    else
    {
        //
        // Disable high speed mode.
        //
        HWREG(ui32Base + UART_O_CTL) &= ~(UART_CTL_HSE);
    }

    //
    // Compute the fractional baud rate divider.
    //
    ui32Div = (((ui32UARTClk * 8) / ui32Baud) + 1) / 2;

    //
    // Set the baud rate.
    //
    HWREG(ui32Base + UART_O_IBRD) = ui32Div / 64;
    HWREG(ui32Base + UART_O_FBRD) = ui32Div % 64;

    //
    // Set parity, data length, and number of stop bits.
    //
    HWREG(ui32Base + UART_O_LCRH) = ui32Config;
}