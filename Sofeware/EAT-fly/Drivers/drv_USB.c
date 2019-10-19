#include "Basic.h"
#include "drv_USB.h"

#include "TM4C123GH6PM.h"
#include "usb.h"
#include "sysctl.h"
#include "gpio.h"
#include "pin_map.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "timer.h"
#include "rom.h"

#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"

#include "USB_def.h"

#include "CommuLink.h"

/*USB状态*/
	static bool usb_connected = false;
	
	static tLineCoding usb_linecoding = 
	{
		115200 ,
		USB_CDC_STOP_BITS_1 ,
		USB_CDC_PARITY_EVEN ,
		8 ,
	};
/*USB状态*/

//*****************************************************************************
//
// Handles CDC driver notifications related to control and setup of the device.
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host.  These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
USBDControlHandler(void *pvCBData, uint32_t ui32Event,
               uint32_t ui32MsgValue, void *pvMsgData)
{
//    uint32_t ui32IntsOff;

    //
    // Which event are we being asked to process?
    //
    switch(ui32Event)
    {
        //
        // We are connected to a host and communication is now possible.
        //
        case USB_EVENT_CONNECTED:
					//更新连接状态
					usb_connected = true;

					//清空缓冲区
					USBBufferFlush(&g_sTxBuffer);
					USBBufferFlush(&g_sRxBuffer);
					break;

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
					//更新连接状态
					usb_connected = false;
					break;

        //
        // Return the current serial communication parameters.
        //
        case USBD_CDC_EVENT_GET_LINE_CODING:
					*( (tLineCoding*)pvMsgData ) = usb_linecoding;
					break;

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_LINE_CODING:
					usb_linecoding = *( (tLineCoding*)pvMsgData );
					break;

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
					//SetControlLineState((uint16_t)ui32MsgValue);
					break;

        //
        // Send a break condition on the serial line.
        //
        case USBD_CDC_EVENT_SEND_BREAK:
					//SendBreak(true);
					break;

        //
        // Clear the break condition on the serial line.
        //
        case USBD_CDC_EVENT_CLEAR_BREAK:
					//SendBreak(false);
					break;

        //
        // Ignore SUSPEND and RESUME for now.
        //
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif

    }

    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the transmit channel (data to
// the USB host).
//
// \param ui32CBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
USBDTxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    //
    // Which event have we been sent?
    //
    switch(ui32Event)
    {
        case USB_EVENT_TX_COMPLETE:
            //
            // Since we are using the USBBuffer, we don't need to do anything
            // here.
            //
            break;

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif

    }
    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the receive channel (data from
// the USB host).
//
// \param ui32CBData is the client-supplied callback data value for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
USBDRxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
//    uint32_t ui32Count;

//    //
//    // Which event are we being sent?
//    //
//    switch(ui32Event)
//    {
//        //
//        // A new packet has been received.
//        //
//        case USB_EVENT_RX_AVAILABLE:
//        {
//            //
//            // Feed some characters into the UART TX FIFO and enable the
//            // interrupt so we are told when there is more space.
//            //
//            USBUARTPrimeTransmit(USB_UART_BASE);
//            ROM_UARTIntEnable(USB_UART_BASE, UART_INT_TX);
//            break;
//        }

//        //
//        // We are being asked how much unprocessed data we have still to
//        // process. We return 0 if the UART is currently idle or 1 if it is
//        // in the process of transmitting something. The actual number of
//        // bytes in the UART FIFO is not important here, merely whether or
//        // not everything previously sent to us has been transmitted.
//        //
//        case USB_EVENT_DATA_REMAINING:
//        {
//            //
//            // Get the number of bytes in the buffer and add 1 if some data
//            // still has to clear the transmitter.
//            //
//            //ui32Count = ROM_UARTBusy(USB_UART_BASE) ? 1 : 0;
//            return(ui32Count);
//        }

//        //
//        // We are being asked to provide a buffer into which the next packet
//        // can be read. We do not support this mode of receiving data so let
//        // the driver know by returning 0. The CDC driver should not be sending
//        // this message but this is included just for illustration and
//        // completeness.
//        //
//        case USB_EVENT_REQUEST_BUFFER:
//        {
//            return(0);
//        }

//        //
//        // We don't expect to receive any other events.  Ignore any that show
//        // up in a release build or hang in a debug build.
//        //
//        default:
//#ifdef DEBUG
//            while(1);
//#else
//            break;
//#endif
//    }

    return(0);
}

void init_drv_USB()
{
//	SysCtlPeripheralEnable( SYSCTL_PERIPH_USB0 );	//开启USB电源
//	delay( 1e-6f );
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); //开启GPIOD电源
	delay( 1e-6f );
	
	GPIOPinTypeUSBAnalog( GPIOD_BASE , GPIO_PIN_4 );	//PD4 USB DM
	GPIOPinTypeUSBAnalog( GPIOD_BASE , GPIO_PIN_5 );	//PD5 USB DP
	
	//初始化发送接收缓冲区
	USBBufferInit(&g_sTxBuffer);
	USBBufferInit(&g_sRxBuffer);
	
	USBStackModeSet(0, eUSBModeForceDevice, 0);
	
	void* usbd_instance = USBDCDCInit(0, &g_sCDCDevice);
	
	IntRegister( INT_USB0 , USB0DeviceIntHandler );
	IntPrioritySet( INT_USB0 , INT_PRIO_4 );
	
	//注册USB端口
	Port usb_port;
	usb_port.read = read_UsbUart;
	usb_port.DataAvailable = UsbUart_DataAvailable;
	usb_port.write = write_UsbUart;
	PortRegister( usb_port );
}

void write_UsbUart( const uint8_t* data , uint16_t length )
{
	USBBufferWrite( &g_sTxBuffer , data , length );
}
uint16_t read_UsbUart( uint8_t* data , uint16_t length )
{
	return USBBufferRead( &g_sRxBuffer , data , length );
}
uint16_t UsbUart_DataAvailable()
{
	return USBBufferDataAvailable(&g_sRxBuffer);
}