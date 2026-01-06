#include <stdint.h>
#include "stm32f411xe.h"
#include "makeshiftDebugger.h"
#include "usb.h"
#include "usbIrqHandlers.h"
#include "usbDemoDescriptors.h"
#include "usbDebug.h"



#define USB_DEVICE_DESCRIPTOR_SIZE_W (USB_DEVICE_DESCRIPTOR_KEYBOARD_DEMO_SIZE_W)
#define USB_DEVICE_DESCRIPTOR_SIZE_B (18U)

#define USB_CONFIG_DESCRIPTOR_SIZE_W (USB_CONFIG_DESCRIPTOR_KEYBOARD_DEMO_SIZE_W)
#define USB_CONFIG_DESCRIPTOR_SIZE_B (USB_CONFIGURATION_DESCRIPTOR_GAMEPAD_DEMO_SIZE_B)

#define USB_CONFIG_COMBINED_DESCRIPTOR_SIZE_W (USB_CONFIG_COMBINED_DESCRIPTOR_GAMEPAD_DEMO_SIZE_W)
#define USB_CONFIG_COMBINED_DESCRIPTOR_SIZE_B (USB_CONFIG_COMBINED_DESCRIPTOR_GAMEPAD_DEMO_SIZE_B)

// keyboard demo
// #define HID_REPORT_DESCRIPTOR_SIZE_W (HID_REPORT_DESCRIPTOR_KEYBOARD_SIZE_W)
// #define HID_REPORT_DESCRIPTOR_SIZE_B (HID_REPORT_DESCRIPTOR_KEYBOARD_SIZE_B)

// gamepad demo
#define HID_REPORT_DESCRIPTOR_SIZE_W (HID_REPORT_DESCRIPTOR_GAMEPAD_DEMO_SIZE_W)
#define HID_REPORT_DESCRIPTOR_SIZE_B (HID_REPORT_DESCRIPTOR_GAMEPAD_DEMO_SIZE_B)


// uint32_t* usbDeviceDescriptorPtr;
// uint32_t* usbConfigurationDescriptorPtr;
// uint32_t* usbCombinedConfigDescriptorsPtr;
// uint32_t* usbHidReportDescriptorPtr;


// -----------------------------------------------------------------------------
//                        USB Demo Descriptors (keyboard)
// -----------------------------------------------------------------------------
extern uint32_t usbDeviceDescriptor[USB_DEVICE_DESCRIPTOR_KEYBOARD_DEMO_SIZE_W];
extern uint32_t usbConfigurationDescriptor[USB_CONFIG_DESCRIPTOR_KEYBOARD_DEMO_SIZE_W];
extern uint32_t usbCombinedConfigDescriptors[USB_CONFIG_COMBINED_DESCRIPTOR_KEYBOARD_DEMO_SIZE_W];
extern uint32_t myKeyboardHidReportDescriptor[HID_REPORT_DESCRIPTOR_KEYBOARD_SIZE_W];

// -----------------------------------------------------------------------------
//                        USB Demo Descriptors (Gamepad)
// -----------------------------------------------------------------------------
extern uint32_t deviceDescriptorGamepadDemo[USB_DEVICE_DESCRIPTOR_GAMEPAD_DEMO_SIZE_W];
extern uint32_t configurationDescriptorGamepadDemo[USB_CONFIGURATION_DESCRIPTOR_GAMEPAD_DEMO_SIZE_W];
extern uint32_t configurationDescriptorCombinedGamepadDemo[USB_CONFIG_COMBINED_DESCRIPTOR_GAMEPAD_DEMO_SIZE_W];
extern uint32_t hidReportDescriptorGamepadDemo[HID_REPORT_DESCRIPTOR_GAMEPAD_DEMO_SIZE_W];


uint32_t* usbDeviceDescriptorPtr = deviceDescriptorGamepadDemo;
uint32_t* usbConfigurationDescriptorPtr = configurationDescriptorGamepadDemo;
uint32_t* usbCombinedConfigDescriptorsPtr = configurationDescriptorCombinedGamepadDemo;
uint32_t* usbHidReportDescriptorPtr = hidReportDescriptorGamepadDemo;


volatile uint8_t isEp1Configured = 0;
uint32_t hidReportDelayCount = 0;
uint8_t shouldSendHidReport = 0;
uint8_t usbRxStatusAlreadyLogged = 0;
uint32_t rxflvlGlobalCallCount = 0;
SetupRequest gSetupRequest;
uint8_t isControlTransferInProgress = 0;
uint16_t usbDeviceAddress = 0; // modified by Set_Address
uint8_t shouldEnableEndpoint1 = 0;
uint8_t isSetAddressInProgress = 0;


// ****************************************************************************
//                     USB OTG USBRST Handler
// ****************************************************************************
void usbHandleIrqUSBRST(void)
{
    debuggerWriteStr(DBG_USART, "+++++++++++++++++++++++++++\n\r");
    debuggerWriteStr(DBG_USART, "USB Reset\n\r");
    debuggerWriteStr(DBG_USART, "+++++++++++++++++++++++++++\n\r");

    // clear global flags
    isEp1Configured = 0;
    shouldSendHidReport = 0;
    hidReportDelayCount = 0;


    // exit the suspend state
    USB_OTG_FS_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;
    // USB_OTG_FS_DEVICE->DCTL |= USB_OTG_DCTL_RWUSIG;

    // flush Tx FIFOs
    // Perform FIFO flush in one go (from HAL)
    USB_OTG_FS->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH | (0x10U << USB_OTG_GRSTCTL_TXFNUM_Pos));
    
    // wait for Tx FIFO flush to take effect
    while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH)
    {
        continue;
    }


    for (uint8_t i = 0; i < USB_OTG_FS_ENDPOINT_COUNT; i++)
    {
        USB_OTG_FS_INEP(i)->DIEPINT = 0xFB7FU;
        USB_OTG_FS_INEP(i)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
        USB_OTG_FS_INEP(i)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
        USB_OTG_FS_OUTEP(i)->DOEPINT = 0xFB7FU;
        USB_OTG_FS_OUTEP(i)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
        USB_OTG_FS_OUTEP(i)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
    }

    // Mask IN Ep 0 and OUT Ep 0
    // USB_OTG_FS_DEVICE->DAINTMSK |= 0x10001U;
    USB_OTG_FS_DEVICE->DAINTMSK |= ((1U << 16U) | (1U << 0U));


    USB_OTG_FS_DEVICE->DOEPMSK |= USB_OTG_DOEPMSK_STUPM |
        USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_EPDM |
        USB_OTG_DOEPMSK_OTEPSPRM | USB_OTG_DOEPMSK_NAKM;

    // TOM = timeout condition
    // XFRCM = transfer completed
    // EPDM = endpoint disabled
    USB_OTG_FS_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_TOM |
        USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_EPDM;


    // device address = 0 (default)
    USB_OTG_FS_DEVICE->DCFG &= ~(USB_OTG_DCFG_DAD);


    #define USB_OTG_CORE_ID (0U) // not an actual ID; it's for tests only
    
    #if (USB_OTG_CORE_ID == 1U)
    #warning "USB_OTG_CORE_ID = 1"
    if ((USB_OTG_FS_OUTEP(0U)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) != USB_OTG_DOEPCTL_EPENA)
    {
        USB_OTG_FS_OUTEP(0U)->DOEPTSIZ = 0U;
        USB_OTG_FS_OUTEP(0U)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19));
        USB_OTG_FS_OUTEP(0U)->DOEPTSIZ |= (3U * 8U);
        USB_OTG_FS_OUTEP(0U)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_STUPCNT;
    }
    #else
    #warning "USB_OTG_CORE_ID = 0"
    USB_OTG_FS_OUTEP(0U)->DOEPTSIZ = 0U;
    USB_OTG_FS_OUTEP(0U)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19));
    USB_OTG_FS_OUTEP(0U)->DOEPTSIZ |= (3U * 8U);
    USB_OTG_FS_OUTEP(0U)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_STUPCNT;
    #endif

    // clear the USB Reset interrupt
    // since most bits in GINTSTS are "rc_w1", then clear this bit with AND (&)
    USB_OTG_FS->GINTSTS &= USB_OTG_GINTSTS_USBRST;
}


// ****************************************************************************
//                     USB OTG ENUMDNE Handler
// ****************************************************************************
void usbHandleIrqENUMDNE(void)
{

    // set maximum packet size for Endpoint 0
    // 0b00 = 64 bytes
    // USB_OTG_FS_INEP(0)->DIEPCTL &= ~(0x3U);
    USB_OTG_FS_INEP(0)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;
    // clear global IN NAK
    USB_OTG_FS_DEVICE->DCTL |= USB_OTG_DCTL_CGINAK;


    // read USB speed
    // uint32_t dstsCopy = USB_OTG_FS_DEVICE->DSTS;
    (void)USB_OTG_FS_DEVICE->DSTS;


    // set USB turnaround time
    USB_OTG_FS->GUSBCFG &= ~(USB_OTG_GUSBCFG_TRDT);
    // USB_OTG_FS->GUSBCFG |= (0x6 << USB_OTG_GUSBCFG_TRDT_Pos);
    USB_OTG_FS->GUSBCFG |= (uint32_t)((0x6U << 10) & USB_OTG_GUSBCFG_TRDT);


    // unmask IN Endpoint 0 interrupts
    // USB_OTG_FS_DEVICE->DAINTMSK |= (1U << 0U);
    USB_OTG_FS_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_IEPM & (uint32_t)(1UL << 0U);
    // debuggerWriteStr(DBG_USART, "setting up DIEPCTL0...\n\r");
    if ((USB_OTG_FS_INEP(0)->DIEPCTL & USB_OTG_DIEPCTL_USBAEP) == 0U)
    {
        // debuggerWriteStr(DBG_USART, "DIEPCTL0_USBAEP = 0\n\r");
        // USB_OTG_FS_INEP(0)->DIEPCTL &= ~(USB_OTG_DIEPCTL_TXFNUM);
        // // 0x80 is Rx FIFO size
        // // IN Ep 0 is the Tx Endpoint (sends data to host)
        // // so its Tx FIFO address is just after Rx FIFO
        // USB_OTG_FS_INEP(0)->DIEPCTL |= (0x80 << USB_OTG_DIEPCTL_TXFNUM_Pos);
        // // "Set DATA0 PID"
        // // this bit is said to be "reserved" in RM0383 Rev 3
        // // it is set in HAL anyway, so I'm gonna leave it set
        // USB_OTG_FS_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_SD0PID_SEVNFRM);
        // // USB activate endpoint (should always be 1 for Endpoint 0)
        // USB_OTG_FS_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_USBAEP);

        // HAL
        USB_OTG_FS_INEP(0)->DIEPCTL |= (64U & USB_OTG_DIEPCTL_MPSIZ) |
                                        (0x00U << 18) | (0 << 22) |
                                        USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
                                        USB_OTG_DIEPCTL_USBAEP;
    }

    // unmask OUT Endpoint 0 interrupts
    USB_OTG_FS_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_OEPM & (uint32_t)(1UL << 16U);
    // debuggerWriteStr(DBG_USART, "setting up DOEPCTL0...\n\r");
    if ((USB_OTG_FS_OUTEP(0)->DOEPCTL & USB_OTG_DOEPCTL_USBAEP) == 0U)
    {
        // debuggerWriteStr(DBG_USART, "DOEPCTL0_USBAEP = 0\n\r");
        // USB_OTG_FS_OUTEP(0)->DOEPCTL &= ~(USB_OTG_DIEPCTL_TXFNUM);
        // // "Set DATA0 PID"
        // // this bit is said to be "reserved" in RM0383 Rev 3
        // // it is set in HAL anyway, so I'm gonna leave it set
        // USB_OTG_FS_OUTEP(0)->DOEPCTL |= (USB_OTG_DOEPCTL_SD0PID_SEVNFRM);
        // // USB activate endpoint (should always be 1 for Endpoint 0)
        // USB_OTG_FS_OUTEP(0)->DOEPCTL |= (USB_OTG_DOEPCTL_USBAEP);

        // HAL
        USB_OTG_FS_OUTEP(0)->DOEPCTL |= (64U & USB_OTG_DOEPCTL_MPSIZ) |
                                        (0x00U << 18) |
                                        USB_OTG_DOEPCTL_SD0PID_SEVNFRM |
                                        USB_OTG_DOEPCTL_USBAEP;
    }

    // 0x10001
    // dbgPrintHex("DAINTMSK = ", USB_OTG_FS_DEVICE->DAINTMSK, "\n\r");

    // clear the ENUMDNE flag
    USB_OTG_FS->GINTSTS &= USB_OTG_GINTSTS_ENUMDNE;
}


// ****************************************************************************
//                     USB OTG RXFLVL Handler
// ****************************************************************************
#define USB_DBG_READ_SETUP_RX_FIFO (0U)
#define USB_DBG_PRINT_SETUP_STRUCT (0U)

void usbHandleIrqRXFLVL(void)
{
    uint32_t rxStatusPop = USB_OTG_FS->GRXSTSP;

    // mask (disable) RXFLVL interrupt
    // this should only be done for reading data from Rx FIFO
    // USB_OTG_FS->GINTMSK &= ~(USB_OTG_GINTMSK_RXFLVLM);

    dbgPrintDec("\n\rRXFLVL handler call #", rxflvlGlobalCallCount+1, "\n\r");
    rxflvlGlobalCallCount++;

    if (!usbRxStatusAlreadyLogged)
    {
        // dbgPrintHex("USB RX Status POP #1 = ", rxStatusPop, "\n\r");
        dbgPrintHex("USB RX Status POP = ", rxStatusPop, "\n\r");

        // dbgPrintHex("endpoint number = ", (rxStatusPop & 0x0FU), "\n\r");
        // dbgPrintHex("byte count = ", ((rxStatusPop >> 4U) & 0x7FFU), "\n\r");
        // dbgPrintHex("Data PID = ", ((rxStatusPop >> 15U) & 0x3U), "\n\r");
        // dbgPrintHex("Packet status = ", ((rxStatusPop >> 17U) & 0xFU), "\n\r");
        // dbgPrintHex("frame number = ", ((rxStatusPop >> 21U) & 0xFU), "\n\r");

        dbgPrintHex("endpoint number = ", GRXSTS_GET_EPNUM(rxStatusPop), "\n\r");
        dbgPrintHex("byte count = ", GRXSTS_GET_BCNT(rxStatusPop), "\n\r");
        dbgPrintHex("Data PID = ", GRXSTS_GET_DPID(rxStatusPop), "\n\r");
        dbgPrintHex("Packet status = ", GRXSTS_GET_PKTSTS(rxStatusPop), "\n\r");
        dbgPrintHex("frame number = ", GRXSTS_GET_FRMNUM(rxStatusPop), "\n\r");

        // usbRxStatusAlreadyLogged = 1;
    }

    // check if Byte Count = 0
    if (GRXSTS_GET_BCNT(rxStatusPop) == 0)
    {
        debuggerWriteStr(DBG_USART, "RXFLVL Byte Count = 0\n\r");

        // mask (disable) all USB interrupts (DEBUG)
        // USB_OTG_FS->GAHBCFG &= ~(USB_OTG_GAHBCFG_GINT);
    }

    uint32_t* fifoPtr = (uint32_t *)EP0_FIFO_ADDR;
    
    #if (USB_DBG_READ_SETUP_RX_FIFO == 1U)
    uint32_t rxFifoData1;
    uint32_t rxFifoData2;
    #endif

    // Endpoint 0
    if (GRXSTS_GET_EPNUM(rxStatusPop) == 0U)
    {
        
        #if (USB_DBG_READ_SETUP_RX_FIFO == 1U)
        rxFifoData1 = *fifoPtr;
        rxFifoData2 = *(fifoPtr + 4U);

        if (!usbRxFifoDataLogged)
        {
            dbgPrintHex("Rx FIFO address = ", (uint32_t)fifoPtr, "\n\r");
            dbgPrintHex("Rx FIFO data #1 = ", rxFifoData1, "\n\r");
            dbgPrintHex("Rx FIFO data #2 = ", rxFifoData2, "\n\r");

            usbRxFifoDataLogged = 1;
        }
        #endif


        if (GRXSTS_GET_PKTSTS(rxStatusPop) == GRXSTS_SETUP_DATA_PACKET_RX)
        {
            // mask (disable) RXFLVL interrupt
            USB_OTG_FS->GINTMSK &= ~(USB_OTG_GINTMSK_RXFLVLM);

            uint64_t setupData = (uint64_t) *fifoPtr;
            setupData |= ((uint64_t) *(fifoPtr + 4U)) << 32U;

            gSetupRequest.bmRequestType = (uint8_t) setupData;
            gSetupRequest.bRequest = (uint8_t) (setupData >> 8U);
            gSetupRequest.wValue = (uint16_t) (setupData >> 16U);
            gSetupRequest.wIndex = (uint16_t) (setupData >> 32U);
            gSetupRequest.wLength = (uint16_t) (setupData >> 48U);


            #if (USB_DBG_PRINT_SETUP_STRUCT == 1U)
            logSetupPacketStruct(&gSetupRequest);
            #endif

            isControlTransferInProgress = 1;

            // unmask (enable) RXFLVL interrupt
            USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
        }
        else if (GRXSTS_GET_PKTSTS(rxStatusPop) == GRXSTS_SETUP_TRANSACTION_COMPLETED)
        {
            debuggerWriteStr(DBG_USART, "packet status: setup transaction completed\n\r");
            debuggerWriteStr(DBG_USART, "STUP interrupt should be triggered now\n\r");

            // mask (disable) all USB interrupts (DEBUG)
            // USB_OTG_FS->GAHBCFG &= ~(USB_OTG_GAHBCFG_GINT);
        }
        else if (GRXSTS_GET_PKTSTS(rxStatusPop) == GRXSTS_OUT_DATA_PACKET_RX)
        {

            debuggerWriteStr(DBG_USART, "--------------------\n\r");
            debuggerWriteStr(DBG_USART, "packet status: Out Data Packet Received\n\r");
            debuggerWriteStr(DBG_USART, "--------------------\n\r");

            if (isControlTransferInProgress == 1)
            {
                isControlTransferInProgress = 0;
            }

            // mask (disable) all USB interrupts (DEBUG)
            // USB_OTG_FS->GAHBCFG &= ~(USB_OTG_GAHBCFG_GINT);

        }
        else if (GRXSTS_GET_PKTSTS(rxStatusPop) == GRXSTS_OUT_TRANSFER_COMPLETED)
        {
            debuggerWriteStr(DBG_USART, "--------------------\n\r");
            debuggerWriteStr(DBG_USART, "packet status: OUT Transfer Completed\n\r");
            debuggerWriteStr(DBG_USART, "--------------------\n\r");
            
            // this event asserts the "Transfer Completed" interrupt
            // and clears, according to DOEPCTL_EPENA description,
            // the EPENA flag (disables the endpoint)

            // enable OUT Ep 0
            USB_OTG_FS_OUTEP(0U)->DOEPCTL |= (USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK);

            // mask (disable) all USB interrupts (DEBUG)
            // USB_OTG_FS->GAHBCFG &= ~(USB_OTG_GAHBCFG_GINT);

        }
        else if (GRXSTS_GET_PKTSTS(rxStatusPop) == GRXSTS_GLOBAL_OUT_NAK)
        {
            debuggerWriteStr(DBG_USART, "--------------------\n\r");
            debuggerWriteStr(DBG_USART, "packet status: Global OUT NAK\n\r");
            debuggerWriteStr(DBG_USART, "--------------------\n\r");

            // mask (disable) all USB interrupts (DEBUG)
            USB_OTG_FS->GAHBCFG &= ~(USB_OTG_GAHBCFG_GINT);
        }
        else
        {
            debuggerWriteStr(DBG_USART, "--------------------\n\r");
            debuggerWriteStr(DBG_USART, "!!!!!! packet status: U N K N O W N !!!!!!\n\r");
            debuggerWriteStr(DBG_USART, "--------------------\n\r");
        }
    }
    else if (GRXSTS_GET_EPNUM(rxStatusPop) == 1U)
    {
        debuggerWriteStr(DBG_USART, "------------------------------------\n\r");
        debuggerWriteStr(DBG_USART, "        RXFLVL   ENDPOINT   1\n\r");
        debuggerWriteStr(DBG_USART, "------------------------------------\n\r");
    }
    else
    {
        // endpoint other than Ep0
        // ...
    }


    // clear RXFLVL? it's read only in reference manual
    // USB_OTG_FS->GINTSTS &= USB_OTG_GINTSTS_RXFLVL;

    // unmask (enable) RXFLVL interrupt
    // this should be done after reading data from Rx FIFO
    // USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;

    // mask (disable) all USB interrupts (DEBUG)
    // USB_OTG_FS->GAHBCFG &= ~(USB_OTG_GAHBCFG_GINT);
}


// ****************************************************************************
//                     USB OTG OEPINT Handler
// ****************************************************************************
void usbHandleIrqOEPINT(void)
{
    uint8_t shouldHandleEp0Stup = 0;


    debuggerWriteStr(DBG_USART, "\n\rhandling OEPINT\n\r");


    // endpoint 0 interrupt is pending
    if ((USB_OTG_FS_DEVICE->DAINT & (1U << 16U)) != 0)
    {
        uint32_t doepintCopy = USB_OTG_FS_OUTEP(0U)->DOEPINT;

        logUsbDeviceOutEndpointPendingInterrupts(doepintCopy, 0);

        debuggerWriteStr(DBG_USART, "pending interrupt on OUT Ep0:");

        // handle Endpoint 0 STUP interrupt
        if (doepintCopy & USB_OTG_DOEPINT_STUP)
        {
            debuggerWriteStr(DBG_USART, " STUP");
            shouldHandleEp0Stup = 1;
        }

        if (doepintCopy & USB_OTG_DOEPINT_XFRC)
        {
            debuggerWriteStr(DBG_USART, " XFRC");
            // ...

            USB_OTG_FS_OUTEP(0U)->DOEPINT &= USB_OTG_DOEPINT_XFRC;
        }

        if (doepintCopy & USB_OTG_DOEPINT_EPDISD)
        {
            debuggerWriteStr(DBG_USART, " EPDISD");
            // ...

            USB_OTG_FS_OUTEP(0U)->DOEPINT &= USB_OTG_DOEPINT_EPDISD;
        }

        if (doepintCopy & USB_OTG_DOEPINT_OTEPSPR)
        {
            // debuggerWriteStr(DBG_USART, " OTEPSPR");
            debuggerWriteStr(DBG_USART, " STSPHSRX");
            // ...

            USB_OTG_FS_OUTEP(0U)->DOEPINT &= USB_OTG_DOEPINT_OTEPSPR;
        }

        if (doepintCopy & USB_OTG_DOEPINT_NAK)
        {
            debuggerWriteStr(DBG_USART, " NAK");
            // ...

            USB_OTG_FS_OUTEP(0U)->DOEPINT &= USB_OTG_DOEPINT_NAK;
        }

        debuggerWriteStr(DBG_USART, "\n\r");
    }

    uint32_t* ep1FifoPtr = (uint32_t *)EP1_FIFO_ADDR;

    // Setup phase done for Endpoint 0
    if (shouldHandleEp0Stup)
    {
        if (gSetupRequest.bRequest == USB_CTRL_REQ_GET_DESCRIPTOR)
        {
            debuggerWriteStr(DBG_USART, "handling Get_Descriptor(?)...\n\r");

            uint32_t* ep0FifoPtr = (uint32_t *)EP0_FIFO_ADDR;
            uint16_t endpointTxSpace;

            if (USB_DESCR_TYPE_FROM_STRUCT(gSetupRequest) == USB_DEVICE_DESCRIPTOR_TYPE_ID)
            {
                
                // debuggerWriteStr(DBG_USART, "handling Get_Descriptor(Device)...\n\r");

                debuggerWriteStr(DBG_USART, "-----------------------------------------\n\r");
                debuggerWriteStr(DBG_USART, "handling Get_Descriptor(Device)...\n\r");
                debuggerWriteStr(DBG_USART, "-----------------------------------------\n\r");
                
                endpointTxSpace = USB_OTG_FS_INEP(0U)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk;

                debuggerWriteStr(DBG_USART, "checking Tx FIFO space... ");
                
                while ((USB_OTG_FS_INEP(0U)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk) == 0U)
                {
                    continue;
                }
                debuggerWriteStr(DBG_USART, "OK, enough space\n\r");
                dbgPrintHex("endpointTxSpace = ", endpointTxSpace, "\n\r");

                // this causes interrupts to trigger constantly
                // USB_OTG_FS_INEP(0U)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK;


                if (endpointTxSpace != 0)
                {
                    // this must be set up before enabling the endpoint
                    USB_OTG_FS_INEP(0U)->DIEPTSIZ = (
                        0x3U << USB_OTG_DIEPTSIZ_MULCNT_Pos | // STUPCNT
                        0x1U << USB_OTG_DIEPTSIZ_PKTCNT_Pos |
                        // device descriptor length in bytes
                        USB_DEVICE_DESCRIPTOR_SIZE_B << USB_OTG_DIEPTSIZ_XFRSIZ_Pos 
                    );

                    // enable the endpoint before writing data to Tx FIFO
                    // USB_OTG_FS_INEP(0U)->DIEPCTL |= (USB_OTG_DIEPCTL_EPENA); // debug
                    USB_OTG_FS_INEP(0U)->DIEPCTL |= (USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK);
                    

                    // send Device Descriptor
                    for (uint8_t i = 0; i < USB_DEVICE_DESCRIPTOR_SIZE_W; i++)
                    {
                        // *(ep0FifoPtr + (i*4U)) = usbDeviceDescriptor[i];
                        *(ep0FifoPtr + (i*4U)) = usbDeviceDescriptorPtr[i];
                    }

                    // experiment; first set Tx FIFO, then disable Ep 0 NAK
                    // the reference manual says that EPENA must be set before
                    // writing to FIFO
                    // USB_OTG_FS_INEP(0U)->DIEPCTL |= (USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK);
                }

                debuggerWriteStr(DBG_USART, "handling Get_Descriptor(Device)... - END\n\r");
                debuggerWriteStr(DBG_USART, "-----------------------------------------\n\r");
            }
            else if (USB_DESCR_TYPE_FROM_STRUCT(gSetupRequest) == USB_CONFIGURATION_DESCRIPTOR_TYPE_ID)
            {
                debuggerWriteStr(DBG_USART, "-----------------------------------------\n\r");
                debuggerWriteStr(DBG_USART, "handling Get_Descriptor(Configuration)...\n\r");
                debuggerWriteStr(DBG_USART, "-----------------------------------------\n\r");

                logSetupPacketStruct(&gSetupRequest);

                endpointTxSpace = USB_OTG_FS_INEP(0U)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk;

                debuggerWriteStr(DBG_USART, "checking Tx FIFO space... ");
                while ((USB_OTG_FS_INEP(0U)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk) == 0U)
                {
                    continue;
                }
                debuggerWriteStr(DBG_USART, "OK, enough space\n\r");
                dbgPrintHex("endpointTxSpace = ", endpointTxSpace, "\n\r");


                if (endpointTxSpace != 0)
                {
                    if (gSetupRequest.wLength == USB_CONFIG_DESCRIPTOR_SIZE_B)
                    {
                        // this must be set up before enabling the endpoint
                        USB_OTG_FS_INEP(0U)->DIEPTSIZ = (
                            0x3U << USB_OTG_DIEPTSIZ_MULCNT_Pos | // STUPCNT
                            0x1U << USB_OTG_DIEPTSIZ_PKTCNT_Pos |
                            // configuration descriptor byte length
                            USB_CONFIG_DESCRIPTOR_SIZE_B << USB_OTG_DIEPTSIZ_XFRSIZ_Pos
                        );

                        // enable the endpoint before writing data to Tx FIFO
                        USB_OTG_FS_INEP(0U)->DIEPCTL |= (USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK);
                        
                        // send Configuration Descriptor
                        for (uint8_t i = 0; i < USB_CONFIG_DESCRIPTOR_SIZE_W; i++)
                        {
                            // *(ep0FifoPtr + (i*4U)) = usbConfigurationDescriptor[i];
                            *(ep0FifoPtr + (i*4U)) = usbConfigurationDescriptorPtr[i];
                        }

                    }
                    else
                    {
                        // combined Get_Configuration

                        // this must be set up before enabling the endpoint
                        USB_OTG_FS_INEP(0U)->DIEPTSIZ = (
                            0x3U << USB_OTG_DIEPTSIZ_MULCNT_Pos | // STUPCNT
                            0x1U << USB_OTG_DIEPTSIZ_PKTCNT_Pos |
                            // combined configuration descriptor length in bytes
                            USB_CONFIG_COMBINED_DESCRIPTOR_SIZE_B << USB_OTG_DIEPTSIZ_XFRSIZ_Pos
                        );


                        // enable the endpoint before writing data to Tx FIFO
                        USB_OTG_FS_INEP(0U)->DIEPCTL |= (USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK);
                        

                        // send Configuration Descriptor
                        for (uint8_t i = 0; i < USB_CONFIG_COMBINED_DESCRIPTOR_SIZE_W; i++)
                        {
                            // *(ep0FifoPtr + (i*4U)) = usbCombinedConfigDescriptors[i];
                            *(ep0FifoPtr + (i*4U)) = usbCombinedConfigDescriptorsPtr[i];
                        }
                    }

                    
                }

                debuggerWriteStr(DBG_USART, "handling Get_Descriptor(Configuration) - END\n\r");
                debuggerWriteStr(DBG_USART, "-----------------------------------------\n\r");
            }
            else if (USB_DESCR_TYPE_FROM_STRUCT(gSetupRequest) == USB_HID_REPORT_DESCRIPTOR_TYPE_ID)
            {
                debuggerWriteStr(DBG_USART, "-----------------------------------------\n\r");
                debuggerWriteStr(DBG_USART, "handling Get_Descriptor(HID Report)...\n\r");
                debuggerWriteStr(DBG_USART, "-----------------------------------------\n\r");
                
                logSetupPacketStruct(&gSetupRequest);

                endpointTxSpace = USB_OTG_FS_INEP(0U)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk;

                debuggerWriteStr(DBG_USART, "checking Tx FIFO space... ");
                while ((USB_OTG_FS_INEP(0U)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk) == 0U)
                {
                    continue;
                }
                debuggerWriteStr(DBG_USART, "OK, enough space\n\r");
                dbgPrintHex("endpointTxSpace = ", endpointTxSpace, "\n\r");


                if (endpointTxSpace != 0)
                {

                    // this must be set up before enabling the endpoint
                    USB_OTG_FS_INEP(0U)->DIEPTSIZ = (
                        0x3U << USB_OTG_DIEPTSIZ_MULCNT_Pos | // STUPCNT
                        0x1U << USB_OTG_DIEPTSIZ_PKTCNT_Pos |
                        // configuration descriptor byte length
                        HID_REPORT_DESCRIPTOR_SIZE_B << USB_OTG_DIEPTSIZ_XFRSIZ_Pos
                    );

                    // enable the endpoint before writing data to Tx FIFO
                    USB_OTG_FS_INEP(0U)->DIEPCTL |= (USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK);
                    
                    // send Descriptor
                    for (uint8_t i = 0; i < HID_REPORT_DESCRIPTOR_SIZE_W; i++)
                    {
                        *(ep0FifoPtr + (i*4U)) = usbHidReportDescriptorPtr[i];
                    }
                }

                debuggerWriteStr(DBG_USART, "handling Get_Descriptor(HID Report) - END\n\r");
                debuggerWriteStr(DBG_USART, "-----------------------------------------\n\r");
            }
            else
            {
                debuggerWriteStr(DBG_USART, "-----------------------------------------\n\r");
                debuggerWriteStr(DBG_USART, "Unknown Get_Descriptor()...\n\r");
                debuggerWriteStr(DBG_USART, "-----------------------------------------\n\r");

                logSetupPacketStruct(&gSetupRequest);

                // mask (disable) all USB interrupts (DEBUG)
                // USB_OTG_FS->GAHBCFG &= ~(USB_OTG_GAHBCFG_GINT);

                debuggerWriteStr(DBG_USART, "Unknown Get_Descriptor() - END\n\r");
                debuggerWriteStr(DBG_USART, "-----------------------------------------\n\r");
            }
        }
        else if (gSetupRequest.bRequest == USB_CTRL_REQ_SET_ADDRESS)
        {

            debuggerWriteStr(DBG_USART, "##################################\n\r");
            debuggerWriteStr(DBG_USART, "handling Set_Address...\n\r");
            debuggerWriteStr(DBG_USART, "##################################\n\r");

            logSetupPacketStruct(&gSetupRequest);

            // turns out it's a wrong approach
            // isSetAddressInProgress = 1;

            // usbDeviceAddress = (uint8_t)gSetupRequest.wValue;
            usbDeviceAddress = gSetupRequest.wValue;
            dbgPrintHex("usbDeviceAddress = ", usbDeviceAddress, "\n\r");


            // set USB address
            USB_OTG_FS_DEVICE->DCFG &= ~(USB_OTG_DCFG_DAD);
            USB_OTG_FS_DEVICE->DCFG |= (((uint32_t)usbDeviceAddress) << USB_OTG_DCFG_DAD_Pos);


            USB_OTG_FS_INEP(0U)->DIEPTSIZ = (
                0x1U << USB_OTG_DIEPTSIZ_PKTCNT_Pos |
                0U << USB_OTG_DIEPTSIZ_XFRSIZ_Pos // sending 0 bytes
            );

            // enable the endpoint to send the empty Status Stage packet
            USB_OTG_FS_INEP(0U)->DIEPCTL |= (USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK);



            // mask (disable) all USB interrupts (DEBUG)
            // USB_OTG_FS->GAHBCFG &= ~(USB_OTG_GAHBCFG_GINT);
        }
        else if (gSetupRequest.bRequest == USB_CTRL_REQ_SET_CONFIGURATION)
        {
            debuggerWriteStr(DBG_USART, "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n\r");
            debuggerWriteStr(DBG_USART, "handling Set_Configuration...\n\r");
            debuggerWriteStr(DBG_USART, "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n\r");

            logSetupPacketStruct(&gSetupRequest);
            dbgPrintHex("selected configuration = ", gSetupRequest.wValue, "\n\r");


            debuggerWriteStr(DBG_USART, "enabling Endpoint 1... ");


            // prepare number of bytes to send by endpoint
            // this must be set up before enabling the endpoint
            // USB_OTG_FS_INEP(1U)->DIEPTSIZ = (
            //     0x1U << USB_OTG_DIEPTSIZ_PKTCNT_Pos |
            //     // configuration descriptor byte length
            //     // HID Report size = 2 bytes (uint16_t)
            //     2U << USB_OTG_DIEPTSIZ_XFRSIZ_Pos
            // );


            // enable active endpoints (Endpoint 1 in this case)
            // active endpoints = endpoints selected in the Combined Configuration descriptor
            // 1. Unmask IN interrupts for Endpoint 1
            // USB_OTG_FS_DEVICE->DAINTMSK |= (2U << USB_OTG_DAINTMSK_IEPM_Pos);

            // IN token received when Tx FIFO empty mask
            // USB_OTG_FS_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_ITTXFEMSK;

            // ????
            // USB_OTG_FS_DEVICE->DIEPEMPMSK |= (1U << 1U);
            // USB_OTG_FS_INEP(1)->DIEPINT |= (USB_OTG_DIEPINT_TXFE);

            // 2. set up Tx FIFO for Endpoint 1
            // already done in initUsb()

            // activate Endpoint 1
            USB_OTG_FS_INEP(1U)->DIEPCTL = (
                USB_OTG_DIEPCTL_SNAK |
                (8U << USB_OTG_DIEPCTL_MPSIZ_Pos) | // max packet size = 8
                (1U << USB_OTG_DIEPCTL_USBAEP_Pos) | // USB active endpoint
                (0U << USB_OTG_DIEPCTL_EONUM_DPID_Pos) | // "start data toggle" ??
                (3U << USB_OTG_DIEPCTL_EPTYP_Pos)  | // endpoint type = interrupt
                (1U << USB_OTG_DIEPCTL_TXFNUM_Pos) // Tx FIFO number = 1
            );
            debuggerWriteStr(DBG_USART, "done.\n\r");

            isEp1Configured = 1;
            


            // write HID Report to Endpoint 1 Tx FIFO
            // *ep1FifoPtr = (uint32_t) hidReportBonus;

            

            // send empty Status Packet (Endpoint 0)
            USB_OTG_FS_INEP(0U)->DIEPTSIZ = (
                0x1U << USB_OTG_DIEPTSIZ_PKTCNT_Pos |
                0U << USB_OTG_DIEPTSIZ_XFRSIZ_Pos // sending 0 bytes
            );

            // enable the endpoint to send the empty Status Stage packet
            USB_OTG_FS_INEP(0U)->DIEPCTL |= (USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK);


            // debuggerWriteStr(DBG_USART, "handling Set_Configuration (END)\n\r");
            // debuggerWriteStr(DBG_USART, "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n\r");
        }
        else if (gSetupRequest.bRequest == USB_CTRL_REQ_SET_IDLE_HID &&
            gSetupRequest.bmRequestType == 0x21)
        {
            debuggerWriteStr(DBG_USART, "\n\r-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n\r");
            debuggerWriteStr(DBG_USART, "handling Set_Idle (HID)...\n\r");
            debuggerWriteStr(DBG_USART, "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n\r");

            // send empty Status Packet
            USB_OTG_FS_INEP(0U)->DIEPTSIZ = (
                0x1U << USB_OTG_DIEPTSIZ_PKTCNT_Pos |
                0U << USB_OTG_DIEPTSIZ_XFRSIZ_Pos // sending 0 bytes
            );

            // enable the endpoint to send the empty Status Stage packet
            USB_OTG_FS_INEP(0U)->DIEPCTL |= (USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK);

            debuggerWriteStr(DBG_USART, "handling Set_Idle (HID) - END\n\r");
            debuggerWriteStr(DBG_USART, "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n\r");
        }
        else if (gSetupRequest.bRequest == 0x01 &&
            gSetupRequest.bmRequestType == 0x02)
        {
            debuggerWriteStr(DBG_USART, "\n\r-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n\r");
            debuggerWriteStr(DBG_USART, "handling Clear_Feature() [??]...\n\r");
            debuggerWriteStr(DBG_USART, "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n\r");

            logSetupPacketStruct(&gSetupRequest);

            // send empty IN Status Stage packet
            USB_OTG_FS_INEP(0U)->DIEPTSIZ = (
                0x1U << USB_OTG_DIEPTSIZ_PKTCNT_Pos |
                0U << USB_OTG_DIEPTSIZ_XFRSIZ_Pos // sending 0 bytes
            );


            // enable Endpoint 1 IN
            // USB_OTG_FS_INEP(1U)->DIEPCTL |= (USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK);
            shouldEnableEndpoint1 = 1;


            // enable the endpoint to send the empty Status Stage packet
            USB_OTG_FS_INEP(0U)->DIEPCTL |= (USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK);

            debuggerWriteStr(DBG_USART, "handling Clear_Feature() [??] - END\n\r");
            debuggerWriteStr(DBG_USART, "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n\r");
        }
        else
        {
            debuggerWriteStr(DBG_USART, "\n\r**************************\n\r");
            debuggerWriteStr(DBG_USART, "Unknown USB Request\n\r");
            debuggerWriteStr(DBG_USART, "**************************\n\r");
            // dbgPrintHex("request = ", gSetupRequest.bRequest, "\n\r");

            logSetupPacketStruct(&gSetupRequest);

            // mask (disable) all USB interrupts (DEBUG)
            USB_OTG_FS->GAHBCFG &= ~(USB_OTG_GAHBCFG_GINT);

            
            debuggerWriteStr(DBG_USART, "Unknown USB Request (end)\n\r");
            debuggerWriteStr(DBG_USART, "**************************\n\r");
        }

        // clear STUP in Endpoint 0
        USB_OTG_FS_OUTEP(0U)->DOEPINT &= USB_OTG_DOEPINT_STUP;

        shouldHandleEp0Stup = 0;
    }


    debuggerWriteStr(DBG_USART, "end: handling OEPINT\n\r");
}


// ****************************************************************************
//                     USB OTG IEPINT Handler
// ****************************************************************************
void usbHandleIrqIEPINT(void)
{
    debuggerWriteStr(DBG_USART, "================================\n\r");
    debuggerWriteStr(DBG_USART, "IEPINT Handler (start)\n\r");
    debuggerWriteStr(DBG_USART, "================================\n\r");

    debuggerWriteStr(DBG_USART, "Unmasked interrupts (DIEPMSK): TOM, XFRCM, and EPDM\n\r");

    uint32_t diepintCopy;

    // Endpoint 0 IN interrups
    if ((USB_OTG_FS_DEVICE->DAINT & (1U << 0U)) != 0)
    {
        diepintCopy = USB_OTG_FS_INEP(0U)->DIEPINT;

        debuggerWriteStr(DBG_USART, "addressed endpoint: 0\n\r");

        logUsbDeviceInEndpointPendingInterrupts(diepintCopy, 0);

        if (diepintCopy & USB_OTG_DIEPINT_NAK)
        {
            debuggerWriteStr(DBG_USART, "USB_OTG_DIEPINT_NAK=1\n\r");

            // clear the NAK flag
            USB_OTG_FS_INEP(0U)->DIEPINT &= USB_OTG_DIEPINT_NAK;
        }

        if (diepintCopy & USB_OTG_DIEPINT_XFRC)
        {
            debuggerWriteStr(DBG_USART, "USB_OTG_DIEPINT_XFRC=1\n\r");

            // mask (disable) all USB interrupts (DEBUG)
            // USB_OTG_FS->GAHBCFG &= ~(USB_OTG_GAHBCFG_GINT);

            shouldEnableEndpoint1 = 0;
            if (shouldEnableEndpoint1)
            {
                debuggerWriteStr(DBG_USART, "Enabling IN Endpoint 1 (IEPINT)\n\r");
                USB_OTG_FS_INEP(1U)->DIEPCTL |= (USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK);

                shouldEnableEndpoint1 = 0;
            }
            


            // IMPORTANT
            // this is a wrong approach; it didn't work;
            // what worked was saving new device address from Set_Address request
            // to DCFG_DAD in OEPING (STUP) handler just before sending the empty
            // status packet
            if (isSetAddressInProgress)
            // if (0U)
            {
                debuggerWriteStr(DBG_USART, "setting new USB Device Address\n\r");

                uint32_t expectedDCFG = (USB_OTG_FS_DEVICE->DCFG & ~USB_OTG_DCFG_DAD_Msk) |
                    ((uint32_t)usbDeviceAddress << USB_OTG_DCFG_DAD_Pos);

                dbgPrintHex("DCFG address mask test: ",  expectedDCFG, "\n\r");
                
                // set new Device Address
                USB_OTG_FS_DEVICE->DCFG &= ~(USB_OTG_DCFG_DAD);
                USB_OTG_FS_DEVICE->DCFG |= (((uint32_t)usbDeviceAddress) << USB_OTG_DCFG_DAD_Pos);


                // this won't work, read errata
                // dbgPrintHex("address in DCFG: ", (USB_OTG_FS_DEVICE->DCFG >> 4U) & 0x7FUL, "\n\r");
                // dbgPrintHex("DCFG: ", USB_OTG_FS_DEVICE->DCFG, "\n\r");

                isSetAddressInProgress = 0;
            }

            USB_OTG_FS_OUTEP(0U)->DOEPCTL |= (USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK);

            // clear the XFRC flag
            USB_OTG_FS_INEP(0U)->DIEPINT &= USB_OTG_DIEPINT_XFRC;
        }
    }
    else if ((USB_OTG_FS_DEVICE->DAINT & (2U << 0U)) != 0)
    {
        debuggerWriteStr(DBG_USART, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\r");
        debuggerWriteStr(DBG_USART, "        Access to Endpoint 1 (IEPINT)\n\r");
        debuggerWriteStr(DBG_USART, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\r");
        
        diepintCopy = USB_OTG_FS_INEP(0U)->DIEPINT;

        logUsbDeviceInEndpointPendingInterrupts(diepintCopy, 0);

        if (diepintCopy & USB_OTG_DIEPINT_INEPNE)
        {
            // debuggerWriteStr(DBG_USART, "USB_OTG_DIEPINT_NAK=1\n\r");

            // clear the flag
            USB_OTG_FS_INEP(1U)->DIEPINT &= USB_OTG_DIEPINT_INEPNE;
        }

        if (diepintCopy & USB_OTG_DIEPINT_TXFE)
        {
            // debuggerWriteStr(DBG_USART, "USB_OTG_DIEPINT_NAK=1\n\r");

            // clear the flag
            USB_OTG_FS_INEP(1U)->DIEPINT &= USB_OTG_DIEPINT_TXFE;
        }

    }
    else
    {
        debuggerWriteStr(DBG_USART, "addressed endpoint other than 0 or 1\n\r");
    }

    debuggerWriteStr(DBG_USART, "IEPINT Handler (end)\n\r");
    debuggerWriteStr(DBG_USART, "================================\n\r");
}


// ****************************************************************************
//                     USB OTG SOF Handler
// ****************************************************************************
void usbHandleIrqSOF(void)
{
    // read frame number from DSTS_FNSOF[13:0]
    // uint16_t usbFrameNumber = (USB_OTG_FS_DEVICE->DSTS >> USB_OTG_DSTS_FNSOF_Pos) & 0x3FFFUL;

    // clear the SOF interrupt flag
    USB_OTG_FS->GINTSTS &= USB_OTG_GINTSTS_SOF;
}