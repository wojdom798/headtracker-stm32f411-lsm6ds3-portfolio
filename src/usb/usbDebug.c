#include <stdint.h>
#include "usb.h"
#include "usbDebug.h"
#include "makeshiftDebugger.h"


// ********************************************************************************
//                  extern variables
// ********************************************************************************
// USART
extern char intToAsciiBuffer[INT_TO_ASCII_BUFFER_SIZE];
// ********************************************************************************
//                end: extern variables
// ********************************************************************************


// uint32_t usbDebugFlags = 0x00000000U;
// disable DSTS logs on first USB IRQ Handler call
uint32_t usbDebugFlags = (0x00000000U | USB_DBG_FLG__ENUMSPD_LOGGED);

uint16_t interruptFlags[INTERRUPT_FLAGS_ARRAY_SIZE];
uint8_t usbIrqCount = 0;

uint8_t isLogOrderOfUsbInterruptsActive = 0;



void logUsbInterruptStatus(void)
{
    uint32_t gintstsCopy = USB_OTG_FS->GINTSTS;

    if (gintstsCopy & USB_OTG_GINTSTS_RXFLVL)
    {
        debuggerWriteStr(DBG_USART, "RXFLVL       = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "RXFLVL       = 0\n\r");
    }
    
    if (gintstsCopy & USB_OTG_GINTSTS_USBRST)
    {
        debuggerWriteStr(DBG_USART, "USBRST       = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "USBRST       = 0\n\r");
    }
    
    if (gintstsCopy & USB_OTG_GINTSTS_ENUMDNE)
    {
        debuggerWriteStr(DBG_USART, "ENUMDNE      = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "ENUMDNE      = 0\n\r");
    }
    
    if (gintstsCopy & USB_OTG_GINTSTS_IEPINT)
    {
        debuggerWriteStr(DBG_USART, "IEPINT       = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "IEPINT       = 0\n\r");
    }
    
    if (gintstsCopy & USB_OTG_GINTSTS_OEPINT)
    {
        debuggerWriteStr(DBG_USART, "OEPINT       = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "OEPINT       = 0\n\r");
    }
    
    if (gintstsCopy & USB_OTG_GINTSTS_IISOIXFR)
    {
        debuggerWriteStr(DBG_USART, "IISOIXFR     = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "IISOIXFR     = 0\n\r");
    }
    
    if (gintstsCopy & USB_OTG_GINTSTS_PXFR_INCOMPISOOUT)
    {
        debuggerWriteStr(DBG_USART, "INCOMPISOOUT = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "INCOMPISOOUT = 0\n\r");
    }
    
    if (gintstsCopy & USB_OTG_GINTSTS_WKUINT)
    {
        debuggerWriteStr(DBG_USART, "WKUINT       = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "WKUINT       = 0\n\r");
    }

    if (gintstsCopy & USB_OTG_GINTSTS_USBSUSP)
    {
        debuggerWriteStr(DBG_USART, "USBSUSP      = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "USBSUSP      = 0\n\r");
    }

    if (gintstsCopy & USB_OTG_GINTSTS_SOF)
    {
        debuggerWriteStr(DBG_USART, "SOF          = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "SOF          = 0\n\r");
    }
}


void dbgLogUsbRegisterAddresses(void)
{
    uint32_t regAddress;

    debuggerWriteStr(DBG_USART, "Selected USB Register addresses (debug)\n\r");

    // GPIO Registers
    regAddress = (uint32_t) &(GPIOC->OTYPER);
    dbgPrintHex("GPIOC->OTYPER:                ", regAddress, "\n\r\n\r");

    // USB Global Registers
    regAddress = (uint32_t) &(USB_OTG_FS->GINTSTS);
    dbgPrintHex("USB_OTG_FS->GINTSTS:          ", regAddress, "\n\r\n\r");

    // USB Device Registers
    regAddress = (uint32_t) &(USB_OTG_FS_DEVICE->DCTL);
    dbgPrintHex("USB_OTG_FS_DEVICE->DCTL:      ", regAddress, "\n\r");

    regAddress = (uint32_t) &(USB_OTG_FS_DEVICE->DAINTMSK);
    dbgPrintHex("USB_OTG_FS_DEVICE->DAINTMSK:  ", regAddress, "\n\r\n\r");
    
    // USB Endpoint 0 Registers
    regAddress = (uint32_t) &(USB_OTG_FS_INEP(0)->DIEPTSIZ);
    dbgPrintHex("USB_OTG_FS_INEP(0)->DIEPTSIZ: ", regAddress, "\n\r");
    
    regAddress = (uint32_t) &(USB_OTG_FS_OUTEP(0)->DOEPINT);
    dbgPrintHex("USB_OTG_FS_OUTEP(0)->DOEPINT: ", regAddress, "\n\r\n\r");


    // USB Endpoint 1 Registers
    regAddress = (uint32_t) &(USB_OTG_FS_INEP(1)->DIEPTSIZ);
    dbgPrintHex("USB_OTG_FS_INEP(1)->DIEPTSIZ: ", regAddress, "\n\r");
    
    regAddress = (uint32_t) &(USB_OTG_FS_OUTEP(1)->DOEPINT);
    dbgPrintHex("USB_OTG_FS_OUTEP(1)->DOEPINT: ", regAddress, "\n\r\n\r");


    // USB Endpoint 2 Registers
    regAddress = (uint32_t) &(USB_OTG_FS_INEP(2)->DIEPTSIZ);
    dbgPrintHex("USB_OTG_FS_INEP(2)->DIEPTSIZ: ", regAddress, "\n\r");
    
    regAddress = (uint32_t) &(USB_OTG_FS_OUTEP(2)->DOEPINT);
    dbgPrintHex("USB_OTG_FS_OUTEP(2)->DOEPINT: ", regAddress, "\n\r\n\r");


    // USB Endpoint 3 Registers
    regAddress = (uint32_t) &(USB_OTG_FS_INEP(3)->DIEPTSIZ);
    dbgPrintHex("USB_OTG_FS_INEP(3)->DIEPTSIZ: ", regAddress, "\n\r");
    
    regAddress = (uint32_t) &(USB_OTG_FS_OUTEP(3)->DOEPINT);
    dbgPrintHex("USB_OTG_FS_OUTEP(3)->DOEPINT: ", regAddress, "\n\r\n\r");
    

    regAddress = (uint32_t) &(USB_OTG_FS_PCGCCTL->PCGCCTL);
    dbgPrintHex("USB_OTG_FS_PCGCCTL->PCGCCTL:  ", regAddress, "\n\r");


    debuggerWriteStr(DBG_USART, "\n\r");
}


void logUsbInterruptFlagsInTriggerOrder(void)
{

    // for (uint32_t i = 0; i < INTERRUPT_FLAGS_ARRAY_SIZE; i++)
    for (uint32_t i = 0; i < usbIrqCount; i++)
    {
        dbgPrintDec("OTG_FS_IRQHandler call #", i+1, "\n\r");

        if (interruptFlags[i] & USB_IRQ_RXFLVL)
        {
            debuggerWriteStr(DBG_USART, "RXFLVL       = 1 !\n\r");
        }
        else
        {
            debuggerWriteStr(DBG_USART, "RXFLVL       = 0\n\r");
        }
        
        if (interruptFlags[i] & USB_IRQ_USBRST)
        {
            debuggerWriteStr(DBG_USART, "USBRST       = 1 !\n\r");
        }
        else
        {
            debuggerWriteStr(DBG_USART, "USBRST       = 0\n\r");
        }
        
        if (interruptFlags[i] & USB_IRQ_ENUMDNE)
        {
            debuggerWriteStr(DBG_USART, "ENUMDNE      = 1 !\n\r");
        }
        else
        {
            debuggerWriteStr(DBG_USART, "ENUMDNE      = 0\n\r");
        }
        
        if (interruptFlags[i] & USB_IRQ_USBSUSP)
        {
            debuggerWriteStr(DBG_USART, "USBSUSP      = 1 !\n\r");
        }
        else
        {
            debuggerWriteStr(DBG_USART, "USBSUSP      = 0\n\r");
        }
        
        if (interruptFlags[i] & USB_IRQ_IEPINT)
        {
            debuggerWriteStr(DBG_USART, "IEPINT       = 1 !\n\r");
        }
        else
        {
            debuggerWriteStr(DBG_USART, "IEPINT       = 0\n\r");
        }
        
        if (interruptFlags[i] & USB_IRQ_OEPINT)
        {
            debuggerWriteStr(DBG_USART, "OEPINT       = 1 !\n\r");
        }
        else
        {
            debuggerWriteStr(DBG_USART, "OEPINT       = 0\n\r");
        }
        
        if (interruptFlags[i] & USB_IRQ_IISOIXFR)
        {
            debuggerWriteStr(DBG_USART, "IISOIXFR     = 1 !\n\r");
        }
        else
        {
            debuggerWriteStr(DBG_USART, "IISOIXFR     = 0\n\r");
        }
        
        if (interruptFlags[i] & USB_IRQ_INCOMPISOOUT)
        {
            debuggerWriteStr(DBG_USART, "INCOMPISOOUT = 1 !\n\r");
        }
        else
        {
            debuggerWriteStr(DBG_USART, "INCOMPISOOUT = 0\n\r");
        }
        
        if (interruptFlags[i] & USB_IRQ_WKUINT)
        {
            debuggerWriteStr(DBG_USART, "WKUINT       = 1 !\n\r");
        }
        else
        {
            debuggerWriteStr(DBG_USART, "WKUINT       = 0\n\r");
        }
        
        if (interruptFlags[i] & USB_IRQ_SOF)
        {
            debuggerWriteStr(DBG_USART, "SOF          = 1 !\n\r");
        }
        else
        {
            debuggerWriteStr(DBG_USART, "SOF          = 0\n\r");
        }

        debuggerWriteStr(DBG_USART, "\n\r");
    }

    debuggerWriteStr(DBG_USART, "\n\r");
}


void logUsbDeviceOutEndpointPendingInterrupts(uint32_t doepintxCopy, uint8_t epNum)
{
    dbgPrintDec("DOEPINT(", epNum, ") flags:\n\r");
    
    if (doepintxCopy & USB_OTG_DOEPINT_XFRC)
    {
        debuggerWriteStr(DBG_USART, "XFRC          = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "XFRC          = 0\n\r");
    }

    if (doepintxCopy & USB_OTG_DOEPINT_EPDISD)
    {
        debuggerWriteStr(DBG_USART, "EPDISD        = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "EPDISD        = 0\n\r");
    }

    if (doepintxCopy & USB_OTG_DOEPINT_STUP)
    {
        debuggerWriteStr(DBG_USART, "STUP          = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "STUP          = 0\n\r");
    }

    if (doepintxCopy & USB_OTG_DOEPINT_OTEPDIS)
    {
        debuggerWriteStr(DBG_USART, "OTEPDIS       = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "OTEPDIS       = 0\n\r");
    }

    // bit #5
    // "RM0383 Rev 3": STSPHSRX
    // "stm32f411xe.h": OTEPSPR
    if (doepintxCopy & USB_OTG_DOEPINT_OTEPSPR)
    {
        debuggerWriteStr(DBG_USART, "STSPHSRX      = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "STSPHSRX      = 0\n\r");
    }

    if (doepintxCopy & USB_OTG_DOEPINT_OUTPKTERR)
    {
        debuggerWriteStr(DBG_USART, "OUTPKTERR     = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "OUTPKTERR     = 0\n\r");
    }

    // BERR - not defined in stm32f411xe.h
    if (doepintxCopy & (1U << 12U))
    {
        debuggerWriteStr(DBG_USART, "BERR          = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "BERR          = 0\n\r");
    }

    if (doepintxCopy & USB_OTG_DOEPINT_NAK)
    {
        debuggerWriteStr(DBG_USART, "NAK           = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "NAK           = 0\n\r");
    }
    
    debuggerWriteStr(DBG_USART, "\n\r");
}


void logUsbDeviceInEndpointPendingInterrupts(uint32_t diepintxCopy, uint8_t epNum)
{
    dbgPrintDec("DIEPINT(", epNum, ") flags:\n\r");
    
    if (diepintxCopy & USB_OTG_DIEPINT_XFRC)
    {
        debuggerWriteStr(DBG_USART, "XFRC          = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "XFRC          = 0\n\r");
    }

    if (diepintxCopy & USB_OTG_DIEPINT_EPDISD)
    {
        debuggerWriteStr(DBG_USART, "EPDISD        = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "EPDISD        = 0\n\r");
    }

    // no mention of this bit in reference manual
    // if (diepintxCopy & USB_OTG_DIEPINT_AHBERR)
    // {
    //     debuggerWriteStr(DBG_USART, "AHBERR        = 1\n\r");
    // }
    // else
    // {
    //     debuggerWriteStr(DBG_USART, "AHBERR        = 0\n\r");
    // }

    if (diepintxCopy & USB_OTG_DIEPINT_TOC)
    {
        debuggerWriteStr(DBG_USART, "TOC           = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "TOC           = 0\n\r");
    }

    if (diepintxCopy & USB_OTG_DIEPINT_ITTXFE)
    {
        debuggerWriteStr(DBG_USART, "ITTXFE        = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "ITTXFE        = 0\n\r");
    }

    if (diepintxCopy & USB_OTG_DIEPINT_INEPNM)
    {
        debuggerWriteStr(DBG_USART, "INEPNM        = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "INEPNM        = 0\n\r");
    }

    if (diepintxCopy & USB_OTG_DIEPINT_INEPNE)
    {
        debuggerWriteStr(DBG_USART, "INEPNE        = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "INEPNE        = 0\n\r");
    }

    if (diepintxCopy & USB_OTG_DIEPINT_TXFE)
    {
        debuggerWriteStr(DBG_USART, "TXFE          = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "TXFE          = 0\n\r");
    }

    // bit 8, no mention of this bit in reference manual
    // if (diepintxCopy & USB_OTG_DIEPINT_TXFIFOUDRN)
    // {
    //     debuggerWriteStr(DBG_USART, "TXFIFOUDRN    = 1\n\r");
    // }
    // else
    // {
    //     debuggerWriteStr(DBG_USART, "TXFIFOUDRN    = 0\n\r");
    // }

    // bit 9, no mention of this bit in reference manual
    // if (diepintxCopy & USB_OTG_DIEPINT_BNA)
    // {
    //     debuggerWriteStr(DBG_USART, "BNA           = 1\n\r");
    // }
    // else
    // {
    //     debuggerWriteStr(DBG_USART, "BNA           = 0\n\r");
    // }

    if (diepintxCopy & USB_OTG_DIEPINT_PKTDRPSTS)
    {
        debuggerWriteStr(DBG_USART, "PKTDRPSTS     = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "PKTDRPSTS     = 0\n\r");
    }

    if (diepintxCopy & USB_OTG_DIEPINT_BERR)
    {
        debuggerWriteStr(DBG_USART, "BERR          = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "BERR          = 0\n\r");
    }

    if (diepintxCopy & USB_OTG_DIEPINT_NAK)
    {
        debuggerWriteStr(DBG_USART, "NAK           = 1\n\r");
    }
    else
    {
        debuggerWriteStr(DBG_USART, "NAK           = 0\n\r");
    }
    
    debuggerWriteStr(DBG_USART, "\n\r");
}


void logSetupPacketStruct(SetupRequest* setupReq)
{
    debuggerWriteStr(DBG_USART, "Setup Packet:\n\r");

    dbgPrintHex("bmRequestType = ", setupReq->bmRequestType, "\n\r");
    dbgPrintHex("bRequest      = ", setupReq->bRequest, "\n\r");
    dbgPrintHex("wValue        = ", setupReq->wValue, "\n\r");
    dbgPrintHex("wIndex        = ", setupReq->wIndex, "\n\r");
    dbgPrintHex("wLength       = ", setupReq->wLength, "\n\r");

    debuggerWriteStr(DBG_USART, "\n\r");
}


// this is a debug/test function, mainly to see the reset value of GUSBCFG
// but also to see if PHYSEL is set to 1 as the reference manual says
void logGusbcfgRegisterValue(USB_OTG_GlobalTypeDef* usbGlobalHandle)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN; // enable USB Periph clock

    uint32_t gusbcfgCopy = usbGlobalHandle->GUSBCFG;
    // gusbcfgCopy = 0x00001440;
    // gusbcfgCopy = 0x12345678;

    debuggerPrintValue(
        intToAsciiBuffer,
        INT_TO_ASCII_BUFFER_SIZE,
        0x2001,
        "GUSBCFG: ",
        "\n\r",
        gusbcfgCopy
    );

    usbGlobalHandle->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

    gusbcfgCopy = usbGlobalHandle->GUSBCFG;

    debuggerPrintValue(
        intToAsciiBuffer,
        INT_TO_ASCII_BUFFER_SIZE,
        0x2001,
        "GUSBCFG (after PHYSEL=1): ",
        "\n\r",
        gusbcfgCopy
    );
}