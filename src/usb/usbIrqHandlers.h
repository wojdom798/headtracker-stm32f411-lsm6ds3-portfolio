#ifndef _STM32F411_USB_IRQ_HANDLERS_H
#define _STM32F411_USB_IRQ_HANDLERS_H


void usbHandleIrqUSBRST(void);
void usbHandleIrqENUMDNE(void);
void usbHandleIrqRXFLVL(void);
void usbHandleIrqOEPINT(void);
void usbHandleIrqIEPINT(void);
void usbHandleIrqSOF(void);


#endif