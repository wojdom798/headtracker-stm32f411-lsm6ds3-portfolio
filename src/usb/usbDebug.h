#ifndef _STM32F411_USB_DEBUG_H
#define _STM32F411_USB_DEBUG_H


#define USB_DBG_FLG__ENUMSPD_LOGGED (0x00000001U)
#define USB_DBG_FLG__RXFLVL_LOGGED  (0x00000002U)
// this flag indicates that the debug address function should be called
// the flag should be set e.g. after a button press
#define USB_DBG_FLG__LOG_ADDR_ACTIVE  (0x00000004U)


// for the "logUsbInterruptFlagsInTriggerOrder" function
#define INTERRUPT_FLAGS_ARRAY_SIZE (30U)

#define USB_IRQ_RXFLVL       (1U << 0U)
#define USB_IRQ_USBRST       (1U << 1U)
#define USB_IRQ_ENUMDNE      (1U << 2U)
#define USB_IRQ_USBSUSP      (1U << 3U)
#define USB_IRQ_IEPINT       (1U << 4U)
#define USB_IRQ_OEPINT       (1U << 5U)
#define USB_IRQ_IISOIXFR     (1U << 6U)
#define USB_IRQ_INCOMPISOOUT (1U << 7U)
#define USB_IRQ_WKUINT       (1U << 8U)
#define USB_IRQ_SOF          (1U << 9U)



void logUsbInterruptStatus(void);
void dbgLogUsbRegisterAddresses(void);

void logUsbInterruptFlagsInTriggerOrder(void);
void logUsbDeviceOutEndpointPendingInterrupts(uint32_t doepintxCopy, uint8_t epNum);
void logSetupPacketStruct(SetupRequest* setupReq);
void logUsbDeviceInEndpointPendingInterrupts(uint32_t diepintxCopy, uint8_t epNum);

void logGusbcfgRegisterValue(USB_OTG_GlobalTypeDef* usbGlobalHandle);

#endif