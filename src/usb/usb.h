#ifndef USB_H
#define USB_H

#include <stdint.h>
#include "stm32f411xe.h"

typedef struct
{
  __IO uint32_t PCGCCTL;   /*!< USB_OTG power and clock gating control register   000h */
} USB_OTG_PcgTypeDef;


typedef struct SetupRequest
{
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} SetupRequest;


#define USB_OTG_FS_DEVICE ((USB_OTG_DeviceTypeDef *) \
    (USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))

/*
#define USB_OTG_FS_INEP ((USB_OTG_INEndpointTypeDef *) \
    (USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE))
*/

/*
#define USB_OTG_FS_OUTEP ((USB_OTG_OUTEndpointTypeDef *) \
    (USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE))
*/

#define USB_OTG_FS_HOST ((USB_OTG_HostTypeDef *) \
    (USB_OTG_FS_PERIPH_BASE + USB_OTG_HOST_BASE))

#define USB_OTG_FS_HOST_CHNL ((USB_OTG_HostChannelTypeDef *) \
    (USB_OTG_FS_PERIPH_BASE + USB_OTG_HOST_CHANNEL_BASE))

#define USB_OTG_FS_PCGCCTL ((USB_OTG_PcgTypeDef *) \
    (USB_OTG_FS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE))


#define USB_OTG_FS_INEP(n) ((USB_OTG_INEndpointTypeDef *) \
    (USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + ((n) * USB_OTG_EP_REG_SIZE)))

#define USB_OTG_FS_OUTEP(n) ((USB_OTG_OUTEndpointTypeDef *) \
    (USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + ((n) * USB_OTG_EP_REG_SIZE)))


// stm32f411 has 4 endpoints; Ep[0,1,2,3]
#define USB_OTG_FS_ENDPOINT_COUNT (4U)



#define USB_DP_GPIOEN (RCC_AHB1ENR_GPIOAEN)
#define USB_DP_GPIO (GPIOA)
#define USB_DP_PIN (12)
#define USB_DP_AF_NUM (10) // alternate function number
// #define USB_DP_AF_NUM (11U) // DEBUG

#define USB_DM_GPIOEN (RCC_AHB1ENR_GPIOAEN)
#define USB_DM_GPIO (GPIOA)
#define USB_DM_PIN (11)
#define USB_DM_AF_NUM (10) // alternate function number
// #define USB_DM_AF_NUM (11U) // DEBUG




// ****************************************************************************
//                     GRXSTSR/GRXSTSP Field Definitions
// ****************************************************************************
// Packet status
#define GRXSTS_GLOBAL_OUT_NAK              (1U)
#define GRXSTS_OUT_DATA_PACKET_RX          (2U)
#define GRXSTS_OUT_TRANSFER_COMPLETED      (3U)
#define GRXSTS_SETUP_TRANSACTION_COMPLETED (4U)
#define GRXSTS_SETUP_DATA_PACKET_RX        (6U)

// Data PID
#define GRXSTS_DATA0 (0U)
#define GRXSTS_DATA1 (1U)
#define GRXSTS_DATA2 (2U)
#define GRXSTS_MDATA (3U)


#define GRXSTS_GET_EPNUM(regCopy)  ((regCopy >> USB_OTG_GRXSTSP_EPNUM_Pos) & 0x0FUL)
#define GRXSTS_GET_BCNT(regCopy)   ((regCopy >> USB_OTG_GRXSTSP_BCNT_Pos) & 0x7FFUL)
#define GRXSTS_GET_DPID(regCopy)   ((regCopy >> USB_OTG_GRXSTSP_DPID_Pos) & 0x3UL)
#define GRXSTS_GET_PKTSTS(regCopy) ((regCopy >> USB_OTG_GRXSTSP_PKTSTS_Pos) & 0xFUL)
#define GRXSTS_GET_FRMNUM(regCopy) ((regCopy >> USB_OTG_FRMNUM_Pos) & 0xFUL)
// ****************************************************************************
//                    end: GRXSTSR/GRXSTSP Field Definitions
// ****************************************************************************

// ****************************************************************************
//                     USB Control Transfer Defines
// ****************************************************************************
// bRequest
#define USB_CTRL_REQ_GET_DESCRIPTOR     (6U)
#define USB_CTRL_REQ_SET_ADDRESS        (5U)
#define USB_CTRL_REQ_SET_CONFIGURATION  (9U)
#define USB_CTRL_REQ_SET_IDLE_HID       (0x0AU)


// descriptor types
#define USB_DEVICE_DESCRIPTOR_TYPE_ID        (1U)
#define USB_CONFIGURATION_DESCRIPTOR_TYPE_ID (2U)
#define USB_STRING_DESCRIPTOR_TYPE_ID        (3U)
#define USB_INTERFACE_DESCRIPTOR_TYPE_ID     (4U)
#define USB_ENDPOINT_DESCRIPTOR_TYPE_ID      (5U)
#define USB_HID_REPORT_DESCRIPTOR_TYPE_ID    (0x22U)


#define USB_DESCR_TYPE_FROM_STRUCT(setupRequestStruct) (\
    (uint8_t) ((setupRequestStruct.wValue) >> 8U))
// ****************************************************************************
//                     USB Control Transfer Defines
// ****************************************************************************


// Endpoint 0 FIFO
#define EP0_FIFO_ADDR (USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE)
// Endpoint 1 FIFO
#define EP1_FIFO_ADDR (USB_OTG_FS_PERIPH_BASE + (USB_OTG_FIFO_BASE + (1U * USB_OTG_FIFO_SIZE)))




void enableUSBClock(RCC_TypeDef* rccHandle);
void initUsb(
    USB_OTG_GlobalTypeDef* usbGlobalHandle,
    USB_OTG_DeviceTypeDef* usbDeviceHandle,
    USB_OTG_PcgTypeDef* usbPcgcctlHandle
);
void usbConfigureGPIOs(void);
void usbEnableInterrupts(uint8_t priority);

#endif