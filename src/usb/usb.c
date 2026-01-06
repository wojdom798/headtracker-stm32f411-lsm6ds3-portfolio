#include "usb.h"


void enableUSBClock(RCC_TypeDef* rccHandle)
{
    // reset USB clock
    rccHandle->AHB2RSTR |= RCC_AHB2RSTR_OTGFSRST;
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");

    // deassert USB clock reset
    rccHandle->AHB2RSTR &= ~(RCC_AHB2RSTR_OTGFSRST);
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");

    // enable clock for the USB peripheral (USB Core)
    rccHandle->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
}


void initUsb(
    USB_OTG_GlobalTypeDef* usbGlobalHandle,
    USB_OTG_DeviceTypeDef* usbDeviceHandle,
    USB_OTG_PcgTypeDef* usbPcgcctlHandle)
{
    // disable global USB interrupts
    usbGlobalHandle->GAHBCFG &= ~(USB_OTG_GAHBCFG_GINT);

    // full speed serial transceiver select
    // RM0383 says that this bit is always 1; HAL library sets this bit to 1 anyway
    usbGlobalHandle->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;


    // perform USB Core soft reset
    // 1. wait for AHB master to get into idle condition
    while ((usbGlobalHandle->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U)
    {
        continue;
    }
    // 2. soft reset
    usbGlobalHandle->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
    // 3. wait for USB Core to clear soft reset bit
    while ((usbGlobalHandle->GRSTCTL & USB_OTG_GRSTCTL_CSRST) != 0U)
    {
        continue;
    }


    // deactivate power down (activate USB transceiver)
    // this is executed by HAL when "cfg.battery_charging_enable" = 0
    usbGlobalHandle->GCCFG |= USB_OTG_GCCFG_PWRDWN;

    // activate power down (deactivate USB transceiver)
    // this is executed by HAL when "cfg.battery_charging_enable" != 0
    // usbGlobalHandle->GCCFG &= ~(USB_OTG_GCCFG_PWRDWN);



    // these bits are missing in the "RM0383 Rev 3" reference manual
    // HAL library sets those bits when "dma_enable" is set to 1
    // I don't think that stm32f411 supports DMA for USB,
    // so I'm gonna leave these 2 lines commented out
    // usbGlobalHandle->GAHBCFG |= USB_OTG_GAHBCFG_HBSTLEN_2;
    // usbGlobalHandle->GAHBCFG |= USB_OTG_GAHBCFG_DMAEN;


    // reset force device mode and force host mode bits
    usbGlobalHandle->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD);
    // set force device mode
    usbGlobalHandle->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
    
    // wait for mode change from Host to Device
    // while (usbGlobalHandle->GINTSTS & USB_OTG_GINTSTS_CMOD)
    // {
    //     continue;
    // }
    // reference manual says it could take 25 ms to change mode
    for (uint32_t i = 0; i < 200000UL; i++)
    {
        continue;
    }


    // reset OTG_FS device IN endpoint transmit FIFO size registers
    usbGlobalHandle->DIEPTXF[0] = 0U; // IN Endpoint 1
    usbGlobalHandle->DIEPTXF[1] = 0U; // IN Endpoint 2
    usbGlobalHandle->DIEPTXF[2] = 0U; // IN Endpoint 3


    // perform a soft disconnect
    // the USB host doesn't see the device connected as long as this bit is set
    // it's reset to 0 later on
    usbDeviceHandle->DCTL |= USB_OTG_DCTL_SDIS;
    // disable V_bus sensing
    usbGlobalHandle->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
    // disable V_bus sensing "B"
    usbGlobalHandle->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
    // disable V_bus sensing "A"
    usbGlobalHandle->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;


    // reset PCGCCTL; stop the PHY clock (??)
    // HAL comment: "Restart the Phy Clock"
    usbPcgcctlHandle->PCGCCTL = 0;
    // usbPcgcctlHandle->PCGCCTL = 0x03; // this will corrupt all interrupts


    // set device speed to Full Speed
    // other values (speed settings) than 0b11 are reserved
    usbDeviceHandle->DCFG |= (0x3U << USB_OTG_DCFG_DSPD_Pos);


    // flush Tx FIFOs
    // 1. wait for AHB master to get into idle condition
    while ((usbGlobalHandle->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U)
    {
        continue;
    }
    
    // 2. select Tx FIFO number to be flushed
    // 2.1. reset FIFO number
    // usbGlobalHandle->GRSTCTL &= ~(USB_OTG_GRSTCTL_TXFNUM);
    // // 2.2. set FIFO number to 0b10000 (flush all Tx FIFOs)
    // usbGlobalHandle->GRSTCTL |= (0x10U << USB_OTG_GRSTCTL_TXFNUM_Pos);
    
    // // 3. perform FIFO flush
    // usbGlobalHandle->GRSTCTL |= USB_OTG_GRSTCTL_TXFFLSH;

    // Perform FIFO flush in one go (from HAL)
    usbGlobalHandle->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH | (0x10U << USB_OTG_GRSTCTL_TXFNUM_Pos));
    
    // 4. wait for Tx FIFO flush to take effect
    while ((usbGlobalHandle->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) != 0U)
    {
        continue;
    }


    // flush Rx FIFO
    // 1. wait for AHB master to get into idle condition
    while ((usbGlobalHandle->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U)
    {
        continue;
    }

    // 2. perform FIFO flush
    // usbGlobalHandle->GRSTCTL |= USB_OTG_GRSTCTL_RXFFLSH;
    usbGlobalHandle->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH; // HAL

    // 3. wait for Rx FIFO flush to take effect
    while ((usbGlobalHandle->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) != 0U)
    {
        continue;
    }


    // Clear all pending Device interrupts
    usbDeviceHandle->DIEPMSK = 0U;
    usbDeviceHandle->DOEPMSK = 0U;
    usbDeviceHandle->DAINTMSK = 0U;

    
    uint32_t i;

    // Reset IN Endpoint registers
    for (i = 0U; i < USB_OTG_FS_ENDPOINT_COUNT; i++)
    {
        // if ((USB_OTG_FS_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) != 0U)
        if ((USB_OTG_FS_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA)
        {
            // check if Endpoint 0
            if (i == 0U)
            {
                // set NAK
                USB_OTG_FS_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_SNAK;
            }
            else
            {
                // for any endpoint other than endpoint 0
                // set NAK and disable the endpoint (EPDIS)
                USB_OTG_FS_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
            }
        }
        else // if endpoint is disabled
        {
            USB_OTG_FS_INEP(i)->DIEPCTL = 0U;
        }

        USB_OTG_FS_INEP(i)->DIEPTSIZ = 0U;

        // clear all bits in DIEPINT (?)
        // most of those bits are rc_w1 (read, cleared by writing 1)
        USB_OTG_FS_INEP(i)->DIEPINT = 0xFB7FU; // mask from HAL
        // 0b 0000 0000 0000 0000 0010 1000 0111 1011 = 0x 0000 287B
        // USB_OTG_FS_INEP(i)->DIEPINT = 0x0000287BU; // my mask (based on reference manual)
    }

    // Reset OUT Endpoint registers
    for (i = 0U; i < USB_OTG_FS_ENDPOINT_COUNT; i++)
    {
        // if ((USB_OTG_FS_OUTEP(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) != 0U)
        if ((USB_OTG_FS_OUTEP(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA)
        {
            // check if Endpoint 0
            if (i == 0U)
            {
                // set NAK
                USB_OTG_FS_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_SNAK;
            }
            else
            {
                // for any other endpoint than endpoint 0
                // set NAK and disable the endpoint (EPDIS)
                USB_OTG_FS_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
            }
        }
        else // if endpoint is disabled
        {
            USB_OTG_FS_OUTEP(i)->DOEPCTL = 0U;
        }

        USB_OTG_FS_OUTEP(i)->DOEPTSIZ = 0U;

        // clear all bits in DOEPINT (?)
        // most of those bits are rc_w1 (read, cleared by writing 1)
        USB_OTG_FS_OUTEP(i)->DOEPINT = 0xFB7FU; // mask from HAL
        // 0b 0000 0000 0000 0000 0011 0001 0011 1011 = 0x 0000 313B
        // USB_OTG_FS_OUTEP(i)->DOEPINT = 0x0000313BU; // my mask
    }

    // there's no DIEPMSK_TXFURM bit in RM0383 Rev 3
    // but this line is still present in the HAL library...
    // found description for this bit in "RM0385 Rev 8" (STM32F75xx)
    usbDeviceHandle->DIEPMSK &= ~(USB_OTG_DIEPMSK_TXFURM);


    // disable all interrupts
    usbGlobalHandle->GINTMSK = 0U;

    // clear any pending interrupts
    usbGlobalHandle->GINTSTS = 0xBFFFFFFFU; // this mask is from HAL
    // 0b 1011 0000 0011 0000 1111 1100 0000 1010 = 0x B030 FC0A
    // usbGlobalHandle->GINTSTS = 0xB030FC0AU; // this is my mask based on RM0383 Rev 3

    // unmask the Rx FIFO non-empty interrupt
    usbGlobalHandle->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;

    // unmask USB Device-related interrupts
    // USBSUSPM (bit 11)
    // USBRST (bit 12)
    // ENUMDNEM (bit 13)
    // IEPINT (bit 18)
    // OEPINT (bit 19)
    // IISOIXFRM (bit 20)
    // IISOOXFRM (bit 21)
    // WUIM (bit 31)
    usbGlobalHandle->GINTMSK |=
        USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST |
        USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |
        USB_OTG_GINTMSK_OEPINT   | USB_OTG_GINTMSK_IISOIXFRM |
        USB_OTG_GINTMSK_PXFRM_IISOOXFRM | USB_OTG_GINTMSK_WUIM;


    // unmask SOFM if SOF is enabled (start of frame)
    // I'm not sure if I need this one
    // usbGlobalHandle->GINTMSK |= USB_OTG_GINTMSK_SOFM;

    // unmask SRQIM and OTGINT if V_bus sensing is enabled
    // SRQIM = session request/new session detected interrupt
    // OTGINT = OTG interrupt
    // usbGlobalHandle->GINTMSK |= (USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT);


    // ungate and restore the phy CLK
    usbPcgcctlHandle->PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);

    // perform a soft disconnect
    usbDeviceHandle->DCTL |= USB_OTG_DCTL_SDIS;


    /*
        Set Rx FIFO size
        It's the easier one, since Rx FIFO is fixed at FIFO address 0
    */
    // usbGlobalHandle->GRXFSIZ &= ~(USB_OTG_GRXFSIZ_RXFD);
    // // set Rx FIFO size to 0x80 (=128 32-bit words)
    // usbGlobalHandle->GRXFSIZ |= (0x80 << USB_OTG_GRXFSIZ_RXFD_Pos);

    // this is how HAL sets RX FIFO
    usbGlobalHandle->GRXFSIZ = 0x80;



    /*
        Set Tx FIFO size
        This is a bit more difficult because each Tx FIFO (other than EP0) must take into account
        the size of all the Tx FIFOs that were configured before them
    */
    // Tx FIFO size for Endpoint 0
    uint32_t txFifoOffset = usbGlobalHandle->GRXFSIZ;
    // usbGlobalHandle->DIEPTXF0_HNPTXFSIZ = 0x00000000; // reset the register value
    // // set Endpoint 0 Tx FIFO size (0x40 = 64 * 32-bit words = 256 bytes)
    // usbGlobalHandle->DIEPTXF0_HNPTXFSIZ |= (0x40 << USB_OTG_TX0FD_Pos);
    // // set Endpoint 0 Tx FIFO RAM start address
    // usbGlobalHandle->DIEPTXF0_HNPTXFSIZ |= (txFifoOffset << USB_OTG_TX0FSA_Pos);
    // HAL
    uint32_t txFifoSize = 0x40U;
    usbGlobalHandle->DIEPTXF0_HNPTXFSIZ = (txFifoSize << USB_OTG_TX0FD_Pos) | txFifoOffset;



    txFifoOffset += 0x40; // add size of Ep0 Tx FIFO to the offset

    // Tx FIFO size for Endpoint 1
    usbGlobalHandle->DIEPTXF[0] = 0x00000000; // reset the register
    // set Endpoint 1 Tx FIFO size (0x40 = 64 * 32-bit words = 256 bytes)
    usbGlobalHandle->DIEPTXF[0] |= (0x40 << USB_OTG_DIEPTXF_INEPTXFD_Pos);
    // set Endpoint 1 Tx FIFO RAM start address
    usbGlobalHandle->DIEPTXF[0] |= (txFifoOffset << USB_OTG_DIEPTXF_INEPTXSA_Pos);


    // activate USB transceiver
    // (I'm not sure if I need this line here; this bit was set before and
    // I don't think anything would reset it until this point; gonna leave it
    // uncommented anyway, just to be safe)
    usbGlobalHandle->GCCFG |= USB_OTG_GCCFG_PWRDWN;

    // enable global USB interrupts
    usbGlobalHandle->GAHBCFG |= (USB_OTG_GAHBCFG_GINT);

    // make sure PHY clock is started and ungated
    usbPcgcctlHandle->PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
    // generate "device connect" event to the Host
    // Host should begin enumeration after this point
    usbDeviceHandle->DCTL &= ~(USB_OTG_DCTL_SDIS);
}


void usbConfigureGPIOs(void)
{
    // enable GPIO clock (RCC)
    RCC->AHB1ENR |= USB_DP_GPIOEN;
    // clear mode before assigning it the right value
    USB_DP_GPIO->MODER &= ~(0x3 << (USB_DP_PIN * 2));
    // set mode to alternate function mode (0b10 = 0x2)
    USB_DP_GPIO->MODER |= (0x2 << (USB_DP_PIN * 2));
    // Port Output Type Register set to 0 (output push-pull)
    USB_DP_GPIO->OTYPER &= ~(1 << USB_DP_PIN);
    // clear Port Output Speed Register (default value = low speed)
    USB_DP_GPIO->OSPEEDR &= ~(0x3 << (USB_DP_PIN * 2));
    // high speed
    USB_DP_GPIO->OSPEEDR |= (0x3 << (USB_DP_PIN * 2));
    // clear pull-up/pull-down register (no pull-up, no pull-down)
    USB_DP_GPIO->PUPDR &= ~(0x3 << (USB_DP_PIN * 2));
    // AFR[0] = AFR Low Register (pins 0..7)
    // AFR[1] = AFR High Register (pins 8..15)
    // clear alternate function (set to zero)
    USB_DP_GPIO->AFR[1] &= ~(0xF << ((USB_DP_PIN - 8U) * 4));
    // select alternate function number
    USB_DP_GPIO->AFR[1] |= (USB_DP_AF_NUM << ((USB_DP_PIN - 8U) * 4));


    // enable GPIO clock (RCC)
    // RCC->AHB1ENR |= USB_DM_GPIOEN;
    // clear mode before assigning it the right value
    USB_DM_GPIO->MODER &= ~(0x3 << (USB_DM_PIN * 2));
    // set mode to alternate function mode (0b10 = 0x2)
    USB_DM_GPIO->MODER |= (0x2 << (USB_DM_PIN * 2));
    // Port Output Type Register set to 0 (output push-pull)
    USB_DM_GPIO->OTYPER &= ~(1 << USB_DM_PIN);
    // clear Port Output Speed Register (default value = low speed)
    USB_DM_GPIO->OSPEEDR &= ~(0x3 << (USB_DM_PIN * 2));
    // high speed
    USB_DM_GPIO->OSPEEDR |= (0x3 << (USB_DM_PIN * 2));
    // clear pull-up/pull-down register (no pull-up, no pull-down)
    USB_DM_GPIO->PUPDR &= ~(0x3 << (USB_DM_PIN * 2));
    // AFR[0] = AFR Low Register (pins 0..7)
    // AFR[1] = AFR High Register (pins 8..15)
    // clear alternate function (set to zero)
    USB_DM_GPIO->AFR[1] &= ~(0xF << ((USB_DM_PIN - 8U) * 4));
    // select alternate function number
    USB_DM_GPIO->AFR[1] |= (USB_DM_AF_NUM << ((USB_DM_PIN - 8U) * 4));
}


void usbEnableInterrupts(uint8_t priority)
{
    NVIC_SetPriority(OTG_FS_IRQn, priority);
    NVIC_EnableIRQ(OTG_FS_IRQn);
}