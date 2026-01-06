#include "utils.h"
#include "main.h"
#include "hardwareSetup.h"
#include "stm32f411Blackpill.h"
#include "makeshiftDebugger.h"
#include "usb.h"
#include "usbIrqHandlers.h"
#include "usbDebug.h"
#include "lsm6ds3.h"


#define SHOW_DBG_MESSAGES_USB_IRQ (0)


// ************************************************************************
//                   Demo (Experiment) Enable/Disable
// ************************************************************************
#define DEMO_ON__PRINT_USB_REG_ADDR              (FALSE)

// enables/disables the "isLogOrderOfUsbInterruptsActive" debug messages
#define SHOULD_LOG_USB_INTERRUPTS_ON_BTN_PRESS   (FALSE)
// ************************************************************************
//                end: Demo (Experiment) Enable/Disable
// ************************************************************************


// ********************************************************************************
//                  extern variables
// ********************************************************************************
// Blackpill User Button Debounce - variables
extern uint32_t bPillButtonDebounceCount;
extern uint8_t blackPillStatusFlags;


// USART debugger
extern char intToAsciiBuffer[INT_TO_ASCII_BUFFER_SIZE];


// USB
extern volatile uint8_t isEp1Configured;
extern uint32_t hidReportDelayCount;
extern uint8_t shouldSendHidReport;

// USB demo (example) HID keyboard
extern uint16_t hidReportBonus;
extern uint8_t shouldToggleHidReportActive;
extern uint8_t isSendingHidReportActive;
extern uint8_t shouldEnableEndpoint1;
extern uint8_t wasHidReportSent;


// USB HID Gamepad
int32_t hidReportGamepad = 0x00000000U;


// USB debug-related variables
extern uint32_t usbDebugFlags;
extern uint16_t interruptFlags[INTERRUPT_FLAGS_ARRAY_SIZE];
extern uint8_t usbIrqCount;
extern uint8_t isLogOrderOfUsbInterruptsActive;


// lsm6ds3
extern uint8_t lsm6ds3IrqReadActive;
// ********************************************************************************
//                end: extern variables
// ********************************************************************************

// global variable where SPI Rx values are stored
// this variable is written to by SPI IRQ Handler
uint8_t spi1IrqReceivedByte;

// button debounce test
uint8_t shouldToggleLED = 0;

// lsm6ds3 SPI test
uint8_t lsm6ds3ShouldReadWhoami = 0;


// ********************************************************************************
//                 lsm6ds3 gyro angle variables
// ********************************************************************************
// #define GYRO_SENSITIVITY (0.00875f)
#define GYRO_SENSITIVITY (1.0f / 114.0f)

// timer 2
// 20 * 500 us = 10 ms
// #define GYRO_TICK_COUNT_MAX (20U)
// #define ANGLE_DELTA_T (0.01f) // 10 ms

// timer 2
// 10 * 500 us = 5 ms
// #define GYRO_TICK_COUNT_MAX (10U)
// #define ANGLE_DELTA_T (0.005f) // 5 ms

// timer 11
// 20 * 50 μs = 1 ms
// #define GYRO_TICK_COUNT_MAX (20U)
// #define ANGLE_DELTA_T (0.001f) // 1 ms

// timer 11
// 400 * 50 μs = 20 ms
#define GYRO_TICK_COUNT_MAX (400U)
#define ANGLE_DELTA_T (0.02f) // 1 ms

// timer 11
// 1000 * 50 μs = 50 ms
// #define GYRO_TICK_COUNT_MAX (1000U)
// #define ANGLE_DELTA_T (0.05f) // 50 ms

// timer 11
// 1400 * 50 μs = 70 ms
// #define GYRO_TICK_COUNT_MAX (1400U)
// #define ANGLE_DELTA_T (0.07f) // 70 ms

// timer 11
// 2000 * 50 μs = 100 ms
// #define GYRO_TICK_COUNT_MAX (2000U)
// #define ANGLE_DELTA_T (0.1f) // 100 ms


float gyroAngle = 0.0f;
float gyroYAngle = 0.0f;


volatile uint32_t gyroTickCount = 0;
volatile uint8_t shouldReadLsm6ds3 = FALSE;

#define LSM6DS3_DATA_BUFFER_SIZE (14U)
uint8_t lsm6ds3Data[LSM6DS3_DATA_BUFFER_SIZE];

float gyroBiasZ = 0.0f;
float gyroBiasY = 0.0f;
// ********************************************************************************
//                end: lsm6ds3 gyro angle variables
// ********************************************************************************


int main(void)
{
    setUpVoltageScaling();
    setUpSystemClock();

    setUpF411Blackpill();
    setUpTimer2();
    setUpUsart2();
    setUpLsm6ds3Gpios();
    setUpSpi1();
    setUpTimer11();

    enableUSBClock(RCC);
    usbConfigureGPIOs();
    initUsb(USB_OTG_FS, USB_OTG_FS_DEVICE, USB_OTG_FS_PCGCCTL);
    
    usbEnableInterrupts(0U);
    enableInterruptsSpi1(3U);
    enableInterruptsTimer2(5U);
    f411BlackpillEnableButtonInterrupts(7U);
    enableInterruptsTimer11(9U);

    // enable accelerometer
    lsm6ds3WriteSingleRegisterSPI(SPI1, LSM6DS3_CTRL1_XL, 0xA0);
    // enable gyroscope
    lsm6ds3WriteSingleRegisterSPI(SPI1, LSM6DS3_CTRL2_G, 0x80);


    BLACKPILL_LED_OFF();


    // basic gyro drift offset calculation
    for (uint16_t i = 0; i < 100; i++)
    {
        lsm6ds3ReadRegistersSPI(
            SPI1,
            &spi1IrqReceivedByte,
            LSM6DS3_OUT_TEMP_L,
            lsm6ds3Data,
            LSM6DS3_DATA_BUFFER_SIZE
        );

        gyroBiasZ += (float) ((lsm6ds3Data[7] << 8U) | lsm6ds3Data[6]);
        gyroBiasY += (float) ((lsm6ds3Data[5] << 8U) | lsm6ds3Data[4]);

        gyroTickCount = 0;
        while (!shouldReadLsm6ds3)
        {
            continue;
        }
    }
    #define GYRO_BIAS_DIV_Z (31000.0f)
    #define GYRO_BIAS_DIV_Y (5900.0f)
    // #define GYRO_BIAS_DIV_Y (100000.0f)
    // #define GYRO_BIAS_DIV_Y (400000.0f)
    // #define GYRO_BIAS_DIV_Y (5000000.0f)
    // #define GYRO_BIAS_DIV_Y (15500.0f)
    // #define GYRO_BIAS_DIV_Y (100.0f)
    // #define GYRO_BIAS_DIV_Y (500.0f)
    // #define GYRO_BIAS_DIV_Y (5000.0f)
    gyroBiasZ /= GYRO_BIAS_DIV_Z;
    gyroBiasY /= GYRO_BIAS_DIV_Y;


    while (1)
    {
        if (IS_BLACKPILL_BTN_PRESSED())
        {
            #if (SHOULD_LOG_USB_INTERRUPTS_ON_BTN_PRESS == TRUE)
            isLogOrderOfUsbInterruptsActive = 1;
            #else
            #warning "interrupt status debug log on button press is disabled"
            #endif

            shouldToggleHidReportActive = 1;

            shouldToggleLED = 1;

            lsm6ds3ShouldReadWhoami = 1;

            // reset the angle
            gyroAngle = 0.0f;
            gyroYAngle = 0.0f;

            CLR_BLACKPILL_DEBOUNCE_IN_PROGRESS();
            CLR_BLACKPILL_BTN_PRESSED();
        }


        if (shouldToggleHidReportActive)
        {
            // hidReportBonus = 0x1402; // shift + 'q'
            // hidReportBonus = 0x0050; // gamepad

            if (hidReportGamepad & 0x01)
            {
                // hidReportGamepad &= ~(0x01);
                hidReportGamepad &= ~(0xFF01); // button and X axis
            }
            else
            {
                // hidReportGamepad |= (0x01);
                hidReportGamepad |= (0x5001); // button and X axis
            }

            shouldToggleHidReportActive = 0;
        }


        if (shouldReadLsm6ds3)
        {
            // if new gyro data available
            if (lsm6ds3ReadSingleRegisterSPI(SPI1, 0x1E, &spi1IrqReceivedByte) & 0x02U)
            {
                lsm6ds3ReadRegistersSPI(
                    SPI1,
                    &spi1IrqReceivedByte,
                    LSM6DS3_OUT_TEMP_L,
                    lsm6ds3Data,
                    LSM6DS3_DATA_BUFFER_SIZE
                );

                // index 0: OUT_TEMP_L (0x20)
                // index 1: OUT_TEMP_H (0x21)
                // index 2: OUTX_L_G   (0x22)
                // index 3: OUTX_H_G   (0x23)
                // index 4: OUTY_L_G   (0x24)
                // index 5: OUTY_H_G   (0x25)
                // index 6: OUTZ_L_G   (0x26)
                // index 7: OUTZ_H_G   (0x27)

                int16_t gyroZRawData = (lsm6ds3Data[7] << 8U) | lsm6ds3Data[6];
                gyroZRawData = -gyroZRawData;
                int16_t gyroYRawData = (lsm6ds3Data[5] << 8U) | lsm6ds3Data[4];
                // gyroYRawData = -gyroYRawData;

                // 0.01f = delta time (10 ms)
                // gyroAngle += ((float)gyroZRawData * GYRO_SENSITIVITY) * 0.01f;
                // gyroYAngle += ((float)gyroYRawData * GYRO_SENSITIVITY) * 0.01f;


                // no gyro bias included
                // gyroAngle += ((float)gyroZRawData * GYRO_SENSITIVITY) * ANGLE_DELTA_T;
                // // gyroAngle += ((float)gyroZRawData * GYRO_SENSITIVITY * 2) * ANGLE_DELTA_T;
                // gyroYAngle += ((float)gyroYRawData * GYRO_SENSITIVITY) * ANGLE_DELTA_T;



                // gyro bias included
                gyroAngle += ((((float)gyroZRawData) - gyroBiasZ) * GYRO_SENSITIVITY) * ANGLE_DELTA_T;
                gyroYAngle += ((((float)gyroYRawData) + gyroBiasY) * GYRO_SENSITIVITY) * ANGLE_DELTA_T;



                // debuggerWriteStr(DBG_USART, "                          \r");
                dbgPrintDec("X angle: ", (uint32_t)gyroAngle, "\n\r");
                dbgPrintDec("Y angle: ", (uint32_t)gyroYAngle, "\n\r");
                


                // int8_t xAxisDeflection = (uint8_t)(180.0f / gyroAngle);
                // int8_t xAxisDeflection = (uint8_t)(255.0f  / gyroAngle);
                // int8_t xAxisDeflection = (uint8_t)(127.0f  / gyroAngle);
                
                // int8_t xAxisDeflection = (uint8_t)(255.0f * (180.0f  / gyroAngle));
                // int8_t xAxisDeflection = (uint8_t)(255.0f * (gyroAngle / 180.0f));
                // int8_t xAxisDeflection = (uint8_t)(255.0f * (gyroAngle / 90.0f));
                
                // int8_t xAxisDeflection = (int8_t)(255.0f * (gyroAngle / 180.0f));
                
                // int8_t xAxisDeflection = (int8_t)(127.0f * (gyroAngle / 180.0f));
                int8_t xAxisDeflection = (int8_t)(127.0f * (gyroAngle / 30.0f));
                // int8_t xAxisDeflection = (int8_t)(127.0f * (gyroAngle / 90.0f));
                // int8_t xAxisDeflection = (int8_t)(127.0f * (gyroAngle / 45.0f));
                // int8_t xAxisDeflection = (uint8_t)(gyroAngle);
                
                // int8_t yAxisDeflection = (int8_t)(127.0f * (gyroYAngle / 60.0f));
                int8_t yAxisDeflection = (int8_t)(127.0f * (gyroYAngle / 30.0f));
                
                // hidReportGamepad = (yAxisDeflection << 16U) | (xAxisDeflection << 8U);
                
                uint8_t gamepadButtonStatus = (uint8_t) hidReportGamepad & 0x01;

                hidReportGamepad = 0;
                hidReportGamepad |= (0xFF0000 & ((int32_t)yAxisDeflection << 16U));
                hidReportGamepad |= (0xFF00 & ((int32_t)xAxisDeflection << 8U));

                hidReportGamepad |= (int32_t) gamepadButtonStatus;
            }

            gyroTickCount = 0;
            shouldReadLsm6ds3 = FALSE;
        }


        if (isEp1Configured)
        {
            uint32_t enpointTxSpace;
            uint32_t* ep1FifoPtr = (uint32_t *)EP1_FIFO_ADDR;



            // debuggerWriteStr(DBG_USART, "********************************\n\r");
            // debuggerWriteStr(DBG_USART, "Sending HID report...\n\r");
            // debuggerWriteStr(DBG_USART, "********************************\n\r");


            enpointTxSpace = USB_OTG_FS_INEP(1U)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk;
            // dbgPrintDec("space left in Tx 1 FIFO: ", enpointTxSpace, "\n\r");

            while (enpointTxSpace == 0)
            {
                continue;
            }
            // debuggerWriteStr(DBG_USART, "enough space.\n\r");

            USB_OTG_FS_INEP(1U)->DIEPTSIZ = (
                0x1U << USB_OTG_DIEPTSIZ_PKTCNT_Pos |
                
                // configuration descriptor byte length
                // HID Report size = 2 bytes (uint16_t)
                // 2U << USB_OTG_DIEPTSIZ_XFRSIZ_Pos
                
                // gamepad with 1 button - 3 bytes
                3U << USB_OTG_DIEPTSIZ_XFRSIZ_Pos
            );

            USB_OTG_FS_INEP(1U)->DIEPCTL |= (USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK);

            // write HID Report to Endpoint 1 Tx FIFO
            // *ep1FifoPtr = (uint32_t) hidReportBonus;
            *ep1FifoPtr = hidReportGamepad;

            // wasHidReportSent = 1;
            // hidReportBonus = 0x0000; // reset HID Report
            
            // debuggerWriteStr(DBG_USART, "waiting for XFRC to be set to 1... ");
            while (!(USB_OTG_FS_INEP(1U)->DIEPINT & USB_OTG_DIEPINT_XFRC))
            {
                continue;
            }
            // debuggerWriteStr(DBG_USART, "done\n\r");

            // debuggerWriteStr(DBG_USART, "clearing XFRC.\n\r");
            USB_OTG_FS_INEP(1U)->DIEPINT &= USB_OTG_DIEPINT_XFRC;

            // shouldSendHidReport = 0; // wait for delay
            // wasHidReportSent = 1;


            // debuggerWriteStr(DBG_USART, "********************************\n\r");
            // debuggerWriteStr(DBG_USART, "Sending HID report (END)\n\r");
            // debuggerWriteStr(DBG_USART, "********************************\n\r");
        }


        if (lsm6ds3ShouldReadWhoami)
        {
            uint8_t whoAmI = lsm6ds3ReadWhoAmI(SPI1, &spi1IrqReceivedByte);

            dbgPrintHex("lsm6ds3 \"Who Am I\": ", whoAmI, "\n\r");

            lsm6ds3ShouldReadWhoami = 0;
        }


        if (isLogOrderOfUsbInterruptsActive)
        {
            logUsbInterruptFlagsInTriggerOrder();

            isLogOrderOfUsbInterruptsActive = 0;
        }

        if (usbDebugFlags & USB_DBG_FLG__LOG_ADDR_ACTIVE)
        {
            #if (DEMO_ON__PRINT_USB_REG_ADDR == TRUE)
            dbgLogUsbRegisterAddresses();
            #endif

            usbDebugFlags &= ~(USB_DBG_FLG__LOG_ADDR_ACTIVE);
        }


        if (shouldToggleLED)
        {
            BLACKPILL_LED_TOGGLE();

            shouldToggleLED = 0;
        }

    }

    return 0;
}


// ****************************************************************************
//                     Timer 11 Interrupt Handler
// ****************************************************************************
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    if (TIM11->SR & TIM_SR_UIF)
    {
        TIM11->SR &= ~(TIM_SR_UIF);


        gyroTickCount++;
        if (gyroTickCount >= GYRO_TICK_COUNT_MAX)
        {
            gyroTickCount = 0;
            shouldReadLsm6ds3 = TRUE;
        }


    }
}
// ****************************************************************************
//                  end: Timer 11 Interrupt Handler
// ****************************************************************************


// ****************************************************************************
//                     Timer 2 Interrupt Handler
// ****************************************************************************
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~(TIM_SR_UIF);


        // gyroTickCount++;
        // if (gyroTickCount >= GYRO_TICK_COUNT_MAX)
        // {
        //     gyroTickCount = 0;
        //     shouldReadLsm6ds3 = TRUE;
        // }


        // blackpill user button debounce
        if (IS_BLACKPILL_BTN_DEBOUNCE_IN_PROGRESS() && !IS_BLACKPILL_BTN_PRESSED())
        {
            bPillButtonDebounceCount++;
            if (bPillButtonDebounceCount >= B_PILL_BTN_DEBOUNCE_TIME)
            {
                SET_BLACKPILL_BTN_PRESSED();
                bPillButtonDebounceCount = 0;
            }
        }
    }
}
// ****************************************************************************
//                  end: Timer 2 Interrupt Handler
// ****************************************************************************


// ****************************************************************************
//                  Blackpill Button Interrupt Handler
// ****************************************************************************
void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & (0x1UL << BLACKPILL_BUTTON_PIN))
    {
        // clear pending interrupt
        EXTI->PR |= (0x1UL << BLACKPILL_BUTTON_PIN);


        usbDebugFlags |= USB_DBG_FLG__LOG_ADDR_ACTIVE;


        if (!IS_BLACKPILL_BTN_DEBOUNCE_IN_PROGRESS())
        {
            SET_BLACKPILL_BTN_DEBOUNCE_IN_PROGRESS();
            bPillButtonDebounceCount = 0;
        }
    }
}
// ****************************************************************************
//                end: Blackpill Button Interrupt Handler
// ****************************************************************************


// ****************************************************************************
//                      USART 6 Interrupt Handler
// ****************************************************************************
void USART6_IRQHandler(void)
{
    if (USART6->SR & USART_SR_RXNE)
    {
        // [...]
        // uint8_t readChar = USART6->DR;
        (void) USART6->DR;
    }
}
// ****************************************************************************
//                    end: USART 6 Interrupt Handler
// ****************************************************************************


// ****************************************************************************
//                      USB OTG Interrupt Handler
// ****************************************************************************
void OTG_FS_IRQHandler(void)
{
    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL)
    {
        if (usbIrqCount < INTERRUPT_FLAGS_ARRAY_SIZE)
        {
            interruptFlags[usbIrqCount] |= USB_IRQ_RXFLVL;
        }

        // debuggerWriteStr(DBG_USART, "****************************************\n\r");
        // debuggerWriteStr(DBG_USART, "Success! RXFLVL = 1\n\r");
        // debuggerWriteStr(DBG_USART, "****************************************\n\r");

        usbHandleIrqRXFLVL();
    }

    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST)
    {
        if (usbIrqCount < INTERRUPT_FLAGS_ARRAY_SIZE)
        {
            interruptFlags[usbIrqCount] |= USB_IRQ_USBRST;
        }

        usbHandleIrqUSBRST();
    }

    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE)
    {
        if (usbIrqCount < INTERRUPT_FLAGS_ARRAY_SIZE)
        {
            interruptFlags[usbIrqCount] |= USB_IRQ_ENUMDNE;
        }

        usbHandleIrqENUMDNE();
    }

    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_IEPINT)
    {
        if (usbIrqCount < INTERRUPT_FLAGS_ARRAY_SIZE)
        {
            interruptFlags[usbIrqCount] |= USB_IRQ_IEPINT;
        }

        usbHandleIrqIEPINT();
    }

    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_OEPINT)
    {
        if (usbIrqCount < INTERRUPT_FLAGS_ARRAY_SIZE)
        {
            interruptFlags[usbIrqCount] |= USB_IRQ_OEPINT;
        }

        usbHandleIrqOEPINT();
    }

    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_IISOIXFR)
    {
        if (usbIrqCount < INTERRUPT_FLAGS_ARRAY_SIZE)
        {
            interruptFlags[usbIrqCount] |= USB_IRQ_IISOIXFR;
        }
    }

    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_PXFR_INCOMPISOOUT)
    {
        if (usbIrqCount < INTERRUPT_FLAGS_ARRAY_SIZE)
        {
            interruptFlags[usbIrqCount] |= USB_IRQ_INCOMPISOOUT;
        }
    }

    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_WKUINT)
    {
        if (usbIrqCount < INTERRUPT_FLAGS_ARRAY_SIZE)
        {
            interruptFlags[usbIrqCount] |= USB_IRQ_WKUINT;
        }
    }

    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBSUSP)
    {
        if (usbIrqCount < INTERRUPT_FLAGS_ARRAY_SIZE)
        {
            interruptFlags[usbIrqCount] |= USB_IRQ_USBSUSP;
        }

        // USB_OTG_FS_DEVICE->DCTL |= USB_OTG_DCTL_RWUSIG;
        // for (uint16_t i = 0; i < 10000U; i++)
        // {
        //     asm("nop");
        // }
        USB_OTG_FS_DEVICE->DCTL &= ~(USB_OTG_DCTL_RWUSIG);

        // clear USBSUSP interrupt (rc_w1)
        // USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_USBSUSP;
        USB_OTG_FS->GINTSTS &= USB_OTG_GINTSTS_USBSUSP;
    }

    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_SOF)
    {
        if (usbIrqCount < INTERRUPT_FLAGS_ARRAY_SIZE)
        {
            interruptFlags[usbIrqCount] |= USB_IRQ_SOF;
        }

        usbHandleIrqSOF();
    }


    // debug
    if (usbIrqCount <= INTERRUPT_FLAGS_ARRAY_SIZE)
    {
        usbIrqCount++;
    }
}
// ****************************************************************************
//                     end: USB OTG Interrupt Handler
// ****************************************************************************


// ****************************************************************************
//                      SPI 1 Interrupt Handler
// ****************************************************************************
void SPI1_IRQHandler(void)
{
    if (SPI1->SR & SPI_SR_RXNE)
    {
        spi1IrqReceivedByte = *(volatile uint8_t *) &(SPI1->DR);

        // debuggerWriteStr(DBG_USART, "SPI RXNE\n\r");

        if (lsm6ds3IrqReadActive)
        {
            lsm6ds3IrqReadActive = 0;
        }
    }

    if (SPI1->SR & SPI_SR_MODF)
    {
        debuggerWriteStr(DBG_USART, "SPI Error: MODF\n\r");
    }

    if (SPI1->SR & SPI_SR_OVR)
    {
        debuggerWriteStr(DBG_USART, "SPI Error: OVR\n\r");
    }

    if (SPI1->SR & SPI_SR_CRCERR)
    {
        debuggerWriteStr(DBG_USART, "SPI Error: CRCERR\n\r");
    }

    if (SPI1->SR & SPI_SR_FRE)
    {
        debuggerWriteStr(DBG_USART, "SPI Error: FRE\n\r");
    }
}
// ****************************************************************************
//                     end: SPI 1 Interrupt Handler
// ****************************************************************************