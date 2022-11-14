/**
  ******************************************************************************
  * File Name          : TouchGFXHAL.cpp
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
#include <TouchGFXHAL.hpp>

/* USER CODE BEGIN TouchGFXHAL.cpp */

#include "stm32f4xx.h"
#include <touchgfx/hal/OSWrappers.hpp>
#include "main.h"
#include <touchgfx/hal/GPIO.hpp>
#include "../Components/otm8009a/otm8009a.h"
#include <CortexMMCUInstrumentation.hpp>
#include <KeySampler.hpp>
#include "FreeRTOS.h"
#include "task.h"

/**
  * @brief  LCD Display OTM8009A ID
  */
#define LCD_OTM8009A_ID  ((uint32_t) 0)

volatile bool displayRefreshing = false;
volatile bool refreshRequested = true;
static int updateRegion = 0;
static uint16_t* currFbBase = 0;

extern "C" {
    extern DSI_HandleTypeDef hdsi;
    extern LTDC_HandleTypeDef hltdc;

    uint8_t pCols[4][4] =
    {
        {0x00, 0x00, 0x00, 0xC7}, /*   0 -> 199 */
        {0x00, 0xC8, 0x01, 0x8F}, /* 200 -> 399 */
        {0x01, 0x90, 0x02, 0x57}, /* 400 -> 599 */
        {0x02, 0x58, 0x03, 0x1F}, /* 600 -> 799 */
    };

    uint8_t pColLeft[] = { 0x00, 0x00, 0x01, 0xbf }; /*   0 -> 447 */
    uint8_t pColRight[] = { 0x01, 0xc0, 0x03, 0x1F }; /* 448 -> 799 */

    uint8_t pPage[] = { 0x00, 0x00, 0x01, 0xDF }; /*   0 -> 479 */
    uint8_t pScanCol[] = { 0x02, 0x15 };             /* Scan @ 533 */

    /* Request tear interrupt at specific scanline. Implemented in BoardConfiguration.cpp */
    void LCD_ReqTear();

    /* Configures display to update indicated region of the screen (200pixel wide chunks) - 16bpp mode */
    void LCD_SetUpdateRegion(int idx);

    /* Configures display to update left half of the screen. Implemented in BoardConfiguration.cpp  - 24bpp mode*/
    void LCD_SetUpdateRegionLeft();

    /* Configures display to update right half of the screen. Implemented in BoardConfiguration.cpp - 24bpp mode*/
    void LCD_SetUpdateRegionRight();
}

using namespace touchgfx;

static CortexMMCUInstrumentation mcuInstr;
static KeySampler btnctrl;


void TouchGFXHAL::initialize()
{
    GPIO::init();

    // Calling parent implementation of initialize().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.
    // Please note, HAL::initialize() must be called to initialize the framework.

    TouchGFXGeneratedHAL::initialize();

    lockDMAToFrontPorch(false);

    mcuInstr.init();
    setMCUInstrumentation(&mcuInstr);
    enableMCULoadCalculation(true);

    setButtonController(&btnctrl);
}

void TouchGFXHAL::taskEntry()
{
    enableLCDControllerInterrupt();
    enableInterrupts();

    OSWrappers::waitForVSync();
    backPorchExited();

    /* Enable the LCD, Send Display on DCS command to display */
    HAL_DSI_ShortWrite(&hdsi, LCD_OTM8009A_ID, DSI_DCS_SHORT_PKT_WRITE_P1, OTM8009A_CMD_DISPON, 0x00);

    for (;;)
    {
        OSWrappers::waitForVSync();
        backPorchExited();
    }
}

/**
 * Gets the frame buffer address used by the TFT controller.
 *
 * @return The address of the frame buffer currently being displayed on the TFT.
 */
uint16_t* TouchGFXHAL::getTFTFrameBuffer() const
{
    // Calling parent implementation of getTFTFrameBuffer().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    return currFbBase;
}

/**
 * Sets the frame buffer address used by the TFT controller.
 *
 * @param [in] address New frame buffer address.
 */
void TouchGFXHAL::setTFTFrameBuffer(uint16_t* address)
{
    // Calling parent implementation of setTFTFrameBuffer(uint16_t* address).
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    TouchGFXGeneratedHAL::setTFTFrameBuffer(address);
}

/**
 * This function is called whenever the framework has performed a partial draw.
 *
 * @param rect The area of the screen that has been drawn, expressed in absolute coordinates.
 *
 * @see flushFrameBuffer().
 */
void TouchGFXHAL::flushFrameBuffer(const touchgfx::Rect& rect)
{
    // Calling parent implementation of flushFrameBuffer(const touchgfx::Rect& rect).
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.
    // Please note, HAL::flushFrameBuffer(const touchgfx::Rect& rect) must
    // be called to notify the touchgfx framework that flush has been performed.

    TouchGFXGeneratedHAL::flushFrameBuffer(rect);
}

bool TouchGFXHAL::blockCopy(void* RESTRICT dest, const void* RESTRICT src, uint32_t numBytes)
{
    return TouchGFXGeneratedHAL::blockCopy(dest, src, numBytes);
}

/**
 * Configures the interrupts relevant for TouchGFX. This primarily entails setting
 * the interrupt priorities for the DMA and LCD interrupts.
 */
void TouchGFXHAL::configureInterrupts()
{
    // Calling parent implementation of configureInterrupts().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    TouchGFXGeneratedHAL::configureInterrupts();
}

/**
 * Used for enabling interrupts set in configureInterrupts()
 */
void TouchGFXHAL::enableInterrupts()
{
    // Calling parent implementation of enableInterrupts().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    TouchGFXGeneratedHAL::enableInterrupts();
}

/**
 * Used for disabling interrupts set in configureInterrupts()
 */
void TouchGFXHAL::disableInterrupts()
{
    // Calling parent implementation of disableInterrupts().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    TouchGFXGeneratedHAL::disableInterrupts();
}

/**
 * Configure the LCD controller to fire interrupts at VSYNC. Called automatically
 * once TouchGFX initialization has completed.
 */
void TouchGFXHAL::enableLCDControllerInterrupt()
{
    // Calling parent implementation of enableLCDControllerInterrupt().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    LCD_ReqTear();
    TouchGFXGeneratedHAL::enableLCDControllerInterrupt();
}

void TouchGFXHAL::setFrameBufferStartAddresses(void* frameBuffer, void* doubleBuffer, void* animationStorage)
{
    currFbBase = (uint16_t*)frameBuffer;
    HAL::setFrameBufferStartAddresses(frameBuffer, doubleBuffer, animationStorage);
}

bool TouchGFXHAL::beginFrame()
{
    refreshRequested = false;
    return HAL::beginFrame();
}

void TouchGFXHAL::endFrame()
{
    HAL::endFrame();
    if (frameBufferUpdatedThisFrame)
    {
        refreshRequested = true;
    }
}

extern "C" {

    /**************************** LINK OTM8009A (Display driver) ******************/
    /**
     * @brief  DCS or Generic short/long write command
     * @param  NbParams: Number of parameters. It indicates the write command mode:
     *                 If inferior to 2, a long write command is performed else short.
     * @param  pParams: Pointer to parameter values table.
     * @retval HAL status
     */
    void DSI_IO_WriteCmd(uint32_t NbrParams, uint8_t* pParams)
    {
        if (NbrParams <= 1)
        {
            HAL_DSI_ShortWrite(&hdsi, LCD_OTM8009A_ID, DSI_DCS_SHORT_PKT_WRITE_P1, pParams[0], pParams[1]);
        }
        else
        {
            HAL_DSI_LongWrite(&hdsi,  LCD_OTM8009A_ID, DSI_DCS_LONG_PKT_WRITE, NbrParams, pParams[NbrParams], pParams);
        }
    }

    /**
     * Request TE at scanline.
     */
    void LCD_ReqTear(void)
    {
        static uint8_t ScanLineParams[2];

        uint16_t scanline = 533;
        ScanLineParams[0] = scanline >> 8;
        ScanLineParams[1] = scanline & 0x00FF;

        HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 2, 0x44, ScanLineParams);
        // set_tear_on
        HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x35, 0x00);
    }

    void LCD_SetUpdateRegion(int idx)
    {
        HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, OTM8009A_CMD_CASET, pCols[idx]);
    }

    void LCD_SetUpdateRegionLeft()
    {
        HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, OTM8009A_CMD_CASET, pColLeft);
    }

    void LCD_SetUpdateRegionRight()
    {
        HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, OTM8009A_CMD_CASET, pColRight);
    }

    void HAL_DSI_TearingEffectCallback(DSI_HandleTypeDef* hdsi)
    {
        GPIO::set(GPIO::VSYNC_FREQ);
        HAL::getInstance()->vSync();
        OSWrappers::signalVSync();

        // In single buffering, only require that the system waits for display update to be finished if we
        // actually intend to update the display in this frame.
        HAL::getInstance()->lockDMAToFrontPorch(refreshRequested);

        if (refreshRequested && !displayRefreshing)
        {
            // We have an update pending.
            if (HAL::getInstance())
            {
                // Swap frame buffers immediately instead of waiting for the task to be scheduled in.
                // Note: task will also swap when it wakes up, but that operation is guarded and will not have
                // any effect if already swapped.
                HAL::getInstance()->swapFrameBuffers();
            }

            // Update region 0 = first area of display first half for 24bpp
            updateRegion = 0;

            //Set update region
            LCD_SetUpdateRegion(updateRegion);

            HAL_DSI_Refresh(hdsi);
            displayRefreshing = true;
        }
        else
        {
            GPIO::clear(GPIO::VSYNC_FREQ);
        }
    }

    void HAL_DSI_EndOfRefreshCallback(DSI_HandleTypeDef* hdsi)
    {
        updateRegion++;
        if (updateRegion < 4)
        {
            DSI->WCR &= ~(DSI_WCR_DSIEN);
            LTDC_Layer1->CFBAR = ((uint32_t)currFbBase) + 200 * 2 * updateRegion;

            uint16_t REAL_WIDTH = 200;
            uint16_t ADJUSTED_WIDTH = 200;
            if (updateRegion == 3)
            {
                ADJUSTED_WIDTH += 32;
            }

            LTDC->AWCR = ((ADJUSTED_WIDTH + 2) << 16) | 0x1E2; //adj
            LTDC->TWCR = ((REAL_WIDTH + 2 + 1 - 1) << 16) | 0x1E3;
            LTDC_Layer1->WHPCR = ((REAL_WIDTH + 2) << 16) | 3;
            LTDC_Layer1->CFBLR = ((832 * 2) << 16) | ((REAL_WIDTH) * 2 + 3);

            LTDC->SRCR = (uint32_t)LTDC_SRCR_IMR;
            LCD_SetUpdateRegion(updateRegion);

            DSI->WCR |= DSI_WCR_DSIEN;
            HAL_DSI_Refresh(hdsi);
        }
        else
        {
            // Otherwise we are done refreshing.

            DSI->WCR &= ~(DSI_WCR_DSIEN);
            LTDC_Layer1->CFBAR = (uint32_t)currFbBase;

            uint16_t WIDTH = 200;
            LTDC->AWCR = ((WIDTH + 2) << 16) | 0x1E2;
            LTDC->TWCR = ((WIDTH + 2 + 1) << 16) | 0x1E3;
            LTDC_Layer1->WHPCR = ((WIDTH + 2) << 16) | 3;
            LTDC_Layer1->CFBLR = (((832 * 2) << 16) | ((WIDTH * 2) + 3));
            LTDC->SRCR = (uint32_t)LTDC_SRCR_IMR;
            LCD_SetUpdateRegion(0);
            DSI->WCR |= DSI_WCR_DSIEN;
            GPIO::clear(GPIO::VSYNC_FREQ);

            displayRefreshing = false;
            if (HAL::getInstance())
            {
                // Signal to the framework that display update has finished.
                HAL::getInstance()->frontPorchEntered();
            }
        }
    }

    portBASE_TYPE IdleTaskHook(void* p)
    {
        if ((int)p) //idle task sched out
        {
            touchgfx::HAL::getInstance()->setMCUActive(true);
        }
        else //idle task sched in
        {
            touchgfx::HAL::getInstance()->setMCUActive(false);
        }
        return pdTRUE;
    }
}

/* USER CODE END TouchGFXHAL.cpp */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
