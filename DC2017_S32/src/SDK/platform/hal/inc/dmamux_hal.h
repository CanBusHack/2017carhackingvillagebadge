/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef DMAMUX_HAL_H
#define DMAMUX_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "device_registers.h"

/*!
 * @addtogroup dmamux_hal
 * @{
 */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name DMAMUX HAL function
 * @{
 */

/*!
 * @brief Initializes the DMAMUX module to the reset state.
 *
 * Initializes the DMAMUX module to the reset state.
 *
 * @param base Register base address for DMAMUX module.
 */
void DMAMUX_HAL_Init(DMAMUX_Type * base);

/*!
 * @brief Enables/Disables the DMAMUX channel.
 *
 * Enables the hardware request. If enabled, the hardware request is  sent to
 * the corresponding DMA channel.
 *
 * @param base Register base address for DMAMUX module.
 * @param channel DMAMUX channel number.
 * @param enable Enables (true) or Disables (false) DMAMUX channel.
 * Implements    : DMAMUX_HAL_SetChannelCmd_Activity
 */
static inline void DMAMUX_HAL_SetChannelCmd(DMAMUX_Type * base, uint32_t channel, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FEATURE_DMAMUX_MODULE_CHANNELS);
#endif
    uint32_t regIndex = FEATURE_DMAMUX_CHN_REG_INDEX(channel);
    uint8_t regValTemp = base->CHCFG[regIndex];
    regValTemp &= (uint8_t)~(DMAMUX_CHCFG_ENBL_MASK);
    regValTemp |= (uint8_t)DMAMUX_CHCFG_ENBL(enable ? 1U : 0U);
    base->CHCFG[regIndex] = regValTemp;
}

#if (FEATURE_DMAMUX_HAS_TRIG == 1)
/*!
 * @brief Enables/Disables the period trigger.
 *
 * @param base Register base address for DMAMUX module.
 * @param channel DMAMUX channel number.
 * @param enable Enables (true) or Disables (false) period trigger.
 * Implements    : DMAMUX_HAL_SetPeriodTriggerCmd_Activity
 */
static inline void DMAMUX_HAL_SetPeriodTriggerCmd(DMAMUX_Type * base, uint32_t channel, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FEATURE_DMAMUX_MODULE_CHANNELS);
#endif
    uint32_t regIndex = FEATURE_DMAMUX_CHN_REG_INDEX(channel);
    uint8_t regValTemp = base->CHCFG[regIndex];
    regValTemp &= (uint8_t)~(DMAMUX_CHCFG_TRIG_MASK);
    regValTemp |= (uint8_t)DMAMUX_CHCFG_TRIG(enable ? 1U : 0U);
    base->CHCFG[regIndex] = regValTemp;
}
#endif

/*!
 * @brief Configures the DMA request for the DMAMUX channel.
 *
 * Selects which DMA source is routed to a DMA channel. The DMA sources are defined in the file
 * <MCU>_Features.h
 *
 * @param base Register base address for DMAMUX module.
 * @param channel DMAMUX channel number.
 * @param source DMA request source.
 * Implements    : DMAMUX_HAL_SetChannelSource_Activity
 */
static inline void DMAMUX_HAL_SetChannelSource(DMAMUX_Type * base, uint32_t channel, uint8_t source)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FEATURE_DMAMUX_MODULE_CHANNELS);
#endif
    uint32_t regIndex = FEATURE_DMAMUX_CHN_REG_INDEX(channel);
    uint8_t regValTemp;
    regValTemp = base->CHCFG[regIndex];
    regValTemp &= (uint8_t)~(DMAMUX_CHCFG_SOURCE_MASK);
    regValTemp |= (uint8_t)DMAMUX_CHCFG_SOURCE(source);
    base->CHCFG[regIndex] = regValTemp;
}

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* DMAMUX_HAL_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/

