/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
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
#if !defined(RCM_HAL_H)
#define RCM_HAL_H

#include "device_registers.h"
#include <stdbool.h>


 /*! @file rcm_hal.h */

/*!
 * @defgroup rcm_hal Reset Control Module (RCM)
 * @ingroup power_manager
 * @brief This module covers the functionality of the Reset Control Module (RCM) peripheral.
 * <p>
 *  RCM HAL provides the API for reading and writing register bit-fields belonging to the RCM module.
 * </p>
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 *  @brief System Reset Source Name definitions
 *  Implements rcm_source_names_t_Class
 */
typedef enum {
    RCM_WAKEUP,                      /* Wakeup */
    RCM_LOW_VOLT_DETECT,             /* Low voltage detect reset */
    RCM_LOSS_OF_CLK,                 /* Loss of clock reset */
    RCM_LOSS_OF_LOCK,                /* Loss of lock reset */
    RCM_WATCH_DOG,                   /* Watch dog reset */
    RCM_EXTERNAL_PIN,                /* External pin reset */
    RCM_POWER_ON,                    /* Power on reset */
    RCM_SJTAG,                       /* JTAG generated reset */
    RCM_CORE_LOCKUP,                 /* core lockup reset */
    RCM_SOFTWARE,                    /* Software reset */
    RCM_SMDM_AP,                     /* MDM-AP system reset */
    RCM_STOP_MODE_ACK_ERR,           /* Stop mode ack error reset */
    RCM_TAMPERR,                     /* Tamperr */
    RCM_CORE1,                       /* Core1 */
    RCM_SRC_NAME_MAX
} rcm_source_names_t;

/*!
 *  @brief Reset pin filter select in Run and Wait modes
 *  Implements rcm_filter_run_wait_modes_t_Class
 */
typedef enum
{
    RCM_FILTER_DISABLED,          /* All filtering disabled */
    RCM_FILTER_BUS_CLK,           /* Bus clock filter enabled */
    RCM_FILTER_LPO_CLK,           /* LPO clock filter enabled */
    RCM_FILTER_RESERVED           /* Reserved setting */
} rcm_filter_run_wait_modes_t;


/*!
 *  @brief Reset delay time
 *  Implements rcm_reset_delay_time_t_Class
 */
typedef enum
{
    RCM_10LPO_CYCLES_DELAY,       /* reset delay time 10 LPO cycles */
    RCM_34LPO_CYCLES_DELAY,       /* reset delay time 34 LPO cycles */
    RCM_130LPO_CYCLES_DELAY,      /* reset delay time 130 LPO cycles */
    RCM_514LPO_CYCLES_DELAY       /* reset delay time 514 LPO cycles */
} rcm_reset_delay_time_t;

/*!
 *  @brief RCM module version number
 *  Implements rcm_version_info_t_Class
 */
typedef struct
{
    uint32_t  majorNumber;       /**< Major Version Number */
    uint32_t  minorNumber;       /**< Minor Version Number */
    uint32_t  featureNumber;     /**< Feature Specification Number */
} rcm_version_info_t;


/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*! @name Reset Control Module APIs*/
/*@{*/

/*!
 * @brief Get the version of the RCM module
 *
 * @param[in]  baseAddr     base address of the RCM module
 * @param[out] versionInfo  Device Version Number
 */
void  RCM_HAL_GetVersion(const RCM_Type* const baseAddr, rcm_version_info_t* const versionInfo);

/*!
 * @brief Checks the existence of the status indication feature for a reset source
 *
 * This function checks the existence of the status indication feature for a specified source.
 *
 * @param[in] baseAddr     Register base address of RCM
 * @param[in] srcName      reset source name
 * @return status          true or false for specified reset source
 */
bool RCM_HAL_GetSrcIndicationFeatureAvailability(const RCM_Type* const baseAddr, const rcm_source_names_t srcName);

/*!
 * @brief Gets the reset source status
 *
 * This function gets the current reset source status for a specified source.
 *
 * @param[in] baseAddr     Register base address of RCM
 * @param[in] srcName      reset source name
 * @return status          true or false for specified reset source
 */
bool RCM_HAL_GetSrcStatusCmd(const RCM_Type* const baseAddr, const rcm_source_names_t srcName);

/*!
 * @brief Enables/disables a specified system reset interrupt.
 *
 * This function will enable/disable the specified system reset interrupt.
 *
 * @param[in] baseAddr        Register base address of RCM
 * @param[in] resetInterrupt  Reset source name
 * @param[in] enable          true or false for the specified reset interrupt
 */
void RCM_HAL_SetResetIntCmd(RCM_Type* const baseAddr, const rcm_source_names_t resetInterrupt, const bool enable);


/*!
 * @brief Enables/disables all system reset interrupts.
 *
 * This function  enables/disables all system reset interrupts.
 *
 * @param[in] baseAddr     Register base address of RCM
 * @param[in] enable       enable or disable the filter in stop mode
 * Implements RCM_HAL_SetAllResetIntCmd_Activity
 */
static inline void RCM_HAL_SetAllResetIntCmd(RCM_Type* const baseAddr, const bool enable)
{
    uint32_t regValue = (uint32_t)baseAddr->SRIE;
    regValue &= (uint32_t)(~(RCM_SRIE_GIE_MASK));
    regValue |= RCM_SRIE_GIE(enable?1UL:0UL);
    baseAddr->SRIE = (uint32_t)regValue;
}

/*!
 * @brief Gets the sticky reset source status.
 *
 * This function gets the current reset source status that have not been cleared
 * by software for a specified source.
 *
 * @param[in] baseAddr     Register base address of RCM
 * @param[in] srcName      reset source name
 * @return status          true or false for specified reset source
 */
bool RCM_HAL_GetStickySrcStatusCmd(const RCM_Type* const baseAddr, const rcm_source_names_t srcName);

/*!
 * @brief Clear the sticky reset source status.
 *
 * This function clears all the sticky system reset flags.
 *
 * @param[in] baseAddr     Register base address of RCM
 */
void RCM_HAL_ClearStickySrcStatus(RCM_Type* const baseAddr);

/*!
 * @brief Sets the reset pin filter in stop mode.
 *
 * This function  sets the reset pin filter enable setting in stop mode.
 *
 * @param[in] baseAddr     Register base address of RCM
 * @param[in] enable       enable or disable the filter in stop mode
 * Implements RCM_HAL_SetFilterStopModeCmd_Activity
 */
static inline void RCM_HAL_SetFilterStopModeCmd(RCM_Type* const baseAddr, const bool enable)
{
    uint32_t regValue = (uint32_t)baseAddr->RPC;
    regValue &= (uint32_t)(~(RCM_RPC_RSTFLTSS_MASK));
    regValue |= RCM_RPC_RSTFLTSS(enable?1UL:0UL);
    baseAddr->RPC = (uint32_t)regValue;
}

/*!
 * @brief Gets the reset pin filter in stop mode.
 *
 * This function gets the reset pin filter enable setting in stop mode.
 *
 * @param[in] baseAddr  Register base address of RCM
 * @return enable       true/false to enable or disable the filter in stop mode
 * Implements RCM_HAL_GetFilterStopModeCmd_Activity
 */
static inline bool RCM_HAL_GetFilterStopModeCmd(const RCM_Type* const baseAddr)
{
    uint32_t regValue = (uint32_t)baseAddr->RPC;
    regValue = (regValue & RCM_RPC_RSTFLTSS_MASK) >> RCM_RPC_RSTFLTSS_SHIFT;
    return (regValue == 0UL) ? false : true;
}

/*!
 * @brief Sets the reset pin filter in run and wait mode.
 *
 * This function sets the reset pin filter enable setting in run/wait mode.
 *
 * @param[in] baseAddr    Register base address of RCM
 * @param[in] mode        to be set for reset filter in run/wait mode
 * Implements RCM_HAL_SetFilterRunWaitMode_Activity
 */
static inline void RCM_HAL_SetFilterRunWaitMode(RCM_Type* const baseAddr, const rcm_filter_run_wait_modes_t mode)
{
    uint32_t regValue = baseAddr->RPC;
    regValue &= ~(RCM_RPC_RSTFLTSRW_MASK);
    regValue |= RCM_RPC_RSTFLTSRW(mode);
    baseAddr->RPC = regValue;
}

/*!
 * @brief Gets the reset pin filter for stop mode.
 *
 * This function gets the reset pin filter enable setting for stop mode.
 *
 * @param[in] baseAddr  Register base address of RCM
 * @return mode  for reset filter in run/wait mode
 * Implements RCM_HAL_GetFilterRunWaitMode_Activity
 */
static inline rcm_filter_run_wait_modes_t RCM_HAL_GetFilterRunWaitMode(const RCM_Type* const baseAddr)
{
    rcm_filter_run_wait_modes_t retValue;
    uint32_t regValue = baseAddr->RPC;
    regValue = (regValue & RCM_RPC_RSTFLTSRW_MASK) >> RCM_RPC_RSTFLTSRW_SHIFT;
    switch(regValue)
    {
        case 0UL:
            retValue = RCM_FILTER_DISABLED;
            break;
        case 1UL:
            retValue = RCM_FILTER_BUS_CLK;
            break;
        case 2UL:
            retValue = RCM_FILTER_LPO_CLK;
            break;
        case 3UL:
        default:
            retValue = RCM_FILTER_RESERVED;
            break;
    }
    return retValue;
}

/*!
 * @brief Sets the reset pin filter width.
 *
 * This function sets the reset pin filter width.
 *
 * @param[in] baseAddr     Register base address of RCM
 * @param[in] width        to be set for reset filter width
 * Implements RCM_HAL_SetFilterWidth_Activity
 */
static inline void RCM_HAL_SetFilterWidth(RCM_Type* const baseAddr, const uint32_t width)
{
    uint32_t regValue = baseAddr->RPC;
    regValue &= ~(RCM_RPC_RSTFLTSEL_MASK);
    regValue |= RCM_RPC_RSTFLTSEL(width);
    baseAddr->RPC = regValue;
}

/*!
 * @brief Gets the reset pin filter for stop mode.
 *
 * This function gets the reset pin filter width.
 *
 * @param[in] baseAddr     Register base address of RCM
 * @return width reset filter width
 * Implements RCM_HAL_GetFilterWidth_Activity
 */
static inline uint32_t RCM_HAL_GetFilterWidth(const RCM_Type* const baseAddr)
{
    uint32_t regValue = baseAddr->RPC;
    regValue = (regValue & RCM_RPC_RSTFLTSEL_MASK) >> RCM_RPC_RSTFLTSEL_SHIFT;
    return (uint32_t)regValue;
}

/*!
 * @brief Sets reset delay time.
 *
 * This function configures the maximum reset delay time from when the interrupt is asserted.
 *
 * @param[in] baseAddr    Register base address of RCM
 * @param[in] value       Reset delay time
 * Implements RCM_HAL_SetResetDelayTimeValue_Activity
 */
static inline void RCM_HAL_SetResetDelayTimeValue(RCM_Type* const baseAddr,
                                                  const rcm_reset_delay_time_t value)
{
    uint32_t regValue = baseAddr->SRIE;
    regValue &= ~(RCM_SRIE_DELAY_MASK);
    regValue |= RCM_SRIE_DELAY(value);
    baseAddr->SRIE = regValue;
}

/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* RCM_HAL_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/

