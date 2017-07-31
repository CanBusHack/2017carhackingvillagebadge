/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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
#if !defined(PMC_HAL_H)
#define PMC_HAL_H

#include "device_registers.h"
#include <stdbool.h>

 /*!
 * @file pmc_hal.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.3, Global typedef not referenced.
 * pmc_config_t is referenced from clock manager.
 */

/*!
 * @defgroup pmc_hal Power Management Controller (PMC)
 * @ingroup power_manager
 * @brief This module covers the functionality of the Power Management Controller (PMC) peripheral.
 * <p>
 *  PMC HAL provides the API for reading and writing register bit-fields belonging to the PMC module.
 * </p>
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 *  @brief Power management control interrupts
 *  Implements pmc_int_select_t_Class
 */
typedef enum {
    PMC_INT_LOW_VOLT_DETECT,                   /*!< Low Voltage Detect Interrupt  */
    PMC_INT_LOW_VOLT_WARN                      /*!< Low Voltage Warning Interrupt */
} pmc_int_select_t;


/*! @brief PMC LPO configuration. */
typedef struct
{
    bool                  initialize;       /*!< Initialize or not the PMC LPO settings. */
    bool                  enable;           /*!< Enable/disable LPO     */
    int8_t                trimValue;        /*!< LPO trimming value     */

} pmc_lpo_clock_config_t;

/*!
 * @brief PMC configure structure.
 */
typedef struct
{
    pmc_lpo_clock_config_t    lpoClockConfig;   /*!< Low Power Clock configuration.     */

} pmc_config_t;




/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*! @name Power Management Controller Control APIs*/
/*@{*/



/*!
 * @brief Enables/Disables the low voltage-related interrupts.
 *
 * This function  enables  the low voltage detection, warning,
 * etc. interrupts.
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @param[in] intSelect interrupt select
 * @param[in] enable    enable/disable the interrupt
 */
void PMC_HAL_SetLowVoltIntCmd(PMC_Type* const baseAddr, const pmc_int_select_t intSelect, const bool enable);


/*!
 * @brief Acknowledges the low voltage-related interrupts.
 *
 * This function  acknowledges  the low voltage detection, warning,
 * etc. interrupts
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @param[in] intSelect interrupt select
 */
void PMC_HAL_SetLowVoltIntAckCmd(PMC_Type* const baseAddr, const pmc_int_select_t intSelect);


/*!
 * @brief Gets the flag for the low voltage-related interrupts.
 *
 * This function gets the flag for the low voltage detection, warning,
 * etc. interrupts
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @param[in] intSelect interrupt select
 * @return status Current low voltage interrupt flag
 *                - true: Low-Voltage Interrupt flag is set
 *                - false: Low-Voltage Interrupt flag is not set
 */
bool PMC_HAL_GetLowVoltIntFlag(const PMC_Type* const baseAddr, const pmc_int_select_t intSelect);


/*!
 * @brief Low-Voltage Detect Hardware Reset Enable/Disable (write once)
 *
 * This function enables/disables the  hardware reset for the low voltage
 * detection. When enabled, if the LVDF (Low Voltage Detect Flag) is set, a
 * hardware reset occurs. This setting is a write-once-only. Any additional writes
 * are ignored.
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @param[in] enable    enable/disable the LVD hardware reset
 * Implements PMC_HAL_SetLowVoltDetectResetCmd_Activity
 */
static inline void PMC_HAL_SetLowVoltDetectResetCmd(PMC_Type* const baseAddr, const bool enable)
{
    uint8_t regValue = baseAddr->LVDSC1;
    regValue &= (uint8_t)(~(PMC_LVDSC1_LVDRE_MASK));
    regValue |= (uint8_t)PMC_LVDSC1_LVDRE(enable ? 1U : 0U);
    baseAddr->LVDSC1 = regValue;
}


/*!
 * @brief Enables/Disables the Bias.
 *
 * This function  enables/disables the Bias.
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @param[in] enable    enable/disable the Bias.
 * Implements PMC_HAL_SetBiasMode_Activity
 */
static inline void PMC_HAL_SetBiasMode(PMC_Type* const baseAddr, const bool enable)
{
    uint8_t regValue = baseAddr->REGSC;
    regValue &= (uint8_t)(~(PMC_REGSC_BIASEN_MASK));
    regValue |= (uint8_t)PMC_REGSC_BIASEN(enable?1U:0U);
    baseAddr->REGSC = regValue;
}

/*!
 * @brief Enables/Disables the Low Power Oscillator.
 *
 * This function  enables/disables the Low Power Oscillator.
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @param[in] enable    enable/disable the Low Power Oscillator.
 * Implements PMC_HAL_SetLpoMode_Activity
 */
static inline void PMC_HAL_SetLpoMode(PMC_Type* const baseAddr, const bool enable)
{
    uint8_t regValue = baseAddr->REGSC;
    regValue &= (uint8_t)(~(PMC_REGSC_LPODIS_MASK));
    regValue |= (uint8_t)PMC_REGSC_LPODIS(enable?0U:1U);
    baseAddr->REGSC = regValue;
}


/*!
 * @brief Gets the Low Power Oscillator status.
 *
 * This function gets the Low Power Oscillator status.
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @return value LPO status
 *               false - LPO is disabled
 *               true - LPO is enabled
 * Implements PMC_HAL_GetLpoMode_Activity
 */
static inline bool PMC_HAL_GetLpoMode(const PMC_Type * const baseAddr)
{
    uint8_t regValue = baseAddr->REGSC;
    regValue = (uint8_t)((regValue & PMC_REGSC_LPODIS_MASK) >> PMC_REGSC_LPODIS_SHIFT);
    return (regValue == 0U) ? true : false;
}


/*!
 * @brief Gets the Regulator regulation status.
 *
 * This function provides the current status of
 * the internal voltage regulator.
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @return value Regulation status
 *               0 - Regulator is in low power mode or transition to/from.
 *               1 - Regulator is in full performance mode.
 * Implements PMC_HAL_GetRegulatorStatus_Activity
 */
static inline uint8_t PMC_HAL_GetRegulatorStatus(const PMC_Type* const baseAddr)
{
    uint8_t regValue = baseAddr->REGSC;
    regValue = (uint8_t)((regValue & PMC_REGSC_REGFPM_MASK) >> PMC_REGSC_REGFPM_SHIFT);
    return regValue;
}

/*!
 * @brief Gets the Low Power Oscillator status.
 *
 * This function provides the current status of
 * the Low Power Oscillator.
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @return value LPO status
 *               0 - Low power oscillator in low phase.
 *               1 - Low power oscillator in high phase.
 * Implements PMC_HAL_GetLpoStatus_Activity
 */
static inline uint8_t PMC_HAL_GetLpoStatus(const PMC_Type* const baseAddr)
{
    uint8_t regValue = baseAddr->REGSC;
    regValue = (uint8_t)((regValue & PMC_REGSC_LPOSTAT_MASK) >> PMC_REGSC_LPOSTAT_SHIFT);
    return regValue;
}

/*!
 * @brief Low Power Oscillator Trimming Value
 *
 * This function sets the trimming value for the low power oscillator
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @param[in] value     Trimming value
 * Implements PMC_HAL_SetLpoTrimValue_Activity
 */
static inline void PMC_HAL_SetLpoTrimValue(PMC_Type* const baseAddr, const int8_t decimalValue)
{
    int8_t decValue = decimalValue;
    uint8_t lpotrim, trimval, regValue;

    if (decValue < 0)
    {
        lpotrim = ((uint8_t)1U) << (PMC_LPOTRIM_LPOTRIM_WIDTH);
        decValue = (int8_t)(decValue + (int8_t)(lpotrim));
    }
    trimval = (uint8_t)decValue;

    DEV_ASSERT(trimval <= (1U << (PMC_LPOTRIM_LPOTRIM_WIDTH - 1U)));

    regValue = baseAddr->LPOTRIM;
    regValue &= (uint8_t)(~(PMC_LPOTRIM_LPOTRIM_MASK));
    regValue |= (uint8_t)PMC_LPOTRIM_LPOTRIM(trimval);
    baseAddr->LPOTRIM = regValue;
}

/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* PMC_HAL_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/

