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



/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, A cast shall not be performed
 * between pointer to void and an arithmetic type.
 * The address of hardware modules is provided as integer so
 * it needs to be cast to pointer.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, A conversion should not be performed
 * between a pointer to object and an integer type.
 * The address of hardware modules is provided as integer so
 * a conversion between a pointer and an integer has to be performed
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application or driver code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower or different essential type.
 * The cast is required to perform a conversion between an unsigned integer and an enum type with many values.
 */

 /*! @file smc_hal.c */

/*!
 * @file smc_hal.c
 */

#include "smc_hal.h"
#include <stddef.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * PROTOTYPES
 ******************************************************************************/

static bool SMC_HAL_WaitForStatChange(const SMC_Type* const baseAddr, const power_mode_stat_t mode, const uint32_t timeout);

/*! Timeout used for waiting to set new mode */
#define SMC_TIMEOUT 1000U


/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : SMC_HAL_GetVersion
 * Description   : Get the version of the SMC module
 * This function will get the version of the SMC module.
 * function for more details.
 *
 * Implements SMC_HAL_GetVersion_Activity
 *END**************************************************************************/
void SMC_HAL_GetVersion(const SMC_Type* const baseAddr, smc_version_info_t* const versionInfo)
{
    uint32_t regValue;

    regValue = baseAddr->VERID;
    regValue = (regValue & SMC_VERID_MAJOR_MASK) >> SMC_VERID_MAJOR_SHIFT;
    versionInfo->majorNumber = regValue;

    regValue = baseAddr->VERID;
    regValue = (regValue & SMC_VERID_MINOR_MASK) >> SMC_VERID_MINOR_SHIFT;
    versionInfo->minorNumber = regValue;

    regValue = baseAddr->VERID;
    regValue = (regValue & SMC_VERID_FEATURE_MASK) >> SMC_VERID_FEATURE_SHIFT;
    versionInfo->featureNumber = regValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMC_HAL_SetPowerMode
 * Description   : Config the power mode
 * This function will configure the power mode control for any run, stop and
 * stop submode if needed. It will also configure the power options for specific
 * power mode. Application should follow the proper procedure to configure and
 * switch power mode between the different run and stop mode. Refer to reference
 * manual for the proper procedure and supported power mode that can be configured
 * and switch between each other. Refer to smc_power_mode_config_t for required
 * parameters to configure the power mode and the supported options. Other options
 * may need to configure through the hal driver individaully. Refer to hal driver
 * header for details.
 *
 * Implements SMC_HAL_SetPowerMode_Activity
 *END**************************************************************************/
status_t SMC_HAL_SetPowerMode(SMC_Type* const baseAddr,
                              const smc_power_mode_config_t* const powerModeConfig)
{
    status_t retCode;
    smc_stop_mode_t stopMode;
    power_manager_modes_t powerModeName = powerModeConfig->powerModeName;

    /*! @brief Power mode transition table
     *         Specifies valid power modes for transitioning to the next mode (in comment)
     */
    static const uint16_t transitionTable[] =
    {
      (uint16_t)STAT_RUN,                                       /* Next mode POWER_MANAGER_HSRUN      */
      (((uint16_t)STAT_VLPR) | ((uint16_t)STAT_HSRUN)),         /* Next mode POWER_MANAGER_RUN        */
      (uint16_t)STAT_RUN,                                       /* Next mode POWER_MANAGER_VLPR       */
      (uint16_t)STAT_RUN,                                       /* Next mode POWER_MANAGER_WAIT       */
      (uint16_t)STAT_VLPR,                                      /* Next mode POWER_MANAGER_VLPW       */
      (uint16_t)STAT_RUN,                                       /* Next mode POWER_MANAGER_STOP       */
      (((uint16_t)STAT_RUN) | ((uint16_t)STAT_VLPR)),           /* Next mode POWER_MANAGER_VLPS       */
    };
    (void) transitionTable;


    /* Verify the power mode name*/
    DEV_ASSERT((size_t)powerModeName < (sizeof(transitionTable) / sizeof(transitionTable[0])));
    /* Check the current and the next power mode in the transitioning table */
    DEV_ASSERT((((uint16_t)SMC_HAL_GetPowerModeStatus(baseAddr)) & transitionTable[powerModeName]) != 0U);

    /* Branch based on power mode name*/
    switch (powerModeName)
    {
    case POWER_MANAGER_RUN:
        /* Set to RUN mode. */
        SMC_HAL_SetRunModeControl(baseAddr, SMC_RUN);
        /* Wait for stat change */
        if (!SMC_HAL_WaitForStatChange(baseAddr,STAT_RUN,SMC_TIMEOUT))
        {
            /* Timeout for power mode change expired. */
            retCode = STATUS_TIMEOUT;
            break;
        }
        retCode = STATUS_SUCCESS;
        break;

    case POWER_MANAGER_VLPR:
        /* "Very-Low-Power Modes" is allowed? */
        DEV_ASSERT(SMC_HAL_GetProtectionMode(baseAddr, ALLOW_VLP));

        /* Set power mode to VLPR*/
        SMC_HAL_SetRunModeControl(baseAddr, SMC_VLPR);
        /* Wait for stat change */
        if (!SMC_HAL_WaitForStatChange(baseAddr,STAT_VLPR,SMC_TIMEOUT))
        {
            /* Timeout for power mode change expired. */
            retCode = STATUS_TIMEOUT;
            break;
        }
        retCode = STATUS_SUCCESS;
        break;

#if FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE
    case POWER_MANAGER_HSRUN:
        /* "High Speed Mode" is allowed? */
        DEV_ASSERT(SMC_HAL_GetProtectionMode(baseAddr, ALLOW_HSRUN));

        /* Set power mode to HSRUN */
        SMC_HAL_SetRunModeControl(baseAddr, SMC_HSRUN);
        /* Wait for stat change */
        if (!SMC_HAL_WaitForStatChange(baseAddr,STAT_HSRUN,SMC_TIMEOUT))
        {
            /* Timeout for power mode change expired. */
            retCode = STATUS_TIMEOUT;
            break;
        }
        retCode = STATUS_SUCCESS;
        break;
#endif
#if FEATURE_SMC_HAS_WAIT_VLPW
    case POWER_MANAGER_WAIT:
        /* Fall-through */
    case POWER_MANAGER_VLPW:
        /* "Very-Low-Power Modes" is allowed? */
        if (powerModeName == POWER_MODE_VLPW)
        {
            DEV_ASSERT(SMC_HAL_GetProtectionMode(baseAddr, ALLOW_VLP));
        }

        /* Clear the SLEEPDEEP bit to disable deep sleep mode - WAIT */
        S32_SCB->SCR &= ~S32_SCB_SCR_SLEEPDEEP_MASK;

        /* Cpu is going into sleep state */
        STANDBY();

        retCode = STATUS_SUCCESS;
        break;
#endif
    case POWER_MANAGER_STOP:
        /* Fall-through */
    case POWER_MANAGER_VLPS:
        if (powerModeName == POWER_MANAGER_STOP)
        {
            stopMode = SMC_STOP;

#if FEATURE_SMC_HAS_STOPO
            if (powerModeConfig->stopOption)
            {

                SMC_HAL_SetStopOption(baseAddr, powerModeConfig->stopOptionValue);
            }
#endif
#if FEATURE_SMC_HAS_PSTOPO
            if (powerModeConfig->pStopOption)
            {
                SMC_HAL_SetPStopOption(baseAddr, powerModeConfig->pStopOptionValue);
            }
#endif
        }
        else
        {
            stopMode = SMC_VLPS;

            /* "Very-Low-Power Modes" is allowed? */
            DEV_ASSERT(SMC_HAL_GetProtectionMode(baseAddr, ALLOW_VLP));
        }

        /* Set power mode to specified STOP mode*/
        SMC_HAL_SetStopModeControl(baseAddr, stopMode);

        /* Set the SLEEPDEEP bit to enable deep sleep mode (STOP)*/
        S32_SCB->SCR |= S32_SCB_SCR_SLEEPDEEP_MASK;

        /* Cpu is going into deep sleep state */
        STANDBY();

        retCode = STATUS_SUCCESS;
        break;
    default:
        retCode = STATUS_UNSUPPORTED;
        break;
    }
    return retCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMC_HAL_SetProtectionMode
 * Description   : Config all power mode protection settings
 * This function will configure the power mode protection settings for
 * supported power mode on the specified chip family. The availabe power modes
 * are defined in smc_power_mode_protection_config_t. Application should provide
 * the protect settings for all supported power mode on the chip and aslo this
 * should be done at early system level init stage. Refer to reference manual
 * for details. This register can only write once after power reset. So either
 * use this function or use the individual set function if you only have single
 * option to set.
 *
 * Implements SMC_HAL_SetProtectionMode_Activity
 *END**************************************************************************/
void SMC_HAL_SetProtectionMode(SMC_Type* const baseAddr,
                               const smc_power_mode_protection_config_t* const protectConfig)
{
    /* Initialize the setting */
    uint32_t regValue = 0U;

    /* Check configurations for each mode and combine the setting together */
    if (protectConfig->vlpProt)
    {
        regValue |= SMC_PMPROT_AVLP(1);
    }

#if FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE
    if (protectConfig->hsrunProt)
    {
        regValue |= SMC_PMPROT_AHSRUN(1);
    }
#endif

    /* Write once into PMPROT register*/
    baseAddr->PMPROT = regValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMC_HAL_GetProtectionMode
 * Description   : Get the current power mode protection setting
 * This function will get the current power mode protection settings for
 * a specified power mode.
 *
 * Implements SMC_HAL_GetProtectionMode_Activity
 *END**************************************************************************/
bool SMC_HAL_GetProtectionMode(const SMC_Type* const baseAddr, const power_modes_protect_t protect)
{
    bool retValue;
    uint32_t regValue = (uint32_t)baseAddr->PMPROT;

    /* Check the mode range */
    DEV_ASSERT(protect < ALLOW_MAX);

    /* Branch according to the mode and read the setting */
    switch (protect)
    {
        case ALLOW_VLP:
            regValue = (regValue & SMC_PMPROT_AVLP_MASK) >> SMC_PMPROT_AVLP_SHIFT;
            break;
#if FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE
        case ALLOW_HSRUN:
            regValue = (regValue & SMC_PMPROT_AHSRUN_MASK) >> SMC_PMPROT_AHSRUN_SHIFT;
            break;
#endif
        default:
            /* Invalid command */
            regValue = 0UL;
            break;
    }
    retValue = (regValue == 0UL) ? false : true;
    return retValue;
}


/*FUNCTION**********************************************************************
 * Function Name : SMC_HAL_WaitForStatChange
 * Description   : Internal function used by SMC_HAL_SetPowerMode function
 * to wait until the state is changed or timeout expires
 *
 * return power mode status change
 *                - true: power mode has been changed successfully
 *                - false: timeout expired, power mode has not been changed
 *END**************************************************************************/
static bool SMC_HAL_WaitForStatChange(const SMC_Type* const baseAddr, const power_mode_stat_t mode, const uint32_t timeout)
{
    uint32_t i;
    bool retValue;

    for (i = 0U; i < timeout; i++)
    {
        if (mode == SMC_HAL_GetPowerModeStatus(baseAddr))
        {
            /* Power mode has been changed successfully */
            break;
        }
    }

    /* If i greater or equal to timeout, then timeout expired (the power mode has not been changed)*/
    retValue = (i < timeout);

    return retValue;
}



/*******************************************************************************
 * EOF
 ******************************************************************************/

