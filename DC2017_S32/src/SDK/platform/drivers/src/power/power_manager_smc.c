/*
 * Copyright (c) 2014-2016, Freescale Semiconductor, Inc.
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


/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, There shall be no occurrence of
 * undefined or critical unspecified behaviour.
 * The addresses of the stack variables are only used at local scope.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.5, object/function previously declared.
 * This requirement is fulfilled since the functions are declared as external only in one file.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, A cast shall not be performed
 * between pointer to void and an arithmetic type.
 * The base address parameter from HAL functions is provided as integer so
 * it needs to be cast to pointer.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, A conversion should not be performed
 * between a pointer to object and an integer type.
 * The base address parameter from HAL functions is provided as integer so
 * a conversion between a pointer and an integer has to be performed
 */

  /*! @file power_manager_smc.c */

/*!
 * @defgroup power_manager Power Manager
 * @brief The S32 SDK Power Manager provides a set of API/services that enables applications
 * to configure and select among various operational and low power modes.
 * @{
 *
 * ## Hardware background ##
 *
 * System mode controller (SMC) is passing the system into and out of all low-power
 * Stop and Run modes. Controls the power, clocks and memories of the system to achieve
 * the power consumption and functionality of that mode.
 *
 *
 * ## Driver consideration ##
 *
 * Power mode entry and sleep-on-exit option are provided at
 * initialization time through the power manager user configuration structure.
 * The available power mode entries are the following ones: HSRUN, RUN, VLPR, WAIT,
 * VLPW, VLPS, PSTOP1 and PSTOP2
 *
 * This is an example of configuration:
 * @code

    power_manager_user_config_t pwrMan1_InitConfig0 = {
        .powerMode = POWER_MANAGER_HSRUN,
        .sleepOnExitOption = false,
        .sleepOnExitValue = false,
    };

    power_manager_user_config_t *powerConfigsArr[] = {
        &pwrMan1_InitConfig0
    };

    power_manager_callback_user_config_t * powerCallbacksConfigsArr[] = {(void *)0};

    if (STATUS_SUCCESS != POWER_SYS_Init(&powerConfigsArr,1,&powerCallbacksConfigsArr,0)) {
        ...
    }
    else {
        ...
    }

    if (STATUS_SUCCESS != POWER_SYS_SetMode(0,POWER_MANAGER_POLICY_AGREEMENT)) {
        ...
    }
    else {
        ...
    }

 * @endcode
 */

#include "power_manager.h"
#include "smc_hal.h"

/*! Timeout used for waiting to set new mode */
#define POWER_SET_MODE_TIMEOUT 1000U

/*! @brief Power manager internal structure. */
extern power_manager_state_t gPowerManagerState;

/*******************************************************************************
 * PROTOTYPES
 ******************************************************************************/

status_t POWER_SYS_DoInit(void);
status_t POWER_SYS_DoDeinit(void);
status_t POWER_SYS_DoSetMode(const power_manager_user_config_t * const configPtr);

static status_t POWER_SYS_WaitForModeStatus(smc_run_mode_t mode);
static status_t POWER_SYS_SwitchToSleepingPowerMode(const power_manager_user_config_t * const configPtr);
static status_t POWER_SYS_SwitchToRunningPowerMode(const power_manager_user_config_t * const configPtr);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
 *
 * It is expected that prior to the POWER_SYS_Init() call the write-once protection
 * register was configured appropriately allowing entry to all required low power
 * modes.
 * The following is an example of how to set up two power modes and one
 * callback, and initialize the Power manager with structures containing their settings.
 * The example shows two possible ways the configuration structures can be stored
 * (ROM or RAM), although it is expected that they will be placed in the read-only
 * memory to save the RAM space. (Note: In the example it is assumed that the programmed chip
 * doesn't support any optional power options described in the power_manager_user_config_t)
 * :
 * @code

  power_manager_user_config_t vlprConfig = {   vlprConfig power mode configuration
      .powerMode = POWER_MANAGER_VLPR,
      .sleepOnExitOption = false,
      .sleepOnExitValue = false,
  };

  power_manager_user_config_t stopConfig = {   stopConfig power mode configuration
      .powerMode = POWER_MANAGER_STOP,
      .sleepOnExitOption = false,
      .sleepOnExitValue = false,
  };

  power_manager_user_config_t const * powerConfigsArr[] = {    Power mode configurations array
      &vlprConfig,
      &stopConfig
  };

  power_manager_callback_user_config_t callbackCfg0 = {  Callback configuration structure callbackCfg0
      .callbackFunction                     = &callback0,
      .callbackType                         = POWER_MANAGER_CALLBACK_BEFORE_AFTER,
      .callbackData                         = (void *)0,
  };

  power_manager_callback_user_config_t const * callbacksConfigsArr[] = {  Callback configuration structures array
      &callbackCfg0
  };

  status_t callback0(power_manager_notify_struct_t * notify,   Definition of power manager callback
                                       power_manager_callback_data_t * dataPtr)
  {
    status_t ret = STATUS_SUCCESS;
    ...
    return ret;
  }

  int main(void) Main function
  {
    status_t ret = STATUS_SUCCESS;

    Calling of init method
    POWER_SYS_Init(&powerConfigsArr, 2U, &powerStaticCallbacksConfigsArr, 1U);

    Switch to VLPR mode
    ret = POWER_SYS_SetMode(MODE_VLPR,POWER_MANAGER_POLICY_AGREEMENT);

    if (ret != STATUS_SUCCESS)
    {
      return -1;
    }
    return 0;
  }

 * @endcode
 *
 *END**************************************************************************/

status_t POWER_SYS_DoInit(void)
{
    smc_power_mode_protection_config_t powerModeProtConfig =
    {
    .vlpProt    =   true,     /* Very low power mode is allowed. */
    .hsrunProt  =   true      /* High speed mode is allowed. */
    };

    /* Very low power modes and high speed mode are not protected. */
    SMC_HAL_SetProtectionMode(SMC, &powerModeProtConfig);

    return STATUS_SUCCESS;
}

status_t POWER_SYS_DoDeinit(void)
{
    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_SYS_DoSetMode
 * Description   : Configures the power mode.
 *
 * This function performs the actual implementation-specific logic to switch to one of the defined power modes.
 *END**************************************************************************/
status_t POWER_SYS_DoSetMode(const power_manager_user_config_t * const configPtr)
{
    status_t returnCode; /* Function return */

    /* Check whether the power mode is a sleeping or a running power mode */
    if (configPtr->powerMode <= POWER_MANAGER_VLPR)
    {
        /* Switch to a running power mode */
        returnCode = POWER_SYS_SwitchToRunningPowerMode(configPtr);
    }
    else
    {
        /* Switch to a sleeping power mode */
        returnCode = POWER_SYS_SwitchToSleepingPowerMode(configPtr);
    }

    return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_SYS_GetCurrentMode
 * Description   : Returns currently running power mode.
 *
 * Implements POWER_SYS_GetCurrentMode_Activity
 *END**************************************************************************/
power_manager_modes_t POWER_SYS_GetCurrentMode(void)
{
    power_manager_modes_t retVal;
    switch (SMC_HAL_GetPowerModeStatus(SMC))
    {
#if FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE
        /* High speed run mode */
        case STAT_HSRUN:
            retVal = POWER_MANAGER_HSRUN;
            break;
#endif
        /* Run mode */
        case STAT_RUN:
            retVal = POWER_MANAGER_RUN;
            break;
        /* Very low power run mode */
        case STAT_VLPR:
            retVal = POWER_MANAGER_VLPR;
            break;
        /* This should never happen - core has to be in some run mode to execute code */
        default:
            retVal = POWER_MANAGER_MAX;
            break;
    }
    return retVal;
}

/*FUNCTION**************************************************************************
 * Function Name : POWER_SYS_WaitForRunStatus
 * Description   :Internal function used by POWER_SYS_SwitchToSleepingPowerMode and
 *                POWER_SYS_SwitchToRunningPowerMode functions
 * mode           The expected running mode
 *
 *END*******************************************************************************/
static status_t POWER_SYS_WaitForModeStatus(smc_run_mode_t mode)
{
    status_t retCode;
    power_mode_stat_t modeStat;
    uint32_t i = 0U;

    switch(mode)
    {
        case SMC_RUN:
            modeStat = STAT_RUN;
            retCode = STATUS_SUCCESS;
            break;
        case SMC_VLPR:
            modeStat = STAT_VLPR;
            retCode = STATUS_SUCCESS;
            break;
#if FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE
        case SMC_HSRUN:
            modeStat = STAT_HSRUN;
            retCode = STATUS_SUCCESS;
            break;
#endif
        default:
            /* invalid parameter */
            modeStat = STAT_INVALID;
            retCode = STATUS_UNSUPPORTED;
            break;
    }
    if (STATUS_SUCCESS == retCode)
    {
        for (; i < POWER_SET_MODE_TIMEOUT ; i++)
        {
            if(SMC_HAL_GetPowerModeStatus(SMC) == modeStat)
            {
                break;
            }
        }
    }
    if (i >= POWER_SET_MODE_TIMEOUT)
    {
        retCode = STATUS_MCU_TRANSITION_FAILED;
    }
    return retCode;
}


/*FUNCTION**********************************************************************************************
 * Function Name : POWER_SYS_SwitchToRunningPowerMode
 * Description   :Internal function used by POWER_SYS_SetMode function to switch to a running power mode
 * configPtr   pointer to the requested user-defined power mode configuration
 *
 *END***************************************************************************************************/
static status_t POWER_SYS_SwitchToRunningPowerMode(const power_manager_user_config_t * const configPtr)
{
    smc_power_mode_config_t halModeConfig; /* SMC HAL layer configuration structure */
    status_t returnCode;

    /* Configure the HAL layer */
    switch (configPtr->powerMode) {
#if FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE
    /* High speed run mode */
    case POWER_MANAGER_HSRUN:
        /* High speed run mode can be entered only from Run mode */
        if (SMC_HAL_GetPowerModeStatus(SMC) != STAT_RUN)
        {
            SMC_HAL_SetRunModeControl(SMC, SMC_RUN);
            returnCode = POWER_SYS_WaitForModeStatus(SMC_RUN);
        }
        else
        {
            returnCode = STATUS_SUCCESS;
        }
        if (STATUS_SUCCESS == returnCode)
        {
            halModeConfig.powerModeName = POWER_MANAGER_HSRUN;

            /* Switch the mode */
            if (SMC_HAL_SetPowerMode(SMC, &halModeConfig) == STATUS_SUCCESS)
            {
                returnCode = POWER_SYS_WaitForModeStatus(SMC_HSRUN);
            }
            else
            {
                returnCode = STATUS_MCU_TRANSITION_FAILED;
            }
        }
        break;
#endif
    /* Run mode */
    case POWER_MANAGER_RUN:
        halModeConfig.powerModeName = POWER_MANAGER_RUN;
        returnCode = STATUS_SUCCESS;
        /* Switch the mode */
        if (SMC_HAL_SetPowerMode(SMC, &halModeConfig) == STATUS_SUCCESS)
        {
            returnCode = POWER_SYS_WaitForModeStatus(SMC_RUN);
        }
        else
        {
            returnCode = STATUS_MCU_TRANSITION_FAILED;
        }
        break;
    /* Very low power run mode */
    case POWER_MANAGER_VLPR:
        /* Very low power run mode can be entered only from Run mode */
        if (SMC_HAL_GetPowerModeStatus(SMC) != STAT_RUN)
        {
            SMC_HAL_SetRunModeControl(SMC, SMC_RUN);
            returnCode = POWER_SYS_WaitForModeStatus(SMC_RUN);
        }
        else
        {
            returnCode = STATUS_SUCCESS;
        }

        if (STATUS_SUCCESS == returnCode)
        {
            halModeConfig.powerModeName = POWER_MANAGER_VLPR;

            /* Switch the mode */
            if (SMC_HAL_SetPowerMode(SMC, &halModeConfig) == STATUS_SUCCESS)
            {
                returnCode = POWER_SYS_WaitForModeStatus(SMC_VLPR);
            }
            else
            {
                returnCode = STATUS_MCU_TRANSITION_FAILED;
            }
        }
        break;
    /* Wait mode */
    default:
        /* invalid power mode */
        returnCode = STATUS_UNSUPPORTED;
        halModeConfig.powerModeName = POWER_MANAGER_MAX;
        break;
    }

    return returnCode;
}


/*FUNCTION**********************************************************************************************
 * Function Name : POWER_SYS_SwitchToSleepingPowerMode
 * Description   :Internal function used by POWER_SYS_SetMode function to switch to a sleeping power mode
 * configPtr   pointer to the requested user-defined power mode configuration
 *
 *END***************************************************************************************************/
static status_t POWER_SYS_SwitchToSleepingPowerMode(const power_manager_user_config_t * const configPtr)
{
    smc_power_mode_config_t halModeConfig; /* SMC HAL layer configuration structure */
    status_t returnCode; /* return value */
    power_mode_stat_t pwrModeStat;         /* power mode stat */

    /* Configure the HAL layer */
    switch (configPtr->powerMode) {
#if FEATURE_SMC_HAS_WAIT_VLPW
    /* Wait mode */
    case POWER_MANAGER_WAIT:
        /* Wait mode can be entered only from Run mode */
        if (SMC_HAL_GetPowerModeStatus(SMC) != STAT_RUN)
        {
            SMC_HAL_SetRunModeControl(SMC, SMC_RUN);
            returnCode = POWER_SYS_WaitForModeStatus(SMC_RUN);
        }
        else
        {
            returnCode = STATUS_SUCCESS;
        }
        halModeConfig.powerModeName = POWER_MANAGER_WAIT;
        break;
    /* Very low power wait mode */
    case POWER_MANAGER_VLPW:
        /* Very low power wait mode can be netered only from Very low power run mode */
        if (SMC_HAL_GetPowerModeStatus(SMC) != STAT_VLPR)
        {
            SMC_HAL_SetRunModeControl(SMC, SMC_VLPR);
            returnCode = POWER_SYS_WaitForModeStatus(SMC_VLPR);
        }
        else
        {
            returnCode = STATUS_SUCCESS;
        }
        halModeConfig.powerModeName = POWER_MANAGER_VLPW;
        break;
#endif
#if FEATURE_SMC_HAS_PSTOPO
        /* Partial stop modes */
    case POWER_MANAGER_PSTOP1:
        /* fall-through */
    case POWER_MANAGER_PSTOP2:
        /* fall-through */
#endif
#if FEATURE_SMC_HAS_STOPO
    /* Stop modes */
    case POWER_MANAGER_STOP1:
    /* fall-through */
    case POWER_MANAGER_STOP2:
    /* fall-through */
#endif
    /* Stop mode */
    case POWER_MANAGER_STOP:
    /* Stop mode can be entered only from Run mode */
        if (SMC_HAL_GetPowerModeStatus(SMC) != STAT_RUN)
        {
          SMC_HAL_SetRunModeControl(SMC, SMC_RUN);
          returnCode = POWER_SYS_WaitForModeStatus(SMC_RUN);
        }
        else
        {
          returnCode = STATUS_SUCCESS;
        }
        halModeConfig.powerModeName = POWER_MANAGER_STOP;
#if FEATURE_SMC_HAS_PSTOPO
        halModeConfig.pstopOption = true;
        /* Set the partial stop option value */
        if (POWER_MANAGER_PSTOP1 == configPtr->powerMode)
        {
            halModeConfig.pstopOptionValue = SMC_PSTOP_STOP1;
        }
        else if(POWER_MANAGER_PSTOP2 == configPtr->powerMode)
        {
            halModeConfig.pstopOptionValue = SMC_PSTOP_STOP2;
        }
        else
        {
            halModeConfig.pstopOptionValue = SMC_PSTOP_STOP;
        }
#endif
#if FEATURE_SMC_HAS_STOPO
        /* Set the stop option value */
        if (POWER_MANAGER_STOP1 == configPtr->powerMode)
        {
          halModeConfig.stopOption = true;
          halModeConfig.stopOptionValue = SMC_STOP1;
        }
        else if(POWER_MANAGER_STOP2 == configPtr->powerMode)
        {
          halModeConfig.stopOption = true;
          halModeConfig.stopOptionValue = SMC_STOP2;
        }
        else
        {
          halModeConfig.stopOption = false;
        }
#endif
        break;
    /* Very low power stop mode */
    case POWER_MANAGER_VLPS:
        pwrModeStat = SMC_HAL_GetPowerModeStatus(SMC);
        /* Very low power stop mode can be entered only from Run mode or Very low power run mode*/
        if ((pwrModeStat != STAT_RUN) && (pwrModeStat != STAT_VLPR))
        {
            SMC_HAL_SetRunModeControl(SMC, SMC_RUN);
            returnCode = POWER_SYS_WaitForModeStatus(SMC_RUN);
        }
        else
        {
            returnCode = STATUS_SUCCESS;
        }
        halModeConfig.powerModeName = POWER_MANAGER_VLPS;
        break;
    default:
        /* invalid power mode */
        returnCode = STATUS_UNSUPPORTED;
        halModeConfig.powerModeName = POWER_MANAGER_MAX;
        break;
    }

    if (STATUS_SUCCESS == returnCode)
    {
        /* Configure ARM core what to do after interrupt invoked in (deep) sleep state */
        if (configPtr->sleepOnExitOption)
        {
            if (configPtr->sleepOnExitValue)
            {
                /* Go back to (deep) sleep state on ISR exit */
                S32_SCB->SCR |= S32_SCB_SCR_SLEEPONEXIT_MASK;
            }
            else
            {
                /* Do not re-enter (deep) sleep state on ISR exit */
                S32_SCB->SCR &= ~(S32_SCB_SCR_SLEEPONEXIT_MASK);
            }
        }

        /* Switch the mode */
        if (SMC_HAL_SetPowerMode(SMC, &halModeConfig) != STATUS_SUCCESS)
        {
            returnCode = STATUS_MCU_TRANSITION_FAILED;
        }
    }
    return returnCode;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

