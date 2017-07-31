/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
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
 * Violates MISRA 2012 Required Rule 1.3, There shall be no occurrence of
 * undefined or critical unspecified behaviour.
 * The addresses of the stack variables are only used at local scope.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower or different essential type.
 * The cast is required to perform a conversion between an unsigned integer and an enum type.
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
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 */


#include "clock_S32K144.h"
#include "clock_manager.h"
#include "device_registers.h"
#include <stddef.h>   /* This header is included for bool type */
/*
 * README:
 * This file provides these APIs:
 * 1. APIs to get the frequency of output clocks in Reference Manual ->
 * Chapter Clock Distribution -> Figure Clocking diagram.
 * 2. APIs for IP modules listed in Reference Manual -> Chapter Clock Distribution
 * -> Module clocks.
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* This frequency values should be set by different boards. */
/* SIM */
uint32_t g_TClkFreq[NUMBER_OF_TCLK_INPUTS];      /* TCLKx clocks    */

/* RTC */
uint32_t g_RtcClkInFreq;                         /* RTC CLKIN clock */

/* SCG */
uint32_t g_xtal0ClkFreq;                         /* EXTAL0 clock    */

/*******************************************************************************
 * INTERNAL FUNCTIONS
 ******************************************************************************/

static status_t CLOCK_SYS_GetScgClockFreq(clock_names_t clockName, uint32_t * frequency);
static status_t CLOCK_SYS_GetSimClockFreq(clock_names_t clockName, uint32_t * frequency);
static status_t CLOCK_SYS_GetPccClockFreq(clock_names_t clockName, uint32_t * frequency);
static uint32_t CLOCK_SYS_GetPeripheralClock(clock_names_t clockName, scg_async_clock_type_t divider);
static scg_system_clock_mode_t CLOCK_SYS_GetCurrentRunMode(const SMC_Type * smc_base);
static status_t CLOCK_SYS_TransitionSystemClock(const scg_system_clock_config_t *to_clk);
static uint32_t CLOCK_SYS_GetSimClkOutFreq(const SIM_Type * base);
static uint32_t CLOCK_SYS_GetScgClkOutFreq(const SCG_Type * base);
static uint32_t CLOCK_SYS_GetSimRtcClkFreq(const SIM_Type * base);
static status_t CLOCK_SYS_ConfigureTemporarySystemClock(void);
static status_t CLOCK_SYS_ConfigureModulesFromScg(const scg_config_t *scgConfig);
static status_t CLOCK_SYS_ConfigureSIRC(const scg_sirc_config_t *sircConfig);
static status_t CLOCK_SYS_ConfigureFIRC(const scg_firc_config_t *fircConfig);
static status_t CLOCK_SYS_ConfigureSOSC(const scg_sosc_config_t *soscConfig);
static status_t CLOCK_SYS_ConfigureSPLL(const scg_spll_config_t *spllConfig);

/*******************************************************************************
 * Code
 ******************************************************************************/


/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_SetConfiguration
 * Description   : This function sets the system to target configuration, it
 * only sets the clock modules registers for clock mode change, but not send
 * notifications to drivers.
 *
 * Implements CLOCK_SYS_SetConfiguration_Activity
 *END**************************************************************************/
status_t CLOCK_SYS_SetConfiguration(clock_manager_user_config_t const* config)
{
    status_t result;
    DEV_ASSERT(config != NULL);

    /* Set SCG settings. */
    result = CLOCK_SYS_SetScgConfiguration(&config->scgConfig);

    if (STATUS_SUCCESS == result)
    {
        /* Set PCC settings. */
        CLOCK_SYS_SetPccConfiguration(&config->pccConfig);

        /* Set SIM settings. */
        CLOCK_SYS_SetSimConfiguration(&config->simConfig);

        /* Set PMC settings. */
        CLOCK_SYS_SetPmcConfiguration(&config->pmcConfig);
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_SetScgConfiguration
 * Description   : This function configures the SCG blocks
 *
 * Implements CLOCK_SYS_SetScgConfiguration_Activity
 *END**************************************************************************/
status_t CLOCK_SYS_SetScgConfiguration(const scg_config_t *scgConfig)
{
    status_t status = STATUS_SUCCESS;
    DEV_ASSERT(scgConfig != NULL);

    if (scgConfig != NULL)
    {
        /* Configure a temporary system clock source: FIRC */
        status = CLOCK_SYS_ConfigureTemporarySystemClock();

        if (status == STATUS_SUCCESS)
        {
            /* Configure clock sources from SCG */
            status = CLOCK_SYS_ConfigureModulesFromScg(scgConfig);
        }

        if (status == STATUS_SUCCESS)
        {
            /* Configure RTC. */
            if (scgConfig->rtcConfig.initialize )
            {
                /* RTC Clock settings. */
                SCG_HAL_SetRtcClkInFreq(SCG, scgConfig->rtcConfig.rtcClkInFreq);
            }

            /* Configure SCG ClockOut. */
            if (scgConfig->clockOutConfig.initialize)
            {
                /* ClockOut settings. */
                SCG_HAL_SetClockoutSourceSel(SCG, scgConfig->clockOutConfig.source);
            }

            /* Configure SCG clock modes. */
            if (scgConfig->clockModeConfig.initialize)
            {
                /* Configure SCG clock modes */
                status = SCG_HAL_SetSystemClockConfig(SCG, SCG_SYSTEM_CLOCK_MODE_RUN,   &(scgConfig->clockModeConfig.rccrConfig));
                if (status == STATUS_SUCCESS)
                {
                    status = SCG_HAL_SetSystemClockConfig(SCG, SCG_SYSTEM_CLOCK_MODE_VLPR,  &(scgConfig->clockModeConfig.vccrConfig));
                }
                if (status == STATUS_SUCCESS)
                {
                    status = SCG_HAL_SetSystemClockConfig(SCG, SCG_SYSTEM_CLOCK_MODE_HSRUN, &(scgConfig->clockModeConfig.hccrConfig));
                }
            }
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_SetPccConfiguration
 * Description   : This function configures the PCC block
 *
 * Implements CLOCK_SYS_SetPccConfiguration_Activity
 *END**************************************************************************/
void CLOCK_SYS_SetPccConfiguration(const pcc_config_t *peripheralClockConfig)
{
    DEV_ASSERT(peripheralClockConfig != NULL);
    PCC_HAL_SetPeripheralClockConfig(PCC, peripheralClockConfig);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_SetSimConfiguration
 * Description   : This function configures the SIM block
 *
 * Implements CLOCK_SYS_SetSimConfiguration_Activity
 *END**************************************************************************/
void CLOCK_SYS_SetSimConfiguration(const sim_clock_config_t *simClockConfig)
{
    DEV_ASSERT(simClockConfig != NULL);
    uint8_t i;

    /* ClockOut settings. */
    if (simClockConfig->clockOutConfig.initialize)
    {
        SIM_HAL_InitClkout(SIM, &(simClockConfig->clockOutConfig));
    }

    /* Low Power Clock settings from SIM. */
    if (simClockConfig->lpoClockConfig.initialize)
    {
        SIM_HAL_SetLpoClocks(SIM, simClockConfig->lpoClockConfig);
    }

    /* Platform Gate Clock settings. */
    if (simClockConfig->platGateConfig.initialize)
    {
        SIM_HAL_SetMscmClockGate(SIM, simClockConfig->platGateConfig.enableMscm);
        SIM_HAL_SetMpuClockGate(SIM,  simClockConfig->platGateConfig.enableMpu);
        SIM_HAL_SetDmaClockGate(SIM,  simClockConfig->platGateConfig.enableDma);
        SIM_HAL_SetErmClockGate(SIM,  simClockConfig->platGateConfig.enableErm);
        SIM_HAL_SetEimClockGate(SIM,  simClockConfig->platGateConfig.enableEim);
    }

    /* TCLK Clock settings. */
    if (simClockConfig->tclkConfig.initialize)
    {
        for( i = 0; i< NUMBER_OF_TCLK_INPUTS; i++)
        {
            SIM_HAL_SetTClkFreq(SIM, i, simClockConfig->tclkConfig.tclkFreq[i]);
        }
    }

    /* Debug trace Clock settings. */
    if (simClockConfig->traceClockConfig.initialize)
    {
        SIM_HAL_InitTraceClock(SIM, &(simClockConfig->traceClockConfig));
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_SetPmcConfiguration
 * Description   : This function configures the PMC block
 *
 * Implements CLOCK_SYS_SetPmcConfiguration_Activity
 *END**************************************************************************/
void CLOCK_SYS_SetPmcConfiguration(const pmc_config_t *pmcConfig)
{
    DEV_ASSERT(pmcConfig != NULL);

    /* Low Power Clock settings from PMC. */
    if (pmcConfig->lpoClockConfig.initialize)
    {
        /* Enable/disable the low power oscillator. */
        PMC_HAL_SetLpoMode(PMC,pmcConfig->lpoClockConfig.enable);

        /* Write trimming value. */
        PMC_HAL_SetLpoTrimValue(PMC,pmcConfig->lpoClockConfig.trimValue);
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetScgClockFreq
 * Description   : This function returns the frequency of a given clock from SCG
 *
 *END**************************************************************************/
static status_t CLOCK_SYS_GetScgClockFreq(clock_names_t clockName,
                                          uint32_t *frequency)
{
    status_t returnCode = STATUS_SUCCESS;
    uint32_t freq = 0U;

    switch (clockName)
    {
        /* Main clocks */
        case CORE_CLOCK:
            freq = SCG_HAL_GetSystemClockFreq(SCG,SCG_SYSTEM_CLOCK_CORE);
            break;
        case BUS_CLOCK:
            freq = SCG_HAL_GetSystemClockFreq(SCG,SCG_SYSTEM_CLOCK_BUS);
            break;
        case SLOW_CLOCK:
            freq = SCG_HAL_GetSystemClockFreq(SCG,SCG_SYSTEM_CLOCK_SLOW);
            break;
        case CLKOUT_CLOCK:
            freq = CLOCK_SYS_GetSimClkOutFreq(SIM);
            break;

        /* Other internal clocks used by peripherals. */
        case SIRC_CLOCK:
            freq = SCG_HAL_GetSircFreq(SCG);
            break;
        case FIRC_CLOCK:
            freq = SCG_HAL_GetFircFreq(SCG);
            break;
        case SOSC_CLOCK:
            freq = SCG_HAL_GetSysOscFreq(SCG);
            break;
        case SPLL_CLOCK:
            freq = SCG_HAL_GetSysPllFreq(SCG);
            break;
        case RTC_CLKIN_CLOCK:
            freq = SCG_HAL_GetRtcClkInFreq(SCG);
            break;
        case SCG_CLKOUT_CLOCK:
            freq = CLOCK_SYS_GetScgClkOutFreq(SCG);
            break;
        default:
            returnCode = STATUS_UNSUPPORTED;
            break;
    }

    if (frequency != NULL)
    {
        *frequency = freq;
    }

    return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetSimClockFreq
 * Description   : This function returns the frequency of a given clock from SIM
 *
 *END**************************************************************************/
static status_t CLOCK_SYS_GetSimClockFreq(clock_names_t clockName,
                                          uint32_t *frequency)
{
    status_t returnCode = STATUS_SUCCESS;
    uint32_t freq = 0U;

    switch (clockName)
    {
        /* SIM clocks */
        case SIM_FTM0_CLOCKSEL:
            freq = SIM_HAL_GetTClkFreq(SIM,(uint8_t)SIM_HAL_GetFtmExternalClkPinMode(SIM, 0U));
            break;
        case SIM_FTM1_CLOCKSEL:
            freq = SIM_HAL_GetTClkFreq(SIM,(uint8_t)SIM_HAL_GetFtmExternalClkPinMode(SIM, 1U));
            break;
        case SIM_FTM2_CLOCKSEL:
            freq = SIM_HAL_GetTClkFreq(SIM,(uint8_t)SIM_HAL_GetFtmExternalClkPinMode(SIM, 2U));
            break;
        case SIM_FTM3_CLOCKSEL:
            freq = SIM_HAL_GetTClkFreq(SIM,(uint8_t)SIM_HAL_GetFtmExternalClkPinMode(SIM, 3U));
            break;
        case SIM_CLKOUTSELL:
            freq = CLOCK_SYS_GetSimClkOutFreq(SIM);
            break;
        case SIM_RTCCLK_CLOCK:
            freq = CLOCK_SYS_GetSimRtcClkFreq(SIM);
            break;
        case SIM_LPO_CLOCK:
            if (PMC_HAL_GetLpoMode(PMC))
            {
                freq = SIM_HAL_GetLpoFreq(SIM);
            }
            break;
        case SIM_LPO_1K_CLOCK:
            if (PMC_HAL_GetLpoMode(PMC))
            {
                freq = SIM_HAL_GetLpo1KFreq(SIM);
            }
            break;
        case SIM_LPO_32K_CLOCK:
            if (PMC_HAL_GetLpoMode(PMC))
            {
                freq = SIM_HAL_GetLpo32KFreq(SIM);
            }
            break;
        case SIM_LPO_128K_CLOCK:
            if (PMC_HAL_GetLpoMode(PMC))
            {
                freq = SIM_HAL_GetLpo128KFreq(SIM);
            }
            break;
        case SIM_EIM_CLOCK:
            if (!SIM_HAL_GetEimClockGate(SIM))
            {
                /* EIM is not clocked. */
                returnCode = STATUS_MCU_GATED_OFF;
            }
            break;
        case SIM_ERM_CLOCK:
            if (!SIM_HAL_GetErmClockGate(SIM))
            {
                /* ERM is not clocked. */
                returnCode = STATUS_MCU_GATED_OFF;
            }
            break;
        case SIM_DMA_CLOCK:
            if (!SIM_HAL_GetDmaClockGate(SIM))
            {
                /* DMA is not clocked. */
                returnCode = STATUS_MCU_GATED_OFF;
            }
            break;
        case SIM_MPU_CLOCK:
            if (!SIM_HAL_GetMpuClockGate(SIM))
            {
                /* MPU is not clocked. */
                returnCode = STATUS_MCU_GATED_OFF;
            }
            break;
        case SIM_MSCM_CLOCK:
            if (!SIM_HAL_GetMscmClockGate(SIM))
            {
                /* MSCM is not clocked. */
                returnCode = STATUS_MCU_GATED_OFF;
            }
            break;
        default:
            returnCode = STATUS_UNSUPPORTED;
            break;
    }

    if (frequency != NULL)
    {
        *frequency = freq;
    }

    return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetPccClockFreq
 * Description   : This function returns the clock frequency of peripheral functional clock.
 *END**************************************************************************/
static status_t CLOCK_SYS_GetPccClockFreq(clock_names_t clockName,
                                          uint32_t *frequency)
{   bool clockMode;
    status_t returnCode = STATUS_SUCCESS;
    uint32_t freq = 0U;



    /* Invalid PCC clock names */
    if ((clockName <= SIM_END_OF_CLOCKS) ||
        (clockName == PCC_END_OF_BUS_CLOCKS) ||
        (clockName == PCC_END_OF_SYS_CLOCKS) ||
        (clockName == PCC_END_OF_SLOW_CLOCKS) ||
        (clockName == PCC_END_OF_ASYNCH_DIV1_CLOCKS) ||
        (clockName == PCC_END_OF_ASYNCH_DIV2_CLOCKS))
    {
        returnCode = STATUS_UNSUPPORTED;
    }
    else
    {
        clockMode = PCC_HAL_GetClockMode(PCC,clockName);
        if (!clockMode)
        {
            /* Module is not clocked. */
            returnCode = STATUS_MCU_GATED_OFF;
        }
        /* Interface clock is BUS CLOCK (peripheral is clocked by BUS CLOCK). */
        else if (clockName < PCC_END_OF_BUS_CLOCKS)
        {
            /* Check whether BUS CLOCK is clocked. */
            returnCode = (status_t)((SCG_HAL_GetSystemClockFreq(SCG, SCG_SYSTEM_CLOCK_BUS) == 0U) ? STATUS_MCU_GATED_OFF : STATUS_SUCCESS);
        }
        /* Interface clock is SYS CLOCK (peripheral is clocked by SYS CLOCK). */
        else if (clockName < PCC_END_OF_SYS_CLOCKS)
        {
            /* Check whether SYS CLOCK is clocked. */
            returnCode = (status_t)((SCG_HAL_GetSystemClockFreq(SCG, SCG_SYSTEM_CLOCK_CORE) == 0U) ? STATUS_MCU_GATED_OFF : STATUS_SUCCESS);
        }
        /* Interface clock is SLOW CLOCK (peripheral is clocked by SLOW CLOCK). */
        else if (clockName < PCC_END_OF_SLOW_CLOCKS)
        {
            /* Check whether SLOW CLOCK is clocked. */
            returnCode = (status_t)((SCG_HAL_GetSystemClockFreq(SCG, SCG_SYSTEM_CLOCK_SLOW) == 0U) ? STATUS_MCU_GATED_OFF : STATUS_SUCCESS);
        }
        /* Peripheral supports functional clock that is clocked by asynchronous source 1th divider */
        else if (clockName < PCC_END_OF_ASYNCH_DIV1_CLOCKS)
        {
            /* Interface clock is SYS_CLK. Check whether SYS CLOCK is clocked. */
            if (SCG_HAL_GetSystemClockFreq(SCG, SCG_SYSTEM_CLOCK_CORE) != 0U)
            {
                /* Check whether the functional clock is clocked */
                freq = CLOCK_SYS_GetPeripheralClock(clockName, SCG_ASYNC_CLOCK_DIV1);
                if (freq == 0U)
                {
                    returnCode = STATUS_MCU_GATED_OFF;
                }
            }
            else
            {
                returnCode = STATUS_MCU_GATED_OFF;
            }
        }
        /* Peripheral supports functional clock that is clocked by asynchronous source 2nd divider */
        else  /* clockName < PCC_END_OF_ASYNCH_DIV2_CLOCKS */
        {
            /* Interface clock is BUS_CLK. Check whether BUS CLOCK is clocked. */
            if (SCG_HAL_GetSystemClockFreq(SCG, SCG_SYSTEM_CLOCK_BUS) != 0U)
            {
                /* Check whether the functional clock is clocked */
                freq = CLOCK_SYS_GetPeripheralClock(clockName, SCG_ASYNC_CLOCK_DIV2);
                if (freq == 0U)
                {
                    returnCode = STATUS_MCU_GATED_OFF;
                }
            }
            else
            {
                returnCode = STATUS_MCU_GATED_OFF;
            }
        }
    }

    /* If frequency reference is provided, write this value */
    if (frequency != NULL)
    {
        *frequency = freq;
    }

    return returnCode;
}




/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetFreq
 * Description   : This function returns the frequency of a given clock
 *
 * Implements CLOCK_SYS_GetFreq_Activity
 *END**************************************************************************/
status_t CLOCK_SYS_GetFreq(clock_names_t clockName,
                           uint32_t *frequency)
{
    status_t returnCode;

    /* Frequency of the clock name from SCG */
    if (clockName < SCG_END_OF_CLOCKS)
    {
        returnCode = CLOCK_SYS_GetScgClockFreq(clockName, frequency);
    }
    /* Frequency of the clock name from SIM */
    else if (clockName < SIM_END_OF_CLOCKS)
    {
        returnCode = CLOCK_SYS_GetSimClockFreq(clockName, frequency);
    }
    /* Frequency of the clock name from PCC */
    else if (clockName < PCC_END_OF_CLOCKS)
    {
        returnCode = CLOCK_SYS_GetPccClockFreq(clockName, frequency);
    }
    /* Invalid clock name */
    else
    {
        returnCode = STATUS_UNSUPPORTED;
    }
    return returnCode;
}

/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_GetPeripheralClock
 * Description   : Internal function used by CLOCK_SYS_GetFreq function
 *END**************************************************************************/

static uint32_t CLOCK_SYS_GetPeripheralClock(clock_names_t clockName, scg_async_clock_type_t divider)
{
    uint32_t frequency = 0;
    peripheral_clock_frac_t  fracValue = PCC_HAL_GetFracValueSel(PCC,clockName);
    peripheral_clock_divider_t divValue = PCC_HAL_GetDividerSel(PCC,clockName);

    /* Check division factor */
    if (((uint32_t)fracValue) <= ((uint32_t)divValue))
    {
        /* Check clock gate */
        if (PCC_HAL_GetClockMode(PCC,clockName))
        {
            /* Check clock source */
            switch (PCC_HAL_GetClockSourceSel(PCC,clockName))
            {
                case CLK_SRC_SOSC:
                    frequency = SCG_HAL_GetSysOscAsyncFreq(SCG,divider);
                    break;
                case CLK_SRC_SIRC:
                    frequency = SCG_HAL_GetSircAsyncFreq(SCG,divider);
                    break;
                case CLK_SRC_FIRC:
                    frequency = SCG_HAL_GetFircAsyncFreq(SCG,divider);
                    break;
                case CLK_SRC_SPLL:
                    frequency = SCG_HAL_GetSysPllAsyncFreq(SCG,divider);
                    break;
                default:
                    frequency = 0;
                    break;
            }
            frequency = frequency / (((uint32_t)divValue)+1U);
            frequency = frequency * (((uint32_t)fracValue)+1U);
        }
    }
    return frequency;
}

/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_GetCurrentRunMode
 * Description   : Internal function used by CLOCK_SYS_SetScgConfiguration function
 *END**************************************************************************/
static scg_system_clock_mode_t CLOCK_SYS_GetCurrentRunMode(const SMC_Type * smc_base)
{
    scg_system_clock_mode_t mode;

    /* Read and convert from SMC run mode to SCG defines*/
    switch (SMC_HAL_GetPowerModeStatus(smc_base))
    {
        /* High speed run mode */
        case STAT_HSRUN:
            mode = SCG_SYSTEM_CLOCK_MODE_HSRUN;
            break;
        /* Run mode */
        case STAT_RUN:
            mode = SCG_SYSTEM_CLOCK_MODE_RUN;
            break;
        /* Very low power run mode */
        case STAT_VLPR:
            mode = SCG_SYSTEM_CLOCK_MODE_VLPR;
            break;
        /* This should never happen - core has to be in some run mode to execute code */
        default:
            mode = SCG_SYSTEM_CLOCK_MODE_NONE;
            break;
    }

    return mode;
}

/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_TransitionSystemClock
 * Description   : Internal function used by CLOCK_SYS_ConfigureTemporarySystemClock and
 * CLOCK_SYS_ConfigureModulesFromScg functions
 *END**************************************************************************/
static status_t CLOCK_SYS_TransitionSystemClock(const scg_system_clock_config_t *to_clk)
{
    scg_system_clock_config_t config;
    scg_system_clock_mode_t run_mode;
    status_t retValue = STATUS_SUCCESS;
    uint32_t timeout;

    /* Check destination clock */
    DEV_ASSERT(to_clk != NULL);
    DEV_ASSERT(to_clk->src != SCG_SYSTEM_CLOCK_SRC_NONE);

    /* Get & Convert Run mode from SMC to SCG defines*/
    run_mode = CLOCK_SYS_GetCurrentRunMode(SMC);

    /* Check the current mode */
    DEV_ASSERT(run_mode != SCG_SYSTEM_CLOCK_MODE_NONE);

    /* Update run mode configuration */
    retValue = SCG_HAL_SetSystemClockConfig(SCG, run_mode, to_clk);

    if (retValue == STATUS_SUCCESS)
    {
        /* Wait for system clock to transition. */
        timeout = 100U;
        do {
            /* Read the new system clock configuration. */
            SCG_HAL_GetSystemClockConfig(SCG, SCG_SYSTEM_CLOCK_MODE_CURRENT, &config);
            timeout--;

        } while ((timeout > 0U) && (config.src != to_clk->src));

        if ( timeout == 0U )
        {
            retValue = STATUS_TIMEOUT;
        }
    }

    return retValue;
}
/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_GetSimClkOutFreq
 * Description   : Internal function used by CLOCK_SYS_GetFreq function
 *END**************************************************************************/
static uint32_t CLOCK_SYS_GetSimClkOutFreq(const SIM_Type * base)
{
    uint32_t frequency;

    /* Check CLKOUT Select */
    sim_clock_out_config_t sim_clkout_config;
    SIM_HAL_GetClkoutConfig(base, &sim_clkout_config);

    if (sim_clkout_config.enable)
    {
        switch (sim_clkout_config.source)
        {
            case SIM_CLKOUT_SEL_SYSTEM_SCG_CLKOUT:
                frequency = CLOCK_SYS_GetScgClkOutFreq(SCG);
                break;
            case SIM_CLKOUT_SEL_SYSTEM_SOSC_DIV2_CLK:
                frequency = SCG_HAL_GetSysOscAsyncFreq(SCG, SCG_ASYNC_CLOCK_DIV2);
                break;
            case SIM_CLKOUT_SEL_SYSTEM_SIRC_DIV2_CLK:
                frequency = SCG_HAL_GetSircAsyncFreq(SCG, SCG_ASYNC_CLOCK_DIV2);
                break;
            case SIM_CLKOUT_SEL_SYSTEM_FIRC_DIV2_CLK:
                frequency = SCG_HAL_GetFircAsyncFreq(SCG, SCG_ASYNC_CLOCK_DIV2);
                break;
            case SIM_CLKOUT_SEL_SYSTEM_SPLL_DIV2_CLK:
                frequency = SCG_HAL_GetSysPllAsyncFreq(SCG, SCG_ASYNC_CLOCK_DIV2);
                break;
            case SIM_CLKOUT_SEL_SYSTEM_LPO_128K_CLK:
                frequency = SIM_HAL_GetLpo128KFreq(SIM);
                break;
            case SIM_CLKOUT_SEL_SYSTEM_LPO_CLK:
                frequency = SIM_HAL_GetLpoFreq(SIM);;
                break;
            case SIM_CLKOUT_SEL_SYSTEM_RTC_CLK:
                frequency = CLOCK_SYS_GetSimRtcClkFreq(SIM);
                break;
            default:
                /* Invalid SIM CLKOUT selection.*/
                frequency = 0U;
                break;
        }

        /* Apply Divide Ratio */
        frequency /= (((uint32_t)sim_clkout_config.divider) + 1U);
    }
    else
    {
        /* Output disabled. */
        frequency = 0U;
    }

    return frequency;
}
/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_GetScgClkOutFreq
 * Description   : Internal function used by CLOCK_SYS_GetFreq function
 *END**************************************************************************/
static uint32_t CLOCK_SYS_GetScgClkOutFreq(const SCG_Type * base)
{
    uint32_t frequency;

    switch (SCG_HAL_GetClockoutSourceSel(base))
    {
        case SCG_CLOCKOUT_SRC_SCG_SLOW:
            frequency = SCG_HAL_GetSystemClockFreq(SCG,SCG_SYSTEM_CLOCK_SLOW);
            break;
        case SCG_CLOCKOUT_SRC_SOSC:
            frequency = SCG_HAL_GetSysOscFreq(SCG);
            break;
        case SCG_CLOCKOUT_SRC_SIRC:
            frequency = SCG_HAL_GetSircFreq(SCG);
            break;
        case SCG_CLOCKOUT_SRC_FIRC:
            frequency = SCG_HAL_GetFircFreq(SCG);
            break;
        case SCG_CLOCKOUT_SRC_SPLL:
            frequency = SCG_HAL_GetSysPllFreq(SCG);
            break;
        default:
            /* Invalid SCG CLKOUT selection.*/
            frequency = 0U;
            break;
    }

    return frequency;
}

/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_GetSimRtcClkFreq
 * Description   : Internal function used by CLOCK_SYS_GetFreq function
 *END**************************************************************************/
static uint32_t CLOCK_SYS_GetSimRtcClkFreq(const SIM_Type * base)
{
    uint32_t frequency;

    /* Check RTCCLK Select */
    switch (SIM_HAL_GetRtcClkSrc(base))
    {
        case SIM_RTCCLK_SEL_SOSCDIV1_CLK:
            frequency = SCG_HAL_GetSysOscAsyncFreq(SCG,SCG_ASYNC_CLOCK_DIV1);
            break;
        case SIM_RTCCLK_SEL_LPO_32K:
            frequency = SIM_HAL_GetLpo32KFreq(SIM);
            break;
        case SIM_RTCCLK_SEL_RTC_CLKIN:
            frequency = SCG_HAL_GetRtcClkInFreq(SCG);
            break;
        case SIM_RTCCLK_SEL_FIRCDIV1_CLK:
            frequency = SCG_HAL_GetFircAsyncFreq(SCG,SCG_ASYNC_CLOCK_DIV1);
            break;
        default:
            /* Invalid RTCCLK selection.*/
            frequency = 0U;
            break;
    }

    return frequency;
}

/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_ConfigureSIRC
 * Description   : Configures SIRC module based on provided configuration.
 *END**************************************************************************/
static status_t CLOCK_SYS_ConfigureSIRC(const scg_sirc_config_t *sircConfig)
{
    status_t status = STATUS_SUCCESS;
    scg_sirc_config_t sircDefaultConfig;
    const scg_sirc_config_t *sircCfg;
    uint32_t timeout;

    if (sircConfig == NULL)
    {
        SCG_HAL_GetSircDefaultConfig(&sircDefaultConfig);
        sircCfg = &sircDefaultConfig;
    }
    else
    {
        sircCfg = sircConfig;
    }

    /* Disable SIRC */
    status = SCG_HAL_DeinitSirc(SCG);

     /* Configure SIRC. */
    if (sircCfg->initialize  && (status == STATUS_SUCCESS))
    {
        /* Setup SIRC. */
        status = SCG_HAL_InitSirc(SCG, sircCfg);

        if (status == STATUS_SUCCESS)
        {
            /* Wait for SIRC to initialize */
            timeout = SIRC_STABILIZATION_TIMEOUT;
            while((SCG_HAL_GetSircFreq(SCG) == 0U) && (timeout > 0U))
            {
                timeout--;
            }
            if (timeout == 0U)
            {
                status = STATUS_TIMEOUT;
            }
        }
    }

    return status;
}


/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_ConfigureFIRC
 * Description   : Configures FIRC module based on provided configuration.
 *END**************************************************************************/
static status_t CLOCK_SYS_ConfigureFIRC(const scg_firc_config_t *fircConfig)
{
    status_t status = STATUS_SUCCESS;
    scg_firc_config_t fircDefaultConfig;
    const scg_firc_config_t *fircCfg;
    uint32_t timeout;

    if (fircConfig == NULL)
    {
        SCG_HAL_GetFircDefaultConfig(&fircDefaultConfig);
        fircCfg = &fircDefaultConfig;
    }
    else
    {
        fircCfg = fircConfig;
    }


    /* Disable FIRC */
    status = SCG_HAL_DeinitFirc(SCG);

     /* Configure FIRC. */
    if (fircCfg->initialize && (status == STATUS_SUCCESS))
    {
        /* Setup FIRC. */
        status = SCG_HAL_InitFirc(SCG, fircCfg);
        if (status == STATUS_SUCCESS)
        {
            /* Wait for FIRC to initialize */
            timeout = FIRC_STABILIZATION_TIMEOUT;
            while((SCG_HAL_GetFircFreq(SCG) == 0U) && (timeout > 0U))
            {
                timeout--;
            }
            if (timeout == 0U)
            {
                status = STATUS_TIMEOUT;
            }
        }
    }

    return status;
}


/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_ConfigureSOSC
 * Description   : Configures SOSC module based on provided configuration.
 *END**************************************************************************/
static status_t CLOCK_SYS_ConfigureSOSC(const scg_sosc_config_t *soscConfig)
{
    status_t status = STATUS_SUCCESS;
    scg_sosc_config_t soscDefaultConfig;
    const scg_sosc_config_t *soscCfg;
    uint32_t timeout;

    if (soscConfig == NULL)
    {
        SCG_HAL_GetSysOscDefaultConfig(&soscDefaultConfig);
        soscCfg = &soscDefaultConfig;
    }
    else
    {
        soscCfg = soscConfig;
    }

    /* Disable SOSC */
    status = SCG_HAL_DeinitSysOsc(SCG);

    /* Configure SOSC. */
    if (soscCfg->initialize && (status == STATUS_SUCCESS))
    {
        /* Setup SOSC. */
        status = SCG_HAL_InitSysOsc(SCG, soscCfg);
        if (status == STATUS_SUCCESS)
        {
        	/* Wait for System OSC to initialize */
        	timeout = SOSC_STABILIZATION_TIMEOUT;
            while((SCG_HAL_GetSysOscFreq(SCG) == 0U) && (timeout > 0U))
            {
                timeout--;
            }
            if (timeout == 0U)
            {
                status = STATUS_TIMEOUT;
            }
        }
    }

    return status;
}


/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_ConfigureSPLL
 * Description   : Configures SPLL module based on provided configuration.
 *END**************************************************************************/
static status_t CLOCK_SYS_ConfigureSPLL(const scg_spll_config_t *spllConfig)
{
    status_t status = STATUS_SUCCESS;
    scg_spll_config_t spllDefaultConfig;
    const scg_spll_config_t *spllCfg;
    uint32_t timeout;

    if (spllConfig == NULL)
    {
        SCG_HAL_GetSysPllDefaultConfig(&spllDefaultConfig);
        spllCfg = &spllDefaultConfig;
    }
    else
    {
        spllCfg = spllConfig;
    }

    /* Disable the SPLL. */
    status = SCG_HAL_DeinitSysPll(SCG);

    /* Configure SPLL. */
    if (spllCfg->initialize && (status == STATUS_SUCCESS))
    {
        /* Setup SPLL. */
        status = SCG_HAL_InitSysPll(SCG, spllCfg);

        if (status == STATUS_SUCCESS)
        {
        	/* Wait for System PLL to initialize */
            timeout = SPLL_STABILIZATION_TIMEOUT;
            while((SCG_HAL_GetSysPllFreq(SCG) == 0U) && (timeout > 0U))
            {
                timeout--;
            }
            if (timeout == 0U)
            {
                status = STATUS_TIMEOUT;
            }
        }


    }

    return status;
}

/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_ConfigureTemporarySystemClock
 * Description   : Configures and transitions to a temporary system clock source: FIRC
 *END**************************************************************************/
static status_t CLOCK_SYS_ConfigureTemporarySystemClock(void)
{
    status_t status = STATUS_SUCCESS;
    scg_system_clock_config_t current_config, sysClockConfig;
    static const scg_system_clock_div_t tmpSysClk[TMP_SYS_CLK_NO][TMP_SYS_DIV_NO] = TMP_SYSTEM_CLOCK_CONFIGS;

    /* Get CURRENT mode configuration. */
    SCG_HAL_GetSystemClockConfig(SCG, SCG_SYSTEM_CLOCK_MODE_CURRENT, &current_config);


    /* If the current system clock source is not FIRC:
     * 1. Enable FIRC (if it's not enabled)
     * 2. Switch to FIRC.
     */
    if (current_config.src != SCG_SYSTEM_CLOCK_SRC_FIRC)
    {
        /* If FIRC is not on, then FIRC is configured
         * with the default configuration */
        if (SCG_HAL_GetFircFreq(SCG) == 0UL)
        {
            status = CLOCK_SYS_ConfigureFIRC(NULL);
        }

        /* FIRC is enabled, transition the system clock source to FIRC. */
        if (status == STATUS_SUCCESS)
        {
            sysClockConfig.src     = SCG_SYSTEM_CLOCK_SRC_FIRC;
            sysClockConfig.divCore = tmpSysClk[TMP_FIRC_CLK][TMP_SYS_DIV];
            sysClockConfig.divBus  = tmpSysClk[TMP_FIRC_CLK][TMP_BUS_DIV];
            sysClockConfig.divSlow = tmpSysClk[TMP_FIRC_CLK][TMP_SLOW_DIV];
            status = CLOCK_SYS_TransitionSystemClock(&sysClockConfig);
        }
    }
    return status;
}

/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_ConfigureModulesFromScg
 * Description   : Configures all modules from SCG (SIRC, FIRC, SOSC and SPLL)
 *END**************************************************************************/
static status_t CLOCK_SYS_ConfigureModulesFromScg(const scg_config_t *scgConfig)
{
    status_t status = STATUS_SUCCESS;
    scg_system_clock_config_t sysClockConfig;
    const scg_system_clock_config_t * nextSysClockConfig;
    static const scg_system_clock_div_t tmpSysClk[TMP_SYS_CLK_NO][TMP_SYS_DIV_NO] = TMP_SYSTEM_CLOCK_CONFIGS;

    /* Configure all clock sources that are different from the
     * current system clock source FIRC (SIRC, SOSC, SPLL). */
    status = CLOCK_SYS_ConfigureSIRC(&scgConfig->sircConfig);
    if (status == STATUS_SUCCESS)
    {
        status = CLOCK_SYS_ConfigureSOSC(&scgConfig->soscConfig);
        if (status == STATUS_SUCCESS)
        {
            status = CLOCK_SYS_ConfigureSPLL(&scgConfig->spllConfig);
        }
    }

    /* Get the next system clock source */
    switch(CLOCK_SYS_GetCurrentRunMode(SMC))
    {
        case SCG_SYSTEM_CLOCK_MODE_RUN:
        {
            nextSysClockConfig = &scgConfig->clockModeConfig.rccrConfig;
        }
        break;
        case SCG_SYSTEM_CLOCK_MODE_VLPR:
        {
            nextSysClockConfig = &scgConfig->clockModeConfig.vccrConfig;
        }
        break;
        case SCG_SYSTEM_CLOCK_MODE_HSRUN:
        {
            nextSysClockConfig = &scgConfig->clockModeConfig.hccrConfig;
        }
        break;
        default:
            DEV_ASSERT(false);
            nextSysClockConfig = NULL;
        break;
    }

    if (status == STATUS_SUCCESS)
    {
        /* The current system clock source is FIRC.
         * Verify whether the next system clock source is FIRC. */
        if (nextSysClockConfig->src == SCG_SYSTEM_CLOCK_SRC_FIRC)
        {
            /* If they are the same, search for a temporary system clock source
             * (use one of the following sources: SPLL, SOSC, SIRC)
             * Assume that a temporary clock is not found status = ERROR. */
            status = STATUS_ERROR;

            /* SPLL is enabled */
            if (scgConfig->spllConfig.initialize /* && (status == STATUS_ERROR) */)
            {
                sysClockConfig.src     = SCG_SYSTEM_CLOCK_SRC_SYS_PLL;
                sysClockConfig.divCore = tmpSysClk[TMP_SPLL_CLK][TMP_SYS_DIV];
                sysClockConfig.divBus  = tmpSysClk[TMP_SPLL_CLK][TMP_BUS_DIV];
                sysClockConfig.divSlow = tmpSysClk[TMP_SPLL_CLK][TMP_SLOW_DIV];
                status = CLOCK_SYS_TransitionSystemClock(&sysClockConfig);
            }

            /* SOSC is enabled and SPLL configuration for system clock source is not valid */
            if (scgConfig->soscConfig.initialize && (status == STATUS_ERROR))
            {
                sysClockConfig.src     = SCG_SYSTEM_CLOCK_SRC_SYS_OSC;
                sysClockConfig.divCore = tmpSysClk[TMP_SOSC_CLK][TMP_SYS_DIV];
                sysClockConfig.divBus  = tmpSysClk[TMP_SOSC_CLK][TMP_BUS_DIV];
                sysClockConfig.divSlow = tmpSysClk[TMP_SOSC_CLK][TMP_SLOW_DIV];
                status = CLOCK_SYS_TransitionSystemClock(&sysClockConfig);
            }

            /* SIRC is enabled and SOSC configuration for system clock source is not valid */
            if (scgConfig->sircConfig.initialize && (status == STATUS_ERROR))
            {
                sysClockConfig.src     = SCG_SYSTEM_CLOCK_SRC_SIRC;
                sysClockConfig.divCore = tmpSysClk[TMP_SIRC_CLK][TMP_SYS_DIV];
                sysClockConfig.divBus  = tmpSysClk[TMP_SIRC_CLK][TMP_BUS_DIV];
                sysClockConfig.divSlow = tmpSysClk[TMP_SIRC_CLK][TMP_SLOW_DIV];
                status = CLOCK_SYS_TransitionSystemClock(&sysClockConfig);
            }

            /* Transitioned to a temporary system clock source. */
            if (status == STATUS_SUCCESS)
            {
                /* Configure the remaining clock source (FIRC). */
                status = CLOCK_SYS_ConfigureFIRC(&scgConfig->fircConfig);

                if (status == STATUS_SUCCESS)
                {
                    /* Transition to the next system clock source. */
                    sysClockConfig.src     = nextSysClockConfig->src;
                    sysClockConfig.divCore = nextSysClockConfig->divCore;
                    sysClockConfig.divBus  = nextSysClockConfig->divBus;
                    sysClockConfig.divSlow = nextSysClockConfig->divSlow;
                    status = CLOCK_SYS_TransitionSystemClock(&sysClockConfig);
                }
            }
        }
        else
        {   /* Transition to the next system clock source. */
            sysClockConfig.src     = nextSysClockConfig->src;
            sysClockConfig.divCore = nextSysClockConfig->divCore;
            sysClockConfig.divBus  = nextSysClockConfig->divBus;
            sysClockConfig.divSlow = nextSysClockConfig->divSlow;
            status = CLOCK_SYS_TransitionSystemClock(&sysClockConfig);

            if (status == STATUS_SUCCESS)
            {
                /* Configure the remaining clock source (FIRC) */
                status = CLOCK_SYS_ConfigureFIRC(&scgConfig->fircConfig);
            }
        }
    }

    return status;
}



/*******************************************************************************
 * EOF
 ******************************************************************************/
