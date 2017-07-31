/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
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

 /*!
 * @file scg_hal.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application or driver code.
 *
 */

#include "scg_hal.h"


/******************************************************************************
 * Definitions
 *****************************************************************************/

/* @brief System PLL base multiplier value, it is the multiplier value when SCG_SPLLCFG[MULT]=0. */
#define SCG_SPLL_MULT_BASE 16U

/*
 * @brief System PLL base divider value, it is the PLL reference clock divider value when
 * SCG_SPLLCFG[PREDIV]=0.
 */
#define SCG_SPLL_PREDIV_BASE 1U

/*
 * @brief System PLL reference clock after SCG_SPLLCFG[PREDIV] should be in the range of
 * SCG_SPLL_REF_MIN to SCG_SPLL_REF_MAX.
 */
#define SCG_SPLL_REF_MIN 8000000U

/*
 * @brief System PLL reference clock after SCG_SPLLCFG[PREDIV] should be in the range of
 * SCG_SPLL_REF_MIN to SCG_SPLL_REF_MAX.
 */
#define SCG_SPLL_REF_MAX 32000000U


#if FEATURE_SOC_SCG_COUNT

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSystemClockFreq
 * Description   : This function gets the SCG system clock frequency, these
 * clocks are used for core, platform, external and bus clock domains.
 *
 * Implements SCG_HAL_GetSystemClockFreq_Activity
 *END*************************************************************************/
uint32_t SCG_HAL_GetSystemClockFreq(const SCG_Type * base,
                                    scg_system_clock_type_t type)
{
    uint32_t freq;
    uint32_t regValue;
    scg_system_clock_src_t src = SCG_HAL_GetSystemClockSrc(base);

    DEV_ASSERT(type < SCG_SYSTEM_CLOCK_MAX);

    switch(src)
    {
        case SCG_SYSTEM_CLOCK_SRC_SYS_OSC:
            freq = SCG_HAL_GetSysOscFreq(base);
            break;
        case SCG_SYSTEM_CLOCK_SRC_SIRC:
            freq = SCG_HAL_GetSircFreq(base);
            break;
        case SCG_SYSTEM_CLOCK_SRC_FIRC:
            freq = SCG_HAL_GetFircFreq(base);
            break;
        case SCG_SYSTEM_CLOCK_SRC_SYS_PLL:
            freq = SCG_HAL_GetSysPllFreq(base);
            break;
        default:
            freq = 0U;
            break;
    }

    regValue = base->CSR;
    regValue = (regValue & SCG_CSR_DIVCORE_MASK) >> SCG_CSR_DIVCORE_SHIFT;
    freq /= (regValue + 1U);

    switch(type)
    {
        case SCG_SYSTEM_CLOCK_CORE:
            /* Intentionally left blank */
            break;
        case SCG_SYSTEM_CLOCK_BUS:
            regValue = base->CSR;
            regValue = (regValue & SCG_CSR_DIVBUS_MASK) >> SCG_CSR_DIVBUS_SHIFT;
            freq /= (regValue + 1U);
            break;
        case SCG_SYSTEM_CLOCK_SLOW:
            regValue = base->CSR;
            regValue = (regValue & SCG_CSR_DIVSLOW_MASK) >> SCG_CSR_DIVSLOW_SHIFT;
            freq /= (regValue + 1U);
            break;
        default:
            freq = 0U;
            break;
    }

    return freq;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_SetSystemClockConfig
 * Description   : This function sets the system configuration for the specified mode.
 *
 * Implements SCG_HAL_SetSystemClockConfig_Activity
 *END*************************************************************************/
status_t SCG_HAL_SetSystemClockConfig(SCG_Type * base,
                                      scg_system_clock_mode_t mode,
                                      scg_system_clock_config_t const *config)
{
    status_t status = STATUS_SUCCESS;
    volatile uint32_t *regAddr = NULL;  /* Address of the register that is written */
    uint32_t srcFreq = 0U;
    uint32_t value = (uint32_t)((((uint32_t)(config->src)     << SCG_CSR_SCS_SHIFT)     & SCG_CSR_SCS_MASK)     |
                                (((uint32_t)(config->divCore) << SCG_CSR_DIVCORE_SHIFT) & SCG_CSR_DIVCORE_MASK) |
                                (((uint32_t)(config->divBus)  << SCG_CSR_DIVBUS_SHIFT)  & SCG_CSR_DIVBUS_MASK)  |
                                (((uint32_t)(config->divSlow) << SCG_CSR_DIVSLOW_SHIFT) & SCG_CSR_DIVSLOW_MASK) );

    /* The maximum clock frequencies in all power modes */
    static const uint32_t maxClocksFreq[MAX_FREQ_MODES_NO][MAX_FREQ_CLK_NO] = CLOCK_MAX_FREQUENCIES;
    uint32_t maxFreqRunMode = 0U;
    const uint32_t sysFreqMul = ((uint32_t)config->divCore) + 1UL;
    const uint32_t busFreqMul = (((uint32_t)config->divCore) + 1UL) * (((uint32_t)config->divBus) + 1UL);
    const uint32_t slowFreqMul = (((uint32_t)config->divCore) + 1UL) * (((uint32_t)config->divSlow) + 1UL);


    DEV_ASSERT(mode != SCG_SYSTEM_CLOCK_MODE_CURRENT);

    switch(config->src)
    {
        case SCG_SYSTEM_CLOCK_SRC_SYS_OSC:
            srcFreq = SCG_HAL_GetSysOscFreq(base);
            break;
        case SCG_SYSTEM_CLOCK_SRC_SIRC:
            srcFreq = SCG_HAL_GetSircFreq(base);
            break;
        case SCG_SYSTEM_CLOCK_SRC_FIRC:
            srcFreq = SCG_HAL_GetFircFreq(base);
            break;
        case SCG_SYSTEM_CLOCK_SRC_SYS_PLL:
            srcFreq = SCG_HAL_GetSysPllFreq(base);
            break;
        default:
            srcFreq = 0U;
            break;
    }


    switch (mode)
    {
        case SCG_SYSTEM_CLOCK_MODE_RUN:       /*!< Run mode.                */
            regAddr = &base->RCCR;
            maxFreqRunMode = MAX_FREQ_RUN;
            break;
        case SCG_SYSTEM_CLOCK_MODE_VLPR:      /*!< Very Low Power Run mode. */
            DEV_ASSERT((SCG_SYSTEM_CLOCK_SRC_SYS_OSC == config->src) ||
                       (SCG_SYSTEM_CLOCK_SRC_SIRC    == config->src));
            regAddr = &base->VCCR;
            maxFreqRunMode = MAX_FREQ_VLPR;
            break;
        case SCG_SYSTEM_CLOCK_MODE_HSRUN:     /*!< High Speed Run mode.     */
            regAddr = &base->VCCR;
            base->HCCR = value;
            maxFreqRunMode = MAX_FREQ_HSRUN;
            break;
        default:
            /* Invalid mode */
            DEV_ASSERT(false);
            break;
    }

    /* Verify the frequencies of sys, bus and slow clocks. */
    if ((srcFreq > (sysFreqMul  * maxClocksFreq[maxFreqRunMode][MAX_FREQ_SYS_CLK])) ||    /* Sys(core) clock */
        (srcFreq > (busFreqMul  * maxClocksFreq[maxFreqRunMode][MAX_FREQ_BUS_CLK])) ||    /* Bus clock */
        (srcFreq > (slowFreqMul * maxClocksFreq[maxFreqRunMode][MAX_FREQ_SLOW_CLK])))     /* Slow clock */
    {
        /* Configuration for the next system clock source is not valid. */
        status = STATUS_ERROR;
    }

    if ((regAddr != NULL) && (status == STATUS_SUCCESS))
    {
        /* Write register. */
        *regAddr = value;
    }

    return status;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSystemClockConfig
 * Description   : This function gets the system configuration for the specified mode.
 *
 * Implements SCG_HAL_GetSystemClockConfig_Activity
 *END*************************************************************************/
void SCG_HAL_GetSystemClockConfig(const SCG_Type * base,
                                  scg_system_clock_mode_t mode,
                                  scg_system_clock_config_t *config)
{
    uint32_t value;
    static const scg_system_clock_div_t systemClockDividerValues[] = {SCG_SYSTEM_CLOCK_DIV_BY_1,
            SCG_SYSTEM_CLOCK_DIV_BY_2,  SCG_SYSTEM_CLOCK_DIV_BY_3,  SCG_SYSTEM_CLOCK_DIV_BY_4,  SCG_SYSTEM_CLOCK_DIV_BY_5,  SCG_SYSTEM_CLOCK_DIV_BY_6,
            SCG_SYSTEM_CLOCK_DIV_BY_7,  SCG_SYSTEM_CLOCK_DIV_BY_8,  SCG_SYSTEM_CLOCK_DIV_BY_9,  SCG_SYSTEM_CLOCK_DIV_BY_10, SCG_SYSTEM_CLOCK_DIV_BY_11,
            SCG_SYSTEM_CLOCK_DIV_BY_12, SCG_SYSTEM_CLOCK_DIV_BY_13, SCG_SYSTEM_CLOCK_DIV_BY_14, SCG_SYSTEM_CLOCK_DIV_BY_15, SCG_SYSTEM_CLOCK_DIV_BY_16};

    switch (mode)
    {
        case SCG_SYSTEM_CLOCK_MODE_CURRENT:   /*!< Current mode.            */
            value = base->CSR;
            break;
        case SCG_SYSTEM_CLOCK_MODE_RUN:       /*!< Run mode.                */
            value = base->RCCR;
            break;
        case SCG_SYSTEM_CLOCK_MODE_VLPR:      /*!< Very Low Power Run mode. */
            value = base->VCCR;
            break;
        case SCG_SYSTEM_CLOCK_MODE_HSRUN:     /*!< High Speed Run mode.     */
            value = base->HCCR;
            break;
        default:
            value = 0U;
            break;
    }

    switch((value & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT)
    {
        case 1U:
            config->src = SCG_SYSTEM_CLOCK_SRC_SYS_OSC;
            break;
        case 2U:
            config->src = SCG_SYSTEM_CLOCK_SRC_SIRC;
            break;
        case 3U:
            config->src = SCG_SYSTEM_CLOCK_SRC_FIRC;
            break;
        case 6U:
            config->src = SCG_SYSTEM_CLOCK_SRC_SYS_PLL;
            break;
        default:
            config->src = SCG_SYSTEM_CLOCK_SRC_NONE;
            break;
    }

    config->divCore = systemClockDividerValues[(value & SCG_CSR_DIVCORE_MASK) >> SCG_CSR_DIVCORE_SHIFT];
    config->divBus  = systemClockDividerValues[(value & SCG_CSR_DIVBUS_MASK)  >> SCG_CSR_DIVBUS_SHIFT ];
    config->divSlow = systemClockDividerValues[(value & SCG_CSR_DIVSLOW_MASK) >> SCG_CSR_DIVSLOW_SHIFT];
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSysOscDefaultConfig
 * Description   : This function gets the default system OSC configuration.
 *
 * Implements SCG_HAL_GetSysOscDefaultConfig_Activity
 *END*************************************************************************/
void SCG_HAL_GetSysOscDefaultConfig(scg_sosc_config_t *config)
{
    DEV_ASSERT(config != NULL);

    config->enableInStop      = false;
    config->enableInLowPower  = false;

    config->monitorMode       = SCG_SOSC_MONITOR_DISABLE;
    config->locked            = false;

    config->div1              = SCG_ASYNC_CLOCK_DISABLE;
    config->div2              = SCG_ASYNC_CLOCK_DISABLE;

    config->extRef            = SCG_SOSC_REF_EXT;
    config->gain              = SCG_SOSC_GAIN_LOW;
    config->range             = SCG_SOSC_RANGE_LOW;
    config->initialize        = true;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_InitSysOsc
 * Description   : This function enables the SCG system OSC clock according
 * to the configuration.
 *
 * Implements SCG_HAL_InitSysOsc_Activity
 *END*************************************************************************/
status_t SCG_HAL_InitSysOsc(SCG_Type * base,
                                  scg_sosc_config_t const *config)
{
    status_t retStatus = STATUS_SUCCESS;
    uint32_t regValue;

    DEV_ASSERT(config != NULL);
    /* Control register can be written. */
    DEV_ASSERT(((base->SOSCCSR & SCG_SOSCCSR_LK_MASK) >> SCG_SOSCCSR_LK_SHIFT) == 0U);

    /* If clock is used by system, return error. */
    if (((base->SOSCCSR & SCG_SOSCCSR_SOSCSEL_MASK) >> SCG_SOSCCSR_SOSCSEL_SHIFT) != 0U)
    {
        retStatus = STATUS_BUSY;
    }
    else
    {
        /* Disable monitor, disable clock and clear error. */
        base->SOSCCSR = SCG_SOSCCSR_SOSCERR_MASK;

        /* Now start to set up OSC clock. */
        /* Step 1. Setup dividers. */
        base->SOSCDIV = SCG_SOSCDIV_SOSCDIV1(config->div1) |
                        SCG_SOSCDIV_SOSCDIV2(config->div2);
        /* Step 2. Set OSC configuration. */
        base->SOSCCFG = SCG_SOSCCFG_RANGE(config->range)        |
                        SCG_SOSCCFG_HGO(config->gain)           |
                        SCG_SOSCCFG_EREFS(config->extRef);
        /* Step 3. Enable clock. */
        base->SOSCCSR = SCG_SOSCCSR_SOSCEN(1U);
        /* Step 4. Enable monitor. */
        base->SOSCCSR = (base->SOSCCSR
                          & ~(SCG_SOSCCSR_SOSCCM_MASK   |
                              SCG_SOSCCSR_SOSCCMRE_MASK |
                              SCG_SOSCCSR_SOSCERR_MASK))
                          |   (uint32_t)config->monitorMode;


        /* Step 4. Lock Control Status Register. */
        regValue = (uint32_t)base->SOSCCSR;
        regValue &= (uint32_t)(~(SCG_SOSCCSR_LK_MASK));
        regValue |= SCG_SOSCCSR_LK(((config->locked) ? 1UL : 0UL));
        base->SOSCCSR = (uint32_t)regValue;

        g_xtal0ClkFreq = config->freq;
    }

    return retStatus;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_DeinitSysOsc
 * Description   : Disables the SCG System OSC clock.
 *
 * Implements SCG_HAL_DeinitSysOsc_Activity
 *END*************************************************************************/
status_t SCG_HAL_DeinitSysOsc(SCG_Type * base)
{
    status_t retStatus = STATUS_SUCCESS;

    /* If clock is used by system, return error. */
    if (((base->SOSCCSR & SCG_SOSCCSR_SOSCSEL_MASK) >> SCG_SOSCCSR_SOSCSEL_SHIFT) != 0U)
    {
        retStatus = STATUS_BUSY;
    }
    else
    {
        /* Clear LK bit field */
        base->SOSCCSR &= (uint32_t)(~(SCG_SOSCCSR_LK_MASK));

        /* Clear SOSCCSR */
        base->SOSCCSR = SCG_SOSCCSR_SOSCERR_MASK;

        g_xtal0ClkFreq = 0U;
    }
    return retStatus;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSysOscFreq
 * Description   : Get SCG System OSC clock frequency (SYSOSC).
 *
 * Implements SCG_HAL_GetSysOscFreq_Activity
 *END*************************************************************************/
uint32_t SCG_HAL_GetSysOscFreq(const SCG_Type * base)
{
    uint32_t retValue;
    if (((base->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK) >> SCG_SOSCCSR_SOSCVLD_SHIFT) != 0U) /* System OSC clock is valid. */
    {
        retValue = g_xtal0ClkFreq;
    }
    else
    {
        retValue = 0U;
    }
    return retValue;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSysOscAsyncFreq
 * Description   : Get SCG asynchronous clock frequency from system OSC.
 *
 * Implements SCG_HAL_GetSysOscAsyncFreq_Activity
 *END*************************************************************************/
uint32_t SCG_HAL_GetSysOscAsyncFreq(const SCG_Type * base,
                                    scg_async_clock_type_t type)
{
    uint32_t regValue, retValue, oscFreq, divider = 0U;

    oscFreq = SCG_HAL_GetSysOscFreq(base);

    /* Get divider. */
    if (oscFreq != 0U)
    {
        switch (type)
        {
            case SCG_ASYNC_CLOCK_DIV2:
                regValue = base->SOSCDIV;
                regValue = (regValue & SCG_SOSCDIV_SOSCDIV2_MASK) >> SCG_SOSCDIV_SOSCDIV2_SHIFT;
                divider = regValue;
                break;
            case SCG_ASYNC_CLOCK_DIV1:
                regValue = base->SOSCDIV;
                regValue = (regValue & SCG_SOSCDIV_SOSCDIV1_MASK) >> SCG_SOSCDIV_SOSCDIV1_SHIFT;
                divider = regValue;
                break;
            default:
                /* Invalid type */
                DEV_ASSERT(false);
                break;
        }
    }
    if (divider != 0U)
    {
        retValue = (oscFreq >> (divider-1U));
    }
    else  /* Output disabled. */
    {
        retValue = 0U;
    }
    return retValue;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSircDefaultConfig
 * Description   : This function gets the default SIRC configuration.
 *
 * Implements SCG_HAL_GetSircDefaultConfig_Activity
 *END*************************************************************************/
void SCG_HAL_GetSircDefaultConfig(scg_sirc_config_t *config)
{
    DEV_ASSERT(config != NULL);

    config->enableInStop      = false;
    config->enableInLowPower  = true;
    config->locked            = false;
    config->div1              = SCG_ASYNC_CLOCK_DISABLE;
    config->div2              = SCG_ASYNC_CLOCK_DISABLE;
    config->range             = SCG_SIRC_RANGE_HIGH;
    config->initialize        = true;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_InitSirc
 * Description   : This function enables the SCG SIRC clock according
 * to the configuration.
 *
 * Implements SCG_HAL_InitSirc_Activity
 *END*************************************************************************/
status_t SCG_HAL_InitSirc(SCG_Type * base,
                                const scg_sirc_config_t *config)
{
    status_t retStatus = STATUS_SUCCESS;
    uint32_t regValue;

    DEV_ASSERT(config != NULL);
    /* Control register can be written. */
    DEV_ASSERT(((base->SIRCCSR & SCG_SIRCCSR_LK_MASK) >> SCG_SIRCCSR_LK_SHIFT) == 0U);

    /* If clock is used by system, return error. */
    if (((base->SIRCCSR & SCG_SIRCCSR_SIRCSEL_MASK) >> SCG_SIRCCSR_SIRCSEL_SHIFT) != 0U)
    {
        retStatus = STATUS_BUSY;
    }
    else
    {
        /* Disable clock. */
        base->SIRCCSR &= (uint32_t)(~(SCG_SIRCCSR_SIRCEN_MASK));

        /* Now start to set up SIRC clock. */
        /* Step 1. Setup dividers. */
        base->SIRCDIV = SCG_SIRCDIV_SIRCDIV1(config->div1) |
                        SCG_SIRCDIV_SIRCDIV2(config->div2);
        /* Step 2. Set SIRC configuration. */
        base->SIRCCFG = SCG_SIRCCFG_RANGE(config->range);
        /* Step 3. Enable clock. */
        base->SIRCCSR = SCG_SIRCCSR_SIRCEN(1U)                                          |
                        SCG_SIRCCSR_SIRCSTEN(((config->enableInStop)     ? 1UL : 0UL )) |
                        SCG_SIRCCSR_SIRCLPEN(((config->enableInLowPower) ? 1UL : 0UL ));
        /* Step 4. Lock Control Status Register.  */
        regValue = (uint32_t)base->SIRCCSR;
        regValue &= (uint32_t)(~(SCG_SIRCCSR_LK_MASK));
        regValue |= SCG_SIRCCSR_LK(((config->locked) ? 1UL : 0UL));
        base->SIRCCSR = (uint32_t)regValue;
    }

    return retStatus;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_DeinitSirc
 * Description   : Disables the SCG slow IRC.
 *
 * Implements SCG_HAL_DeinitSirc_Activity
 *END*************************************************************************/
status_t SCG_HAL_DeinitSirc(SCG_Type * base)
{
    status_t retStatus;

    /* If clock is used by system, return error. */
    if (((base->SIRCCSR & SCG_SIRCCSR_SIRCSEL_MASK) >> SCG_SIRCCSR_SIRCSEL_SHIFT) != 0U)
    {
        retStatus = STATUS_BUSY;
    }
    else
    {
        /* Clear LK bit field */
        base->SIRCCSR &= (uint32_t)(~(SCG_SIRCCSR_LK_MASK));

        /* Clear SIRCCSR */
        base->SIRCCSR = 0;

        retStatus = STATUS_SUCCESS;
    }
    return retStatus;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSircFreq
 * Description   : Get SCG SIRC clock frequency.
 *
 * Implements SCG_HAL_GetSircFreq_Activity
 *END*************************************************************************/
uint32_t SCG_HAL_GetSircFreq(const SCG_Type * base)
{
    uint32_t retValue;

    if (((base->SIRCCSR & SCG_SIRCCSR_SIRCVLD_MASK) >> SCG_SIRCCSR_SIRCVLD_SHIFT) != 0U) /* SIRC is valid. */
    {
        retValue =  (((base->SIRCCFG & SCG_SIRCCFG_RANGE_MASK) >> SCG_SIRCCFG_RANGE_SHIFT) != 0U) ?
                                  FEATURE_SCG_SIRC_HIGH_RANGE_FREQ : FEATURE_SCG_SIRC_LOW_RANGE_FREQ;
    }
    else
    {
        retValue = 0U;
    }
    return retValue;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSircAsyncFreq
 * Description   : Get SCG asynchronous clock frequency from SIRC.
 *
 * Implements SCG_HAL_GetSircAsyncFreq_Activity
 *END*************************************************************************/
uint32_t SCG_HAL_GetSircAsyncFreq(const SCG_Type * base,
                                  scg_async_clock_type_t type)
{
    uint32_t regValue, retValue, sircFreq, divider = 0U;

    sircFreq = SCG_HAL_GetSircFreq(base);

    /* Get divider. */
    if (sircFreq != 0U)
    {
        switch (type)
        {
            case SCG_ASYNC_CLOCK_DIV2:
                regValue = base->SIRCDIV;
                regValue = (regValue & SCG_SIRCDIV_SIRCDIV2_MASK) >> SCG_SIRCDIV_SIRCDIV2_SHIFT;
                divider = regValue;
                break;
            case SCG_ASYNC_CLOCK_DIV1:
                regValue = base->SIRCDIV;
                regValue = (regValue & SCG_SIRCDIV_SIRCDIV1_MASK) >> SCG_SIRCDIV_SIRCDIV1_SHIFT;
                divider = regValue;
                break;
            default:
                /* Invalid type */
                DEV_ASSERT(false);
                break;
        }
    }
    if (divider != 0U)
    {
        retValue = (sircFreq >> (divider-1U));
    }
    else  /* Output disabled. */
    {
        retValue = 0U;
    }
    return retValue;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetFircDefaultConfig
 * Description   : This function gets the default FIRC configuration.
 *
 * Implements SCG_HAL_GetFircDefaultConfig_Activity
 *END*************************************************************************/
void SCG_HAL_GetFircDefaultConfig(scg_firc_config_t *config)
{
    DEV_ASSERT(config != NULL);

    config->enableInStop      = false;
    config->enableInLowPower  = false;
    config->regulator         = true;
    config->locked            = false;

    config->div1              = SCG_ASYNC_CLOCK_DISABLE;
    config->div2              = SCG_ASYNC_CLOCK_DISABLE;

    config->range             = SCG_FIRC_RANGE_48M;

    config->initialize        = true;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_InitFirc
 * Description   : This function enables the SCG FIRC clock according
 * to the configuration.
 *
 * Implements SCG_HAL_InitFirc_Activity
 *END*************************************************************************/
status_t SCG_HAL_InitFirc(SCG_Type * base,
                                const scg_firc_config_t *config)
{
    status_t retStatus = STATUS_SUCCESS;
    uint32_t regValue;

    DEV_ASSERT(config != NULL);
    /* Control register can be written. */
    DEV_ASSERT(((base->FIRCCSR & SCG_FIRCCSR_LK_MASK) >> SCG_FIRCCSR_LK_SHIFT) == 0U);

    /* If clock is used by system, return error. */
    if (((base->FIRCCSR & SCG_FIRCCSR_FIRCSEL_MASK) >> SCG_FIRCCSR_FIRCSEL_SHIFT) != 0U)
    {
        retStatus = STATUS_BUSY;
    }
    else
    {
        /* Disable clock. */
        base->FIRCCSR &= (uint32_t)(~(SCG_FIRCCSR_FIRCEN_MASK));
        /* Clear error. */
        base->FIRCCSR |= SCG_FIRCCSR_FIRCERR(1U);

        /* Now start to set up FIRC clock. */
        /* Step 1. Setup dividers. */
        base->FIRCDIV = SCG_FIRCDIV_FIRCDIV1(config->div1) |
                        SCG_FIRCDIV_FIRCDIV2(config->div2);

        /* Step 2. Set FIRC configuration. */
        base->FIRCCFG = SCG_FIRCCFG_RANGE(config->range);


        /* Step 3. Enable clock. */
        base->FIRCCSR = (base->FIRCCSR
                       & ~((SCG_FIRCCSR_FIRCEN_MASK)   |
                           (SCG_FIRCCSR_FIRCREGOFF_MASK)))
                       | (SCG_FIRCCSR_FIRCEN(1U)                                 |
                          SCG_FIRCCSR_FIRCREGOFF((config->regulator ? 0UL : 1UL)));



        /* Step 5. Lock Control Status Register  */
        regValue = (uint32_t)base->FIRCCSR;
        regValue &= (uint32_t)(~(SCG_FIRCCSR_LK_MASK));
        regValue |= SCG_FIRCCSR_LK(((config->locked) ? 1UL : 0UL));
        base->FIRCCSR = (uint32_t)regValue;

        /* Check if an error was detected */
        if (((base->FIRCCSR & SCG_FIRCCSR_FIRCERR_MASK) >> SCG_FIRCCSR_FIRCERR_SHIFT) != 0U)
        {
            retStatus = STATUS_ERROR;
        }
    }
    return retStatus;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_DeinitFirc
 * Description   : Disables the SCG fast IRC.
 *
 * Implements SCG_HAL_DeinitFirc_Activity
 *END*************************************************************************/
status_t SCG_HAL_DeinitFirc(SCG_Type * base)
{
    status_t retStatus = STATUS_SUCCESS;

    /* If clock is used by system, return error. */
    if (((base->FIRCCSR & SCG_FIRCCSR_FIRCSEL_MASK) >> SCG_FIRCCSR_FIRCSEL_SHIFT) != 0U)
    {
        retStatus = STATUS_BUSY;
    }
    else
    {
        /* Clear LK bit field */
        base->FIRCCSR &= (uint32_t)(~(SCG_FIRCCSR_LK_MASK));

        /* Clear FIRCCSR */
        base->FIRCCSR = SCG_FIRCCSR_FIRCERR_MASK;
    }
    return retStatus;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetFircFreq
 * Description   : Get SCG FIRC clock frequency.
 *
 * Implements SCG_HAL_GetFircFreq_Activity
 *END*************************************************************************/
uint32_t SCG_HAL_GetFircFreq(const SCG_Type * base)
{
    uint32_t retValue;

    static const uint32_t fircFreq[] = {
        FEATURE_SCG_FIRC_FREQ0,
        FEATURE_SCG_FIRC_FREQ1,
        FEATURE_SCG_FIRC_FREQ2,
        FEATURE_SCG_FIRC_FREQ3,
    };

    if (((base->FIRCCSR & SCG_FIRCCSR_FIRCVLD_MASK) >> SCG_FIRCCSR_FIRCVLD_SHIFT) != 0U) /* FIRC is valid. */
    {
        uint32_t regValue = base->FIRCCFG;
        regValue = (regValue & SCG_FIRCCFG_RANGE_MASK) >> SCG_FIRCCFG_RANGE_SHIFT;
        retValue = fircFreq[regValue];
    }
    else
    {
        retValue = 0U;
    }
    return retValue;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetFircAsyncFreq
 * Description   : Get SCG asynchronous clock frequency from FIRC.
 *
 * Implements SCG_HAL_GetFircAsyncFreq_Activity
 *END*************************************************************************/
uint32_t SCG_HAL_GetFircAsyncFreq(const SCG_Type * base,
                                  scg_async_clock_type_t type)
{
    uint32_t fircFreq = SCG_HAL_GetFircFreq(base);
    uint32_t regValue, retValue, divider = 0U;

    /* Get divider. */
    if (fircFreq != 0U)
    {
        switch (type)
        {
            case SCG_ASYNC_CLOCK_DIV2:
                regValue = base->FIRCDIV;
                regValue = (regValue & SCG_FIRCDIV_FIRCDIV2_MASK) >> SCG_FIRCDIV_FIRCDIV2_SHIFT;
                divider = regValue;
                break;
            case SCG_ASYNC_CLOCK_DIV1:
                regValue = base->FIRCDIV;
                regValue = (regValue & SCG_FIRCDIV_FIRCDIV1_MASK) >> SCG_FIRCDIV_FIRCDIV1_SHIFT;
                divider = regValue;
                break;
            default:
                /* Invalid type */
                DEV_ASSERT(false);
                break;
        }
    }
    if (divider != 0U)
    {
        retValue = (fircFreq >> (divider-1U));
    }
    else  /* Output disabled. */
    {
        retValue = 0U;
    }
    return retValue;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSysPllDefaultConfig
 * Description   : This function gets the default system PLL configuration.
 *
 * Implements SCG_HAL_GetSysPllDefaultConfig_Activity
 *END*************************************************************************/
void SCG_HAL_GetSysPllDefaultConfig(scg_spll_config_t *config)
{
    DEV_ASSERT(config != NULL);

    config->enableInStop   = false;
    config->monitorMode    = SCG_SPLL_MONITOR_DISABLE;
    config->locked         = false;

    config->div1           = SCG_ASYNC_CLOCK_DISABLE;
    config->div2           = SCG_ASYNC_CLOCK_DISABLE;

    config->prediv         = 0;
    config->mult           = 0;
    config->src            = 0;
    config->initialize     = true;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_InitSysPll
 * Description   : This function enables the SCG system PLL clock according
 * to the configuration.
 *
 * Implements SCG_HAL_InitSysPll_Activity
 *END*************************************************************************/
status_t SCG_HAL_InitSysPll(SCG_Type * base,
                                scg_spll_config_t const *config)
{
    status_t retStatus = STATUS_SUCCESS;
    uint32_t srcFreq, regValue;

    DEV_ASSERT(config != NULL);
    /* Control register can be written. */
    DEV_ASSERT(((base->SPLLCSR & SCG_SPLLCSR_LK_MASK) >> SCG_SPLLCSR_LK_SHIFT) == 0U);

    /* If clock is used by system, return error. */
    if (((base->SPLLCSR & SCG_SPLLCSR_SPLLSEL_MASK) >> SCG_SPLLCSR_SPLLSEL_SHIFT) != 0U)
    {
        retStatus = STATUS_BUSY;
    }
    else
    {
        /* Get clock source frequency. */
        srcFreq = SCG_HAL_GetSysOscFreq(base);

        DEV_ASSERT(srcFreq != 0U);

        /* Pre-divider checking. */
        srcFreq /= (((uint32_t)config->prediv) + SCG_SPLL_PREDIV_BASE);

        DEV_ASSERT((srcFreq >= SCG_SPLL_REF_MIN) && (srcFreq <= SCG_SPLL_REF_MAX));

        /* Disable monitor, disable clock and clear error. */
        base->SPLLCSR = SCG_SPLLCSR_SPLLERR_MASK;

        /* Now start to set up PLL clock. */
        /* Step 1. Setup dividers. */
        base->SPLLDIV = SCG_SPLLDIV_SPLLDIV1(config->div1) |
                        SCG_SPLLDIV_SPLLDIV2(config->div2);
        /* Step 2. Set PLL configuration. */
        base->SPLLCFG = SCG_SPLLCFG_PREDIV(config->prediv)  |
                        SCG_SPLLCFG_MULT(config->mult);
        /* Step 3. Enable clock. */
        base->SPLLCSR = SCG_SPLLCSR_SPLLEN(1U);
        /* Step 4. Enable monitor. */
        base->SPLLCSR = (base->SPLLCSR
                          & ~(SCG_SPLLCSR_SPLLCM_MASK   |
                              SCG_SPLLCSR_SPLLCMRE_MASK |
                              SCG_SPLLCSR_SPLLERR_MASK))
                          |   (uint32_t)config->monitorMode;

        /* Step 5. Lock Control Status Register */
        regValue = (uint32_t)base->SPLLCSR;
        regValue &= (uint32_t)(~(SCG_SPLLCSR_LK_MASK));
        regValue |= SCG_SPLLCSR_LK(((config->locked) ? 1UL : 0UL));
        base->SPLLCSR = (uint32_t)regValue;
    }
    return retStatus;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_DeinitSysPll
 * Description   : Disables the SCG system PLL.
 *
 * Implements SCG_HAL_DeinitSysPll_Activity
 *END*************************************************************************/
status_t SCG_HAL_DeinitSysPll(SCG_Type * base)
{
    status_t retStatus = STATUS_SUCCESS;

    /* If clock is used by system, return error. */
    if (((base->SPLLCSR & SCG_SPLLCSR_SPLLSEL_MASK) >> SCG_SPLLCSR_SPLLSEL_SHIFT) != 0U)
    {
        retStatus = STATUS_BUSY;
    }
    else
    {
        /* Clear LK bit field */
        base->SPLLCSR &= (uint32_t)(~(SCG_SPLLCSR_LK_MASK));

        /* Clear SPLLCSR */
        base->SPLLCSR = SCG_SPLLCSR_SPLLERR_MASK;
    }
    return retStatus;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSysPllFreq
 * Description   : Get SCG system PLL clock frequency.
 *
 * Implements SCG_HAL_GetSysPllFreq_Activity
 *END*************************************************************************/
uint32_t SCG_HAL_GetSysPllFreq(const SCG_Type * base)
{
    uint32_t freq, regValue, retValue;

    if (((base->SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK) >> SCG_SPLLCSR_SPLLVLD_SHIFT) != 0U) /* System PLL is valid. */
    {
        /* Get System OSC. frequency. */
        freq = SCG_HAL_GetSysOscFreq(base);

        if (freq != 0U) /* If source is valid. */
        {
            regValue = base->SPLLCFG;
            regValue = (regValue & SCG_SPLLCFG_PREDIV_MASK) >> SCG_SPLLCFG_PREDIV_SHIFT;
            freq /= (regValue + SCG_SPLL_PREDIV_BASE);  /* Pre-divider. */

            regValue = base->SPLLCFG;
            regValue = (regValue & SCG_SPLLCFG_MULT_MASK) >> SCG_SPLLCFG_MULT_SHIFT;
            freq *= (regValue + SCG_SPLL_MULT_BASE);      /* Multiplier. */

            freq = freq >> 1U;  /* Divide VCO by 2. */
        }

        retValue = freq;
    }
    else
    {
        retValue = 0U;
    }
    return retValue;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSysPllAsyncFreq
 * Description   : Get SCG asynchronous clock frequency from system PLL.
 *
 * Implements SCG_HAL_GetSysPllAsyncFreq_Activity
 *END*************************************************************************/
uint32_t SCG_HAL_GetSysPllAsyncFreq(const SCG_Type * base,
                                    scg_async_clock_type_t type)
{
    uint32_t pllFreq = SCG_HAL_GetSysPllFreq(base);
    uint32_t regValue, retValue, divider = 0U;

    /* Get divider. */
    if (pllFreq != 0U)
    {
        switch (type)
        {
            case SCG_ASYNC_CLOCK_DIV2:
                regValue = base->SPLLDIV;
                regValue = (regValue & SCG_SPLLDIV_SPLLDIV2_MASK) >> SCG_SPLLDIV_SPLLDIV2_SHIFT;
                divider = regValue;
                break;
            case SCG_ASYNC_CLOCK_DIV1:
                regValue = base->SPLLDIV;
                regValue = (regValue & SCG_SPLLDIV_SPLLDIV1_MASK) >> SCG_SPLLDIV_SPLLDIV1_SHIFT;
                divider = regValue;
                break;
            default:
                /* Invalid type */
                DEV_ASSERT(false);
                break;
        }
    }
    if (divider != 0U)
    {
        retValue = (pllFreq >> (divider-1U));
    }
    else  /* Output disabled. */
    {
        retValue = 0U;
    }
    return retValue;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_SetRtcClkInFreq
 * Description   : Configures SCG RTC CLKIN clock frequency.
 *
 * Implements SCG_HAL_SetRtcClkInFreq_Activity
 *END*************************************************************************/
void SCG_HAL_SetRtcClkInFreq(SCG_Type * base, uint32_t frequency)
{
    (void) (base);
    g_RtcClkInFreq = frequency;;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetRtcClkInFreq
 * Description   : Get SCG RTC CLKIN clock frequency.
 *
 * Implements SCG_HAL_GetRtcClkInFreq_Activity
 *END*************************************************************************/
uint32_t SCG_HAL_GetRtcClkInFreq(SCG_Type * base)
{
    (void) (base);
    return g_RtcClkInFreq;
}

#endif

/******************************************************************************
 * EOF
 *****************************************************************************/

