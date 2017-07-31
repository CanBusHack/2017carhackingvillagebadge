/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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

#include "device_registers.h"
#include "sim_hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* @brief LPO fixed clock frequency. */
#define LPO_128K_FREQUENCY 128000
#define LPO_32K_FREQUENCY   32000
#define LPO_1K_FREQUENCY     1000

/*******************************************************************************
 * APIs
 ******************************************************************************/

/*FUNCTION*********************************************************************
 *
 * Function Name : SIM_HAL_GetClkoutDefaultConfig
 * Description   : This function gets the default SIM CLKOUT configuration.
 *
 * Implements SIM_HAL_GetClkoutDefaultConfig_Activity
 *END*************************************************************************/
void SIM_HAL_GetClkoutDefaultConfig(sim_clock_out_config_t *config)
{
    DEV_ASSERT(config != NULL);

    config->initialize        = true;
    config->enable            = false;
    config->source            = SIM_CLKOUT_SEL_SYSTEM_SCG_CLKOUT;
    config->divider           = SIM_CLKOUT_DIV_BY_1;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SIM_HAL_GetClkoutDefaultConfig
 * Description   : This function gets the default SIM CLKOUT configuration.
 *
 * Implements SIM_HAL_GetClkoutConfig_Activity
 *END*************************************************************************/
void SIM_HAL_GetClkoutConfig(const SIM_Type * base, sim_clock_out_config_t *config)
{
    uint32_t value;
    DEV_ASSERT(config != NULL);

    value = base->CHIPCTL;

    config->enable  = (((value & SIM_CHIPCTL_CLKOUTEN_MASK) >> SIM_CHIPCTL_CLKOUTEN_SHIFT) == 0U) ? false : true;

    switch((value & SIM_CHIPCTL_CLKOUTSEL_MASK) >> SIM_CHIPCTL_CLKOUTSEL_SHIFT)
    {
        case 14U:
            config->source = SIM_CLKOUT_SEL_SYSTEM_RTC_CLK;
            break;
        case 12U:
            config->source = SIM_CLKOUT_SEL_SYSTEM_LPO_CLK;
            break;
        case 10U:
            config->source = SIM_CLKOUT_SEL_SYSTEM_LPO_128K_CLK;
            break;
        case 8U:
            config->source = SIM_CLKOUT_SEL_SYSTEM_SPLL_DIV2_CLK;
            break;
        case 6U:
            config->source = SIM_CLKOUT_SEL_SYSTEM_FIRC_DIV2_CLK;
            break;
        case 4U:
            config->source = SIM_CLKOUT_SEL_SYSTEM_SIRC_DIV2_CLK;
            break;
        case 2U:
            config->source = SIM_CLKOUT_SEL_SYSTEM_SOSC_DIV2_CLK;
            break;
        case 0U:
            /* Pass-though */
        default:
            config->source = SIM_CLKOUT_SEL_SYSTEM_SCG_CLKOUT;
            break;
    }

    switch((value & SIM_CHIPCTL_CLKOUTDIV_MASK) >> SIM_CHIPCTL_CLKOUTDIV_SHIFT)
    {
        case 7U:
            config->divider = SIM_CLKOUT_DIV_BY_8;
            break;
        case 6U:
            config->divider = SIM_CLKOUT_DIV_BY_7;
            break;
        case 5U:
            config->divider = SIM_CLKOUT_DIV_BY_6;
            break;
        case 4U:
            config->divider = SIM_CLKOUT_DIV_BY_5;
            break;
        case 3U:
            config->divider = SIM_CLKOUT_DIV_BY_4;
            break;
        case 2U:
            config->divider = SIM_CLKOUT_DIV_BY_3;
            break;
        case 1U:
            config->divider = SIM_CLKOUT_DIV_BY_2;
            break;
        case 0U:
            /* Pass-though */
        default:
            config->divider = SIM_CLKOUT_DIV_BY_1;
            break;
    }
}
/*FUNCTION*********************************************************************
 *
 * Function Name : SIM_HAL_InitClkout
 * Description   : This function enables the SIM CLKOUT according to the configuration.
 * @note This function ignores initialize member
 *
 * Implements SIM_HAL_InitClkout_Activity
 *END*************************************************************************/
void SIM_HAL_InitClkout(SIM_Type * base, const sim_clock_out_config_t *config)
{
    uint32_t regValue;
    DEV_ASSERT(config != NULL);

    /* Read current configuration. */
    regValue = base->CHIPCTL;

    /* Clear previous values. */
    regValue &= ~( SIM_CHIPCTL_CLKOUTEN_MASK  |
                   SIM_CHIPCTL_CLKOUTSEL_MASK |
                   SIM_CHIPCTL_CLKOUTDIV_MASK );

    /* Configure based on configuration*/
    regValue |= SIM_CHIPCTL_CLKOUTEN(config->enable ? 1UL : 0UL);
    regValue |= SIM_CHIPCTL_CLKOUTSEL(config->source);
    regValue |= SIM_CHIPCTL_CLKOUTDIV(config->divider);

    /* Write value to register. */
    base->CHIPCTL = regValue;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetFtmExternalClkPinMode
 * Description   : Set FlexTimer x external clock pin select setting
 * This function will select the source of FTMx external clock pin select
 *
 * Implements SIM_HAL_SetFtmExternalClkPinMode_Activity
 *END**************************************************************************/
void SIM_HAL_SetFtmExternalClkPinMode(SIM_Type * base,
                                      uint32_t instance,
                                      sim_ftm_clk_sel_t select)
{
    uint32_t regValue = base->FTMOPT0;
    uint32_t selectValue = (uint32_t)select;
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);

    switch (instance)
    {
    case 0U:
        regValue &= ~(SIM_FTMOPT0_FTM0CLKSEL_MASK);
        regValue |= SIM_FTMOPT0_FTM0CLKSEL(selectValue);
        break;
    case 1U:
        regValue &= ~(SIM_FTMOPT0_FTM1CLKSEL_MASK);
        regValue |= SIM_FTMOPT0_FTM1CLKSEL(selectValue);
        break;
    case 2U:
        regValue &= ~(SIM_FTMOPT0_FTM2CLKSEL_MASK);
        regValue |= SIM_FTMOPT0_FTM2CLKSEL(selectValue);
        break;
    case 3U:
        regValue &= ~(SIM_FTMOPT0_FTM3CLKSEL_MASK);
        regValue |= SIM_FTMOPT0_FTM3CLKSEL(selectValue);
        break;
#if FTM_INSTANCE_COUNT > 4U
    case 4U:
        regValue &= ~(SIM_FTMOPT0_FTM4CLKSEL_MASK);
        regValue |= SIM_FTMOPT0_FTM4CLKSEL(selectValue);
        break;
#endif
#if FTM_INSTANCE_COUNT > 5U
    case 5U:
        regValue &= ~(SIM_FTMOPT0_FTM5CLKSEL_MASK);
        regValue |= SIM_FTMOPT0_FTM5CLKSEL(selectValue);
        break;
#endif
#if FTM_INSTANCE_COUNT > 6U
    case 6U:
        regValue &= ~(SIM_FTMOPT0_FTM6CLKSEL_MASK);
        regValue |= SIM_FTMOPT0_FTM6CLKSEL(selectValue);
        break;
#endif
#if FTM_INSTANCE_COUNT > 7U
    case 7U:
        regValue &= ~(SIM_FTMOPT0_FTM7CLKSEL_MASK);
        regValue |= SIM_FTMOPT0_FTM7CLKSEL(selectValue);
        break;
#endif
    default:
        /* Nothing to do, error is caught by DEV_ASSERT(instance < FTM_INSTANCE_COUNT) */
        break;
    }
    base->FTMOPT0 = regValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetFtmExternalClkPinMode
 * Description   : Get FlexTimer x external clock pin select setting
 * This function will get FlexTimer x external clock pin select setting.
 *
 * Implements SIM_HAL_GetFtmExternalClkPinMode_Activity
 *END**************************************************************************/
sim_ftm_clk_sel_t SIM_HAL_GetFtmExternalClkPinMode(const SIM_Type * base,
                                                   uint32_t instance)
{
    sim_ftm_clk_sel_t retValue;
    uint32_t regValue = base->FTMOPT0;
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);

    switch (instance)
    {
    case 0U:
        regValue = (regValue & SIM_FTMOPT0_FTM0CLKSEL_MASK) >> SIM_FTMOPT0_FTM0CLKSEL_SHIFT;
        break;
    case 1U:
        regValue = (regValue & SIM_FTMOPT0_FTM1CLKSEL_MASK) >> SIM_FTMOPT0_FTM1CLKSEL_SHIFT;
        break;
    case 2U:
        regValue = (regValue & SIM_FTMOPT0_FTM2CLKSEL_MASK) >> SIM_FTMOPT0_FTM2CLKSEL_SHIFT;
        break;
    case 3U:
        regValue = (regValue & SIM_FTMOPT0_FTM3CLKSEL_MASK) >> SIM_FTMOPT0_FTM3CLKSEL_SHIFT;
        break;
#if FTM_INSTANCE_COUNT > 4U
    case 4U:
        regValue = (regValue & SIM_FTMOPT0_FTM4CLKSEL_MASK) >> SIM_FTMOPT0_FTM4CLKSEL_SHIFT;
        break;
#endif
#if FTM_INSTANCE_COUNT > 5U
    case 5U:
        regValue = (regValue & SIM_FTMOPT0_FTM5CLKSEL_MASK) >> SIM_FTMOPT0_FTM5CLKSEL_SHIFT;
        break;
#endif
#if FTM_INSTANCE_COUNT > 6U
    case 6U:
        regValue = (regValue & SIM_FTMOPT0_FTM6CLKSEL_MASK) >> SIM_FTMOPT0_FTM6CLKSEL_SHIFT;
        break;
#endif
#if FTM_INSTANCE_COUNT > 7U
    case 7U:
        regValue = (regValue & SIM_FTMOPT0_FTM7CLKSEL_MASK) >> SIM_FTMOPT0_FTM7CLKSEL_SHIFT;
        break;
#endif
    default:
        regValue = 0U;
        break;
    }

    switch(regValue)
    {
    case 3U:
        retValue = SIM_FTM_CLK_SEL_11;
        break;
    case 2U:
        retValue = SIM_FTM_CLK_SEL_10;
        break;
    case 1U:
        retValue = SIM_FTM_CLK_SEL_01;
        break;
    case 0U:
    /* Pass-through */
    default:
        retValue = SIM_FTM_CLK_SEL_00;
        break;
    }

    return retValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetFtmFaultSelMode
 * Description   : Set FlexTimer x faults select settings
 * This function will set the FlexTimer x faults select settings.
 *
 * Implements SIM_HAL_SetFtmFaultSelMode_Activity
 *END**************************************************************************/
void SIM_HAL_SetFtmFaultSelMode(SIM_Type * base,
                                uint32_t instance,
                                uint8_t select)
{
    uint32_t regValue;
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(select < (1U<<SIM_FTMOPT0_FTM0FLTxSEL_WIDTH));

    /* TODO: verify that all SIM_FTMOPT0_FTM1FLTxSEL_WIDTH are equal */

    switch (instance)
    {
        case 0U:
            regValue = base->FTMOPT0;
            regValue &= ~(SIM_FTMOPT0_FTM0FLTxSEL_MASK);
            regValue |= SIM_FTMOPT0_FTM0FLTxSEL(select);
            base->FTMOPT0 = regValue;
            break;
        case 1U:
            regValue = base->FTMOPT0;
            regValue &= ~(SIM_FTMOPT0_FTM1FLTxSEL_MASK);
            regValue |= SIM_FTMOPT0_FTM1FLTxSEL(select);
            base->FTMOPT0 = regValue;
            break;
        case 2U:
            regValue = base->FTMOPT0;
            regValue &= ~(SIM_FTMOPT0_FTM2FLTxSEL_MASK);
            regValue |= SIM_FTMOPT0_FTM2FLTxSEL(select);
            base->FTMOPT0 = regValue;
            break;
        case 3U:
            regValue = base->FTMOPT0;
            regValue &= ~(SIM_FTMOPT0_FTM3FLTxSEL_MASK);
            regValue |= SIM_FTMOPT0_FTM3FLTxSEL(select);
            base->FTMOPT0 = regValue;
            break;
        default:
            /* Instance invalid value, this error is caught by DEV_ASSERT(instance < ADC_INSTANCE_COUNT) */
            break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetFtmFaultSelMode
 * Description   : Get FlexTimer x faults select settings
 * This function will get FlexTimer x faults select settings.
 *
 * Implements SIM_HAL_GetFtmFaultSelMode_Activity
 *END**************************************************************************/
uint8_t SIM_HAL_GetFtmFaultSelMode(const SIM_Type * base,
                                   uint32_t instance)
{
    uint8_t retValue;

    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);

    switch (instance)
    {
    case 0U:
        retValue = (uint8_t)((base->FTMOPT0 & SIM_FTMOPT0_FTM0FLTxSEL_MASK) >> SIM_FTMOPT0_FTM0FLTxSEL_SHIFT);
        break;
    case 1U:
        retValue = (uint8_t)((base->FTMOPT0 & SIM_FTMOPT0_FTM1FLTxSEL_MASK) >> SIM_FTMOPT0_FTM1FLTxSEL_SHIFT);
        break;
    case 2U:
        retValue = (uint8_t)((base->FTMOPT0 & SIM_FTMOPT0_FTM2FLTxSEL_MASK) >> SIM_FTMOPT0_FTM2FLTxSEL_SHIFT);
        break;
    case 3U:
        retValue = (uint8_t)((base->FTMOPT0 & SIM_FTMOPT0_FTM3FLTxSEL_MASK) >> SIM_FTMOPT0_FTM3FLTxSEL_SHIFT);
        break;
    default:
        /* Instance invalid value, this error is caught by DEV_ASSERT(instance < ADC_INSTANCE_COUNT) */
        retValue = 0U;
        break;
    }

    return retValue;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SIM_HAL_GetLpoFreq
 * Description   : Get SIM LPO 1KHz clock frequency (LPO_CLOCK).
 *
 * Implements SIM_HAL_GetLpoFreq_Activity
 *END*************************************************************************/
uint32_t SIM_HAL_GetLpoFreq(const SIM_Type * base)
{
    uint32_t freq = 0U;

    switch ((base->LPOCLKS & SIM_LPOCLKS_LPOCLKSEL_MASK) >> SIM_LPOCLKS_LPOCLKSEL_SHIFT)
    {
        case 0U:  /* SIM_LPO_CLK_SEL_LPO_128K */
            freq = LPO_128K_FREQUENCY;
            break;
        case 1U:  /* SIM_LPO_CLK_SEL_NO_CLOCK: */
            freq = 0U;
            break;
        case 2U:  /* SIM_LPO_CLK_SEL_LPO_32K: */
            freq = SIM_HAL_GetLpo32KFreq(base);
            break;
        case 3U:  /* SIM_LPO_CLK_SEL_LPO_1K:  */
            freq = SIM_HAL_GetLpo1KFreq(base);
            break;
        default:
            /* Invalid LPOCLKSEL selection.*/
            DEV_ASSERT(false);
            break;
    }

    return freq;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SIM_HAL_GetLpo128KFreq
 * Description   : Get SIM LPO 128KHz clock frequency (LPO_128K_CLOCK).
 *
 * Implements SIM_HAL_GetLpo128KFreq_Activity
 *END*************************************************************************/
uint32_t SIM_HAL_GetLpo128KFreq(const SIM_Type * base)
{
    (void)base;
    return LPO_128K_FREQUENCY;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SIM_HAL_GetLpo32KFreq
 * Description   : Get SIM LPO 32KHz clock frequency (LPO_32K_CLOCK).
 *
 * Implements SIM_HAL_GetLpo32KFreq_Activity
 *END*************************************************************************/
uint32_t SIM_HAL_GetLpo32KFreq(const SIM_Type * base)
{
    uint32_t retValue = 0U;
    if (((base->LPOCLKS & SIM_LPOCLKS_LPO32KCLKEN_MASK) >> SIM_LPOCLKS_LPO32KCLKEN_SHIFT) != 0U) {
        retValue = LPO_32K_FREQUENCY;
    }
    return retValue;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SIM_HAL_GetLpo1KFreq
 * Description   : Get SIM LPO 1KHz clock frequency (LPO_1K_CLOCK).
 *
 * Implements SIM_HAL_GetLpo1KFreq_Activity
 *END*************************************************************************/
uint32_t SIM_HAL_GetLpo1KFreq(const SIM_Type * base)
{
    uint32_t retValue = 0U;
    if (((base->LPOCLKS & SIM_LPOCLKS_LPO1KCLKEN_MASK) >> SIM_LPOCLKS_LPO1KCLKEN_SHIFT) != 0U) {
        retValue = LPO_1K_FREQUENCY;
    }
    return retValue;
}

 /*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetAdcPreTriggerMode
 * Description   : Set ADCx pre-trigger select setting
 * This function will select the ADCx pre-trigger source
 *
 * Implements SIM_HAL_SetAdcPreTriggerMode_Activity
 *END**************************************************************************/
void SIM_HAL_SetAdcPreTriggerMode(SIM_Type * base,
                                  uint32_t instance,
                                  sim_adc_pretrg_sel_t select)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    switch (instance)
    {
    case 0U:
        REG_RMW32(&(base->ADCOPT), SIM_ADCOPT_ADC0PRETRGSEL_MASK, SIM_ADCOPT_ADC0PRETRGSEL(select));
        break;
    case 1U:
        REG_RMW32(&(base->ADCOPT), SIM_ADCOPT_ADC1PRETRGSEL_MASK, SIM_ADCOPT_ADC1PRETRGSEL(select));
        break;
    default:
        /* Instance invalid value, this error is caught by DEV_ASSERT(instance < ADC_INSTANCE_COUNT) */
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetAdcPreTriggerMode
 * Description   : Get ADCx pre-trigger select setting
 * This function will get ADCx pre-trigger select setting.
 *
 * Implements SIM_HAL_GetAdcPreTriggerMode_Activity
 *END**************************************************************************/
sim_adc_pretrg_sel_t SIM_HAL_GetAdcPreTriggerMode(const SIM_Type * base,
                                                  uint32_t instance)
{
    sim_adc_pretrg_sel_t retValue;
    uint32_t value;

    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    switch (instance)
    {
    case 0U:
        value = ((base->ADCOPT & SIM_ADCOPT_ADC0PRETRGSEL_MASK) >> SIM_ADCOPT_ADC0PRETRGSEL_SHIFT);
        break;
    case 1U:
        value = ((base->ADCOPT & SIM_ADCOPT_ADC1PRETRGSEL_MASK) >> SIM_ADCOPT_ADC1PRETRGSEL_SHIFT);
        break;
    default:
        /* Instance invalid value, this error is caught by DEV_ASSERT(instance < ADC_INSTANCE_COUNT) */
        value = 0U;
        break;
    }

    switch(value)
    {
        case 0U:
            retValue = SIM_ADC_PRETRG_SEL_PDB;
            break;
        case 1U:
            retValue = SIM_ADC_PRETRG_SEL_TRGMUX;
            break;
        case 2U:
            retValue = SIM_ADC_PRETRG_SEL_SOFTWARE;
            break;
        case 3U:
            /* Pass-through */
        default:
            retValue = SIM_ADC_PRETRG_SEL_RESERVED;
            break;
    }

    return retValue;
}

 /*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetAdcSwPreTriggerMode
 * Description   : Set ADCx software pre-trigger select setting
 * This function will select the ADCx software pre-trigger source
 *
 * Implements SIM_HAL_SetAdcSwPreTriggerMode_Activity
 *END**************************************************************************/
void SIM_HAL_SetAdcSwPreTriggerMode(SIM_Type * base,
                                  uint32_t instance,
                                  sim_adc_sw_pretrg_sel_t select)
{
    uint32_t regValue;
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    regValue = base->ADCOPT;
    switch (instance)
    {
        case 0U:
            regValue &= ~(SIM_ADCOPT_ADC0SWPRETRG_MASK);
            regValue |= SIM_ADCOPT_ADC0SWPRETRG(select);
            break;
        case 1U:
            regValue &= ~(SIM_ADCOPT_ADC1SWPRETRG_MASK);
            regValue |= SIM_ADCOPT_ADC1SWPRETRG(select);
            break;
    default:
        /* Instance invalid value, this error is caught by DEV_ASSERT(instance < ADC_INSTANCE_COUNT) */
        break;
    }
    base->ADCOPT = regValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetAdcSwPreTriggerMode
 * Description   : Get ADCx software pre-trigger select setting
 * This function will get ADCx software pre-trigger select setting.
 *
 * Implements SIM_HAL_GetAdcSwPreTriggerMode_Activity
 *END**************************************************************************/
sim_adc_sw_pretrg_sel_t SIM_HAL_GetAdcSwPreTriggerMode(const SIM_Type * base,
                                                       uint32_t instance)
{
    sim_adc_sw_pretrg_sel_t retValue;
    uint32_t regValue = base->ADCOPT;

    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    switch (instance)
    {
    case 0U:
        regValue = (regValue & SIM_ADCOPT_ADC0SWPRETRG_MASK) >> SIM_ADCOPT_ADC0SWPRETRG_SHIFT;
        break;
    case 1U:
        regValue = (regValue & SIM_ADCOPT_ADC1SWPRETRG_MASK) >> SIM_ADCOPT_ADC1SWPRETRG_SHIFT;
        break;
    default:
        /* Instance invalid value, this error is caught by DEV_ASSERT(instance < ADC_INSTANCE_COUNT) */
        break;
    }

    switch(regValue)
    {
        case 7U:
            retValue = SIM_ADC_SW_PRETRG_SEL_3;
            break;
        case 6U:
            retValue = SIM_ADC_SW_PRETRG_SEL_2;
            break;
        case 5U:
            retValue = SIM_ADC_SW_PRETRG_SEL_1;
            break;
        case 4U:
            retValue = SIM_ADC_SW_PRETRG_SEL_0;
            break;
        case 3U:
            retValue = SIM_ADC_SW_PRETRG_SEL_RESERVED2;
            break;
        case 2U:
            retValue = SIM_ADC_SW_PRETRG_SEL_RESERVED1;
            break;
        case 1U:
            retValue = SIM_ADC_SW_PRETRG_SEL_RESERVED0;
            break;
        case 0U:
            /* Pass-through */
        default:
            retValue = SIM_ADC_SW_PRETRG_SEL_DISABLED;
            break;
    }
    return retValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetAdcTriggerMode
 * Description   : Set ADCx trigger select setting
 * This function will select the ADCx trigger source
 *
 * Implements SIM_HAL_SetAdcTriggerMode_Activity
 *END**************************************************************************/
void SIM_HAL_SetAdcTriggerMode(SIM_Type * base,
                               uint32_t instance,
                               sim_adc_trg_sel_t select)
{
    uint32_t regValue, selectValue;
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    switch(select)
    {
        case SIM_ADC_TRG_SEL_TRGMUX:
            selectValue = 1U;
            break;
        case SIM_ADC_TRG_SEL_PDB:
        /* Pass through */
        default:
            selectValue = 0U;
            break;
    }


    regValue = (uint32_t)base->ADCOPT;

    switch (instance)
    {
        case 0:
            regValue &= (uint32_t)(~(SIM_ADCOPT_ADC0TRGSEL_MASK));
            regValue |= SIM_ADCOPT_ADC0TRGSEL(selectValue);
            break;
        case 1:
            regValue &= (uint32_t)(~(SIM_ADCOPT_ADC1TRGSEL_MASK));
            regValue |= SIM_ADCOPT_ADC1TRGSEL(selectValue);
            break;
        default:
            /* Instance invalid value, this error is caught by DEV_ASSERT(instance < ADC_INSTANCE_COUNT) */
            break;
    }

    base->ADCOPT = (uint32_t)regValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetAdcTriggerMode
 * Description   : Get ADCx trigger select setting
 * This function will get ADCx trigger select setting.
 *
 * Implements SIM_HAL_GetAdcTriggerMode_Activity
 *END**************************************************************************/
sim_adc_trg_sel_t SIM_HAL_GetAdcTriggerMode(const SIM_Type * base, uint32_t instance)
{
    sim_adc_trg_sel_t retValue;
    uint32_t regValue = (uint32_t)base->ADCOPT;

    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    switch (instance)
    {
    case 0:
        regValue = (regValue & SIM_ADCOPT_ADC0TRGSEL_MASK) >> SIM_ADCOPT_ADC0TRGSEL_SHIFT;
        break;
    case 1:
        regValue = (regValue & SIM_ADCOPT_ADC1TRGSEL_MASK) >> SIM_ADCOPT_ADC1TRGSEL_SHIFT;
        break;
    default:
        /* Instance invalid value, this error is caught by DEV_ASSERT(instance < ADC_INSTANCE_COUNT) */
        break;
    }

    switch(regValue)
    {
        case 1U:
            retValue = SIM_ADC_TRG_SEL_TRGMUX;
            break;
        case 0U:
            /* Pass-through */
        default:
            retValue = SIM_ADC_TRG_SEL_PDB;
            break;
    }
    return retValue;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_CH_OUT_SRC_MASK
 * Description   : Internal function for FTMxOCHySRC
 *END**************************************************************************/
static inline uint32_t FTM_CH_OUT_SRC_MASK(uint32_t instance,
                                           uint8_t channel)
{
    return 1UL << ((((instance)>>1U)*8U) + channel + 16U);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetFtmChOutSrcMode
 * Description   : FlexTimer x channel y output source select setting.
 * This function will select FlexTimer x channel y output source
 *
 * Implements SIM_HAL_SetFtmChOutSrcMode_Activity
 *END**************************************************************************/
void SIM_HAL_SetFtmChOutSrcMode(SIM_Type * base,
                                uint32_t instance,
                                uint8_t channel,
                                sim_ftm_ch_out_src_t select)
{
    DEV_ASSERT((0U==instance) || (3U==instance));
    DEV_ASSERT(SIM_FTMOPT1_FTM3_OUTSEL_WIDTH > channel);

    switch(select)
    {
        case SIM_FTM_CH_OUT_SRC_0:
            base->FTMOPT1 = base->FTMOPT1 & ~(FTM_CH_OUT_SRC_MASK(instance, channel));
            break;
        case SIM_FTM_CH_OUT_SRC_1:
            base->FTMOPT1 = base->FTMOPT1 | (FTM_CH_OUT_SRC_MASK(instance, channel));
            break;
        default:
            /* invalid select value */
            DEV_ASSERT(false);
            break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetFtmChOutSrcMode
 * Description   : Get FlexTimer x channel y output source select setting
 * This function will get FlexTimer x channel y output source select
 * setting.
 *
 * Implements SIM_HAL_GetFtmChOutSrcMode_Activity
 *END**************************************************************************/
sim_ftm_ch_out_src_t SIM_HAL_GetFtmChOutSrcMode(const SIM_Type * base,
                                                uint32_t instance,
                                                uint8_t channel)
{
    sim_ftm_ch_out_src_t retValue;

    DEV_ASSERT((0U==instance) || (3U==instance));
    DEV_ASSERT(SIM_FTMOPT1_FTM3_OUTSEL_WIDTH > channel);

    if ((base->FTMOPT1 & FTM_CH_OUT_SRC_MASK(instance, channel)) != 0U)
    {
        retValue = SIM_FTM_CH_OUT_SRC_1;
    }
    else
    {
        retValue = SIM_FTM_CH_OUT_SRC_0;
    }
    return retValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetFtmChSrcMode
 * Description   : FlexTimer x channel y input source select setting
 * This function will select FlexTimer x channel y input source
 *
 * Implements SIM_HAL_SetFtmChSrcMode_Activity
 *END**************************************************************************/
void SIM_HAL_SetFtmChSrcMode(SIM_Type * base,
                             uint32_t instance,
                             uint8_t  channel,
                             sim_ftm_ch_src_t select)
{
    uint32_t regValue = base->FTMOPT1;
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT((instance == 2U) && (((uint8_t)select) > SIM_FTMOPT1_FTM2CH1SEL_WIDTH));

    switch (instance)
    {
    case 1:
        switch (channel)
        {
        case 0:
            regValue &= ~(SIM_FTMOPT1_FTM1CH0SEL_MASK);
            regValue |= SIM_FTMOPT1_FTM1CH0SEL(select);
            break;
        default:
            /* Invalid channel value */
            DEV_ASSERT(false);
            break;
        }
        break;
    case 2:
        switch (channel)
        {
            case 0:
                regValue &= ~(SIM_FTMOPT1_FTM2CH0SEL_MASK);
                regValue |= SIM_FTMOPT1_FTM2CH0SEL(select);
                break;
            case 1:
                regValue &= ~(SIM_FTMOPT1_FTM2CH1SEL_MASK);
                regValue |= SIM_FTMOPT1_FTM2CH1SEL(select);
                break;
            default:
                /* Invalid channel value */
                DEV_ASSERT(false);
                break;
        }
        break;
    default:
        /* Invalid instance value, this error is caught by development checking: DEV_ASSERT(instance < FTM_INSTANCE_COUNT) */
        break;
    }
    base->FTMOPT1 = regValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetFtmChSrcMode
 * Description   : Get FlexTimer x channel y input source select setting
 * This function will get FlexTimer x channel y input source select
 * setting.
 *
 * Implements SIM_HAL_GetFtmChSrcMode_Activity
 *END**************************************************************************/
sim_ftm_ch_src_t SIM_HAL_GetFtmChSrcMode(const SIM_Type * base,
                                         uint32_t instance,
                                         uint8_t channel)
{
    sim_ftm_ch_src_t retValue;
    uint32_t regValue;

    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);

    regValue = base->FTMOPT1;
    switch (instance)
    {
    case 1:
        switch (channel)
        {
        case 0:
            regValue = (regValue & SIM_FTMOPT1_FTM1CH0SEL_MASK) >> SIM_FTMOPT1_FTM1CH0SEL_SHIFT;
            break;
        default:
            /* Invalid channel value */
            DEV_ASSERT(false);
            break;
        }
        break;
    case 2:
        switch (channel)
        {
        case 0:
            regValue = (regValue & SIM_FTMOPT1_FTM2CH0SEL_MASK) >> SIM_FTMOPT1_FTM2CH0SEL_SHIFT;
            break;
        case 1:
            regValue = (regValue & SIM_FTMOPT1_FTM2CH1SEL_MASK) >> SIM_FTMOPT1_FTM2CH1SEL_SHIFT;
            break;
        default:
            /* Invalid channel value */
            DEV_ASSERT(false);
            break;
        }
        break;
    default:
        /* Nothing to do, error is caught by DEV_ASSERT(instance < FTM_INSTANCE_COUNT) */
        break;
    }

    switch(regValue)
    {
        case 3U:
            retValue = SIM_FTM_CH_SRC_3;
            break;
        case 2U:
            retValue = SIM_FTM_CH_SRC_2;
            break;
        case 1U:
            retValue = SIM_FTM_CH_SRC_1;
            break;
        case 0U:
            /* Pass-through */
        default:
            retValue = SIM_FTM_CH_SRC_0;
            break;
    }
    return retValue;
}

 /*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetFtmSyncCmd
 * Description   : Set FTMxSYNCBIT
 * This function sets FlexTimer x hardware trigger 0 software synchronization.
 *
 * Implements SIM_HAL_SetFtmSyncCmd_Activity
 *END**************************************************************************/
void SIM_HAL_SetFtmSyncCmd(SIM_Type * base, uint32_t instance, bool sync)
{
    uint32_t syncValue = (sync == false) ? 0UL : 1UL;
    uint32_t regValue = base->FTMOPT1;
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);

    switch (instance)
    {
    case 0U:
        regValue &= ~(SIM_FTMOPT1_FTM0SYNCBIT_MASK);
        regValue |= SIM_FTMOPT1_FTM0SYNCBIT(syncValue);
        break;
    case 1U:
        regValue &= ~(SIM_FTMOPT1_FTM1SYNCBIT_MASK);
        regValue |= SIM_FTMOPT1_FTM1SYNCBIT(syncValue);
        break;
    case 2U:
        regValue &= ~(SIM_FTMOPT1_FTM2SYNCBIT_MASK);
        regValue |= SIM_FTMOPT1_FTM2SYNCBIT(syncValue);
        break;
    case 3U:
        regValue &= ~(SIM_FTMOPT1_FTM3SYNCBIT_MASK);
        regValue |= SIM_FTMOPT1_FTM3SYNCBIT(syncValue);
        break;
#if FTM_INSTANCE_COUNT > 4U
    case 4U:
        regValue &= ~(SIM_FTMOPT1_FTM4SYNCBIT_MASK);
        regValue |= SIM_FTMOPT1_FTM4SYNCBIT(syncValue);
        break;
#endif
#if FTM_INSTANCE_COUNT > 5U
    case 5U:
        regValue &= ~(SIM_FTMOPT1_FTM5SYNCBIT_MASK);
        regValue |= SIM_FTMOPT1_FTM5SYNCBIT(syncValue);
        break;
#endif
#if FTM_INSTANCE_COUNT > 6U
    case 6U:
        regValue &= ~(SIM_FTMOPT1_FTM6SYNCBIT_MASK);
        regValue |= SIM_FTMOPT1_FTM6SYNCBIT(syncValue);
        break;
#endif
#if FTM_INSTANCE_COUNT > 7U
    case 7U:
        regValue &= ~(SIM_FTMOPT1_FTM7SYNCBIT_MASK);
        regValue |= SIM_FTMOPT1_FTM7SYNCBIT(syncValue);
        break;
#endif
    default:
        /* Nothing to do, error is caught by DEV_ASSERT(instance < FTM_INSTANCE_COUNT) */
        break;
    }
    base->FTMOPT1 = regValue;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SIM_HAL_GetTraceClockDefaultConfig
 * Description   : This function gets the default Debug Trace Clock configuration.
 *
 * Implements SIM_HAL_GetTraceClockDefaultConfig_Activity
 *END*************************************************************************/
void SIM_HAL_GetTraceClockDefaultConfig(sim_trace_clock_config_t *config)
{
    DEV_ASSERT(config != NULL);

    config->initialize        = true;
    config->divEnable         = true;
    config->source            = CLOCK_TRACE_SRC_CORE_CLK;
    config->divider           = 0U;
    config->divFraction       = false;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SIM_HAL_InitTraceClock
 * Description   : This function enables the SIM Debug Trace clock according
 * to the configuration.
 * @note This function ignores initialize member
 *
 * Implements SIM_HAL_InitTraceClock_Activity
 *END*************************************************************************/
void SIM_HAL_InitTraceClock(SIM_Type * base,
                            const sim_trace_clock_config_t *config)
{
    uint32_t regValue;
    DEV_ASSERT(config != NULL);

    /* Disable divider. */
    base->CLKDIV4 &= (uint32_t)(~(SIM_CLKDIV4_TRACEDIVEN_MASK));

    /* Configure source. */
    regValue = (uint32_t)base->CHIPCTL;
    regValue &= (uint32_t)(~(SIM_CHIPCTL_TRACECLK_SEL_MASK));
    regValue |= SIM_CHIPCTL_TRACECLK_SEL(config->source);
    base->CHIPCTL = (uint32_t)regValue;

    /* Configure divider. */
    regValue = base->CLKDIV4;
    regValue &= ~(SIM_CLKDIV4_TRACEDIV_MASK);
    regValue |= SIM_CLKDIV4_TRACEDIV(config->divider);
    base->CLKDIV4 = regValue;

    /* Configure fraction. */
    regValue = (uint32_t)base->CLKDIV4;
    regValue &= (uint32_t)(~(SIM_CLKDIV4_TRACEFRAC_MASK));
    regValue |= SIM_CLKDIV4_TRACEFRAC(config->divFraction ? 1UL : 0UL);
    base->CLKDIV4 = (uint32_t)regValue;

    /* Configure divider enable. */
    regValue = (uint32_t)base->CLKDIV4;
    regValue &= (uint32_t)(~(SIM_CLKDIV4_TRACEDIVEN_MASK));
    regValue |= SIM_CLKDIV4_TRACEDIVEN(config->divEnable ? 1UL : 0UL);
    base->CLKDIV4 = (uint32_t)regValue;
}


/*******************************************************************************
 * EOF
 ******************************************************************************/

