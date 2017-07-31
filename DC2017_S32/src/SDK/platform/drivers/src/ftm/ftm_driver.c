/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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
/*!
 * @file ftm_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable. A source of error in
 * writing dynamic code is that the stack segment may be different from the data
 * segment.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, external symbol defined without a prior
 * declaration.
 * The symbols are declared in the driver common file as external; they are needed
 * at driver initialization to install the correct interrupt handler, but are not
 * a part of the public API.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * The function is defined for use by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, could define variable at block scope
 * The variables are defined in the source file to make transition to other
 * platforms easier.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from
 * 'essentially enum<i>' to 'essentially Boolean'. This is required by
 * the conversion of a enum type into a bool type.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 */

#include "ftm_driver.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Table of base addresses for FTM instances. */
FTM_Type * const g_ftmBase[FTM_INSTANCE_COUNT] = FTM_BASE_PTRS;

/*! @brief Interrupt vectors for the FTM peripheral. */
const IRQn_Type g_ftmIrqId[FTM_INSTANCE_COUNT][FEATURE_FTM_CHANNEL_COUNT] = FTM_IRQS;
const IRQn_Type g_ftmFaultIrqId[FTM_INSTANCE_COUNT] = FTM_Fault_IRQS;
const IRQn_Type g_ftmOverflowIrqId[FTM_INSTANCE_COUNT] = FTM_Overflow_IRQS;
const IRQn_Type g_ftmReloadIrqId[FTM_INSTANCE_COUNT] = FTM_Reload_IRQS;

/*! @brief Pointer to runtime state structure. */
ftm_state_t * ftmStatePtr[FTM_INSTANCE_COUNT] = {NULL};

/*! @brief  Select external clock pin or clock source for peripheral */
static const clock_names_t g_ftmExtClockSel[FTM_INSTANCE_COUNT][2] = {{SIM_FTM0_CLOCKSEL, PCC_FTM0_CLOCK},
                                                                      {SIM_FTM1_CLOCKSEL, PCC_FTM1_CLOCK},
                                                                      {SIM_FTM2_CLOCKSEL, PCC_FTM2_CLOCK},
                                                                      {SIM_FTM3_CLOCKSEL, PCC_FTM3_CLOCK}};

/*******************************************************************************
 * Code
 ******************************************************************************/
static void FTM_DRV_InputCaptureHandler(uint32_t instance,
                                        uint8_t channelPair);

static void FTM_DRV_IrqHandler(uint32_t instance,
                               uint8_t channelPair);

void FTM0_Ch0_Ch1_IRQHandler(void);

void FTM0_Ch2_Ch3_IRQHandler(void);

void FTM0_Ch4_Ch5_IRQHandler(void);

void FTM0_Ch6_Ch7_IRQHandler(void);

void FTM1_Ch0_Ch1_IRQHandler(void);

void FTM1_Ch2_Ch3_IRQHandler(void);

void FTM1_Ch4_Ch5_IRQHandler(void);

void FTM1_Ch6_Ch7_IRQHandler(void);

void FTM2_Ch0_Ch1_IRQHandler(void);

void FTM2_Ch2_Ch3_IRQHandler(void);

void FTM2_Ch4_Ch5_IRQHandler(void);

void FTM2_Ch6_Ch7_IRQHandler(void);

void FTM3_Ch0_Ch1_IRQHandler(void);

void FTM3_Ch2_Ch3_IRQHandler(void);

void FTM3_Ch4_Ch5_IRQHandler(void);

void FTM3_Ch6_Ch7_IRQHandler(void);

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_Init
 * Description   : Initializes the FTM driver and get the clock frequency value
 * which select one of three possible clock sources for the FTM counter.
 *
 * Implements    : FTM_DRV_Init_Activity
 *END**************************************************************************/
status_t FTM_DRV_Init(uint32_t instance,
                      const ftm_user_config_t * info,
                      ftm_state_t * state)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(info != NULL);
    DEV_ASSERT(state != NULL);
    FTM_Type * ftmBase = g_ftmBase[instance];
    status_t status = STATUS_SUCCESS;

    /* Check if this instance is already initialized */
    if (ftmStatePtr[instance] != NULL)
    {
        status = STATUS_ERROR;
    }
    else
    {
        /* Configure state structure. */
        state->ftmClockSource = info->ftmClockSource;
        state->ftmMode = FTM_MODE_NOT_INITIALIZED;
        state->ftmPeriod = 0U;
        ftmStatePtr[instance] = state;
        /* The reset operation doesn't care about write protection. FTM_HAL_Reset will
         * disable this protection.*/
        FTM_HAL_Reset(ftmBase);
        FTM_HAL_Init(ftmBase, info->ftmPrescaler);
        /* Get clock name used to configure the FlexTimer module */
        state->ftmSourceClockFrequency = FTM_DRV_GetFrequency(instance);
        /* Check the functional clock is selected for FTM */
        DEV_ASSERT(state->ftmSourceClockFrequency > 0U);
    }

    if (STATUS_SUCCESS == status)
    {
        /* Check if the mode operation in PWM mode */
        if ((FTM_MODE_EDGE_ALIGNED_PWM == info->ftmMode) || (FTM_MODE_CEN_ALIGNED_PWM == info->ftmMode) || (FTM_MODE_OUTPUT_COMPARE == info->ftmMode))
        {
            /* Configure sync for between registers and buffers */
            status = FTM_DRV_SetSync(instance, &(info->syncMethod));
        }

        /* Enable the generation of initialization trigger on chip module */
        FTM_HAL_SetInitTriggerCmd(ftmBase, info->enableInitializationTrigger);
        FTM_HAL_SetBdmMode(ftmBase, info->BDMMode);

        /* Check if enable interrupt in counter mode */
        if (info->isTofIsrEnabled)
        {
            FTM_HAL_SetTimerOverflowInt(ftmBase, true);
            INT_SYS_EnableIRQ(g_ftmOverflowIrqId[instance]);
        }
        else
        {
            FTM_HAL_SetTimerOverflowInt(ftmBase, false);
            INT_SYS_DisableIRQ(g_ftmOverflowIrqId[instance]);
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_Deinit
 * Description   : Shuts down the FTM driver.
 * First, FTM_DRV_Init must be called. Then this function will disables the FTM module.
 *
 * Implements    : FTM_DRV_Deinit_Activity
 *END**************************************************************************/
status_t FTM_DRV_Deinit(uint32_t instance)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    /* Reset all FTM register */
    FTM_HAL_Reset(ftmBase);
    ftmStatePtr[instance] = NULL;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_InitCounter
 * Description   : Initializes the FTM counter. This function provides access to the
 * FTM counter settings. The counter can be run in Up counting or Up-down counting modes.
 * To run the counter in Free running mode, choose Up counting option and provide
 * 0x0 for the countStartVal and 0xFFFF for countFinalVal. Please call this
 * function only when FTM is used as timer/counter.
 *
 * Implements    : FTM_DRV_InitCounter_Activity
 *END**************************************************************************/
status_t FTM_DRV_InitCounter(uint32_t instance,
                             const ftm_timer_param_t * timer)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(timer != NULL);
    DEV_ASSERT((FTM_MODE_UP_TIMER == timer->mode) || (FTM_MODE_UP_DOWN_TIMER == timer->mode));
    FTM_Type * ftmBase = g_ftmBase[instance];
    ftm_state_t * state = ftmStatePtr[instance];
    status_t retStatus = STATUS_SUCCESS;
    uint8_t channel = 0U;

    if ((NULL != state) && (FTM_MODE_NOT_INITIALIZED == state->ftmMode))
    {
        /* Disable counter clock */
        FTM_HAL_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
        /* Clear the overflow flag */
        FTM_HAL_ClearTimerOverflow(ftmBase);
        /* Set counter initial and maximum values */
        FTM_HAL_SetCounterInitVal(ftmBase, timer->initialValue);
        FTM_HAL_SetMod(ftmBase, timer->finalValue);
        /* Disable the quadrature decoder mode */
        FTM_HAL_SetQuadDecoderCmd(ftmBase, false);
        /* Use FTM as counter, disable all the channels */
        for (channel = 0U; channel < FEATURE_FTM_CHANNEL_COUNT; channel++)
        {
            FTM_HAL_SetChnEdgeLevel(ftmBase, channel, 0U);
        }

        /* Check the FTM counter modes */
        if (FTM_MODE_UP_TIMER == timer->mode)
        {
            FTM_HAL_SetCpwms(ftmBase, false);
        }
        else if (FTM_MODE_UP_DOWN_TIMER == timer->mode)
        {
            FTM_HAL_SetCpwms(ftmBase, true);
        }
        else
        {
            /* Do nothing */
        }

        state->ftmMode = timer->mode;
    }
    else
    {
        retStatus = STATUS_ERROR;
    }

    return retStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_CounterStart
 * Description   : Starts the FTM counter.
 *
 * Implements    : FTM_DRV_CounterStart_Activity
 *END**************************************************************************/
status_t FTM_DRV_CounterStart(uint32_t instance)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    const ftm_state_t * state = ftmStatePtr[instance];
    /* Check the clock source is available for FTM counter */
    DEV_ASSERT(state->ftmSourceClockFrequency > 0U);
    /* Enable counter clock */
    FTM_HAL_SetClockSource(ftmBase, state->ftmClockSource);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_CounterStop
 * Description   : Stops the FTM counter.
 *
 * Implements    : FTM_DRV_CounterStop_Activity
 *END**************************************************************************/
status_t FTM_DRV_CounterStop(uint32_t instance)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];
    ftm_state_t * state = ftmStatePtr[instance];

    /* Stop the FTM counter */
    FTM_HAL_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
    state->ftmMode = FTM_MODE_NOT_INITIALIZED;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_CounterRead
 * Description   : Reads back the current value of the FTM counter.
 *
 * Implements    : FTM_DRV_CounterRead_Activity
 *END**************************************************************************/
uint32_t FTM_DRV_CounterRead(uint32_t instance)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type const * ftmBase = g_ftmBase[instance];

    return FTM_HAL_GetCounter(ftmBase);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_DeinitPwm
 * Description   : Stops all PWM channels.
 *
 * Implements    : FTM_DRV_DeinitPwm_Activity
 *END**************************************************************************/
status_t FTM_DRV_DeinitPwm(uint32_t instance)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];
    uint8_t channel;
    uint8_t chnlPairNum;
    ftm_state_t * state = ftmStatePtr[instance];

    /* Stop the FTM counter */
    FTM_HAL_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
    for (channel = 0U; channel < FEATURE_FTM_CHANNEL_COUNT; channel++)
    {
        chnlPairNum = (uint8_t)(channel >> 1U);
        /* Disable PWM mode in hardware */
        FTM_HAL_SetChnCountVal(ftmBase, channel, 0U);
        FTM_HAL_SetChnEdgeLevel(ftmBase, channel, 0U);
        FTM_HAL_SetChnMSnBAMode(ftmBase, channel, 0U);
        FTM_HAL_SetCpwms(ftmBase, false);
        /* Configure polarity bit */
        FTM_HAL_SetChnOutputPolarityCmd(ftmBase, channel, FTM_POLARITY_LOW);
        FTM_HAL_DisablePwmChannelOutputs(ftmBase, channel);
        /* Clear the PWM synchronization */
        FTM_HAL_SetDualChnPwmSyncCmd(ftmBase, chnlPairNum, false);
        /* Clear combination for each pair of channels */
        FTM_HAL_SetDualChnMofCombineCmd(ftmBase, chnlPairNum, false);
        FTM_HAL_SetDualChnCombineCmd(ftmBase, chnlPairNum, false);
        FTM_HAL_SetDualChnDeadtimeCmd(ftmBase, chnlPairNum, false);
        FTM_HAL_SetDualChnFaultCmd(ftmBase, chnlPairNum, false);
    }

    /* Clear the dead-time pre-scaler and value */
    FTM_HAL_SetExtDeadtimeValue(ftmBase, 0U);
    FTM_HAL_SetDeadtimePrescale(ftmBase, FTM_DEADTIME_DIVID_BY_1);
    FTM_HAL_SetDeadtimeCount(ftmBase, 0U);
    /* Clear fault control register */
    FTM_HAL_ClearFaultControl(ftmBase);
    /* Disable fault interrupt */
    FTM_HAL_SetFaultInt(ftmBase, false);
    /* Disable fault control */
    FTM_HAL_SetFaultControlMode(ftmBase, FTM_FAULT_CONTROL_DISABLED);
    /* Clear the module value of the registers */
    FTM_HAL_SetMod(ftmBase, 0U);
    FTM_HAL_SetCounter(ftmBase, 0U);
    state->ftmMode = FTM_MODE_NOT_INITIALIZED;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_InitPwm
 * Description   : Configures duty cycle and frequency and starts outputting
 * PWM on specified channels.
 *
 * Implements    : FTM_DRV_InitPwm_Activity
 *END**************************************************************************/
status_t FTM_DRV_InitPwm(uint32_t instance,
                         const ftm_pwm_param_t * param)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(param != NULL);
    status_t retVal = STATUS_SUCCESS;
    uint8_t index = 0U;
    uint8_t hwChannel = 0U;
    uint8_t fltChannel = 0U;
    uint8_t chnlPairNum = 0U;
    uint8_t channelId = 0U;
    ftm_state_t * state = ftmStatePtr[instance];
    FTM_Type * ftmBase = g_ftmBase[instance];

    if ((NULL != state) && (FTM_MODE_NOT_INITIALIZED == state->ftmMode))
    {
        /* Disable counter clock */
        FTM_HAL_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
        /* Clear the overflow flag */
        FTM_HAL_ClearTimerOverflow(ftmBase);
        /* Disable write protection */
        FTM_HAL_SetWriteProtectionCmd(ftmBase, false);
        /* Configure FTM mode */
        state->ftmMode = param->mode;

        /* Configure independent PWM channels */
        for (index = 0U; index < param->nNumIndependentPwmChannels; index++)
        {
            channelId = param->pwmIndependentChannelConfig[index].hwChannelId;
            chnlPairNum =  (uint8_t)(channelId >> 1U);
            FTM_HAL_SetDualEdgeCaptureCmd(ftmBase, chnlPairNum, false);
            /* Set ELSB bit and clear ELSA bit*/
            FTM_HAL_SetChnEdgeLevel(ftmBase, channelId, 2U);
            /* Set MSB and MSA bits*/
            FTM_HAL_SetChnMSnBAMode(ftmBase, channelId, 3U);
            /* Write FTMn_PWMLOAD register to enable synchronized loading points for the given channel */
            FTM_HAL_EnablePwmChannelOutputs(ftmBase, channelId);
            FTM_HAL_SetChnOutputPolarityCmd(ftmBase, channelId, param->pwmIndependentChannelConfig[index].polarity);
            FTM_HAL_SetDualChnFaultCmd(ftmBase, chnlPairNum, ((param->faultConfig)->faultMode != FTM_FAULT_CONTROL_DISABLED) ? true : false);
            /* Enable sync control for channels*/
            FTM_HAL_SetDualChnPwmSyncCmd(ftmBase, chnlPairNum, true);
            FTM_HAL_SetDualChnCombineCmd(ftmBase, chnlPairNum, false);
            FTM_HAL_SetDualChnMofCombineCmd(ftmBase, chnlPairNum, false);
            /* Enable the generation a trigger on chip module */
            FTM_HAL_SetChnTriggerCmd(ftmBase, channelId, param->pwmIndependentChannelConfig[index].enableExternalTrigger);
        }

        /* Configure combined PWM channels */
        for (index = 0U; index < param->nNumCombinedPwmChannels; index++)
        {
            channelId = param->pwmCombinedChannelConfig[index].hwChannelId;
            chnlPairNum =  (uint8_t)(channelId >> 1U);
            FTM_HAL_SetDualEdgeCaptureCmd(ftmBase, chnlPairNum, false);
            /* Set ELSB bit and clear ELSA bit*/
            FTM_HAL_SetChnEdgeLevel(ftmBase, channelId, 2U);
            /* Set MSB and MSA bits */
            FTM_HAL_SetChnMSnBAMode(ftmBase, channelId, 3U);
            /* Write FTMn_PWMLOAD register to enable synchronized loading points for the given channel */
            /* Enable channel (n) output */
            FTM_HAL_EnablePwmChannelOutputs(ftmBase, channelId);
            /* Enable channel (n+1) output */
            if (param->pwmCombinedChannelConfig[index].enableSecondChannelOutput)
            {
                FTM_HAL_EnablePwmChannelOutputs(ftmBase, (uint8_t)(channelId + 1U));
                /* When ELSB = 0 and ELSA = 0 for channel (n+1) output is not available */
                FTM_HAL_SetChnEdgeLevel(ftmBase, (uint8_t)(channelId + 1U), 2U);
                /* Configure complementary mode for channel (n+1) */
                FTM_HAL_SetDualChnCompCmd(ftmBase, chnlPairNum, param->pwmCombinedChannelConfig[index].secondChannelPolarity);
            }
            else
            {
                FTM_HAL_DisablePwmChannelOutputs(ftmBase, (uint8_t)(channelId + 1U));
            }

            FTM_HAL_SetChnOutputPolarityCmd(ftmBase, channelId, param->pwmCombinedChannelConfig[index].mainChannelPolarity);
            FTM_HAL_SetDualChnFaultCmd(ftmBase, chnlPairNum, ((param->faultConfig)->faultMode != FTM_FAULT_CONTROL_DISABLED) ? true : false);
            /* Enable sync control for channels */
            FTM_HAL_SetDualChnPwmSyncCmd(ftmBase, chnlPairNum, true);
            /* Enable the combine mode */
            FTM_HAL_SetDualChnCombineCmd(ftmBase, chnlPairNum, true);
            /* Configure the modified combine mode */
            FTM_HAL_SetDualChnMofCombineCmd(ftmBase, chnlPairNum, param->pwmCombinedChannelConfig[index].enableModifiedCombine);
            /* Configure dead time */
            FTM_HAL_SetDualChnDeadtimeCmd(ftmBase, chnlPairNum, param->pwmCombinedChannelConfig[index].deadTime);
            /* Enable the generation a trigger on the channel (n) */
            FTM_HAL_SetChnTriggerCmd(ftmBase, (chnlPairNum << 1U), param->pwmCombinedChannelConfig[index].enableExternalTrigger);
            /* Enable the generation a trigger on the channel (n+1) */
            FTM_HAL_SetChnTriggerCmd(ftmBase, (chnlPairNum << 1U) + 1U, param->pwmCombinedChannelConfig[index].enableExternalTriggerOnNextChn);
        }

        /* Set enable outputs to be set to Initial/default value */
        FTM_HAL_SetInitChnOutputCmd(ftmBase, true);
        /* Enable faults (if faults were configured) */
        if ((param->faultConfig)->faultMode != FTM_FAULT_CONTROL_DISABLED)
        {
            /* Configure PWM Output behavior */
            FTM_HAL_SetPwmFaultBehavior(ftmBase, ((param->faultConfig)->pwmOutputStateOnFault) ? true : false);
            /* Configure fault filter value */
            FTM_HAL_SetFaultInputFilterVal(ftmBase, ((param->faultConfig)->faultFilterValue));
            for (fltChannel = 0U; fltChannel < FTM_FEATURE_FAULT_CHANNELS; fltChannel++)
            {
                if (true == (param->faultConfig)->ftmFaultChannelParam[fltChannel].faultChannelEnabled)
                {
                    /* Enable fault channel */
                    FTM_HAL_SetFaultInputCmd(ftmBase, fltChannel, true);
                    /* Configure fault filter */
                    FTM_HAL_SetFaultInputFilterCmd(ftmBase,
                                                   fltChannel,
                                                   ((param->faultConfig)->ftmFaultChannelParam[fltChannel].faultFilterEnabled) ? true : false);
                    /* Configure fault outputs */
                    FTM_HAL_SetChnFaultInputPolarityCmd(ftmBase, fltChannel,
                                                        ((param->faultConfig)->ftmFaultChannelParam[fltChannel].ftmFaultPinPolarity));
                }
            }

            /* Set fault interrupt */
            if (true == ((param->faultConfig)->pwmFaultInterrupt))
            {
                FTM_HAL_SetFaultInt(ftmBase, true);
            }

            /* Enable fault control */
            FTM_HAL_SetFaultControlMode(ftmBase, (param->faultConfig)->faultMode);
        }

        /* Configure PWM mode: edge or center aligned */
        FTM_HAL_SetCpwms(ftmBase, (param->mode == FTM_MODE_CEN_ALIGNED_PWM) ? true : false);
        /* Calculate frequency of the give FTM hardware module - all channels will run at the same frequency */
        state->ftmPeriod = FTM_DRV_ConvertFreqToPeriodTicks(instance, param->uFrequencyHZ);
        /* Based on reference manual, in PWM mode CNTIN is to be set 0*/
        FTM_HAL_SetCounterInitVal(ftmBase, 0U);
        /* Write MOD register with the value of the period */
        /* For center aligned mode MOD register should be divided by 2 */
        /* For edge aligned mode period is determined by: MOD-CNTIN+1 */
        if (param->mode == FTM_MODE_CEN_ALIGNED_PWM)
        {
            FTM_HAL_SetMod(ftmBase, (uint16_t)(state->ftmPeriod >> 1U));
        }
        else
        {
            FTM_HAL_SetMod(ftmBase, (uint16_t)(state->ftmPeriod - 1U));
        }

        for (index = 0U; index < param->nNumIndependentPwmChannels; index++)
        {
            hwChannel = param->pwmIndependentChannelConfig[index].hwChannelId;
            /* Write CnV registers and setup duty cycle and phase values */
            retVal = FTM_DRV_UpdatePwmChannel(instance,
                                              hwChannel,
                                              FTM_PWM_UPDATE_IN_DUTY_CYCLE,
                                              param->pwmIndependentChannelConfig[index].uDutyCyclePercent,
                                              0U,
                                              false);
        }

        for (index = 0U; index < param->nNumCombinedPwmChannels; index++)
        {
            hwChannel = param->pwmCombinedChannelConfig[index].hwChannelId;
            /* Write CnV registers and setup duty cycle and phase values */
            retVal = FTM_DRV_UpdatePwmChannel(instance,
                                              hwChannel,
                                              FTM_PWM_UPDATE_IN_DUTY_CYCLE,
                                              param->pwmCombinedChannelConfig[index].firstEdge,
                                              param->pwmCombinedChannelConfig[index].secondEdge,
                                              false);
        }

        if (STATUS_SUCCESS == retVal)
        {
            /* Configure dead time for combine mode */
            FTM_HAL_SetDeadtimeCount(ftmBase, param->deadTimeValue);
            FTM_HAL_SetDeadtimePrescale(ftmBase, param->deadTimePrescaler);
            FTM_HAL_Enable(ftmBase, true);
            FTM_HAL_SetPwmSyncMode(ftmBase, true);
            /* Set clock source to start counter */
            FTM_HAL_SetClockSource(ftmBase, state->ftmClockSource);
        }
        else
        {
            /* Restore FTM mode if initialization fails */
            state->ftmMode = FTM_MODE_NOT_INITIALIZED;
        }
    }
    else
    {
        retVal = STATUS_ERROR;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_UpdatePwmChannel
 * Description   : This function will update the duty cycle of PWM output.
 * - If the type of update in the duty cycle, this function will convert the input parameters representing
 * frequency in Hz to a period value in ticks needed by the hardware timer. Period is calculated depending
 * on the operating mode of the FTM module and is stored in internal state structure.
 * firstEdge and secondEdge can have value between 0 - FTM_MAX_DUTY_CYCLE(0 = 0% duty
 * and FTM_MAX_DUTY_CYCLE = 100% duty). secondEdge value is used only whenFTM module is used in PWM combine mode.
 * - If the type of update in ticks, this function will get value in ticks to the hardware timer.
 * firstEdge and secondEdge variables can have value between 0 and ftmPeriod is stored in the state structure.
 * - in the modified combine mode, the firstEdge parameter is fixed value and only can modify the secondEdge variables
 * which is the initial value in the channel (n+1) match edge when the FTM counter has been ran.
 *
 * Implements    : FTM_DRV_UpdatePwmChannel_Activity
 *END**************************************************************************/
status_t FTM_DRV_UpdatePwmChannel(uint32_t instance,
                                  uint8_t channel,
                                  ftm_pwm_update_option_t typeOfUpdate,
                                  uint16_t firstEdge,
                                  uint16_t secondEdge,
                                  bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);
    uint16_t hwFirstEdge = 0U;
    uint16_t hwSecondEdge = 0U;
    uint16_t ftmPeriod = 0U;
    uint8_t chnlPairNum = (uint8_t)(channel >> 1U);
    FTM_Type * ftmBase = g_ftmBase[instance];
    ftm_state_t * state = ftmStatePtr[instance];
    status_t retStatus = STATUS_SUCCESS;

    /* Get the newest period in the MOD register */
    ftmPeriod = FTM_HAL_GetMod(ftmBase);
    /* Check the mode operation in FTM module */
    if (state->ftmMode == FTM_MODE_CEN_ALIGNED_PWM)
    {
        ftmPeriod = (uint16_t)(ftmPeriod << 1U);
    }
    else if (state->ftmMode == FTM_MODE_EDGE_ALIGNED_PWM)
    {
        ftmPeriod = (uint16_t)(ftmPeriod + 1U);
    }
    else
    {
        retStatus = STATUS_ERROR;
    }

    /* Check the type of update for PWM */
    if (FTM_PWM_UPDATE_IN_DUTY_CYCLE == typeOfUpdate)
    {
        if ((firstEdge <= FTM_MAX_DUTY_CYCLE) && (secondEdge <= FTM_MAX_DUTY_CYCLE))
        {
            /* Calculate DutyCycle based of the previously calculated frequency*/
            /* For greater resolution the DutyCycle values are in the range [0. FTM_MAX_DUTY_CYCLE]
             *  where 0 = 0% or PWM always at Low and FTM_MAX_DUTY_CYCLE = 100% or PWM always HIGH;
             *  a value of 0x4000 is equivalent of 50% DutyCycle. */
            hwFirstEdge = (uint16_t)((ftmPeriod * firstEdge) >> FTM_DUTY_TO_TICKS_SHIFT);
            hwSecondEdge = (uint16_t)((ftmPeriod * secondEdge) >> FTM_DUTY_TO_TICKS_SHIFT);
            /* adjust DutyCycle if 100% value is to be achieved. */
            if (FTM_MAX_DUTY_CYCLE == firstEdge)
            {
                /* If expected duty is 100% then increase by 1 the value that is to be written
                 *  to Hardware so it will exceed value of period */
                hwFirstEdge = (uint16_t)(hwFirstEdge + 1U);
            }
        }
        else
        {
            retStatus = STATUS_ERROR;
        }
    }
    else
    {
        if ((firstEdge <= (state->ftmPeriod)) && (secondEdge <= (state->ftmPeriod)))
        {
            hwFirstEdge = firstEdge;
            hwSecondEdge = secondEdge;
        }
        else
        {
            retStatus = STATUS_ERROR;
        }
    }

    if (STATUS_SUCCESS == retStatus)
    {
        if (true == FTM_HAL_GetDualChnCombineCmd(ftmBase, chnlPairNum))
        {
            if (true == FTM_HAL_GetDualChnMofCombineCmd(ftmBase, chnlPairNum))
            {
                /* Check the clock source for FTM counter is disabled or not */
                if (FTM_HAL_GetClockSource(ftmBase) == 0U)
                {
                    FTM_HAL_SetChnCountVal(ftmBase, (uint8_t)(chnlPairNum * 2U), hwFirstEdge);
                }
            }
            else
            {
                FTM_HAL_SetChnCountVal(ftmBase, (uint8_t)(chnlPairNum * 2U), hwFirstEdge);
            }

            /* Modify the initial value in the channel (n+1) match edge */
            FTM_HAL_SetChnCountVal(ftmBase, (uint8_t)((chnlPairNum * 2U) + 1U), hwSecondEdge);
        }
        else
        {
            /* Channel value is divided by 2 for up down counter mode to keep same duty */
            if (true == FTM_HAL_GetCpwms(ftmBase))
            {
                FTM_HAL_SetChnCountVal(ftmBase, channel, (uint16_t)(hwFirstEdge >> 1U));
            }
            else
            {
                FTM_HAL_SetChnCountVal(ftmBase, channel, hwFirstEdge);
            }
        }

        /* Software trigger is generated to change CnV registers */
        /* Before this please configure sync mechanism to use software trigger */
        if (softwareTrigger)
        {
            FTM_HAL_SetSoftwareTriggerCmd(ftmBase, true);
        }

        /* Store the PWM period in the state structure */
        state->ftmPeriod = ftmPeriod;
    }

    return retStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_UpdatePwmPeriod
 * Description   : This function will update the new period in the frequency or
 * in the counter value into mode register which modify the period of PWM signal
 * on the channel output.
 * - If the type of update in the duty cycle which is reused in FTM_DRV_UpdatePwmChannel
 * function to convert the newValue parameters representing frequency in Hz to
 * a period value to update the MOD register. The newValue parameter must be value
 * between 1U and maximum is the frequency of the FTM counter.
 * - If the type of update in ticks, this function will get value in counting to
 * the MOD register. The newValue parameter must be value between 1U and 0xFFFFU
 *
 * Implements : FTM_DRV_UpdatePwmPeriod_Activity
 *END**************************************************************************/
status_t FTM_DRV_UpdatePwmPeriod(uint32_t instance,
                                 ftm_pwm_update_option_t typeOfUpdate,
                                 uint32_t newValue,
                                 bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(newValue != 0U);
    uint32_t ftmPeriod = 0U;
    FTM_Type * ftmBase = g_ftmBase[instance];
    const ftm_state_t * state = ftmStatePtr[instance];
    status_t retStatus = STATUS_SUCCESS;

    /* Check the type of update for period in PWM mode */
    if (FTM_PWM_UPDATE_IN_TICKS == typeOfUpdate)
    {
        ftmPeriod = newValue;
    }
    else
    {
        if (newValue <= state->ftmSourceClockFrequency)
        {
            ftmPeriod = (uint32_t)FTM_DRV_ConvertFreqToPeriodTicks(instance, newValue);
        }
        else
        {
            retStatus = STATUS_ERROR;
        }
    }

    if (STATUS_SUCCESS == retStatus)
    {
        /* Check the ftmPeriod is invalid */
        DEV_ASSERT(ftmPeriod <= 0xFFFFU);
        /* Check the signal operation in which PWM mode */
        DEV_ASSERT((FTM_MODE_CEN_ALIGNED_PWM == state->ftmMode) || (FTM_MODE_EDGE_ALIGNED_PWM == state->ftmMode));
        if (FTM_MODE_CEN_ALIGNED_PWM == state->ftmMode)
        {
            ftmPeriod = (ftmPeriod >> 1U);
        }
        else
        {
            ftmPeriod = (ftmPeriod - 1U);
        }
        /* Set the new modulo value into MOD register */
        FTM_HAL_SetMod(ftmBase, (uint16_t)(ftmPeriod));
        /* Software trigger is generated to change MOD registers */
        if (softwareTrigger)
        {
            FTM_HAL_SetSoftwareTriggerCmd(ftmBase, true);
        }
    }

    return retStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_MaskOutputChannels
 * Description   : This function will mask the output of the channels and at match
 * events will be ignored by the masked channels.
 *
 * Implements : FTM_DRV_MaskOutputChannels_Activity
 *END**************************************************************************/
status_t FTM_DRV_MaskOutputChannels(uint32_t instance,
                                    uint32_t channelsMask,
                                    bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    FTM_HAL_SetOutmaskReg(ftmBase, channelsMask);
    if (softwareTrigger)
    {
        FTM_HAL_SetSoftwareTriggerCmd(ftmBase, true);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetInitialCounterValue
 * Description   : This function configure the initial counter value. The counter
 * will get this value after an overflow event.
 *
 * Implements : FTM_DRV_SetInitialCounterValue_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetInitialCounterValue(uint32_t instance,
                                        uint16_t counterValue,
                                        bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    FTM_HAL_SetCounterInitVal(ftmBase, counterValue);
    if (softwareTrigger)
    {
        FTM_HAL_SetSoftwareTriggerCmd(ftmBase, true);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetHalfCycleReloadPoint
 * Description   : This function configure the value of the counter which will
 * generates an reload point.
 *
 * Implements : FTM_DRV_SetHalfCycleReloadPoint_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetHalfCycleReloadPoint(uint32_t instance,
                                         uint16_t reloadPoint,
                                         bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    FTM_HAL_SetHalfCycleValue(ftmBase, reloadPoint);
    if (softwareTrigger)
    {
        FTM_HAL_SetSoftwareTriggerCmd(ftmBase, true);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetSoftwareOutputChannelValue
 * Description   : This function will force the output value of a channel to a specific value.
 * Before using this function it's mandatory to mask the match events using
 * FTM_DRV_MaskOutputChannels and to enable software output control using
 * FTM_DRV_SetSoftwareOutputChannelControl.
 *
 * Implements : FTM_DRV_SetSoftOutChnValue_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetSoftOutChnValue(uint32_t instance,
                                    uint8_t channelsValues,
                                    bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];
    FTM_HAL_SetAllChnSoftwareCtrlVal(ftmBase, channelsValues);
    if (softwareTrigger)
    {
        FTM_HAL_SetSoftwareTriggerCmd(ftmBase, true);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetSoftwareOutputChannelControl
 * Description   : This function will configure which output channel can be
 * software controlled.
 *
 * Implements : FTM_DRV_SetSoftwareOutputChannelControl_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetSoftwareOutputChannelControl(uint32_t instance,
                                                 uint8_t channelsMask,
                                                 bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];
    FTM_HAL_SetAllChnSoftwareCtrlCmd(ftmBase, channelsMask);
    if (softwareTrigger)
    {
        FTM_HAL_SetSoftwareTriggerCmd(ftmBase, true);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetInvertingControl
 * Description   : This function will configure if the second channel of a pair
 * will be inverted or not.
 *
 * Implements : FTM_DRV_SetInvertingControl_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetInvertingControl(uint32_t instance,
                                     uint8_t channelsPairMask,
                                     bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    FTM_HAL_SetInvctrlReg(ftmBase, channelsPairMask);
    if (softwareTrigger)
    {
        FTM_HAL_SetSoftwareTriggerCmd(ftmBase, true);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetModuloCounterValue
 * Description   : This function configure the maximum counter value.
 *
 * Implements : FTM_DRV_SetModuloCounterValue_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetModuloCounterValue(uint32_t instance,
                                       uint16_t counterValue,
                                       bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    FTM_HAL_SetMod(ftmBase, counterValue);
    if (softwareTrigger)
    {
        FTM_HAL_SetSoftwareTriggerCmd(ftmBase, true);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetSync
 * Description   : This function configure the synchronization for PWM register
 * (CnV, MOD, CINT, HCR, OUTMASK).If this function is used whit wrong parameters
 * it's possible to generate wrong waveform. Registers synchronization need to
 * be configured for PWM and output compare mode.
 *
 * Implements : FTM_DRV_SetSync_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetSync(uint32_t instance,
                         const ftm_pwm_sync_t * param)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(param != NULL);
    FTM_Type * ftmBase = g_ftmBase[instance];
    status_t retStatus = STATUS_SUCCESS;
    bool hardwareSync = param->hardwareSync0 || param->hardwareSync1 || param->hardwareSync2;

    /* Software and hardware triggers are not allowed in the same time */
    if ((param->softwareSync && hardwareSync) || (true != (param->softwareSync || hardwareSync)))
    {
        retStatus = STATUS_ERROR;
    }
    else if (param->softwareSync)
    {
        /* Configure sync for OUTMASK register */
        FTM_HAL_SetOutmaskSoftwareSyncModeCmd(ftmBase, true);
        /* Configure sync for INVCTRL register */
        FTM_HAL_SetInvctrlSoftwareSyncModeCmd(ftmBase, true);
        /* Configure sync for SWOCTRL register */
        FTM_HAL_SetSwoctrlSoftwareSyncModeCmd(ftmBase, true);
        /* Configure sync for MOD, HCR, CNTIN, and CnV registers */
        FTM_HAL_SetModCntinCvSoftwareSyncModeCmd(ftmBase, true);
        /* Configure synchronization method (waiting next loading point or now) */
        FTM_HAL_SetCounterSoftwareSyncModeCmd(ftmBase, param->syncPoint);
    }
    else
    {
        /* Configure sync for OUTMASK register */
        FTM_HAL_SetOutmaskHardwareSyncModeCmd(ftmBase, true);
        /* Configure sync for INVCTRL register */
        FTM_HAL_SetInvctrlHardwareSyncModeCmd(ftmBase, true);
        /* Configure sync for SWOCTRL register */
        FTM_HAL_SetSwoctrlHardwareSyncModeCmd(ftmBase, true);
        /* Configure sync for MOD, HCR, CNTIN, and CnV registers */
        FTM_HAL_SetModCntinCvHardwareSyncModeCmd(ftmBase, true);
        /* Configure synchronization method (waiting next loading point or now) */
        FTM_HAL_SetCounterHardwareSyncModeCmd(ftmBase, (bool)param->syncPoint);
    }

    if (STATUS_SUCCESS == retStatus)
    {
        /* Enhanced PWM sync is used */
        FTM_HAL_SetPwmSyncModeCmd(ftmBase, true);
        /* Configure trigger source for sync */
        FTM_HAL_SetHardwareSyncTriggerSrc(ftmBase, 0U, param->hardwareSync0);
        FTM_HAL_SetHardwareSyncTriggerSrc(ftmBase, 1U, param->hardwareSync1);
        FTM_HAL_SetHardwareSyncTriggerSrc(ftmBase, 2U, param->hardwareSync2);
        /* Configure loading points */
        FTM_HAL_SetMaxLoadingCmd(ftmBase, param->maxLoadingPoint);
        FTM_HAL_SetMinLoadingCmd(ftmBase, param->minLoadingPoint);
        /* Configure sync for OUTMASK register */
        FTM_HAL_SetOutmaskPwmSyncModeCmd(ftmBase, (bool)param->maskRegSync);
        /* Configure sync for INVCTRL register */
        FTM_HAL_SetInvctrlPwmSyncModeCmd(ftmBase, param->inverterSync);
        /* Configure sync for SWOCTRL register */
        FTM_HAL_SetSwoctrlPwmSyncModeCmd(ftmBase, param->outRegSync);
        /* Configure sync for MOD, HCR, CNTIN, and CnV registers */
        FTM_HAL_SetCntinPwmSyncModeCmd(ftmBase, param->initCounterSync);
        /* Configure if FTM clears TRIGj (j=0,1,2) when the hardware trigger j is detected. */
        FTM_HAL_SetHwTriggerSyncModeCmd(ftmBase, param->autoClearTrigger);
    }

    return retStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_InitOutputCompare
 * Description   : Configures the FTM to generate timed pulses
 * When the FTM counter matches the value of compareVal argument (this is
 * written into CnV register), the channel output is changed based on what is specified
 * in the compareMode argument.
 *
 * Implements    : FTM_DRV_InitOutputCompare_Activity
 *END**************************************************************************/
status_t FTM_DRV_InitOutputCompare(uint32_t instance,
                                   const ftm_output_cmp_param_t * param)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(param != NULL);
    FTM_Type * ftmBase = g_ftmBase[instance];
    uint8_t index = 0U;
    uint8_t hwChannel = 0U;
    uint8_t chnlPairNum = 0U;
    ftm_state_t * state = ftmStatePtr[instance];
    status_t retStatus = STATUS_SUCCESS;

    if ((NULL != state) && (FTM_MODE_NOT_INITIALIZED == state->ftmMode))
    {
        FTM_HAL_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
        FTM_HAL_SetCpwms(ftmBase, false);
        /* Clear the overflow flag */
        FTM_HAL_ClearTimerOverflow(ftmBase);
        FTM_HAL_SetCounterInitVal(ftmBase, 0U);
        FTM_HAL_SetMod(ftmBase, param->maxCountValue);
        FTM_HAL_SetCounter(ftmBase, 0U);
        FTM_HAL_SetQuadDecoderCmd(ftmBase, false);
        /* Use FTM as counter, disable all the channels */
        for (index = 0U; index < param->nNumOutputChannels; index++)
        {
            hwChannel = param->outputChannelConfig[index].hwChannelId;
            chnlPairNum =  (uint8_t)(hwChannel >> 1U);
            FTM_HAL_SetDualChnMofCombineCmd(ftmBase, chnlPairNum, false);
            FTM_HAL_SetDualChnCombineCmd(ftmBase, chnlPairNum, false);
            FTM_HAL_SetDualEdgeCaptureCmd(ftmBase, chnlPairNum, false);
            /* Set Channel Output mode */
            FTM_HAL_SetChnEdgeLevel(ftmBase, hwChannel, (uint8_t)(param->outputChannelConfig[index].chMode));
            /* Enter counter mode for all configured channels */
            FTM_HAL_SetChnMSnBAMode(ftmBase, hwChannel, 1U);
            /* Write initial count value for all channels */
            FTM_HAL_SetChnCountVal(ftmBase, hwChannel, param->outputChannelConfig[index].comparedValue);
            /* Enable channel output */
            FTM_HAL_EnablePwmChannelOutputs(ftmBase, hwChannel);
            /* Enable the generation a trigger on chip module */
            FTM_HAL_SetChnTriggerCmd(ftmBase, hwChannel, param->outputChannelConfig[index].enableExternalTrigger);
        }

        /* Set software trigger */
        FTM_HAL_SetSoftwareTriggerCmd(ftmBase, true);
        state->ftmMode = param->mode;
        /* Set clock source to start the counter */
        FTM_HAL_SetClockSource(ftmBase, state->ftmClockSource);
    }
    else
    {
        retStatus = STATUS_ERROR;
    }

    return retStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_DeinitOutputCompare
 * Description   : Disables compare match output control and clears FTM timer configuration
 *
 * Implements    : FTM_DRV_DeinitOutputCompare_Activity
 *END**************************************************************************/
status_t FTM_DRV_DeinitOutputCompare(uint32_t instance,
                                     const ftm_output_cmp_param_t * param)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(param != NULL);
    FTM_Type * ftmBase = g_ftmBase[instance];
    uint8_t index = 0U;
    uint8_t hwChannel = 0U;
    ftm_state_t * state = ftmStatePtr[instance];

    /* Stop the FTM counter */
    FTM_HAL_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
    /* Clear the overflow flag */
    FTM_HAL_ClearTimerOverflow(ftmBase);
    FTM_HAL_SetCounterInitVal(ftmBase, 0U);
    for (index = 0U; index < param->nNumOutputChannels; index++)
    {
        hwChannel = param->outputChannelConfig[index].hwChannelId;
        /* Disable Channel Output mode */
        FTM_HAL_SetChnEdgeLevel(ftmBase, hwChannel, (uint8_t)0U);
        /* Write initial count value for all channels to 0xFFFF */
        FTM_HAL_SetChnCountVal(ftmBase, hwChannel, 0U);
        /* Disable channel output */
        FTM_HAL_DisablePwmChannelOutputs(ftmBase, hwChannel);
    }

    /* Clear out the registers */
    FTM_HAL_SetMod(ftmBase, 0U);
    FTM_HAL_SetCounter(ftmBase, 0U);
    state->ftmMode = FTM_MODE_NOT_INITIALIZED;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_UpdateOutputCompareChannel
 * Description   : Sets the next compare match value on the given channel starting
 *                 from the current counter value.
 *
 * Implements    : FTM_DRV_UpdateOutputCompareChannel_Activity
 *END**************************************************************************/
status_t FTM_DRV_UpdateOutputCompareChannel(uint32_t instance,
                                            uint8_t channel,
                                            uint16_t nextComparematchValue,
                                            ftm_output_compare_update_t update,
                                            bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);
    uint16_t counterValue = FTM_HAL_GetCounter(g_ftmBase[instance]);
    uint16_t compareValue = 0U;
    uint16_t maxCounterValue;
    FTM_Type * ftmBase = g_ftmBase[instance];

    if (update == FTM_RELATIVE_VALUE)
    {
        maxCounterValue = FTM_HAL_GetMod(g_ftmBase[instance]);
        /* Configure channel compare register */
        if ((uint16_t)(counterValue + nextComparematchValue) > maxCounterValue)
        {
            compareValue = (uint16_t)(nextComparematchValue - (maxCounterValue - counterValue));
        }
        else
        {
            compareValue = (uint16_t)(counterValue + nextComparematchValue);
        }
    }
    else
    {
        compareValue = nextComparematchValue;
    }

    /* Set CnV value and use software trigger for sync */
    FTM_HAL_SetChnCountVal(g_ftmBase[instance], channel, compareValue);
    if (softwareTrigger)
    {
        FTM_HAL_SetSoftwareTriggerCmd(ftmBase, true);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_InitInputCapture
 * Description   : Configures Channel Input Capture for either getting time-stamps on edge detection
 * or on signal measurement . When the edge specified in the captureMode
 * argument occurs on the channel the FTM counter is captured into the CnV register.
 * The user will have to read the CnV register separately to get this value. The filter
 * function is disabled if the filterVal argument passed in is 0. The filter function
 * is available only on channels 0,1,2,3.
 *
 * Implements    : FTM_DRV_InitInputCapture_Activity
 *END**************************************************************************/
status_t FTM_DRV_InitInputCapture(uint32_t instance,
                                  const ftm_input_param_t * param)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(param != NULL);
    FTM_Type * ftmBase = g_ftmBase[instance];
    uint8_t chnlPairNum = 0U;
    uint8_t index = 0U;
    uint8_t hwChannel = 0U;
    ftm_signal_measurement_mode_t measurementType;
    ftm_state_t * state = ftmStatePtr[instance];
    status_t retStatus = STATUS_SUCCESS;

    if ((NULL != state) && (FTM_MODE_NOT_INITIALIZED == state->ftmMode))
    {
        FTM_HAL_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
        FTM_HAL_SetCounterInitVal(ftmBase, 0U);
        FTM_HAL_SetMod(ftmBase, param->nMaxCountValue);
        FTM_HAL_SetCpwms(ftmBase, false);
        /* Disable the combine mode */
        FTM_HAL_SetDualChnMofCombineCmd(ftmBase, chnlPairNum, false);
        FTM_HAL_SetDualChnCombineCmd(ftmBase, chnlPairNum, false);

        for (index = 0U; index < param->nNumChannels; index++)
        {
            hwChannel = param->inputChConfig[index].hwChannelId;
            chnlPairNum =  (uint8_t)(hwChannel >> 1U);
            /* Save in state structure user define handlers */
            state->channelsCallbacksParams[hwChannel] =  param->inputChConfig[index].channelsCallbacksParams;
            state->channelsCallbacks[hwChannel] = param->inputChConfig[index].channelsCallbacks;
            /* Enable filtering for input channels */
            if (hwChannel < CHAN4_IDX)
            {
                if (true == param->inputChConfig[index].filterEn)
                {
                    FTM_HAL_SetChnInputCaptureFilter(ftmBase, hwChannel, (uint8_t)param->inputChConfig[index].filterValue);
                }
                else
                {
                    FTM_HAL_SetChnInputCaptureFilter(ftmBase, hwChannel, 0U);
                }
            }

            if (FTM_EDGE_DETECT == param->inputChConfig[index].inputMode)
            {
                /* Disable the dual edge mode */
                FTM_HAL_SetDualEdgeCaptureCmd(ftmBase, chnlPairNum, false);
                /* Set input capture mode */
                FTM_HAL_SetChnMSnBAMode(ftmBase, hwChannel, 0U);
                /* Check if no edge is selected */
                DEV_ASSERT (FTM_NO_PIN_CONTROL != param->inputChConfig[index].edgeAlignement);
                /* Set the event which will generate the interrupt */
                FTM_HAL_SetChnEdgeLevel(ftmBase, hwChannel, (uint8_t)param->inputChConfig[index].edgeAlignement);
                /* Enable interrupt request for the current channel */
                FTM_HAL_EnableChnInt(ftmBase, hwChannel);
                INT_SYS_EnableIRQ(g_ftmIrqId[instance][hwChannel]);
            }
            else if (FTM_SIGNAL_MEASUREMENT == param->inputChConfig[index].inputMode)
            {
                /* Enable the dual edge mode */
                FTM_HAL_SetDualEdgeCaptureCmd(ftmBase, chnlPairNum, true);
                /* Enable dual edge input capture */
                FTM_HAL_SetDualChnDecapCmd(ftmBase, chnlPairNum, true);
                /* If continuous mode is set*/
                if (true == param->inputChConfig[index].continuousModeEn)
                {
                    /* Set MSnA and MSnB bit*/
                    FTM_HAL_SetChnMSnBAMode(ftmBase, hwChannel, 3U);
                }
                else
                {
                    /* Clear MSnA and Set MSnB bit*/
                    FTM_HAL_SetChnMSnBAMode(ftmBase, hwChannel, 2U);
                }

                measurementType = param->inputChConfig[index].measurementType;
                /* Check If want to measure a pulse width or period of the signal */
                if ((FTM_PERIOD_ON_MEASUREMENT == measurementType) || (FTM_RISING_EDGE_PERIOD_MEASUREMENT== measurementType))
                {
                    FTM_HAL_SetChnEdgeLevel(ftmBase, hwChannel, 1U);
                    if (FTM_PERIOD_ON_MEASUREMENT == measurementType)
                    {
                        /* Measure time between rising and falling edge - positive duty */
                        FTM_HAL_SetChnEdgeLevel(ftmBase, (uint8_t)(hwChannel + 1U), 2U);
                    }
                    else
                    {
                        /* If channel (n) is configured to capture falling edges (ELS(n)B:ELS(n)A = 0:1)
                         * then channel (n+1) also captures falling edges (ELS(n+1)B:ELS(n+1)A = 0:1) */
                        FTM_HAL_SetChnEdgeLevel(ftmBase, (uint8_t)(hwChannel + 1U), 1U);
                    }
                }
                else if ((FTM_PERIOD_OFF_MEASUREMENT == measurementType) || (FTM_FALLING_EDGE_PERIOD_MEASUREMENT == measurementType))
                {
                    FTM_HAL_SetChnEdgeLevel(ftmBase, hwChannel, 2U);
                    if (FTM_PERIOD_OFF_MEASUREMENT == measurementType)
                    {
                        /* Measure time between falling and rising edge - negative duty */
                        FTM_HAL_SetChnEdgeLevel(ftmBase, (uint8_t)(hwChannel + 1U), 1U);
                    }
                    else
                    {
                        /* If channel (n) is configured to capture rising edges (ELS(n)B:ELS(n)A = 1:0) than
                         * channel (n+1) is setup to capture also raising edges (ELS(n+1)B:ELS(n+1)A = 1:0) */
                        FTM_HAL_SetChnEdgeLevel(ftmBase, (uint8_t)(hwChannel + 1U), 2U);
                    }
                }
                else
                {
                    retStatus = STATUS_ERROR;
                    break;
                }

                /* Enable the interrupt request for the channel which will indicate that the measurement is done. */
                FTM_HAL_EnableChnInt(ftmBase, (uint8_t)(hwChannel + 1U));
                INT_SYS_EnableIRQ(g_ftmIrqId[instance][hwChannel]);
            }
            else
            {
                /* Do nothing */
            }
        }

        if (STATUS_SUCCESS == retStatus)
        {
            state->ftmMode = FTM_MODE_INPUT_CAPTURE;
            /* Set clock source to start the counter */
            FTM_HAL_SetClockSource(ftmBase, state->ftmClockSource);
        }
    }
    else
    {
        retStatus = STATUS_ERROR;
    }

    return retStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_DeinitInputCapture
 * Description   : Disables Channel Input Capture
 *
 * Implements    : FTM_DRV_DeinitInputCapture_Activity
 *END**************************************************************************/
status_t FTM_DRV_DeinitInputCapture(uint32_t instance,
                                    const ftm_input_param_t * param)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(param != NULL);
    FTM_Type * ftmBase = g_ftmBase[instance];
    uint8_t chnlPairNum = 0U;
    uint8_t index = 0U;
    uint8_t hwChannel = 0U;
    ftm_state_t * state = ftmStatePtr[instance];

    /* FTM counter is disabled */
    FTM_HAL_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
    FTM_HAL_SetCounterInitVal(ftmBase, 0U);
    FTM_HAL_SetMod(ftmBase, 0xFFFFU);
    FTM_HAL_SetCpwms(ftmBase, false);
    for (index = 0U; index < param->nNumChannels; index++)
    {
        hwChannel = param->inputChConfig[index].hwChannelId;
        chnlPairNum =  (uint8_t)(hwChannel >> 1U);
        /* Disable filtering for input channels */
        if (hwChannel < CHAN4_IDX)
        {
            FTM_HAL_SetChnInputCaptureFilter(ftmBase, hwChannel, 0U);
        }

        FTM_HAL_SetDualChnCombineCmd(ftmBase, chnlPairNum, false);
        FTM_HAL_SetDualEdgeCaptureCmd(ftmBase, chnlPairNum, false);
        FTM_HAL_SetChnEdgeLevel(ftmBase, hwChannel, (uint8_t)0U);
        FTM_HAL_DisableChnInt(ftmBase, hwChannel);
    }

    /* Clear Callbacks function from the state structure */
    for (index = 0U; index < FEATURE_FTM_CHANNEL_COUNT; index++)
    {
        state->channelsCallbacksParams[index] =  NULL;
        state->channelsCallbacks[index] = NULL;
    }

    state->ftmMode = FTM_MODE_NOT_INITIALIZED;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_GetInputCaptureMeasurement
 * Description   : This function is used to calculate the measurement and/or time stamps values
 * which are read from the C(n, n+1)V registers and stored to the static buffers.
 *
 * Implements    : FTM_DRV_GetInputCaptureMeasurement_Activity
 *END**************************************************************************/
uint16_t FTM_DRV_GetInputCaptureMeasurement(uint32_t instance,
                                            uint8_t channel)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);
    const ftm_state_t * state = ftmStatePtr[instance];

    return state->measurementResults[channel];
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_StartNewSignalMeasurement
 * Description   : This function starts new Signal Measurements on a dual input compare channel
 * that is configured as single-shot measurement.
 *
 * Implements    : FTM_DRV_StartNewSignalMeasurement_Activity
 *END**************************************************************************/
status_t FTM_DRV_StartNewSignalMeasurement(uint32_t instance,
                                           uint8_t channel)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);
    /* Clear CH(n)F and CH(n+1)F  flags and Set DECAP bit */
    FTM_Type * ftmBase = g_ftmBase[instance];
    uint8_t chnlPairNum = (uint8_t)(channel >> 1U);

    /* Get channel mode */
    if (FTM_FEATURE_INPUT_CAPTURE_SINGLE_SHOT == FTM_HAL_GetChnMode(ftmBase, channel))
    {
        if (FTM_HAL_GetDualChnCombineCmd(ftmBase, chnlPairNum))
        {
            /* Clear event flags for channel n and n + 1 */
            FTM_HAL_ClearChnEventFlag(ftmBase, (uint8_t)(channel + 1U));
            FTM_HAL_ClearChnEventFlag(ftmBase, channel);
            /* Set DECAP bit to start measurement */
            FTM_HAL_SetDualChnDecapCmd(ftmBase, chnlPairNum, true);
        }
    }
    else
    {
        /* Nothing to do */
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_QuadDecodeStart
 * Description   : Configures the parameters needed and activates quadrature
 * decode mode.
 *
 * Implements    : FTM_DRV_QuadDecodeStart_Activity
 *END**************************************************************************/
status_t FTM_DRV_QuadDecodeStart(uint32_t instance,
                                 const ftm_quad_decode_config_t * config)
{
    DEV_ASSERT((instance == FTM1_IDX) || (instance == FTM2_IDX));
    DEV_ASSERT(config != NULL);
    FTM_Type * ftmBase = g_ftmBase[instance];
    ftm_state_t * state = ftmStatePtr[instance];
    status_t retStatus = STATUS_SUCCESS;

    if ((NULL != state) && (FTM_MODE_NOT_INITIALIZED == state->ftmMode))
    {
        /* Disable Quadrature Decoder */
        FTM_HAL_SetQuadDecoderCmd(ftmBase, false);
        FTM_HAL_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
        /* Configure Quadrature Decoder */
        /* Set decoder mode Speed and direction or Phase A and Phase B encoding */
        FTM_HAL_SetQuadMode(ftmBase, config->mode);
        /* Set filter state for Phase A (enable/disable) */
        FTM_HAL_SetQuadPhaseAFilterCmd(ftmBase, config->phaseAConfig.phaseInputFilter);
        /* Set Phase A filter value if phase filter is enabled */
        if (config->phaseAConfig.phaseInputFilter)
        {
            FTM_HAL_SetChnInputCaptureFilter(ftmBase, CHAN0_IDX, config->phaseAConfig.phaseFilterVal);
        }

        /* Set filter state for Phase B (enable/disable) */
        FTM_HAL_SetQuadPhaseBFilterCmd(ftmBase, config->phaseBConfig.phaseInputFilter);
        /* Set Phase B filter value if phase filter is enabled */
        if (config->phaseBConfig.phaseInputFilter)
        {
            FTM_HAL_SetChnInputCaptureFilter(ftmBase, CHAN1_IDX, config->phaseBConfig.phaseFilterVal);
        }

        /* Set polarity for Phase A and Phase B */
        FTM_HAL_SetQuadPhaseAPolarity(ftmBase, config->phaseAConfig.phasePolarity);
        FTM_HAL_SetQuadPhaseBPolarity(ftmBase, config->phaseBConfig.phasePolarity);
        /* Configure counter (initial value and maximum value) */
        FTM_HAL_SetCounterInitVal(ftmBase, config->initialVal);
        FTM_HAL_SetMod(ftmBase, config->maxVal);
        FTM_HAL_SetCounter(ftmBase, config->initialVal);
        /* Enable Quadrature Decoder */
        FTM_HAL_SetQuadDecoderCmd(ftmBase, true);
        state->ftmMode = FTM_MODE_QUADRATURE_DECODER;
    }
    else
    {
        retStatus = STATUS_ERROR;
    }

    return retStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_QuadDecodeStop
 * Description   : De-activates quadrature decoder mode.
 *
 * Implements    : FTM_DRV_QuadDecodeStop_Activity
 *END**************************************************************************/
status_t FTM_DRV_QuadDecodeStop(uint32_t instance)
{
    DEV_ASSERT((instance == FTM1_IDX) || (instance == FTM2_IDX));
    FTM_Type * ftmBase = g_ftmBase[instance];
    ftm_state_t * state = ftmStatePtr[instance];

    /* Disable Quadrature decoder */
    FTM_HAL_SetQuadDecoderCmd(ftmBase, false);
    state->ftmMode = FTM_MODE_NOT_INITIALIZED;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_QuadGetState
 * Description   : Return the current quadrature decoder state
 * (counter value, overflow flag and overflow direction)
 *
 * Implements    : FTM_DRV_QuadGetState_Activity
 *END**************************************************************************/
ftm_quad_decoder_state_t FTM_DRV_QuadGetState(uint32_t instance)
{
    DEV_ASSERT((instance == FTM1_IDX) || (instance == FTM2_IDX));
    FTM_Type const * ftmBase = g_ftmBase[instance];
    ftm_quad_decoder_state_t state;

    state.counterDirection = FTM_HAL_GetQuadDir(ftmBase);
    state.overflowDirection = FTM_HAL_GetQuadTimerOverflowDir(ftmBase);
    state.overflowFlag = FTM_HAL_HasTimerOverflowed(ftmBase);
    state.counter = FTM_HAL_GetCounter(ftmBase);

    return state;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_GetFrequency
 * Description   : Retrieves the frequency of the clock source feeding the FTM counter.
 * Function will return a 0 if no clock source is selected and the FTM counter is disabled.
 * The returned value is clock sources for the FTM counter.
 *
 * Implements    : FTM_DRV_GetFrequency_Activity
 *END**************************************************************************/
uint32_t FTM_DRV_GetFrequency(uint32_t instance)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    PCC_Type const * pccBase = PCC_BASE_PTRS;
    FTM_Type const * ftmBase = g_ftmBase[instance];
    clock_names_t ftmClkName;
    peripheral_clock_source_t clockSelect;
    uint8_t clkPs;
    uint32_t frequency = 0U;
    const ftm_state_t * state = ftmStatePtr[instance];
    clkPs = (uint8_t)(1U << FTM_HAL_GetClockPs(ftmBase));

    switch (state->ftmClockSource)
    {
        case FTM_CLOCK_SOURCE_EXTERNALCLK:
            clockSelect = PCC_HAL_GetClockSourceSel(pccBase, g_ftmExtClockSel[instance][1]);
            if (CLK_SRC_OFF == clockSelect)
            {
                ftmClkName = g_ftmExtClockSel[instance][0];
            }
            else
            {
                ftmClkName = g_ftmExtClockSel[instance][1];
            }

            /* Get the clock frequency value */
            (void)CLOCK_SYS_GetFreq(ftmClkName, &frequency);
            break;
        case FTM_CLOCK_SOURCE_FIXEDCLK:
            /* Get the clock frequency value */
            (void)CLOCK_SYS_GetFreq(SIM_RTCCLK_CLOCK, &frequency);
            break;
        case FTM_CLOCK_SOURCE_SYSTEMCLK:
            /* Get the clock frequency value */
            (void)CLOCK_SYS_GetFreq(CORE_CLOCK, &frequency);
            break;
        default:
            /* Nothing to do */
            break;
    }

    return (uint32_t)(frequency / clkPs);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_ConvertFreqToPeriodTicks
 * Description   : This function converts the input parameters representing
 * frequency in Hz to a period value in ticks needed by the hardware timer.
 *
 * Implements    : FTM_DRV_ConvertFreqToPeriodTicks_Activity
 *END**************************************************************************/
uint16_t FTM_DRV_ConvertFreqToPeriodTicks(uint32_t instance,
                                          uint32_t freqencyHz)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(freqencyHz != 0U);
    uint32_t uFTMhz;
    const ftm_state_t * state = ftmStatePtr[instance];
    uFTMhz = state->ftmSourceClockFrequency;

    return (uint16_t)(uFTMhz / freqencyHz);
}

/* Implementation of FTM0_Ch0_Ch1_IRQHandler master handler named in startup code. */
void FTM0_Ch0_Ch1_IRQHandler(void)
{
    FTM_DRV_IrqHandler(0U, 0U);
}

/* Implementation of FTM0_Ch2_Ch3_IRQHandler master handler named in startup code. */
void FTM0_Ch2_Ch3_IRQHandler(void)
{
    FTM_DRV_IrqHandler(0U, 1U);
}

/* Implementation of FTM0_Ch4_Ch5_IRQHandler master handler named in startup code. */
void FTM0_Ch4_Ch5_IRQHandler(void)
{
    FTM_DRV_IrqHandler(0U, 2U);
}

/* Implementation of FTM0_Ch6_Ch7_IRQHandler master handler named in startup code. */
void FTM0_Ch6_Ch7_IRQHandler(void)
{
    FTM_DRV_IrqHandler(0U, 3U);
}

/* Implementation of FTM1_Ch0_Ch1_IRQHandler master handler named in startup code. */
void FTM1_Ch0_Ch1_IRQHandler(void)
{
    FTM_DRV_IrqHandler(1U, 0U);
}

/* Implementation of FTM1_Ch2_Ch3_IRQHandler master handler named in startup code. */
void FTM1_Ch2_Ch3_IRQHandler(void)
{
    FTM_DRV_IrqHandler(1U, 1U);
}

/* Implementation of FTM1_Ch4_Ch5_IRQHandler master handler named in startup code. */
void FTM1_Ch4_Ch5_IRQHandler(void)
{
    FTM_DRV_IrqHandler(1U, 2U);
}

/* Implementation of FTM1_Ch6_Ch7_IRQHandler master handler named in startup code. */
void FTM1_Ch6_Ch7_IRQHandler(void)
{
    FTM_DRV_IrqHandler(1U, 3U);
}

/* Implementation of FTM2_Ch0_Ch1_IRQHandler master handler named in startup code. */
void FTM2_Ch0_Ch1_IRQHandler(void)
{
    FTM_DRV_IrqHandler(2U, 0U);
}

/* Implementation of FTM2_Ch2_Ch3_IRQHandler master handler named in startup code. */
void FTM2_Ch2_Ch3_IRQHandler(void)
{
    FTM_DRV_IrqHandler(2U, 1U);
}

/* Implementation of FTM2_Ch4_Ch5_IRQHandler master handler named in startup code. */
void FTM2_Ch4_Ch5_IRQHandler(void)
{
    FTM_DRV_IrqHandler(2U, 2U);
}

/* Implementation of FTM2_Ch6_Ch7_IRQHandler master handler named in startup code. */
void FTM2_Ch6_Ch7_IRQHandler(void)
{
    FTM_DRV_IrqHandler(2U, 3U);
}

/* Implementation of FTM3_Ch0_Ch1_IRQHandler master handler named in startup code. */
void FTM3_Ch0_Ch1_IRQHandler(void)
{
    FTM_DRV_IrqHandler(3U, 0U);
}

/* Implementation of FTM3_Ch2_Ch3_IRQHandler master handler named in startup code. */
void FTM3_Ch2_Ch3_IRQHandler(void)
{
    FTM_DRV_IrqHandler(3U, 1U);
}

/* Implementation of FTM3_Ch4_Ch5_IRQHandler master handler named in startup code. */
void FTM3_Ch4_Ch5_IRQHandler(void)
{
    FTM_DRV_IrqHandler(3U, 2U);
}

/* Implementation of FTM3_Ch6_Ch7_IRQHandler master handler named in startup code. */
void FTM3_Ch6_Ch7_IRQHandler(void)
{
    FTM_DRV_IrqHandler(3U, 3U);
}

static void FTM_DRV_IrqHandler(uint32_t instance,
                               uint8_t channelPair)
{
    const ftm_state_t * state = ftmStatePtr[instance];
    switch (state->ftmMode)
    {
        case FTM_MODE_INPUT_CAPTURE:
            FTM_DRV_InputCaptureHandler(instance, channelPair);
            break;
        default:
            /* Nothing to do */
            break;
    }
}

static void FTM_DRV_InputCaptureHandler(uint32_t instance,
                                        uint8_t channelPair)
{
    ftm_state_t * state = ftmStatePtr[instance];
    FTM_Type * ftmBase = g_ftmBase[instance];

    /* Verify the mode for current pair of channels */
    if (FTM_HAL_GetDualEdgeCaptureBit(ftmBase, channelPair))
    {
        /* Dual edge input capture case */
        uint16_t first_event_time = FTM_HAL_GetChnCountVal(ftmBase, (uint8_t)(channelPair << 1U));
        uint16_t second_event_time = FTM_HAL_GetChnCountVal(ftmBase, (uint8_t)((channelPair << 1U) + 1U));
        if (second_event_time < first_event_time)
        {
            /* Measurement when overflow occurred */
            state->measurementResults[channelPair << 1U] = (uint16_t)(second_event_time + (FTM_HAL_GetMod(ftmBase) - first_event_time));
        }
        else
        {
            /* Measurement when overflow doesn't occurred */
            state->measurementResults[channelPair << 1U] = (uint16_t)(second_event_time - first_event_time);
        }

        /* Clear flags for channels n and n+1 */
        FTM_HAL_ClearChnEventFlag(ftmBase, (uint8_t)(channelPair << 1U));
        FTM_HAL_ClearChnEventFlag(ftmBase, (uint8_t)((channelPair << 1U) + 1U));
        /* If the callback is define use it */
        if ((state->channelsCallbacks[(channelPair << 1U)]) != NULL)
        {
            state->channelsCallbacks[(channelPair << 1U)](state->channelsCallbacksParams[channelPair << 1U]);
        }
    }
    else
    {
        /* To get the channel interrupt source the both channels flag must be checked */
        if (FTM_HAL_HasChnEventOccurred(ftmBase, (uint8_t)(channelPair << 1U)))
        {
            /* Get the time stamp of the event */
            state->measurementResults[channelPair << 1U] = FTM_HAL_GetChnCountVal(ftmBase, (uint8_t)(channelPair << 1U));
            /* Clear the flag for C(n) channel */
            FTM_HAL_ClearChnEventFlag(ftmBase, (uint8_t)(channelPair << 1U));
            /* If the callback is define use it */
            if ((state->channelsCallbacks[channelPair << 1U]) != NULL)
            {
                state->channelsCallbacks[channelPair << 1U](state->channelsCallbacksParams[channelPair << 1U]);
            }
        }
        else
        {
            /* Get the time stamp of the event */
            state->measurementResults[(channelPair << 1U) + 1U] = FTM_HAL_GetChnCountVal(ftmBase, (uint8_t)((channelPair << 1U) + 1U));
            /* Clear the flag for C(n+1) channel */
            FTM_HAL_ClearChnEventFlag(ftmBase, (uint8_t)((channelPair << 1U) + 1U));
            /* If the callback is define use it */
            if ((state->channelsCallbacks[(channelPair << 1U) + 1U]) != NULL)
            {
                state->channelsCallbacks[(channelPair << 1U) + 1U](state->channelsCallbacksParams[(channelPair << 1U) + 1U]);
            }
        }
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
