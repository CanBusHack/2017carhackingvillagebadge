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
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application or driver code.
 *
 */

/*! @file pmc_hal.c */

#include "pmc_hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : PMC_HAL_SetLowVoltIntCmd
 * Description   : Enable/Disable low voltage related interrupts
 * This function  enables  the the low voltage detection, warning, etc. interrupts.
 * When enabled, if the LVDF (Low Voltage Detect Flag) is set, a hardware
 * interrupt occurs.
 *
 * Implements PMC_HAL_SetLowVoltIntCmd_Activity
 *END**************************************************************************/
void PMC_HAL_SetLowVoltIntCmd(PMC_Type* const baseAddr, const pmc_int_select_t intSelect, const bool enable)
{
    uint8_t regValue;
    uint8_t enableValue = (uint8_t)(enable ? 1U : 0U);

    switch (intSelect)
    {
        case PMC_INT_LOW_VOLT_DETECT:    /* Low Voltage Detect */
            regValue = baseAddr->LVDSC1;
            regValue &= (uint8_t)(~(PMC_LVDSC1_LVDIE_MASK));
            regValue |= (uint8_t)PMC_LVDSC1_LVDIE(enableValue);
            baseAddr->LVDSC1 = regValue;
            break;
        case PMC_INT_LOW_VOLT_WARN:      /* Low Voltage Warning */
            regValue = baseAddr->LVDSC2;
            regValue &= (uint8_t)(~(PMC_LVDSC2_LVWIE_MASK));
            regValue |= (uint8_t)PMC_LVDSC2_LVWIE(enableValue);
            baseAddr->LVDSC2 = regValue;
            break;
        default:
            /* invalid command */
            break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PMC_HAL_SetLowVoltIntAckCmd
 * Description   : Acknowledges low voltage related interrupts
 * This function acknowledges the low voltage detection, warning,
 * etc. interrupts
 *
 * Implements PMC_HAL_SetLowVoltIntAckCmd_Activity
 *END**************************************************************************/
void PMC_HAL_SetLowVoltIntAckCmd(PMC_Type* const baseAddr, const pmc_int_select_t intSelect)
{
    uint8_t regValue;

    switch (intSelect)
    {
        case PMC_INT_LOW_VOLT_DETECT:    /* Low Voltage Detect */
            regValue = (uint8_t)baseAddr->LVDSC1;
            regValue &= (uint8_t)(~(PMC_LVDSC1_LVDACK_MASK));
            regValue |= (uint8_t)PMC_LVDSC1_LVDACK(1U);
            baseAddr->LVDSC1 = (uint8_t)regValue;
            break;
        case PMC_INT_LOW_VOLT_WARN:      /* Low Voltage Warning */
            regValue = (uint8_t)baseAddr->LVDSC2;
            regValue &= (uint8_t)(~(PMC_LVDSC2_LVWACK_MASK));
            regValue |= (uint8_t)PMC_LVDSC2_LVWACK(1U);
            baseAddr->LVDSC2 = (uint8_t)regValue;
            break;
        default:
            /* invalid command */
            break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PMC_HAL_GetLowVoltIntFlag
 * Description   : Returns the flag for low voltage related interrupts
 * This function returns the flag for the low voltage detection, warning,
 * etc. interrupts
 *
 * Implements PMC_HAL_GetLowVoltIntFlag_Activity
 *END**************************************************************************/
bool PMC_HAL_GetLowVoltIntFlag(const PMC_Type* const baseAddr, const pmc_int_select_t intSelect)
{
    bool flag;
    uint8_t regValue;

    switch (intSelect)
    {
        case PMC_INT_LOW_VOLT_DETECT:    /* Low Voltage Detect */
            regValue = baseAddr->LVDSC1;
            regValue = (uint8_t)((regValue & PMC_LVDSC1_LVDF_MASK) >> PMC_LVDSC1_LVDF_SHIFT);
            break;
        case PMC_INT_LOW_VOLT_WARN:      /* Low Voltage Warning */
            regValue = baseAddr->LVDSC2;
            regValue = (uint8_t)((regValue & PMC_LVDSC2_LVWF_MASK) >> PMC_LVDSC2_LVWF_SHIFT);
            break;
        default:
            regValue = 0U;
            break;
    }
    flag = (regValue == 0U) ? false : true;
    return flag;
}






/*******************************************************************************
 * EOF
 ******************************************************************************/

