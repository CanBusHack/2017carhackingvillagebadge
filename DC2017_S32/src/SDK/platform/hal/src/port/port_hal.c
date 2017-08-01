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

#include "port_hal.h"

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 */

#if FEATURE_SOC_PORT_COUNT > 0

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : PORT_HAL_SetLowGlobalPinCtrlCmd
 * Description   : Configure low half of pin control register for the same settings,
 *                 this function operates pin 0 -15 of one specific port.
 *
 * Implements    : PORT_HAL_SetLowGlobalPinCtrlCmd_Activity
 *END**************************************************************************/
void PORT_HAL_SetLowGlobalPinCtrlCmd(PORT_Type* const base, const uint16_t lowPinSelect, const uint16_t config)
{
    uint32_t combine = lowPinSelect;
    combine = (combine << 16U) + config;
    base->GPCLR = combine;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PORT_HAL_SetHighGlobalPinCtrlCmd
 * Description   : Configure high half of pin control register for the same
 *                 settings, this function operates pin 16 -31 of one specific port.
 *
 * Implements    : PORT_HAL_SetHighGlobalPinCtrlCmd_Activity
 *END**************************************************************************/
void PORT_HAL_SetHighGlobalPinCtrlCmd(PORT_Type* const base, const uint16_t highPinSelect, const uint16_t config)
{
    uint32_t combine = highPinSelect;
    combine = (combine << 16U) + config;
    base->GPCHR = combine;
}

#endif /* FEATURE_SOC_PORT_COUNT */
/*******************************************************************************
 * EOF
 ******************************************************************************/

