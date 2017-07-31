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

/*!
 * @file pcc_hal.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application or driver code.
 *
 */

#include "pcc_hal.h"
#include <stddef.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*! @brief Clock name mappings
 *         Constant array storing the mappings between clock names and peripheral clock control indexes.
 *         If there is no peripheral clock control index for a clock name, then the corresponding value is
 *         PCC_INVALID_INDEX.
 */
const uint16_t clockNameMappings[] = PCC_CLOCK_NAME_MAPPINGS;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION*********************************************************************
 *
 * Function Name : PCC_HAL_SetPeripheralClockConfig
 * Description   : This function sets the peripheral clock configuration
 *
 * Implements PCC_HAL_SetPeripheralClockConfig_Activity
 *END*************************************************************************/
void PCC_HAL_SetPeripheralClockConfig(PCC_Type* const base,
                                                const pcc_config_t* const config)
{
    uint32_t i, clkGate;
    const peripheral_clock_config_t *peripheral_clock_config;

    if ((config != NULL) && (config->peripheralClocks != NULL))
    {
        for (i = 0U; i < config->count; i++) {

            peripheral_clock_config = &config->peripheralClocks[i];

            /* Disable the peripheral clock */
            base->PCCn[clockNameMappings[peripheral_clock_config->clockName]] &= (uint32_t)(~(PCC_PCCn_CGC_MASK));

            /* Clock gate value */
            clkGate = (peripheral_clock_config->clkGate == true) ? 1UL : 0UL;

            /* Configure the peripheral clock source, the fractional clock divider and the clock gate */
            base->PCCn[clockNameMappings[peripheral_clock_config->clockName]] = PCC_PCCn_PCS(peripheral_clock_config->clkSrc)   |
                                          PCC_PCCn_FRAC(peripheral_clock_config->frac)    |
                                          PCC_PCCn_PCD(peripheral_clock_config->divider)  |
                                          PCC_PCCn_CGC(clkGate);

        }
    }
}



/*******************************************************************************
 * EOF
 ******************************************************************************/
