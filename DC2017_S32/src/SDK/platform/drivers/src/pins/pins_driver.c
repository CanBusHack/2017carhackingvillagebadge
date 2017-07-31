/*
 * Copyright (c) 2014 - 2015, Freescale Semiconductor, Inc.
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

#include "pins_driver.h"
#include "clock_manager.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, A conversion should not be performed
 * between a pointer to object and an integer type.
 * The cast is required to initialize a pointer with an unsigned int define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, A cast shall not be performed
 * between pointer to void and an arithmetic type.
 * The cast is required to initialize a pointer with an unsigned int define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_Init
 * Description   : This function configures the pins with the options provided
 * in the provided structure.
 *
 * Implements    : PINS_DRV_Init_Activity
 *END**************************************************************************/
status_t PINS_DRV_Init(const uint32_t pin_count, const pin_settings_config_t config[])
{
    uint32_t i;
#if (defined (DEV_ERROR_DETECT) || defined (CUSTOM_DEVASSERT))
    bool porta_checked = false;
    bool portb_checked = false;
    bool portc_checked = false;
    bool portd_checked = false;
    bool porte_checked = false;
#endif

    for (i = 0U; i < pin_count; i++) {
    	/*Check if the clock is enabled for ports*/
#if (defined (DEV_ERROR_DETECT) || defined (CUSTOM_DEVASSERT))
    	if (config[i].base == PORTA)
    	{
    		if (porta_checked == false)
    		{
    			porta_checked  = true;
        		DEV_ASSERT(CLOCK_SYS_GetFreq(PCC_PORTA_CLOCK, NULL) == STATUS_SUCCESS);
    		}
    	}
    	else if (config[i].base == PORTB)
    	{
    		if (portb_checked == false)
    		{
    			portb_checked  = true;
    			DEV_ASSERT(CLOCK_SYS_GetFreq(PCC_PORTB_CLOCK, NULL) == STATUS_SUCCESS);
    		}
    	}
    	else if (config[i].base == PORTC)
    	{
    		if(portc_checked == false)
    		{
    			portc_checked  = true;
    			DEV_ASSERT(CLOCK_SYS_GetFreq(PCC_PORTC_CLOCK, NULL) == STATUS_SUCCESS);
    		}
    	}
    	else if (config[i].base == PORTD)
    	{
    		if (portd_checked == false)
    		{
    			portd_checked  = true;
    			DEV_ASSERT(CLOCK_SYS_GetFreq(PCC_PORTD_CLOCK, NULL) == STATUS_SUCCESS);
    		}

    	}
    	else if (config[i].base == PORTE)
    	{
    		if (porte_checked == false)
    		{
    			porte_checked  = true;
    			DEV_ASSERT(CLOCK_SYS_GetFreq(PCC_PORTE_CLOCK, NULL) == STATUS_SUCCESS);
    		}
    	}
    	else
    	{
    		/* not a port */
    		DEV_ASSERT(false);
    	}
#endif

#if FEATURE_PORT_HAS_PULL_SELECTION
        PORT_HAL_SetPullSel(          config[i].base, config[i].pinPortIdx, config[i].pullConfig);
#endif
#if FEATURE_PORT_HAS_SLEW_RATE
        PORT_HAL_SetSlewRateMode(     config[i].base, config[i].pinPortIdx, config[i].rateSelect);
#endif
#if FEATURE_PORT_HAS_PASSIVE_FILTER
        PORT_HAL_SetPassiveFilterMode(config[i].base, config[i].pinPortIdx, config[i].passiveFilter);
#endif
#if FEATURE_PORT_HAS_OPEN_DRAIN
        PORT_HAL_SetOpenDrainMode(    config[i].base, config[i].pinPortIdx, config[i].openDrain);
#endif
#if FEATURE_PORT_HAS_DRIVE_STRENGTH
        PORT_HAL_SetDriveStrengthMode(config[i].base, config[i].pinPortIdx, config[i].driveSelect);
#endif
        PORT_HAL_SetMuxModeSel(       config[i].base, config[i].pinPortIdx, config[i].mux);
#if FEATURE_PORT_HAS_PIN_CONTROL_LOCK
        PORT_HAL_SetPinCtrlLockMode(  config[i].base, config[i].pinPortIdx, config[i].pinLock);
#endif
        PORT_HAL_SetPinIntSel(        config[i].base, config[i].pinPortIdx, config[i].intConfig);
        if(config[i].clearIntFlag){
            PORT_HAL_ClearPinIntFlagCmd(  config[i].base, config[i].pinPortIdx);
        }
        if (PORT_MUX_AS_GPIO == config[i].mux)
        {
            switch(config[i].direction)
            {
                case GPIO_INPUT_DIRECTION:
                    GPIO_HAL_SetPinDirection(config[i].gpioBase, config[i].pinPortIdx, 0U);
                    break;
                case GPIO_OUTPUT_DIRECTION:
                    GPIO_HAL_SetPinDirection(config[i].gpioBase, config[i].pinPortIdx, 1U);
                    break;
                case GPIO_UNSPECIFIED_DIRECTION:
                    /* pass-through */
                default:
                    /* nothing to configure */
                	DEV_ASSERT(false);
                    break;
            }
        }
    }
    return STATUS_SUCCESS;
}


/******************************************************************************
 * EOF
 *****************************************************************************/
