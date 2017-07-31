/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
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
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application or driver code.
 */

 /*! @file rcm_hal.c */


#include "rcm_hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : RCM_HAL_GetVersion
 * Description   : Get the version of the RCM module
 * This function will get the version of the RCM module.
 * function for more details.
 *
 * Implements RCM_HAL_GetVersion_Activity
 *END**************************************************************************/
void  RCM_HAL_GetVersion(const RCM_Type* const baseAddr, rcm_version_info_t* const versionInfo)
{
    uint32_t regValue;

    regValue = baseAddr->VERID;
    regValue = (regValue & RCM_VERID_MAJOR_MASK) >> RCM_VERID_MAJOR_SHIFT;
    versionInfo->majorNumber = regValue;

    regValue = baseAddr->VERID;
    regValue = (regValue & RCM_VERID_MINOR_MASK) >> RCM_VERID_MINOR_SHIFT;
    versionInfo->minorNumber = regValue;

    regValue = baseAddr->VERID;
    regValue = (regValue & RCM_VERID_FEATURE_MASK) >> RCM_VERID_FEATURE_SHIFT;
    versionInfo->featureNumber = regValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : RCM_HAL_GetSrcIndicationFeatureAvailability
 * Description   : Checks the existence of the status indication feature for a reset source
 *
 * This function will check the existence of the status indication feature for a specified source.
 *
 * Implements RCM_HAL_GetSrcIndicationFeatureAvailability_Activity
 *END**************************************************************************/
bool RCM_HAL_GetSrcIndicationFeatureAvailability(const RCM_Type* const baseAddr, const rcm_source_names_t srcName)
{
    uint32_t regValue = (uint32_t)baseAddr->PARAM;

    DEV_ASSERT(srcName < RCM_SRC_NAME_MAX);

    switch (srcName)
    {
        case RCM_WAKEUP:                      /* wakeup */
            regValue = (regValue & RCM_PARAM_EWAKEUP_MASK) >> RCM_PARAM_EWAKEUP_SHIFT;
            break;
        case RCM_LOW_VOLT_DETECT:              /* low voltage detect reset */
            regValue = (regValue & RCM_PARAM_ELVD_MASK) >> RCM_PARAM_ELVD_SHIFT;
            break;
        case RCM_LOSS_OF_CLK:                  /* loss of clock reset */
            regValue = (regValue & RCM_PARAM_ELOC_MASK) >> RCM_PARAM_ELOC_SHIFT;
            break;
        case RCM_LOSS_OF_LOCK:                 /* loss of lock reset */
            regValue = (regValue & RCM_PARAM_ELOL_MASK) >> RCM_PARAM_ELOL_SHIFT;
            break;
        case RCM_WATCH_DOG:                    /* watch dog reset */
            regValue = (regValue & RCM_PARAM_EWDOG_MASK) >> RCM_PARAM_EWDOG_SHIFT;
            break;
        case RCM_EXTERNAL_PIN:                 /* external pin reset */
            regValue = (regValue & RCM_PARAM_EPIN_MASK) >> RCM_PARAM_EPIN_SHIFT;
            break;
        case RCM_POWER_ON:                     /* power on reset */
            regValue = (regValue & RCM_PARAM_EPOR_MASK) >> RCM_PARAM_EPOR_SHIFT;
            break;
        case RCM_SJTAG:                        /* JTAG generated reset */
            regValue = (regValue & RCM_PARAM_EJTAG_MASK) >> RCM_PARAM_EJTAG_SHIFT;
            break;
        case RCM_CORE_LOCKUP:                  /* core lockup reset */
            regValue = (regValue & RCM_PARAM_ELOCKUP_MASK) >> RCM_PARAM_ELOCKUP_SHIFT;
            break;
        case RCM_SOFTWARE:                     /* software reset */
            regValue = (regValue & RCM_PARAM_ESW_MASK) >> RCM_PARAM_ESW_SHIFT;
            break;
        case RCM_SMDM_AP:                      /* MDM-AP system reset */
            regValue = (regValue & RCM_PARAM_EMDM_AP_MASK) >> RCM_PARAM_EMDM_AP_SHIFT;
            break;
        case RCM_STOP_MODE_ACK_ERR:            /* stop mode ack error reset */
            regValue = (regValue & RCM_PARAM_ESACKERR_MASK) >> RCM_PARAM_ESACKERR_SHIFT;
            break;
        case RCM_TAMPERR:                      /* wakeup */
            regValue = (regValue & RCM_PARAM_ETAMPER_MASK) >> RCM_PARAM_ETAMPER_SHIFT;
            break;
        case RCM_CORE1:                      /* wakeup */
            regValue = (regValue & RCM_PARAM_ECORE1_MASK) >> RCM_PARAM_ECORE1_SHIFT;
            break;
        default:
            /* invalid source, this error is caught by development checking: DEV_ASSERT(srcName < RCM_SRC_NAME_MAX) */
            break;
    }
    return (regValue == 0UL) ? false:true;
}



/*FUNCTION**********************************************************************
 *
 * Function Name : RCM_HAL_GetSrcStatusCmd
 * Description   : Get the reset source status
 *
 * This function will get the current reset source status for specified source
 *
 * Implements RCM_HAL_GetSrcStatusCmd_Activity
 *END**************************************************************************/
bool RCM_HAL_GetSrcStatusCmd(const RCM_Type* const baseAddr, const rcm_source_names_t srcName)
{
    bool retValue;
    uint32_t regValue = (uint32_t)baseAddr->SRS;

    DEV_ASSERT(srcName < RCM_SRC_NAME_MAX);

    switch (srcName)
    {
        case RCM_LOW_VOLT_DETECT:              /* low voltage detect reset */
            regValue = (regValue & RCM_SRS_LVD_MASK) >> RCM_SRS_LVD_SHIFT;
            break;
        case RCM_LOSS_OF_CLK:                  /* loss of clock reset */
            regValue = (regValue & RCM_SRS_LOC_MASK) >> RCM_SRS_LOC_SHIFT;
            break;
        case RCM_LOSS_OF_LOCK:                 /* loss of lock reset */
            regValue = (regValue & RCM_SRS_LOL_MASK) >> RCM_SRS_LOL_SHIFT;
            break;
        case RCM_WATCH_DOG:                    /* watch dog reset */
            regValue = (regValue & RCM_SRS_WDOG_MASK) >> RCM_SRS_WDOG_SHIFT;
            break;
        case RCM_EXTERNAL_PIN:                 /* external pin reset */
            regValue = (regValue & RCM_SRS_PIN_MASK) >> RCM_SRS_PIN_SHIFT;
            break;
        case RCM_POWER_ON:                     /* power on reset */
            regValue = (regValue & RCM_SRS_POR_MASK) >> RCM_SRS_POR_SHIFT;
            break;
        case RCM_SJTAG:                        /* JTAG generated reset */
            regValue = (regValue & RCM_SSRS_SJTAG_MASK) >> RCM_SSRS_SJTAG_SHIFT;
            break;
        case RCM_CORE_LOCKUP:                  /* core lockup reset */
            regValue = (regValue & RCM_SRS_LOCKUP_MASK) >> RCM_SRS_LOCKUP_SHIFT;
            break;
        case RCM_SOFTWARE:                     /* software reset */
            regValue = (regValue & RCM_SRS_SW_MASK) >> RCM_SRS_SW_SHIFT;
            break;
        case RCM_SMDM_AP:                      /* MDM-AP system reset */
            regValue = (regValue & RCM_SSRS_SMDM_AP_MASK) >> RCM_SSRS_SMDM_AP_SHIFT;
            break;
        case RCM_STOP_MODE_ACK_ERR:            /* stop mode ack error reset */
            regValue = (regValue & RCM_SRS_SACKERR_MASK) >> RCM_SRS_SACKERR_SHIFT;
            break;
        default:
            /* invalid command */
            regValue = 0U;
            break;
    }
    retValue = (regValue == 0UL) ? false:true;
    return retValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : RCM_HAL_SetResetIntCmd
 * Description   : Enables/disables a specified system reset interrupt.
 *
 * This function will enable/disable the specified system reset interrupt
 *
 * Implements RCM_HAL_SetResetIntCmd_Activity
 *END**************************************************************************/
void RCM_HAL_SetResetIntCmd(RCM_Type* const baseAddr, const rcm_source_names_t resetInterrupt, const bool enable)
{
    uint32_t regValue = (uint32_t)baseAddr->SRIE;
    uint32_t enableValue = (enable) ? 1UL : 0UL;

    DEV_ASSERT(resetInterrupt < RCM_SRC_NAME_MAX);

    switch (resetInterrupt)
    {
        case RCM_LOW_VOLT_DETECT:            /* low-voltage detect reset */
            regValue &= (uint32_t)(~(RCM_SSRS_SLVD_MASK));
            regValue |= RCM_SSRS_SLVD(enableValue);
            break;
        case RCM_LOSS_OF_CLK:                  /* loss of clock reset */
            regValue &= (uint32_t)(~(RCM_SRIE_LOC_MASK));
            regValue |= RCM_SRIE_LOC(enableValue);
            break;
        case RCM_LOSS_OF_LOCK:                 /* loss of lock reset */
            regValue &= (uint32_t)(~(RCM_SRIE_LOL_MASK));
            regValue |= RCM_SRIE_LOL(enableValue);
            break;
        case RCM_WATCH_DOG:                   /* watch dog reset */
            regValue &= (uint32_t)(~(RCM_SRIE_WDOG_MASK));
            regValue |= RCM_SRIE_WDOG(enableValue);
            break;
        case RCM_EXTERNAL_PIN:                /* external pin reset */
            regValue &= (uint32_t)(~(RCM_SRIE_PIN_MASK));
            regValue |= RCM_SRIE_PIN(enableValue);
            break;
        case RCM_SJTAG:                /* JTAG generated reset */
            regValue &= (uint32_t)(~(RCM_SRIE_JTAG_MASK));
            regValue |= RCM_SRIE_JTAG(enableValue);
            break;
        case RCM_CORE_LOCKUP:                 /* core lockup reset */
            regValue &= (uint32_t)(~(RCM_SRIE_LOCKUP_MASK));
            regValue |= RCM_SRIE_LOCKUP(enableValue);
            break;
        case RCM_SOFTWARE:                   /* software reset */
            regValue &= (uint32_t)(~(RCM_SRIE_SW_MASK));
            regValue |= RCM_SRIE_SW(enableValue);
            break;
        case RCM_SMDM_AP:                    /* MDM-AP system reset */
            regValue &= (uint32_t)(~(RCM_SRIE_MDM_AP_MASK));
            regValue |= RCM_SRIE_MDM_AP(enableValue);
            break;
        case RCM_STOP_MODE_ACK_ERR:          /* stop mode ack error reset */
            regValue &= (uint32_t)(~(RCM_SRIE_SACKERR_MASK));
            regValue |= RCM_SRIE_SACKERR(enableValue);
            break;
        default:
            /* invalid command */
            break;
    }
    baseAddr->SRIE = (uint32_t)regValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : RCM_HAL_GetStickySrcStatusCmd
 * Description   : Get the sticky reset source status
 *
 * This function gets the current reset source status that have not been cleared
 * by software for a specified source.
 *
 * Implements RCM_HAL_GetStickySrcStatusCmd_Activity
 *END**************************************************************************/
bool RCM_HAL_GetStickySrcStatusCmd(const RCM_Type* const baseAddr, const rcm_source_names_t srcName)
{
    bool retValue;
    uint32_t regValue;

    DEV_ASSERT(srcName < RCM_SRC_NAME_MAX);

    regValue = (uint32_t)baseAddr->SSRS;

    switch (srcName)
    {
        case RCM_LOW_VOLT_DETECT:             /* low voltage detect reset */
            regValue = (regValue & RCM_SSRS_SLVD_MASK) >> RCM_SSRS_SLVD_SHIFT;
            break;
        case RCM_LOSS_OF_CLK:                 /* loss of clock reset */
            regValue = (regValue & RCM_SSRS_SLOC_MASK) >> RCM_SSRS_SLOC_SHIFT;
            break;
        case RCM_LOSS_OF_LOCK:                /* loss of lock reset */
            regValue = (regValue & RCM_SSRS_SLOL_MASK) >> RCM_SSRS_SLOL_SHIFT;
            break;
        case RCM_WATCH_DOG:                   /* watch dog reset */
            regValue = (regValue & RCM_SSRS_SWDOG_MASK) >> RCM_SSRS_SWDOG_SHIFT;
            break;
        case RCM_EXTERNAL_PIN:                /* external pin reset */
            regValue = (regValue & RCM_SSRS_SPIN_MASK) >> RCM_SSRS_SPIN_SHIFT;
            break;
        case RCM_POWER_ON:                    /* power on reset */
            regValue = (regValue & RCM_SSRS_SPOR_MASK) >> RCM_SSRS_SPOR_SHIFT;
            break;
        case RCM_SJTAG:                       /* JTAG generated reset */
            regValue = (regValue & RCM_SSRS_SJTAG_MASK) >> RCM_SSRS_SJTAG_SHIFT;
            break;
        case RCM_CORE_LOCKUP:                 /* core lockup reset */
            regValue = (regValue & RCM_SSRS_SLOCKUP_MASK) >> RCM_SSRS_SLOCKUP_SHIFT;
            break;
        case RCM_SOFTWARE:                    /* software reset */
            regValue = (regValue & RCM_SSRS_SSW_MASK) >> RCM_SSRS_SSW_SHIFT;
            break;
        case RCM_SMDM_AP:                     /* MDM-AP system reset */
            regValue = (regValue & RCM_SSRS_SMDM_AP_MASK) >> RCM_SSRS_SMDM_AP_SHIFT;
            break;
        case RCM_STOP_MODE_ACK_ERR:           /* stop mode ack error reset */
            regValue = (regValue & RCM_SSRS_SSACKERR_MASK) >> RCM_SSRS_SSACKERR_SHIFT;
            break;
        default:
            regValue = 0U;
            break;
    }
    retValue = (regValue == 0UL) ? false:true;

    return retValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : RCM_HAL_ClearStickySrcStatus
 * Description   : Clear the sticky reset source status
 *
 * This function clears all the sticky system reset flags.
 *
 * Implements RCM_HAL_ClearStickySrcStatus_Activity
 *END**************************************************************************/
void RCM_HAL_ClearStickySrcStatus(RCM_Type* const baseAddr)
{
    uint32_t status;

    status = baseAddr->SSRS;
    baseAddr->SSRS = status;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
