/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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
 * The function is defined for use by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable. A source of error in
 * writing dynamic code is that the stack segment may be different from the data
 * segment.
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

#include "crc_driver.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Table of base addresses for CRC instances. */
CRC_Type * const g_crcBase[] = CRC_BASE_PTRS;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_DRV_Init
 * Description   : This function initializes CRC driver based on user configuration input.
 * The user must make sure that the clock is enabled.
 *
 * Implements    : CRC_DRV_Init_Activity
 *END**************************************************************************/
status_t CRC_DRV_Init(uint32_t instance,
                      const crc_user_config_t * userConfigPtr)
{
    DEV_ASSERT(instance < CRC_INSTANCE_COUNT);
    DEV_ASSERT(userConfigPtr != NULL);
    CRC_Type * base = g_crcBase[instance];
    status_t retStatus = STATUS_SUCCESS;

    /* Set the default configuration */
    CRC_HAL_Init(base);
    /* Set the CRC configuration */
    retStatus = CRC_DRV_Configure(instance, userConfigPtr);

    return retStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_DRV_Deinit
 * Description   : This function sets the default configuration.
 *
 * Implements    : CRC_DRV_Deinit_Activity
 *END**************************************************************************/
status_t CRC_DRV_Deinit(uint32_t instance)
{
    DEV_ASSERT(instance < CRC_INSTANCE_COUNT);
    CRC_Type * base = g_crcBase[instance];

    /* Set the default configuration */
    CRC_HAL_Init(base);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_DRV_WriteData
 * Description   : This function appends a block of bytes to the current CRC calculation.
 *
 * Implements    : CRC_DRV_WriteData_Activity
 *END**************************************************************************/
void CRC_DRV_WriteData(uint32_t instance,
                       const uint8_t * data,
                       uint32_t dataSize)
{
    DEV_ASSERT(instance < CRC_INSTANCE_COUNT);
    DEV_ASSERT(data != NULL);
    uint32_t i;
    CRC_Type * base = g_crcBase[instance];

    /* 8-bit reads and writes till end of data buffer */
    for (i = 0U; i < dataSize; i++)
    {
        CRC_HAL_SetDataLLReg(base, data[i]);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_DRV_GetCrcResult
 * Description   : This function returns the current result of the CRC calculation.
 *
 * Implements    : CRC_DRV_GetCrcResult_Activity
 *END**************************************************************************/
uint32_t CRC_DRV_GetCrcResult(uint32_t instance)
{
    DEV_ASSERT(instance < CRC_INSTANCE_COUNT);
    const CRC_Type * base = g_crcBase[instance];

    /* Result of the CRC calculation */
    return CRC_HAL_GetCrcResult(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_DRV_Configure
 * Description   : This function configures the CRC module from a user configuration structure.
 *
 * Implements    : CRC_DRV_Configure_Activity
 *END**************************************************************************/
status_t CRC_DRV_Configure(uint32_t instance,
                           const crc_user_config_t * userConfigPtr)
{
    DEV_ASSERT(instance < CRC_INSTANCE_COUNT);
    DEV_ASSERT(userConfigPtr != NULL);
    CRC_Type * base = g_crcBase[instance];

    /* Set CRC mode */
    CRC_HAL_SetProtocolWidth(base, userConfigPtr->crcWidth);

    /* Set transposes and complement options */
    CRC_HAL_SetWriteTranspose(base, userConfigPtr->writeTranspose);
    CRC_HAL_SetReadTranspose(base, userConfigPtr->readTranspose);
    CRC_HAL_SetFXorMode(base, userConfigPtr->complementChecksum);

    /* 3Write a polynomial */
    CRC_HAL_SetPolyReg(base, userConfigPtr->polynomial);

    /* Write a seed - initial checksum */
    CRC_HAL_SetSeedOrDataMode(base, true);
    CRC_HAL_SetDataReg(base, userConfigPtr->seed);
    CRC_HAL_SetSeedOrDataMode(base, false);

    return STATUS_SUCCESS;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
