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
 */

#include "crc_hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Default polynomial 0x1021U */
#define CRC_DEFAULT_POLYNOMIAL  (0x1021U)
/* Initial checksum */
#define CRC_INITIAL_SEED        (0U)

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_HAL_Init
 * Description   : This function initializes the module to default configuration
 * (Initial checksum: 0U,
 * Default polynomial: 0x1021U,
 * Type of read transpose: CRC_TRANSPOSE_NONE,
 * Type of write transpose: CRC_TRANSPOSE_NONE,
 * No complement of checksum read,
 * 32-bit CRC).
 *
 * Implements    : CRC_HAL_Init_Activity
 *END**************************************************************************/
void CRC_HAL_Init(CRC_Type * const base)
{
    /* Set CRC mode to 32-bit */
    CRC_HAL_SetProtocolWidth(base, CRC_BITS_32);

    /* Set read/write transpose and complement checksum to none */
    CRC_HAL_SetWriteTranspose(base, CRC_TRANSPOSE_NONE);
    CRC_HAL_SetReadTranspose(base, CRC_TRANSPOSE_NONE);
    CRC_HAL_SetFXorMode(base, false);

    /* Write polynomial to 0x1021U */
    CRC_HAL_SetPolyReg(base, CRC_DEFAULT_POLYNOMIAL);

    /* Write seed to zero */
    CRC_HAL_SetSeedOrDataMode(base, true);
    CRC_HAL_SetDataReg(base, CRC_INITIAL_SEED);
    CRC_HAL_SetSeedOrDataMode(base, false);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_HAL_GetCrc32
 * Description   : This function appends 32-bit data to the current CRC calculation
 * and returns new result. If the newSeed is true, seed set and result are calculated
 * from the seed new value (new CRC calculation).
 *
 * Implements    : CRC_HAL_GetCrc32_Activity
 *END**************************************************************************/
uint32_t CRC_HAL_GetCrc32(CRC_Type * const base,
                          uint32_t data,
                          bool newSeed,
                          uint32_t seed)
{
    if (newSeed)
    {
        CRC_HAL_SetSeedOrDataMode(base, true);
        CRC_HAL_SetDataReg(base, seed);
        CRC_HAL_SetSeedOrDataMode(base, false);
    }

    CRC_HAL_SetDataReg(base, data);

    return CRC_HAL_GetCrcResult(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_HAL_GetCrc16
 * Description   : This function appends 16-bit data to the current CRC calculation
 * and returns new result. If the newSeed is true, seed set and result are calculated
 * from the seed new value (new CRC calculation).
 *
 * Implements    : CRC_HAL_GetCrc16_Activity
 *END**************************************************************************/
uint32_t CRC_HAL_GetCrc16(CRC_Type * const base,
                          uint16_t data,
                          bool newSeed,
                          uint32_t seed)
{
    if (newSeed)
    {
        CRC_HAL_SetSeedOrDataMode(base, true);
        CRC_HAL_SetDataReg(base, seed);
        CRC_HAL_SetSeedOrDataMode(base, false);
    }

    CRC_HAL_SetDataLReg(base, data);

    return CRC_HAL_GetCrcResult(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_HAL_GetCrc8
 * Description   : This function appends 8-bit data to the current CRC calculation
 * and returns new result. If the newSeed is true, seed set and result are calculated
 * from the seed new value (new CRC calculation).
 *
 * Implements    : CRC_HAL_GetCrc8_Activity
 *END**************************************************************************/
uint32_t CRC_HAL_GetCrc8(CRC_Type * const base,
                         uint8_t data,
                         bool newSeed,
                         uint32_t seed)
{
    if (newSeed)
    {
        CRC_HAL_SetSeedOrDataMode(base, true);
        CRC_HAL_SetDataReg(base, seed);
        CRC_HAL_SetSeedOrDataMode(base, false);
    }

    CRC_HAL_SetDataLLReg(base, data);

    return CRC_HAL_GetCrcResult(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_HAL_GetCrcResult
 * Description   : This function returns the current result of the CRC calculation.
 *
 * Implements    : CRC_HAL_GetCrcResult_Activity
 *END**************************************************************************/
uint32_t CRC_HAL_GetCrcResult(const CRC_Type * const base)
{
    crc_bit_width_t width = CRC_HAL_GetProtocolWidth(base);
    crc_transpose_t transpose;
    uint32_t result;

    if (width == CRC_BITS_16)
    {
        transpose = CRC_HAL_GetReadTranspose(base);
        if ((transpose == CRC_TRANSPOSE_BITS_AND_BYTES) || (transpose == CRC_TRANSPOSE_BYTES))
        {
            /* Returns upper 16 bits of CRC because of transposition in 16-bit mode */
            result = (uint32_t)CRC_HAL_GetDataHReg(base);
        }
        else
        {
            result = (uint32_t)CRC_HAL_GetDataLReg(base);
        }
    }
    else
    {
        result = CRC_HAL_GetDataReg(base);
    }

    return result;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
