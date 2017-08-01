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

#ifndef CRC_HAL_H
#define CRC_HAL_H

/*! @file crc_hal.h */

#include <stddef.h>
#include <stdbool.h>
#include "device_registers.h"

/*!
 * @defgroup crc_hal CRC HAL
 * @ingroup crc
 * @details This section describes the programming interface of the CRC HAL.
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @brief CRC type of transpose of read write data
 * Implements : crc_transpose_t_Class
 */
typedef enum
{
    CRC_TRANSPOSE_NONE              = 0x00U,    /*!< No transpose */
    CRC_TRANSPOSE_BITS              = 0x01U,    /*!< Transpose bits in bytes */
    CRC_TRANSPOSE_BITS_AND_BYTES    = 0x02U,    /*!< Transpose bytes and bits in bytes */
    CRC_TRANSPOSE_BYTES             = 0x03U     /*!< Transpose bytes */
} crc_transpose_t;

/*!
 * @brief CRC bit width
 * Implements : crc_bit_width_t_Class
 */
typedef enum
{
    CRC_BITS_16 = 0U,   /*!< Generate 16-bit CRC code */
    CRC_BITS_32 = 1U    /*!< Generate 32-bit CRC code */
} crc_bit_width_t;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name CRC HAL API
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the CRC module
 *
 * This function initializes the module to default configuration
 * (Initial checksum: 0U,
 * Default polynomial: 0x1021U,
 * Type of read transpose: CRC_TRANSPOSE_NONE,
 * Type of write transpose: CRC_TRANSPOSE_NONE,
 * No complement of checksum read,
 * 32-bit CRC)
 *
 * @param[in] base The CRC peripheral base address
 */
void CRC_HAL_Init(CRC_Type * const base);

/*!
 * @brief Appends 32-bit data to the current CRC calculation and returns new result
 *
 * This function appends 32-bit data to the current CRC calculation
 * and returns new result. If the newSeed is true, seed set and
 * result are calculated from the seed new value (new CRC calculation)
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] data Input data for CRC calculation
 * @param[in] newSeed Sets new CRC calculation
 *            - true: New seed set and used for new calculation.
 *            - false: Seed argument ignored, continues old calculation.
 * @param[in] seed New seed if newSeed is true, else ignored
 * @return New CRC result
 */
uint32_t CRC_HAL_GetCrc32(CRC_Type * const base,
                          uint32_t data,
                          bool newSeed,
                          uint32_t seed);

/*!
 * @brief Appends 16-bit data to the current CRC calculation and returns new result
 *
 * This function appends 16-bit data to the current CRC calculation
 * and returns new result. If the newSeed is true, seed set and
 * result are calculated from the seed new value (new CRC calculation)
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] data Input data for CRC calculation
 * @param[in] newSeed Sets new CRC calculation
 *            - true: New seed set and used for new calculation.
 *            - false: Seed argument ignored, continues old calculation.
 * @param[in] seed New seed if newSeed is true, else ignored
 * @return New CRC result
 */
uint32_t CRC_HAL_GetCrc16(CRC_Type * const base,
                          uint16_t data,
                          bool newSeed,
                          uint32_t seed);

/*!
 * @brief Appends 8-bit data to the current CRC calculation and returns new result
 *
 * This function appends 8-bit data to the current CRC calculation
 * and returns new result. If the newSeed is true, seed set and
 * result are calculated from the seed new value (new CRC calculation)
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] data Input data for CRC calculation
 * @param[in] newSeed Sets new CRC calculation
 *            - true: New seed set and used for new calculation.
 *            - false: Seed argument ignored, continues old calculation.
 * @param[in] seed New seed if newSeed is true, else ignored
 * @return New CRC result
 */
uint32_t CRC_HAL_GetCrc8(CRC_Type * const base,
                         uint8_t data,
                         bool newSeed,
                         uint32_t seed);

/*!
 * @brief Returns the current result of the CRC calculation
 *
 * This function returns the current result of the CRC calculation
 *
 * @param[in] base The CRC peripheral base address
 * @return Result of CRC calculation
 */
uint32_t CRC_HAL_GetCrcResult(const CRC_Type * const base);

/*!
 * @brief Gets the current CRC result
 *
 * This function gets the current CRC result from the data register
 *
 * @param[in] base The CRC peripheral base address
 * @return Returns the current CRC result
 * Implements : CRC_HAL_GetDataReg_Activity
 */
static inline uint32_t CRC_HAL_GetDataReg(const CRC_Type * const base)
{
    return base->DATAu.DATA;
}

/*!
 * @brief Sets the 32 bits of CRC data register
 *
 * This function sets the 32 bits of CRC data register
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] value New data for CRC computation
 * Implements : CRC_HAL_SetDataReg_Activity
 */
static inline void CRC_HAL_SetDataReg(CRC_Type * const base,
                                      uint32_t value)
{
    base->DATAu.DATA = value;
}

/*!
 * @brief Gets the upper 16 bits of the current CRC result
 *
 * This function gets the upper 16 bits of the current CRC result from the data register
 *
 * @param[in] base The CRC peripheral base address
 * @return Returns the upper 16 bits of the current CRC result
 * Implements : CRC_HAL_GetDataHReg_Activity
 */
static inline uint16_t CRC_HAL_GetDataHReg(const CRC_Type * const base)
{
    return base->DATAu.DATA_16.H;
}

/*!
 * @brief Sets the upper 16 bits of CRC data register
 *
 * This function sets the upper 16 bits of CRC data register
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] value New data for CRC computation
 * Implements : CRC_HAL_SetDataHReg_Activity
 */
static inline void CRC_HAL_SetDataHReg(CRC_Type * const base,
                                       uint16_t value)
{
    base->DATAu.DATA_16.H = value;
}

/*!
 * @brief Gets the lower 16 bits of the current CRC result
 *
 * This function gets the lower 16 bits of the current CRC result from the data register
 *
 * @param[in] base The CRC peripheral base address
 * @return Returns the lower 16 bits of the current CRC result
 * Implements : CRC_HAL_GetDataLReg_Activity
 */
static inline uint16_t CRC_HAL_GetDataLReg(const CRC_Type * const base)
{
    return base->DATAu.DATA_16.L;
}

/*!
 * @brief Sets the lower 16 bits of CRC data register
 *
 * This function sets the lower 16 bits of CRC data register
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] value New data for CRC computation
 * Implements : CRC_HAL_SetDataLReg_Activity
 */
static inline void CRC_HAL_SetDataLReg(CRC_Type * const base,
                                       uint16_t value)
{
    base->DATAu.DATA_16.L = value;
}

/*!
 * @brief Sets the High Upper Byte - HU
 *
 * This function sets the High Upper Byte - HU of CRC data register
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] value New data for CRC computation
 * Implements : CRC_HAL_SetDataHUReg_Activity
 */
static inline void CRC_HAL_SetDataHUReg(CRC_Type * const base,
                                        uint8_t value)
{
    base->DATAu.DATA_8.HU = value;
}

/*!
 * @brief Sets the High Lower Byte - HL
 *
 * This function sets the High Lower Byte - HL of CRC data register
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] value New data for CRC computation
 * Implements : CRC_HAL_SetDataHLReg_Activity
 */
static inline void CRC_HAL_SetDataHLReg(CRC_Type * const base,
                                        uint8_t value)
{
    base->DATAu.DATA_8.HL = value;
}

/*!
 * @brief Sets the Low Upper Byte - LU
 *
 * This function sets the Low Upper Byte - LU of CRC data register
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] value New data for CRC computation
 * Implements : CRC_HAL_SetDataLUReg_Activity
 */
static inline void CRC_HAL_SetDataLUReg(CRC_Type * const base,
                                        uint8_t value)
{
    base->DATAu.DATA_8.LU = value;
}

/*!
 * @brief Sets the Low Lower Byte - LL
 *
 * This function sets the Low Lower Byte - LL of CRC data register
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] value New data for CRC computation
 * Implements : CRC_HAL_SetDataLLReg_Activity
 */
static inline void CRC_HAL_SetDataLLReg(CRC_Type * const base,
                                        uint8_t value)
{
    base->DATAu.DATA_8.LL = value;
}

/*!
 * @brief Gets the polynomial register value
 *
 * This function gets the polynomial register value
 *
 * @param[in] base The CRC peripheral base address
 * @return Returns the polynomial register value
 * Implements : CRC_HAL_GetPolyReg_Activity
 */
static inline uint32_t CRC_HAL_GetPolyReg(const CRC_Type * const base)
{
    return base->GPOLY;
}

/*!
 * @brief Sets the polynomial register
 *
 * This function sets the polynomial register
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] value Polynomial value
 * Implements : CRC_HAL_SetPolyReg_Activity
 */
static inline void CRC_HAL_SetPolyReg(CRC_Type * const base,
                                      uint32_t value)
{
    base->GPOLY = value;
}

/*!
 * @brief Gets the upper 16 bits of polynomial register
 *
 * This function gets the upper 16 bits of polynomial register.
 * Note that this upper part of the register is not used in 16-bit CRC mode
 *
 * @param[in] base The CRC peripheral base address
 * @return Returns the upper 16 bits of polynomial register
 * Implements : CRC_HAL_GetPolyHReg_Activity
 */
static inline uint16_t CRC_HAL_GetPolyHReg(const CRC_Type * const base)
{
    return (uint16_t)((base->GPOLY & CRC_GPOLY_HIGH_MASK) >> CRC_GPOLY_HIGH_SHIFT);
}

/*!
 * @brief Sets the upper 16 bits of polynomial register
 *
 * This function sets the upper 16 bits of polynomial register.
 * Note that this upper part of the register is ignored in 16-bit CRC mode
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] value Polynomial value
 * Implements : CRC_HAL_SetPolyHReg_Activity
 */
static inline void CRC_HAL_SetPolyHReg(CRC_Type * const base,
                                       uint16_t value)
{
    uint32_t gpolyTemp = base->GPOLY;

    gpolyTemp &= ~(CRC_GPOLY_HIGH_MASK);
    gpolyTemp |= CRC_GPOLY_HIGH(value);
    base->GPOLY = gpolyTemp;
}

/*!
 * @brief Gets the lower 16 bits of polynomial register
 *
 * This function gets the lower 16 bits of polynomial register
 *
 * @param[in] base The CRC peripheral base address
 * @return Returns the lower 16 bits of polynomial register
 * Implements : CRC_HAL_GetPolyLReg_Activity
 */
static inline uint16_t CRC_HAL_GetPolyLReg(const CRC_Type * const base)
{
    return (uint16_t)((base->GPOLY & CRC_GPOLY_LOW_MASK) >> CRC_GPOLY_LOW_SHIFT);
}

/*!
 * @brief Sets the lower 16 bits of polynomial register
 *
 * This function sets the lower 16 bits of polynomial register
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] value Polynomial value
 * Implements : CRC_HAL_SetPolyLReg_Activity
 */
static inline void CRC_HAL_SetPolyLReg(CRC_Type * const base,
                                       uint16_t value)
{
    uint32_t gpolyTemp = base->GPOLY;

    gpolyTemp &= ~(CRC_GPOLY_LOW_MASK);
    gpolyTemp |= CRC_GPOLY_LOW(value);
    base->GPOLY = gpolyTemp;
}

/*!
 * @brief Gets the CRC_DATA register mode
 *
 * This function gets the CRC_DATA register mode
 *
 * @param[in] base The CRC peripheral base address
 * @return CRC_DATA register mode
 *         -true: CRC_DATA register is used for seed values.
 *         -false: CRC_DATA register is used for data values.
 * Implements : CRC_HAL_GetSeedOrDataMode_Activity
 */
static inline bool CRC_HAL_GetSeedOrDataMode(const CRC_Type * const base)
{
    return ((base->CTRL & CRC_CTRL_WAS_MASK) >> CRC_CTRL_WAS_SHIFT) != 0U;
}

/*!
 * @brief Sets the CRC_DATA register mode
 *
 * This function sets the CRC_DATA register mode
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] enable Enable CRC data register to use for seed value
 *            -true: use CRC data register for seed values.
 *            -false: use CRC data register for data values.
 * Implements : CRC_HAL_SetSeedOrDataMode_Activity
 */
static inline void CRC_HAL_SetSeedOrDataMode(CRC_Type * const base,
                                             bool enable)
{
    uint32_t ctrlTemp = base->CTRL;

    ctrlTemp &= ~(CRC_CTRL_WAS_MASK);
    ctrlTemp |= CRC_CTRL_WAS(enable ? 1UL : 0UL);
    base->CTRL = ctrlTemp;
}

/*!
 * @brief Gets complement read of CRC data register
 *
 * This function gets complement read of CRC data register.
 * Some CRC protocols require the final checksum to be XORed with 0xFFFFFFFF
 * or 0xFFFF. Complement mode enables "on the fly" complementing of read data
 *
 * @param[in] base The CRC peripheral base address
 * @return Complement read
 *         -true: Invert or complement the read value of the CRC Data register.
 *         -false: No XOR on reading.
 * Implements : CRC_HAL_GetFXorMode_Activity
 */
static inline bool CRC_HAL_GetFXorMode(const CRC_Type * const base)
{
    return ((base->CTRL & CRC_CTRL_FXOR_MASK) >> CRC_CTRL_FXOR_SHIFT) != 0U;
}

/*!
 * @brief Sets complement read of CRC data register
 *
 * This function sets complement read of CRC data register.
 * Some CRC protocols require the final checksum to be XORed with 0xFFFFFFFF
 * or 0xFFFF. Complement mode enables "on the fly" complementing of read data
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] enable Enable or disable complementing of read data
 * Implements : CRC_HAL_SetFXorMode_Activity
 */
static inline void CRC_HAL_SetFXorMode(CRC_Type * const base,
                                       bool enable)
{
    uint32_t ctrlTemp = base->CTRL;

    ctrlTemp &= ~(CRC_CTRL_FXOR_MASK);
    ctrlTemp |= CRC_CTRL_FXOR(enable ? 1UL : 0UL);
    base->CTRL = ctrlTemp;
}

/*!
 * @brief Gets the CRC protocol width
 *
 * This function gets the CRC protocol width
 *
 * @param[in] base The CRC peripheral base address
 * @return CRC protocol width
 *         - CRC_BITS_16: 16-bit CRC protocol.
 *         - CRC_BITS_32: 32-bit CRC protocol.
 * Implements : CRC_HAL_GetProtocolWidth_Activity
 */
static inline crc_bit_width_t CRC_HAL_GetProtocolWidth(const CRC_Type * const base)
{
    crc_bit_width_t retVal = CRC_BITS_16;

    if (((base->CTRL & CRC_CTRL_TCRC_MASK) >> CRC_CTRL_TCRC_SHIFT) != 0U)
    {
        retVal = CRC_BITS_32;
    }

    return retVal;
}

/*!
 * @brief Sets the CRC protocol width
 *
 * This function sets the CRC protocol width
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] width The CRC protocol width
 *            - CRC_BITS_16: 16-bit CRC protocol.
 *            - CRC_BITS_32: 32-bit CRC protocol.
 * Implements : CRC_HAL_SetProtocolWidth_Activity
 */
static inline void CRC_HAL_SetProtocolWidth(CRC_Type * const base,
                                            crc_bit_width_t width)
{
    uint32_t ctrlTemp = base->CTRL;

    ctrlTemp &= ~(CRC_CTRL_TCRC_MASK);
    ctrlTemp |= CRC_CTRL_TCRC(width);
    base->CTRL = ctrlTemp;
}

/*!
 * @brief Gets the CRC transpose type for writes
 *
 * This function gets the CRC transpose type for writes
 *
 * @param[in] base The CRC peripheral base address
 * @return CRC input transpose type for writes
 * Implements : CRC_HAL_GetWriteTranspose_Activity
 */
static inline crc_transpose_t CRC_HAL_GetWriteTranspose(const CRC_Type * const base)
{
    crc_transpose_t type;
    uint32_t temp = (base->CTRL & CRC_CTRL_TOT_MASK) >> CRC_CTRL_TOT_SHIFT;

    switch (temp)
    {
        case 1U:
            type = CRC_TRANSPOSE_BITS;
            break;
        case 2U:
            type = CRC_TRANSPOSE_BITS_AND_BYTES;
            break;
        case 3U:
            type = CRC_TRANSPOSE_BYTES;
            break;
        default:
            type = CRC_TRANSPOSE_NONE;
            break;
    }

    return type;
}

/*!
 * @brief Sets the CRC transpose type for writes
 *
 * This function sets the CRC transpose type for writes
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] transp The CRC input transpose type
 * Implements : CRC_HAL_SetWriteTranspose_Activity
 */
static inline void CRC_HAL_SetWriteTranspose(CRC_Type * const base,
                                             crc_transpose_t transp)
{
    uint32_t ctrlTemp = base->CTRL;

    ctrlTemp &= ~(CRC_CTRL_TOT_MASK);
    ctrlTemp |= CRC_CTRL_TOT(transp);
    base->CTRL = ctrlTemp;
}

/*!
 * @brief Gets the CRC transpose type for reads
 *
 * This function gets the CRC transpose type for reads
 *
 * @param[in] base The CRC peripheral base address
 * @return CRC output transpose type
 * Implements : CRC_HAL_GetReadTranspose_Activity
 */
static inline crc_transpose_t CRC_HAL_GetReadTranspose(const CRC_Type * const base)
{
    crc_transpose_t type;
    uint32_t temp = (base->CTRL & CRC_CTRL_TOTR_MASK) >> CRC_CTRL_TOTR_SHIFT;

    switch (temp)
    {
        case 1U:
            type = CRC_TRANSPOSE_BITS;
            break;
        case 2U:
            type = CRC_TRANSPOSE_BITS_AND_BYTES;
            break;
        case 3U:
            type = CRC_TRANSPOSE_BYTES;
            break;
        default:
            type = CRC_TRANSPOSE_NONE;
            break;
    }

    return type;
}

/*!
 * @brief Sets the CRC transpose type for reads
 *
 * This function sets the CRC transpose type for reads
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] transp The CRC output transpose type
 * Implements : CRC_HAL_SetReadTranspose_Activity
 */
static inline void CRC_HAL_SetReadTranspose(CRC_Type * const base,
                                            crc_transpose_t transp)
{
    uint32_t ctrlTemp = base->CTRL;

    ctrlTemp &= ~(CRC_CTRL_TOTR_MASK);
    ctrlTemp |= CRC_CTRL_TOTR(transp);
    base->CTRL = ctrlTemp;
}

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* CRC_HAL_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
