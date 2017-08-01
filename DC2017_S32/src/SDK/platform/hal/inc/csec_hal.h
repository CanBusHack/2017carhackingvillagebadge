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

#ifndef CSEC_HAL_H
#define CSEC_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "device_registers.h"

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, global macro not referenced
 * There are some global macros (CSEC_STATUS_*) used for accessing bits in the
 * csec_status_t value returned by CSEC_HAL_ReadStatus.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5,
 * Impermissible cast; cannot cast from 'essentially unsigned' to 'essentially enum<i>'.
 * All possible values are covered by the enumeration, direct casting is used to optimize code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3,
 * Expression assigned to a narrower or different essential type.
 * All possible values are covered by the enumeration, direct casting is used to optimize code.
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

/*! @file */

/*!
 * @defgroup csec_hal CSEc HAL
 * @ingroup csec
 * @addtogroup csec_hal
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#include <stddef.h>
#include "status.h"

/*! @brief The bit is set whenever SHE is processing a command. */
#define CSEC_STATUS_BUSY              (0x1U)
/*! @brief The bit is set if the secure booting is activated. */
#define CSEC_STATUS_SECURE_BOOT       (0x2U)
/*! @brief The bit is set if the secure booting has been personalized during the
boot sequence. */
#define CSEC_STATUS_BOOT_INIT         (0x4U)
/*! @brief The bit is set when the secure booting has been finished by calling
either CMD_BOOT_FAILURE or CMD_BOOT_OK or if CMD_SECURE_BOOT failed in verifying
BOOT_MAC. */
#define CSEC_STATUS_BOOT_FINISHED     (0x8U)
/*! @brief The bit is set if the secure booting (CMD_SECURE_BOOT) succeeded. If
CMD_BOOT_FAILURE is called the bit is erased. */
#define CSEC_STATUS_BOOT_OK           (0x10U)
/*! @brief The bit is set if the random number generator has been initialized. */
#define CSEC_STATUS_RND_INIT          (0x20U)
/*! @brief The bit is set if an external debugger is connected to the chip. */
#define CSEC_STATUS_EXT_DEBUGGER      (0x40U)
/*! @brief The bit is set if the internal debugging mechanisms of SHE are
activated. */
#define CSEC_STATUS_INT_DEBUGGER      (0x80U)

/*!
 * @brief Represents the status of the CSEc module. Provides one bit for each
 * status code as per SHE specification. CSEC_STATUS_* masks can be used for
 * verifying the status.
 *
 * Implements : csec_status_t_Class
 */
typedef uint8_t csec_status_t;

/*!
 * @brief CSEc commands which follow the same values as the SHE command definition.
 *
 * Implements : csec_cmd_t_Class
 */
typedef enum {
    CSEC_CMD_ENC_ECB = 0x1U,
    CSEC_CMD_ENC_CBC,
    CSEC_CMD_DEC_ECB,
    CSEC_CMD_DEC_CBC,
    CSEC_CMD_GENERATE_MAC,
    CSEC_CMD_VERIFY_MAC,
    CSEC_CMD_LOAD_KEY,
    CSEC_CMD_LOAD_PLAIN_KEY,
    CSEC_CMD_EXPORT_RAM_KEY,
    CSEC_CMD_INIT_RNG,
    CSEC_CMD_EXTEND_SEED,
    CSEC_CMD_RND,
    CSEC_CMD_RESERVED_1,
    CSEC_CMD_BOOT_FAILURE,
    CSEC_CMD_BOOT_OK,
    CSEC_CMD_GET_ID,
    CSEC_CMD_BOOT_DEFINE,
    CSEC_CMD_DBG_CHAL,
    CSEC_CMD_DBG_AUTH,
    CSEC_CMD_TRNG_RND,
    CSEC_CMD_RESERVED_2,
    CSEC_CMD_MP_COMPRESS
} csec_cmd_t;

/*!
 * @brief Specify the KeyID to be used to implement the requested cryptographic
 * operation.
 *
 * Implements : csec_key_id_t_Class
 */
typedef enum {
    CSEC_SECRET_KEY = 0x0U,
    CSEC_MASTER_ECU,
    CSEC_BOOT_MAC_KEY,
    CSEC_BOOT_MAC,
    CSEC_KEY_1,
    CSEC_KEY_2,
    CSEC_KEY_3,
    CSEC_KEY_4,
    CSEC_KEY_5,
    CSEC_KEY_6,
    CSEC_KEY_7,
    CSEC_KEY_8,
    CSEC_KEY_9,
    CSEC_KEY_10,
    CSEC_RAM_KEY = 0xFU,
    CSEC_KEY_11 = 0x14U,
    CSEC_KEY_12,
    CSEC_KEY_13,
    CSEC_KEY_14,
    CSEC_KEY_15,
    CSEC_KEY_16,
    CSEC_KEY_17,
    CSEC_KEY_18,
    CSEC_KEY_19,
    CSEC_KEY_20,
    CSEC_KEY_21
} csec_key_id_t;

/*!
 * @brief Specifies how the data is transferred to/from the CSE.
 * There are two use cases. One is to copy all data and the command function call
 * method and the other is a pointer and function call method.
 *
 * Implements : csec_func_format_t_Class
 */
typedef enum {
    CSEC_FUNC_FORMAT_COPY,
    CSEC_FUNC_FORMAT_ADDR
} csec_func_format_t;

/*!
 * @brief Specifies if the information is the first or a following function call.
 *
 * Implements : csec_call_sequence_t_Class
 */
typedef enum {
    CSEC_CALL_SEQ_FIRST,
    CSEC_CALL_SEQ_SUBSEQUENT
} csec_call_sequence_t;

/*! @brief Represents the result of the execution of a command. Provides one bit
for each error code as per SHE specification. */
#define CSEC_NO_ERROR            0x1U
#define CSEC_SEQUENCE_ERROR      0x2U
#define CSEC_KEY_NOT_AVAILABLE   0x4U
#define CSEC_KEY_INVALID         0x8U
#define CSEC_KEY_EMPTY           0x10U
#define CSEC_NO_SECURE_BOOT      0x20U
#define CSEC_KEY_WRITE_PROTECTED 0x40U
#define CSEC_KEY_UPDATE_ERROR    0x80U
#define CSEC_RNG_SEED            0x100U
#define CSEC_NO_DEBUGGING        0x200U
#define CSEC_MEMORY_FAILURE      0x400U
#define CSEC_GENERAL_ERROR       0x800U


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Writes the command header to CSE_PRAM.
 *
 * This function writes the command header to CSE_PRAM, triggering the CSEc
 * operation.
 *
 * @param[in] funcId The ID of the operation to be started.
 * @param[in] funcFormat Specifies how the data is transferred to/from the CSE.
 * @param[in] callSeq Specifies if the information is the first or a following
 * function call.
 * @param[in] keyId Specify the KeyID to be used to implement the requested
 * cryptographic operation.
 *
 * Implements : CSEC_HAL_WriteCommandHeader_Activity
 */
static inline void CSEC_HAL_WriteCommandHeader(csec_cmd_t funcId,
                                       csec_func_format_t funcFormat,
                                       csec_call_sequence_t callSeq,
                                       csec_key_id_t keyId)
{
    CSE_PRAM->RAMn[0].DATA_32 =
        CSE_PRAM_RAMn_DATA_32_BYTE_0(funcId) |
        CSE_PRAM_RAMn_DATA_32_BYTE_1(funcFormat) |
        CSE_PRAM_RAMn_DATA_32_BYTE_2(callSeq) |
        CSE_PRAM_RAMn_DATA_32_BYTE_3(keyId);
}

/*!
 * @brief Writes command bytes to CSE_PRAM.
 *
 * This function writes command bytes to CSE_PRAM, at a 32-bit aligned offset.
 *
 * @param[in] offset The offset (in bytes) at which the bytes shall be written.
 * @param[in] bytes The buffer containing the bytes to be written.
 * @param[in] numBytes The number of bytes to be written.
 */
void CSEC_HAL_WriteCommandBytes(uint8_t offset, const uint8_t *bytes, uint8_t numBytes);

/*!
 * @brief Writes a command half word to CSE_PRAM.
 *
 * This function writes a command half word to CSE_PRAM, at a 16-bit aligned
 * offset.
 *
 * @param[in] offset The offset (in bytes) at which the half word shall be
 * written.
 * @param[in] halfWord The half word to be written.
 */
void CSEC_HAL_WriteCommandHalfWord(uint8_t offset, uint16_t halfWord);

/*!
 * @brief Writes a command byte to CSE_PRAM.
 *
 * This function writes a command byte to CSE_PRAM.
 *
 * @param[in] offset The offset (in bytes) at which the byte shall be written.
 * @param[in] byte The byte to be written.
 */
void CSEC_HAL_WriteCommandByte(uint8_t offset, uint8_t byte);

/*!
 * @brief Writes command words to CSE_PRAM.
 *
 * This function writes command words to CSE_PRAM, at a 32-bit aligned offset.
 *
 * @param[in] offset The offset (in bytes) at which the words shall be written.
 * @param[in] words The buffer containing the words to be written.
 * @param[in] numWords The number of words to be written.
 */
void CSEC_HAL_WriteCommandWords(uint8_t offset, const uint32_t *words, uint8_t numWords);

/*!
 * @brief Reads command bytes from CSE_PRAM.
 *
 * This function reads command bytes from CSE_PRAM, from a 32-bit aligned offset.
 *
 * @param[in] offset The offset (in bytes) from which the bytes shall be read.
 * @param[out] bytes The buffer containing the bytes read.
 * @param[in] numBytes The number of bytes to be read.
 */
void CSEC_HAL_ReadCommandBytes(uint8_t offset, uint8_t *bytes, uint8_t numBytes);

/*!
 * @brief Reads a command half word from CSE_PRAM.
 *
 * This function reads a command half word from CSE_PRAM, from a 16-bit aligned
 * offset.
 *
 * @param[in] offset The offset (in bytes) from which the half word shall be
 * read.
 * @return The half word read.
 */
uint16_t CSEC_HAL_ReadCommandHalfWord(uint8_t offset);

/*!
 * @brief Reads a command byte from CSE_PRAM.
 *
 * This function reads a command byte from CSE_PRAM.
 *
 * @param[in] offset The offset (in bytes) from which the byte shall be read.
 * @return The byte read.
 */
uint8_t CSEC_HAL_ReadCommandByte(uint8_t offset);

/*!
 * @brief Reads command words from CSE_PRAM.
 *
 * This function reads command words from CSE_PRAM, from a 32-bit aligned offset.
 *
 * @param[in] offset The offset (in bytes) from which the words shall be read.
 * @param[out] words The buffer containing the words read.
 * @param[in] numWords The number of words to be read.
 */
void CSEC_HAL_ReadCommandWords(uint8_t offset, uint32_t *words, uint8_t numWords);

/*!
 * @brief Waits for the completion of a CSEc command.
 *
 * This function waits for the completion of a CSEc command.
 *
 * Implements : CSEC_HAL_WaitCommandCompletion_Activity
 */
static inline void CSEC_HAL_WaitCommandCompletion(void)
{
    while ((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK) == 0U)
    {
        /* Wait until the CCIF flag is set */
    }
}

/*!
 * @brief Reads the error bits from PRAM.
 *
 * This function reads the error bits reported after running a CSEc command.
 *
 * @return Error Code after command execution.
 *
 * Implements : CSEC_HAL_ReadErrorBits_Activity
 */
static inline status_t CSEC_HAL_ReadErrorBits(void)
{
    uint16_t errBits = CSEC_HAL_ReadCommandHalfWord(FEATURE_CSEC_ERROR_BITS_OFFSET);
    status_t stat;

    switch (errBits)
    {
    case CSEC_NO_ERROR:
        stat = STATUS_SUCCESS;
        break;
    case CSEC_SEQUENCE_ERROR:
        stat = STATUS_CSEC_SEQUENCE_ERROR;
        break;
    case CSEC_KEY_NOT_AVAILABLE:
        stat = STATUS_CSEC_KEY_NOT_AVAILABLE;
        break;
    case CSEC_KEY_INVALID:
        stat = STATUS_CSEC_KEY_INVALID;
        break;
    case CSEC_KEY_EMPTY:
        stat = STATUS_CSEC_KEY_EMPTY;
        break;
    case CSEC_NO_SECURE_BOOT:
        stat = STATUS_CSEC_NO_SECURE_BOOT;
        break;
    case CSEC_KEY_WRITE_PROTECTED:
        stat = STATUS_CSEC_KEY_WRITE_PROTECTED;
        break;
    case CSEC_KEY_UPDATE_ERROR:
        stat = STATUS_CSEC_KEY_UPDATE_ERROR;
        break;
    case CSEC_RNG_SEED:
        stat = STATUS_CSEC_RNG_SEED;
        break;
    case CSEC_NO_DEBUGGING:
        stat = STATUS_CSEC_NO_DEBUGGING;
        break;
    case CSEC_MEMORY_FAILURE:
        stat = STATUS_CSEC_MEMORY_FAILURE;
        break;
    case CSEC_GENERAL_ERROR:
    default:
        stat = STATUS_ERROR;
        break;
    }

    return stat;
}

/*!
 * @brief Reads the status of the CSEc module.
 *
 * This function reads the contents od the status register.
 *
 * @return Status represented as a mask of CSEC_STATUS_*.
 *
 * Implements : CSEC_HAL_ReadStatus_Activity
 */
static inline csec_status_t CSEC_HAL_ReadStatus(void)
{
    return (FTFC->FCSESTAT);
}

/*!
 * @brief Writes the command header to CSE_PRAM and waits for completion.
 *
 * This function writes the header of a command and waits for completion.
 * The function is always located in RAM, and is used for CSEc commands using
 * pointer methods, in order to allow the MGATE to read from FLASH without
 * causing a read collision.
 *
 * @param[in] funcId The ID of the operation to be started.
 * @param[in] funcFormat Specifies how the data is transferred to/from the CSE.
 * @param[in] callSeq Specifies if the information is the first or a following
 * function call.
 * @param[in] keyId Specify the KeyID to be used to implement the requested
 * cryptographic operation.
 */
START_FUNCTION_DECLARATION_RAMSECTION
void CSEC_HAL_WriteCmdAndWait(csec_cmd_t funcId,
        csec_func_format_t funcFormat,
        csec_call_sequence_t callSeq,
        csec_key_id_t keyId)
END_FUNCTION_DECLARATION_RAMSECTION

/*!
 * @brief Enables/Disables the command completion interrupt.
 *
 * This function enables/disables the command completion interrupt.
 *
 * @param[in] enable Enable/Disable the command completion interrupt.
 *
 * Implements : CSEC_HAL_SetInterrupt_Activity
 */
static inline void CSEC_HAL_SetInterrupt(bool enable)
{
    uint8_t tmp = FTFC->FCNFG;

    FTFC->FCNFG = (uint8_t)((tmp & ~FTFC_FCNFG_CCIE_MASK) | FTFC_FCNFG_CCIE(enable ? 1U : 0U));
}

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* CSEC_HAL_H */

/*******************************************************************************
 * EOF
 ******************************************************************************/
