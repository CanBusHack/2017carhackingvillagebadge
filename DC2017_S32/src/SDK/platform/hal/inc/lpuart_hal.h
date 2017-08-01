/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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
 * @file lpuart_hal.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.3, Global typedef not referenced.
 * This increases ease of use: allows users to access the corresponding field in the register
 * using an already defined type.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * The macros were defined for consistency reasons, all the registers have a corresponding ID.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower or different essential type.
 * The cast is required to perform a conversion between an unsigned integer and an enum type.
 */

#ifndef LPUART_HAL_H__
#define LPUART_HAL_H__

#include "device_registers.h"
#include "status.h"
#include <stdbool.h>
#include <stddef.h>

/*!
 * @addtogroup lpuart_hal
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LPUART_SHIFT (16U)
#define LPUART_BAUD_REG_ID (1U)
#define LPUART_STAT_REG_ID (2U)
#define LPUART_CTRL_REG_ID (3U)
#define LPUART_DATA_REG_ID (4U)
#define LPUART_MATCH_REG_ID (5U)
#define LPUART_MODIR_REG_ID (6U)
#define LPUART_FIFO_REG_ID (7U)
#define LPUART_WATER_REG_ID (8U)

/*! @brief LPUART number of stop bits
 *
 * Implements : lpuart_stop_bit_count_t_Class
 */
typedef enum
{
    LPUART_ONE_STOP_BIT = 0x0U, /*!< one stop bit */
    LPUART_TWO_STOP_BIT = 0x1U  /*!< two stop bits */
} lpuart_stop_bit_count_t;

/*! @brief LPUART parity mode
 *
 * Implements : lpuart_parity_mode_t_Class
 */
typedef enum
{
    LPUART_PARITY_DISABLED = 0x0U, /*!< parity disabled */
    LPUART_PARITY_EVEN     = 0x2U, /*!< parity enabled, type even, bit setting: PE|PT = 10 */
    LPUART_PARITY_ODD      = 0x3U  /*!< parity enabled, type odd,  bit setting: PE|PT = 11 */
} lpuart_parity_mode_t;

/*! @brief LPUART number of bits in a character
 *
 * Implements : lpuart_bit_count_per_char_t_Class
 */
typedef enum
{
    LPUART_8_BITS_PER_CHAR  = 0x0U, /*!< 8-bit data characters */
    LPUART_9_BITS_PER_CHAR  = 0x1U, /*!< 9-bit data characters */
    LPUART_10_BITS_PER_CHAR = 0x2U  /*!< 10-bit data characters */
} lpuart_bit_count_per_char_t;

/*! @brief LPUART operation configuration constants
 *
 * Implements : lpuart_operation_config_t_Class
 */
typedef enum
{
    LPUART_OPERATES = 0x0U, /*!< LPUART continues to operate normally.*/
    LPUART_STOPS    = 0x1U  /*!< LPUART stops operation. */
} lpuart_operation_config_t;

/*! @brief LPUART wakeup from standby method constants
 *
 * Implements : lpuart_wakeup_method_t_Class
 */
typedef enum
{
    LPUART_IDLE_LINE_WAKE = 0x0U, /*!< Idle-line wakes the LPUART receiver from standby. */
    LPUART_ADDR_MARK_WAKE = 0x1U  /*!< Addr-mark wakes LPUART receiver from standby.*/
} lpuart_wakeup_method_t;

/*!
 * @brief LPUART break character length settings for transmit/detect.
 *
 * The actual maximum bit times may vary depending on the LPUART instance.
 *
 * Implements : lpuart_break_char_length_t_Class
 */
typedef enum
{
    LPUART_BREAK_CHAR_10_BIT_MINIMUM = 0x0U, /*!< LPUART break char length 10 bit times (if M = 0, SBNS = 0)
                                                  or 11 (if M = 1, SBNS = 0 or M = 0, SBNS = 1) or 12 (if M = 1,
                                                  SBNS = 1 or M10 = 1, SNBS = 0) or 13 (if M10 = 1, SNBS = 1) */
    LPUART_BREAK_CHAR_13_BIT_MINIMUM = 0x1U  /*!< LPUART break char length 13 bit times (if M = 0, SBNS = 0
                                                  or M10 = 0, SBNS = 1) or 14 (if M = 1, SBNS = 0 or M = 1,
                                                  SBNS = 1) or 15 (if M10 = 1, SBNS = 1 or M10 = 1, SNBS = 0) */
} lpuart_break_char_length_t;

/*! @brief LPUART single-wire mode TX direction
 *
 * Implements : lpuart_singlewire_txdir_t_Class
 */
typedef enum
{
    LPUART_SINGLE_WIRE_TX_DIR_IN  = 0x0U, /*!< LPUART Single Wire mode TXDIR input*/
    LPUART_SINGLE_WIRE_TX_DIR_OUT = 0x1U  /*!< LPUART Single Wire mode TXDIR output*/
} lpuart_singlewire_txdir_t;

/*! @brief LPUART Configures the match addressing mode used.
 *
 * Implements : lpuart_match_config_t_Class
 */
typedef enum
{
    LPUART_ADDRESS_MATCH_WAKEUP     = 0x0U, /*!< Address Match Wakeup*/
    LPUART_IDLE_MATCH_WAKEUP        = 0x1U, /*!< Idle Match Wakeup*/
    LPUART_MATCH_ON_AND_MATCH_OFF   = 0x2U, /*!< Match On and Match Off*/
    LPUART_ENABLE_RWU_ON_DATA_MATCH = 0x3U  /*!< Enables RWU on Data Match and Match On/Off for transmitter CTS input*/
} lpuart_match_config_t;

/*! @brief LPUART infra-red transmitter pulse width options
 *
 * Implements : lpuart_ir_tx_pulsewidth_t_Class
 */
typedef enum
{
    LPUART_IR_THREE_SIXTEENTH_WIDTH   = 0x0U, /*!< 3/16 pulse*/
    LPUART_IR_ONE_SIXTEENTH_WIDTH     = 0x1U, /*!< 1/16 pulse*/
    LPUART_IR_ONE_THIRTYSECOND_WIDTH  = 0x2U, /*!< 1/32 pulse*/
    LPUART_IR_ONE_FOURTH_WIDTH        = 0x3U  /*!< 1/4 pulse*/
} lpuart_ir_tx_pulsewidth_t;

/*!
 * @brief LPUART Configures the number of idle characters that must be received
 * before the IDLE flag is set.
 *
 * Implements : lpuart_idle_char_t_Class
 */
typedef enum
{
    LPUART_1_IDLE_CHAR   = 0x0U, /*!< 1 idle character*/
    LPUART_2_IDLE_CHAR   = 0x1U, /*!< 2 idle character*/
    LPUART_4_IDLE_CHAR   = 0x2U, /*!< 4 idle character*/
    LPUART_8_IDLE_CHAR   = 0x3U, /*!< 8 idle character*/
    LPUART_16_IDLE_CHAR  = 0x4U, /*!< 16 idle character*/
    LPUART_32_IDLE_CHAR  = 0x5U, /*!< 32 idle character*/
    LPUART_64_IDLE_CHAR  = 0x6U, /*!< 64 idle character*/
    LPUART_128_IDLE_CHAR = 0x7U  /*!< 128 idle character*/
} lpuart_idle_char_t;

/*! @brief LPUART Transmits the CTS Configuration. Configures the source of the CTS input.
 *
 * Implements : lpuart_cts_source_t_Class
 */
typedef enum
{
    LPUART_CTS_SOURCE_PIN                     = 0x0U,  /*!< CTS input is the LPUART_CTS pin.*/
    LPUART_CTS_SOURCE_INVERTED_RECEIVER_MATCH = 0x1U   /*!< CTS input is the inverted Receiver Match result.*/
} lpuart_cts_source_t;

/*!
 * @brief LPUART Transmits CTS Source.Configures if the CTS state is checked at
 * the start of each character or only when the transmitter is idle.
 *
 * Implements : lpuart_cts_config_t_Class
 */
typedef enum
{
    LPUART_CTS_SAMPLED_ON_EACH_CHAR = 0x0U, /*!< CTS input is sampled at the start of each character.*/
    LPUART_CTS_SAMPLED_ON_IDLE      = 0x1U  /*!< CTS input is sampled when the transmitter is idle.*/
} lpuart_cts_config_t;

/*! @brief Structure for idle line configuration settings
 *
 * Implements : lpuart_idle_line_config_t_Class
 */
typedef struct
{
    unsigned idleLineType : 1; /*!< ILT, Idle bit count start: 0 - after start bit (default),
                                    1 - after stop bit */
    unsigned rxWakeIdleDetect : 1; /*!< RWUID, Receiver Wake Up Idle Detect. IDLE status bit
                                        operation during receive standbyControls whether idle
                                        character that wakes up receiver will also set
                                        IDLE status bit 0 - IDLE status bit doesn't
                                        get set (default), 1 - IDLE status bit gets set*/
} lpuart_idle_line_config_t;

/*!
 * @brief LPUART status flags.
 *
 * This provides constants for the LPUART status flags for use in the UART functions.
 *
 * Implements : lpuart_status_flag_t_Class
 */
typedef enum
{
    LPUART_TX_DATA_REG_EMPTY          = (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) \
                                        | (uint32_t)LPUART_STAT_TDRE_SHIFT), \
                                        /*!< Tx data register empty flag, sets when Tx buffer is empty */
    LPUART_TX_COMPLETE                = (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) \
                                        | (uint32_t)LPUART_STAT_TC_SHIFT), \
                                        /*!< Transmission complete flag, sets when transmission activity complete */
    LPUART_RX_DATA_REG_FULL           = (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) \
                                        | (uint32_t)LPUART_STAT_RDRF_SHIFT), \
                                        /*!< Rx data register full flag, sets when the receive data buffer is full */
    LPUART_IDLE_LINE_DETECT           = (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) \
                                        | (uint32_t)LPUART_STAT_IDLE_SHIFT), \
                                        /*!< Idle line detect flag, sets when idle line detected */
    LPUART_RX_OVERRUN                 = (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) \
                                        | (uint32_t)LPUART_STAT_OR_SHIFT), \
                                        /*!< Rx Overrun sets if new data is received before data is read */
    LPUART_NOISE_DETECT               = (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) \
                                        | (uint32_t)LPUART_STAT_NF_SHIFT), \
                                        /*!< Rx takes 3 samples of each received bit. If these differ, the flag sets */
    LPUART_FRAME_ERR                  = (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) \
                                        | (uint32_t)LPUART_STAT_FE_SHIFT), \
                                        /*!< Frame error flag, sets if logic 0 was detected where stop bit expected */
    LPUART_PARITY_ERR                 = (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) \
                                        | (uint32_t)LPUART_STAT_PF_SHIFT), \
                                        /*!< If parity enabled, sets upon parity error detection */
    LPUART_LIN_BREAK_DETECT           = (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) \
                                        | (uint32_t)LPUART_STAT_LBKDIF_SHIFT), \
                                        /*!< LIN break detect interrupt flag, sets when LIN break char detected */
    LPUART_RX_ACTIVE_EDGE_DETECT      = (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) \
                                        | (uint32_t)LPUART_STAT_RXEDGIF_SHIFT), \
                                        /*!< Rx pin active edge interrupt flag, sets when active edge detected */
    LPUART_RX_ACTIVE                  = (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) \
                                        | (uint32_t)LPUART_STAT_RAF_SHIFT), \
                                        /*!< Receiver Active Flag (RAF), sets at beginning of valid start bit */
    LPUART_NOISE_IN_CURRENT_WORD      = (((uint32_t)LPUART_DATA_REG_ID << (uint32_t)LPUART_SHIFT) \
                                        | (uint32_t)LPUART_DATA_NOISY_SHIFT), \
                                        /*!< NOISY bit, sets if noise detected in current data word */
    LPUART_PARITY_ERR_IN_CURRENT_WORD = (((uint32_t)LPUART_DATA_REG_ID << (uint32_t)LPUART_SHIFT) \
                                        | (uint32_t)LPUART_DATA_PARITYE_SHIFT), \
                                        /*!< PARITYE bit, sets if noise detected in current data word */
#if FEATURE_LPUART_HAS_ADDRESS_MATCHING
    LPUART_MATCH_ADDR_ONE             = (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) \
                                        | (uint32_t)LPUART_STAT_MA1F_SHIFT), \
                                        /*!< Address one match flag */
    LPUART_MATCH_ADDR_TWO             = (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) \
                                        | (uint32_t)LPUART_STAT_MA2F_SHIFT), \
                                        /*!< Address two match flag */
#endif
#if FEATURE_LPUART_FIFO_SIZE > 0U
    LPUART_FIFO_TX_OF                 = (((uint32_t)LPUART_FIFO_REG_ID << (uint32_t)LPUART_SHIFT) \
                                        | (uint32_t)LPUART_FIFO_TXOF_SHIFT), \
                                        /*!< Transmitter FIFO buffer overflow */
    LPUART_FIFO_RX_UF                 = (((uint32_t)LPUART_FIFO_REG_ID << (uint32_t)LPUART_SHIFT) \
                                        | (uint32_t)LPUART_FIFO_RXUF_SHIFT) \
                                        /*!< Receiver FIFO buffer underflow */
#endif
} lpuart_status_flag_t;

/*! @brief LPUART interrupt configuration structure, default settings are 0 (disabled)
 *
 * Implements : lpuart_interrupt_t_Class
 */
typedef enum
{
    LPUART_INT_LIN_BREAK_DETECT  = (((uint32_t)LPUART_BAUD_REG_ID << (uint32_t)LPUART_SHIFT) \
                                   | (uint32_t)LPUART_BAUD_LBKDIE_SHIFT),  /*!< LIN break detect. */
    LPUART_INT_RX_ACTIVE_EDGE    = (((uint32_t)LPUART_BAUD_REG_ID << (uint32_t)LPUART_SHIFT) \
                                   | (uint32_t)LPUART_BAUD_RXEDGIE_SHIFT), /*!< RX Active Edge. */
    LPUART_INT_TX_DATA_REG_EMPTY = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) \
                                   | (uint32_t)LPUART_CTRL_TIE_SHIFT),     /*!< Transmit data register empty. */
    LPUART_INT_TX_COMPLETE       = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) \
                                   | (uint32_t)LPUART_CTRL_TCIE_SHIFT),    /*!< Transmission complete. */
    LPUART_INT_RX_DATA_REG_FULL  = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) \
                                   | (uint32_t)LPUART_CTRL_RIE_SHIFT),     /*!< Receiver data register full. */
    LPUART_INT_IDLE_LINE         = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) \
                                   | (uint32_t)LPUART_CTRL_ILIE_SHIFT),    /*!< Idle line. */
    LPUART_INT_RX_OVERRUN        = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) \
                                   | (uint32_t)LPUART_CTRL_ORIE_SHIFT),    /*!< Receiver Overrun. */
    LPUART_INT_NOISE_ERR_FLAG    = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) \
                                   | (uint32_t)LPUART_CTRL_NEIE_SHIFT),    /*!< Noise error flag. */
    LPUART_INT_FRAME_ERR_FLAG    = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) \
                                   | (uint32_t)LPUART_CTRL_FEIE_SHIFT),    /*!< Framing error flag. */
    LPUART_INT_PARITY_ERR_FLAG   = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) \
                                   | (uint32_t)LPUART_CTRL_PEIE_SHIFT),    /*!< Parity error flag. */
#if FEATURE_LPUART_HAS_ADDRESS_MATCHING
    LPUART_INT_MATCH_ADDR_ONE    = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) \
                                   | (uint32_t)LPUART_CTRL_MA1IE_SHIFT),   /*!< Match address one flag. */
    LPUART_INT_MATCH_ADDR_TWO    = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) \
                                   | (uint32_t)LPUART_CTRL_MA2IE_SHIFT),   /*!< Match address two flag. */
#endif
#if FEATURE_LPUART_FIFO_SIZE > 0U
    LPUART_INT_FIFO_TXOF         = (((uint32_t)LPUART_FIFO_REG_ID << (uint32_t)LPUART_SHIFT) \
                                   | (uint32_t)LPUART_FIFO_TXOFE_SHIFT),    /*!< Transmitter FIFO buffer interrupt */
    LPUART_INT_FIFO_RXUF         = (((uint32_t)LPUART_FIFO_REG_ID << (uint32_t)LPUART_SHIFT) \
                                   | (uint32_t)LPUART_FIFO_RXUFE_SHIFT)     /*!< Receiver FIFO buffer interrupt */
#endif
} lpuart_interrupt_t;


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name LPUART Common Configurations
 * @{
 */

/*!
 * @brief Initializes the LPUART controller.
 *
 * This function Initializes the LPUART controller to known state.
 *
 *
 * @param base LPUART base pointer.
 */
void LPUART_HAL_Init(LPUART_Type * base);

/*!
 * @brief Enable/Disable the LPUART transmitter.
 *
 * This function enables or disables the LPUART transmitter, based on the
 * parameter received.
 *
 *
 * @param base LPUART base pointer.
 * @param enable Enable(true) or disable(false) transmitter.
 * Implements : LPUART_HAL_SetTransmitterCmd_Activity
 */
static inline void LPUART_HAL_SetTransmitterCmd(LPUART_Type * base, bool enable)
{
    base->CTRL = (base->CTRL & ~LPUART_CTRL_TE_MASK) | ((enable ? 1UL : 0UL) << LPUART_CTRL_TE_SHIFT);
    /* Wait for the register write operation to complete */
    while((bool)((base->CTRL & LPUART_CTRL_TE_MASK) != 0U) != enable) {}
}

/*!
 * @brief Gets the LPUART transmitter enabled/disabled configuration.
 *
 * This function returns true if the LPUART transmitter is enabled, or
 * false, when the transmitter is disabled.
 *
 *
 * @param base LPUART base pointer
 * @return State of LPUART transmitter enable(true)/disable(false)
 * Implements : LPUART_HAL_GetTransmitterCmd_Activity
 */
static inline bool LPUART_HAL_GetTransmitterCmd(const LPUART_Type * base)
{
    return (((base->CTRL >> LPUART_CTRL_TE_SHIFT) & 1U) > 0U);
}

/*!
 * @brief Enable/Disable the LPUART receiver.
 *
 * This function enables or disables the LPUART receiver, based on the
 * parameter received.
 *
 *
 * @param base LPUART base pointer
 * @param enable Enable(true) or disable(false) receiver.
 * Implements : LPUART_HAL_SetReceiverCmd_Activity
 */
static inline void LPUART_HAL_SetReceiverCmd(LPUART_Type * base, bool enable)
{
    base->CTRL = (base->CTRL & ~LPUART_CTRL_RE_MASK) | ((enable ? 1UL : 0UL) << LPUART_CTRL_RE_SHIFT);
    /* Wait for the register write operation to complete */
    while((bool)((base->CTRL & LPUART_CTRL_RE_MASK) != 0U) != enable) {}
}

/*!
 * @brief Gets the LPUART receiver enabled/disabled configuration.
 *
 * This function returns true if the LPUART receiver is enabled, or
 * false, when the transmitter is disabled.
 *
 *
 * @param base LPUART base pointer
 * @return State of LPUART receiver enable(true)/disable(false)
 * Implements : LPUART_HAL_GetReceiverCmd_Activity
 */
static inline bool LPUART_HAL_GetReceiverCmd(const LPUART_Type * base)
{
    return (((base->CTRL >> LPUART_CTRL_RE_SHIFT) & 1U) > 0U);
}

/*!
 * @brief Configures the LPUART baud rate.
 *
 * This function configures the LPUART baud rate.
 * In some LPUART instances the user must disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer.
 * @param sourceClockInHz LPUART source input clock in Hz.
 * @param desiredBaudRate LPUART desired baud rate.
 * @return STATUS_SUCCESS if successful or STATUS_ERROR if an error occured
 */
status_t LPUART_HAL_SetBaudRate(LPUART_Type * base,
                                uint32_t sourceClockInHz,
                                uint32_t desiredBaudRate);

/*!
 * @brief Sets the LPUART baud rate modulo divisor.
 *
 * This function sets the LPUART baud rate modulo divisor.
 *
 *
 * @param base LPUART base pointer.
 * @param baudRateDivisor The baud rate modulo division "SBR"
 * Implements : LPUART_HAL_SetBaudRateDivisor_Activity
 */
static inline void LPUART_HAL_SetBaudRateDivisor(LPUART_Type * base, uint32_t baudRateDivisor)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT((baudRateDivisor < 0x1FFFU) && (baudRateDivisor > 1U));
#endif
    uint32_t baudRegValTemp;

    baudRegValTemp = base->BAUD;
    baudRegValTemp &= ~(LPUART_BAUD_SBR_MASK);
    /* Removed the shift operation as the SBR field position is zero; shifting with 0 violates MISRA */
    baudRegValTemp |= baudRateDivisor & LPUART_BAUD_SBR_MASK;
    base->BAUD = baudRegValTemp;
}

#if FEATURE_LPUART_HAS_BAUD_RATE_OVER_SAMPLING_SUPPORT
/*!
 * @brief Sets the LPUART baud rate oversampling ratio
 *
 * This function sets the LPUART baud rate oversampling ratio.
 * (Note: Feature available on select LPUART instances used together with baud rate programming)
 * The oversampling ratio should be set between 4x (00011) and 32x (11111). Writing
 * an invalid oversampling ratio results in an error and is set to a default
 * 16x (01111) oversampling ratio.
 * Disable the transmitter/receiver before calling this function.
 *
 *
 * @param base LPUART base pointer.
 * @param overSamplingRatio The oversampling ratio "OSR"
 * Implements : LPUART_HAL_SetOversamplingRatio_Activity
 */
static inline void LPUART_HAL_SetOversamplingRatio(LPUART_Type * base, uint32_t overSamplingRatio)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(overSamplingRatio < 0x1FU);
#endif
    uint32_t baudRegValTemp;

    baudRegValTemp = base->BAUD;
    baudRegValTemp &= ~(LPUART_BAUD_OSR_MASK);
    baudRegValTemp |= LPUART_BAUD_OSR(overSamplingRatio);
    base->BAUD = baudRegValTemp;
}
#endif

#if FEATURE_LPUART_HAS_BOTH_EDGE_SAMPLING_SUPPORT
/*!
 * @brief Configures the LPUART baud rate both edge sampling
 *
 * This function configures the LPUART baud rate both edge sampling.
 * (Note: Feature available on select LPUART instances used with baud rate programming)
 * When enabled, the received data is sampled on both edges of the baud rate clock.
 * This must be set when the oversampling ratio is between 4x and 7x.
 * This function should only be called when the receiver is disabled.
 *
 *
 * @param base LPUART base pointer.
 * @param enable   Enable (1) or Disable (0) Both Edge Sampling
 * Implements : LPUART_HAL_SetBothEdgeSamplingCmd_Activity
 */
static inline void LPUART_HAL_SetBothEdgeSamplingCmd(LPUART_Type * base, bool enable)
{
    base->BAUD = (base->BAUD & ~LPUART_BAUD_BOTHEDGE_MASK) | ((enable ? 1UL : 0UL) << LPUART_BAUD_BOTHEDGE_SHIFT);
}
#endif

/*!
 * @brief Returns whether the receive data is inverted or not.
 *
 * This function returns the polarity of the receive data.
 *
 * @param base LPUART base pointer.
 * @return Rx data polarity; true: inverted, false: not inverted.
 * Implements : LPUART_HAL_GetRxDataPolarity_Activity
 */
static inline bool LPUART_HAL_GetRxDataPolarity(const LPUART_Type * base)
{
    return (((base->STAT >> LPUART_STAT_RXINV_SHIFT) & 1U) > 0U);
}

/*!
 * @brief Sets whether the recevie data is inverted or not.
 *
 * This function sets the polarity of the receive data.
 *
 * @param base LPUART base pointer.
 * @param polarity  Rx Data polarity; true: inverted, false: not inverted.
 * Implements : LPUART_HAL_SetRxDataPolarity_Activity
 */
static inline void LPUART_HAL_SetRxDataPolarity(LPUART_Type * base, bool polarity)
{
    base->STAT = (base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK & ~LPUART_STAT_RXINV_MASK)) | \
                 ((polarity ? 1UL : 0UL) << LPUART_STAT_RXINV_SHIFT);
}

/*!
 * @brief Configures the number of bits per character in the LPUART controller.
 *
 * This function configures the number of bits per character in the LPUART controller.
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer.
 * @param bitCountPerChar  Number of bits per char (8, 9, or 10, depending on the LPUART instance)
 */
void LPUART_HAL_SetBitCountPerChar(LPUART_Type * base, lpuart_bit_count_per_char_t bitCountPerChar);

/*!
 * @brief Configures parity mode in the LPUART controller.
 *
 * This function configures parity mode in the LPUART controller.
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer.
 * @param parityModeType  Parity mode (enabled, disable, odd, even - see parity_mode_t struct)
 */
void LPUART_HAL_SetParityMode(LPUART_Type * base, lpuart_parity_mode_t parityModeType);

/*!
 * @brief Configures the number of stop bits in the LPUART controller.
 *
 * This function configures the number of stop bits in the LPUART controller.
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer.
 * @param stopBitCount Number of stop bits (1 or 2 - see lpuart_stop_bit_count_t struct)
 * Implements : LPUART_HAL_SetStopBitCount_Activity
 */
static inline void LPUART_HAL_SetStopBitCount(LPUART_Type * base, lpuart_stop_bit_count_t stopBitCount)
{
    base->BAUD = (base->BAUD & ~LPUART_BAUD_SBNS_MASK) | ((uint32_t)stopBitCount << LPUART_BAUD_SBNS_SHIFT);
}

/*!
 * @brief  Get LPUART tx/rx data register address.
 *
 * This function returns LPUART tx/rx data register address.
 *
 *
 * @param base LPUART base pointer.
 * @return  LPUART tx/rx data register address.
 * Implements : LPUART_HAL_GetDataRegAddr_Activity
 */
static inline const volatile void * LPUART_HAL_GetDataRegAddr(const LPUART_Type * base)
{
    return &(base->DATA);
}

/*@}*/

/*!
 * @name LPUART Interrupts and DMA
 * @{
 */

/*!
 * @brief Configures the LPUART module interrupts.
 *
 * This function configures the LPUART module interrupts to enable/disable various interrupt sources.
 *
 *
 * @param   base LPUART module base pointer.
 * @param   intSrc LPUART interrupt configuration data.
 * @param   enable   true: enable, false: disable.
 */
void LPUART_HAL_SetIntMode(LPUART_Type * base, lpuart_interrupt_t intSrc, bool enable);

/*!
 * @brief Returns LPUART module interrupts state.
 *
 * This function returns whether a certain LPUART module interrupt is enabled or disabled.
 *
 *
 * @param   base LPUART module base pointer.
 * @param   intSrc LPUART interrupt configuration data.
 * @return  true: enable, false: disable.
 */
bool LPUART_HAL_GetIntMode(const LPUART_Type * base, lpuart_interrupt_t intSrc);

#if FEATURE_LPUART_HAS_DMA_ENABLE
/*!
 * @brief Configures DMA requests.
 *
 * This function configures DMA requests for LPUART Transmitter.
 *
 *
 * @param base LPUART base pointer
 * @param enable Transmit DMA request configuration (enable:1 /disable: 0)
 * Implements : LPUART_HAL_SetTxDmaCmd_Activity
 */
static inline void LPUART_HAL_SetTxDmaCmd(LPUART_Type * base, bool enable)
{
    base->BAUD = (base->BAUD & ~LPUART_BAUD_TDMAE_MASK) | ((enable ? 1UL : 0UL) << LPUART_BAUD_TDMAE_SHIFT);
}

/*!
 * @brief Configures DMA requests.
 *
 * This function configures DMA requests for LPUART Receiver.
 *
 *
 * @param base LPUART base pointer
 * @param enable Receive DMA request configuration (enable: 1/disable: 0)
 * Implements : LPUART_HAL_SetRxDmaCmd_Activity
 */
static inline void LPUART_HAL_SetRxDmaCmd(LPUART_Type * base, bool enable)
{
    base->BAUD = (base->BAUD & ~LPUART_BAUD_RDMAE_MASK) | ((enable ? 1UL : 0UL) << LPUART_BAUD_RDMAE_SHIFT);
}

/*!
 * @brief Gets the LPUART DMA request configuration.
 *
 * This function returns the LPUART Transmit DMA request configuration.
 *
 *
 * @param base LPUART base pointer
 * @return Transmit DMA request configuration (enable: 1/disable: 0)
 * Implements : LPUART_HAL_IsTxDmaEnabled_Activity
 */
static inline bool LPUART_HAL_IsTxDmaEnabled(const LPUART_Type * base)
{
    return (((base->BAUD >> LPUART_BAUD_TDMAE_SHIFT) & 1U) > 0U);
}

/*!
 * @brief Gets the LPUART DMA request configuration.
 *
 * This function returns the LPUART Receive DMA request configuration.
 *
 *
 * @param base LPUART base pointer
 * @return Receives the DMA request configuration (enable: 1/disable: 0).
 * Implements : LPUART_HAL_IsRxDmaEnabled_Activity
 */
static inline bool LPUART_HAL_IsRxDmaEnabled(const LPUART_Type * base)
{
    return (((base->BAUD >> LPUART_BAUD_RDMAE_SHIFT) & 1U) > 0U);
}

#endif

/*@}*/

/*!
 * @name LPUART Transfer Functions
 * @{
 */

/*!
 * @brief Sends the LPUART 8-bit character.
 *
 * This functions sends an 8-bit character.
 *
 *
 * @param base LPUART Instance
 * @param data     data to send (8-bit)
 * Implements : LPUART_HAL_Putchar_Activity
 */
static inline void LPUART_HAL_Putchar(LPUART_Type * base, uint8_t data)
{
    volatile uint8_t * dataRegBytes = (volatile uint8_t *)(&(base->DATA));
    dataRegBytes[0] = data;
}

/*!
 * @brief Sends the LPUART 9-bit character.
 *
 * This functions sends a 9-bit character.
 *
 *
 * @param base LPUART Instance
 * @param data     data to send (9-bit)
 */
void LPUART_HAL_Putchar9(LPUART_Type * base, uint16_t data);

/*!
 * @brief Sends the LPUART 10-bit character (Note: Feature available on select LPUART instances).
 *
 * This functions sends a 10-bit character.
 *
 *
 * @param base LPUART Instance
 * @param data   data to send (10-bit)
 */
void LPUART_HAL_Putchar10(LPUART_Type * base, uint16_t data);

/*!
 * @brief Gets the LPUART 8-bit character.
 *
 * This functions receives an 8-bit character.
 *
 *
 * @param base LPUART base pointer
 * @param readData Data read from receive (8-bit)
 * Implements : LPUART_HAL_Getchar_Activity
 */
static inline void LPUART_HAL_Getchar(const LPUART_Type * base, uint8_t *readData)
{
    DEV_ASSERT(readData != NULL);

    *readData = (uint8_t)base->DATA;
}

/*!
 * @brief Gets the LPUART 9-bit character.
 *
 * This functions receives a 9-bit character.
 *
 *
 * @param base LPUART base pointer
 * @param readData Data read from receive (9-bit)
 */
void LPUART_HAL_Getchar9(const LPUART_Type * base, uint16_t *readData);

/*!
 * @brief Gets the LPUART 10-bit character.
 *
 * This functions receives a 10-bit character.
 *
 *
 * @param base LPUART base pointer
 * @param readData Data read from receive (10-bit)
 */
void LPUART_HAL_Getchar10(const LPUART_Type * base, uint16_t *readData);

/*!
 * @brief Send out multiple bytes of data using polling method.
 *
 * This function only supports 8-bit transaction.
 *
 * @param   base LPUART module base pointer.
 * @param   txBuff The buffer pointer which saves the data to be sent.
 * @param   txSize Size of data to be sent in unit of byte.
 */
void LPUART_HAL_SendDataPolling(LPUART_Type * base, const uint8_t *txBuff, uint32_t txSize);

/*!
 * @brief Receive multiple bytes of data using polling method.
 *
 * This function only supports 8-bit transaction.
 *
 * @param   base LPUART module base pointer.
 * @param   rxBuff The buffer pointer which saves the data to be received.
 * @param   rxSize Size of data need to be received in unit of byte.
 * @return  STATUS_SUCCESS if the transaction is success or STATUS_UART_RX_OVERRUN if rx overrun.
 */
status_t LPUART_HAL_ReceiveDataPolling(LPUART_Type * base, uint8_t *rxBuff, uint32_t rxSize);

/*@}*/

/*!
 * @name LPUART Status Flags
 * @{
 */

/*!
 * @brief  LPUART get status flag
 *
 * This function returns the state of a status flag.
 *
 *
 * @param base LPUART base pointer
 * @param statusFlag  The status flag to query
 * @return Whether the current status flag is set(true) or not(false).
 */
bool LPUART_HAL_GetStatusFlag(const LPUART_Type * base, lpuart_status_flag_t statusFlag);

/*!
 * @brief LPUART clears an individual status flag.
 *
 * This function clears an individual status flag (see lpuart_status_flag_t for list of status bits).
 *
 *
 * @param base LPUART base pointer
 * @param statusFlag  Desired LPUART status flag to clear
 * @return STATUS_SUCCESS if successful or STATUS_ERROR if an error occured
 */
status_t LPUART_HAL_ClearStatusFlag(LPUART_Type * base, lpuart_status_flag_t statusFlag);

/*@}*/

/*!
 * @name LPUART Special Feature Configurations
 * @{
 */

/*!
 * @brief Configures the number of idle characters.
 *
 * This function Configures the number of idle characters that must be
 * received before the IDLE flag is set.
 *
 *
 * @param base LPUART base pointer
 * @param idleConfig Idle characters configuration
 * Implements : LPUART_HAL_SetIdleChar_Activity
 */
static inline void LPUART_HAL_SetIdleChar(LPUART_Type * base, lpuart_idle_char_t idleConfig)
{
    uint32_t ctrlRegValTemp;

    ctrlRegValTemp = base->CTRL;
    ctrlRegValTemp &= ~(LPUART_CTRL_IDLECFG_MASK);
    ctrlRegValTemp |= LPUART_CTRL_IDLECFG(idleConfig);
    base->CTRL = ctrlRegValTemp;
}

/*!
 * @brief Gets the number of idle characters for IDLE flag.
 *
 * This function returns the number of idle characters that must be received
 * before the IDLE flag is set.
 *
 *
 * @param base LPUART base pointer
 * @return  idle characters configuration
 * Implements : LPUART_HAL_GetIdleChar_Activity
 */
static inline lpuart_idle_char_t LPUART_HAL_GetIdleChar(const LPUART_Type * base)
{
    lpuart_idle_char_t retVal = LPUART_1_IDLE_CHAR;
    uint32_t ctrlRegVal = base->CTRL;
    ctrlRegVal = (ctrlRegVal & LPUART_CTRL_IDLECFG_MASK) >> LPUART_CTRL_IDLECFG_SHIFT;

    switch (ctrlRegVal)
    {
        case 0x0U:
            retVal = LPUART_1_IDLE_CHAR;
            break;
        case 0x1U:
            retVal = LPUART_2_IDLE_CHAR;
            break;
        case 0x2U:
            retVal = LPUART_4_IDLE_CHAR;
            break;
        case 0x3U:
            retVal = LPUART_8_IDLE_CHAR;
            break;
        case 0x4U:
            retVal = LPUART_16_IDLE_CHAR;
            break;
        case 0x5U:
            retVal = LPUART_32_IDLE_CHAR;
            break;
        case 0x6U:
            retVal = LPUART_64_IDLE_CHAR;
            break;
        case 0x7U:
            retVal = LPUART_128_IDLE_CHAR;
            break;
        default:
            /* IDLECFG field is three bits wide, so the result is between 0 and 7 */
            break;
    }

    return retVal;
}

/*!
 * @brief Checks whether the current data word was received with noise.
 *
 * This function returns whether the current data word was received with noise.
 *
 *
 * @param base LPUART base pointer.
 * @return The status of the NOISY bit in the LPUART extended data register
 * Implements : LPUART_HAL_IsCurrentDataWithNoise_Activity
 */
static inline bool LPUART_HAL_IsCurrentDataWithNoise(const LPUART_Type * base)
{
    return (((base->DATA >> LPUART_DATA_NOISY_SHIFT) & 1U) > 0U);
}

/*!
 * @brief Checks whether the current data word was received with frame error.
 *
 * This function returns whether the current data word was received with frame error.
 *
 *
 * @param base LPUART base pointer
 * @return The status of the FRETSC bit in the LPUART extended data register
 * Implements : LPUART_HAL_IsCurrentDataWithFrameError_Activity
 */
static inline bool LPUART_HAL_IsCurrentDataWithFrameError(const LPUART_Type * base)
{
    return (((base->DATA >> LPUART_DATA_FRETSC_SHIFT) & 1U) > 0U);
}

/*!
 * @brief Indicates a special character is to be transmitted.
 *
 * This function sets this bit to indicate a break or idle character is to be transmitted
 * instead of the contents in DATA[T9:T0].
 *
 *
 * @param base LPUART base pointer
 * @param specialChar -> 0 - break character, 1 - idle character
 * Implements : LPUART_HAL_SetTxSpecialChar_Activity
 */
static inline void LPUART_HAL_SetTxSpecialChar(LPUART_Type * base, uint8_t specialChar)
{
    base->DATA = ((base->DATA & ~LPUART_DATA_R9T9_MASK) | LPUART_DATA_R9T9(specialChar)) | LPUART_DATA_FRETSC_MASK;
}

/*!
 * @brief Checks whether the current data word was received with parity error.
 *
 * This function returns whether the current data word was received with parity error.
 *
 *
 * @param base LPUART base pointer
 * @return The status of the PARITYE bit in the LPUART extended data register
 * Implements : LPUART_HAL_IsCurrentDataWithParityError_Activity
 */
static inline bool LPUART_HAL_IsCurrentDataWithParityError(const LPUART_Type * base)
{
    return (((base->DATA >> LPUART_DATA_PARITYE_SHIFT) & 1U) > 0U);
}

/*!
 * @brief Checks whether the receive buffer is empty.
 *
 * This function returns whether the receive buffer is empty.
 *
 *
 * @param base LPUART base pointer
 * @return TRUE if the receive-buffer is empty, else FALSE.
 * Implements : LPUART_HAL_IsReceiveBufferEmpty_Activity
 */
static inline bool LPUART_HAL_IsReceiveBufferEmpty(const LPUART_Type * base)
{
    return (((base->DATA >> LPUART_DATA_RXEMPT_SHIFT) & 1U) > 0U);
}

/*!
 * @brief Checks whether the previous BUS state was idle before this byte is received.
 *
 * This function returns whether the previous BUS state was idle before this byte is received.
 *
 *
 * @param base LPUART base pointer
 * @return TRUE if the previous BUS state was IDLE, else FALSE.
 * Implements : LPUART_HAL_WasPreviousReceiverStateIdle_Activity
 */
static inline bool LPUART_HAL_WasPreviousReceiverStateIdle(const LPUART_Type * base)
{
    return (((base->DATA >> LPUART_DATA_IDLINE_SHIFT) & 1U) > 0U);
}

/*!
 * @brief Configures the LPUART operation in wait mode (operates or stops operations in wait mode).
 *
 * This function configures the LPUART operation in wait mode (operates or stops operations in wait mode).
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer
 * @param mode     LPUART wait mode operation - operates or stops to operate in wait mode.
 * Implements : LPUART_HAL_SetWaitModeOperation_Activity
 */
static inline void  LPUART_HAL_SetWaitModeOperation(LPUART_Type * base, lpuart_operation_config_t mode)
{
    /* In CPU wait mode: 0 - lpuart clocks continue to run; 1 - lpuart clocks freeze */
    base->CTRL = (base->CTRL & ~LPUART_CTRL_DOZEEN_MASK) | ((uint32_t)mode << LPUART_CTRL_DOZEEN_SHIFT);
}

/*!
 * @brief Gets the LPUART operation in wait mode.
 *
 * This function returns the LPUART operation in wait mode
 * (operates or stops operations in wait mode).
 *
 *
 * @param base LPUART base pointer
 * @return LPUART wait mode operation configuration - LPUART_OPERATES or LPUART_STOPS in wait mode
 * Implements : LPUART_HAL_GetWaitModeOperation_Activity
 */
static inline lpuart_operation_config_t LPUART_HAL_GetWaitModeOperation(const LPUART_Type * base)
{
    /* In CPU wait mode: 0 - lpuart clocks continue to run; 1 - lpuart clocks freeze  */
    return (lpuart_operation_config_t)((((base->CTRL >> LPUART_CTRL_DOZEEN_SHIFT) & 1U) > 0U) ? LPUART_STOPS : LPUART_OPERATES);
}

/*!
 * @brief Configures the LPUART loopback operation (enable/disable loopback operation)
 *
 * This function configures the LPUART loopback operation (enable/disable loopback operation).
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer
 * @param enable   LPUART loopback mode - disabled (0) or enabled (1)
 */
void LPUART_HAL_SetLoopbackCmd(LPUART_Type * base, bool enable);

/*!
 * @brief Configures the LPUART single-wire operation (enable/disable single-wire mode).
 *
 * This function configures the LPUART single-wire operation (enable/disable single-wire mode).
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer
 * @param enable   LPUART loopback mode - disabled (0) or enabled (1)
 */
void LPUART_HAL_SetSingleWireCmd(LPUART_Type * base, bool enable);

/*!
 * @brief Configures the LPUART transmit direction while in single-wire mode.
 *
 * This function configures the LPUART transmit direction while in single-wire mode.
 *
 *
 * @param base LPUART base pointer
 * @param direction LPUART single-wire transmit direction - input or output
 * Implements : LPUART_HAL_SetTxdirInSinglewireMode_Activity
 */
static inline void LPUART_HAL_SetTxdirInSinglewireMode(LPUART_Type * base,
                                                       lpuart_singlewire_txdir_t direction)
{
    base->CTRL = (base->CTRL & ~LPUART_CTRL_TXDIR_MASK) | ((uint32_t)direction << LPUART_CTRL_TXDIR_SHIFT);
}

/*!
 * @brief  Places the LPUART receiver in standby mode.
 *
 * This function places the LPUART receiver in standby mode.
 *
 *
 * @param base LPUART base pointer
 * @return STATUS_SUCCESS if successful or STATUS_ERROR if an error occured
 */
status_t LPUART_HAL_SetReceiverInStandbyMode(LPUART_Type * base);

/*!
 * @brief  Places the LPUART receiver in a normal mode.
 *
 * This function places the LPUART receiver in a normal mode (disable standby mode operation).
 *
 *
 * @param base LPUART base pointer
 * Implements : LPUART_HAL_PutReceiverInNormalMode_Activity
 */
static inline void LPUART_HAL_PutReceiverInNormalMode(LPUART_Type * base)
{
    base->CTRL &= ~LPUART_CTRL_RWU_MASK;
}

/*!
 * @brief  Checks whether the LPUART receiver is in a standby mode.
 *
 * This function returns whether the LPUART receiver is in a standby mode.
 *
 *
 * @param base LPUART base pointer
 * @return LPUART in normal more (0) or standby (1)
 * Implements : LPUART_HAL_IsReceiverInStandby_Activity
 */
static inline bool LPUART_HAL_IsReceiverInStandby(const LPUART_Type * base)
{
    return (((base->CTRL >> LPUART_CTRL_RWU_SHIFT) & 1U) > 0U);
}

/*!
 * @brief  Sets the LPUART receiver wakeup method from standby mode.
 *
 * This function sets the LPUART receiver wakeup method (idle line or addr-mark)
 * from standby mode.
 *
 *
 * @param base LPUART base pointer
 * @param method   LPUART wakeup method: 0 - Idle-line wake (default), 1 - addr-mark wake
 * Implements : LPUART_HAL_SetReceiverWakeupMode_Activity
 */
static inline void LPUART_HAL_SetReceiverWakeupMode(LPUART_Type * base,
                                                    lpuart_wakeup_method_t method)
{
    base->CTRL = (base->CTRL & ~LPUART_CTRL_WAKE_MASK) | ((uint32_t)method << LPUART_CTRL_WAKE_SHIFT);
}

/*!
 * @brief  Gets the LPUART receiver wakeup method from standby mode.
 *
 * This function returns the LPUART receiver wakeup method (idle line or addr-mark)
 * from standby mode.
 *
 *
 * @param base LPUART base pointer
 * @return  LPUART wakeup method: LPUART_IDLE_LINE_WAKE: 0 - Idle-line wake (default),
 *          LPUART_ADDR_MARK_WAKE: 1 - addr-mark wake
 * Implements : LPUART_HAL_GetReceiverWakeupMode_Activity
 */
static inline lpuart_wakeup_method_t LPUART_HAL_GetReceiverWakeupMode(const LPUART_Type * base)
{
    return (lpuart_wakeup_method_t)((((base->CTRL >> LPUART_CTRL_WAKE_SHIFT) & 1U) > 0U) ? LPUART_ADDR_MARK_WAKE : LPUART_IDLE_LINE_WAKE);
}

/*!
 * @brief  LPUART idle-line detect operation configuration.
 *
 * This function configures idle-line detect operation configuration
 * (idle line bit-count start and wake up affect on IDLE status bit).
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer
 * @param config   LPUART configuration data for idle line detect operation
 */
void LPUART_HAL_SetIdleLineDetect(LPUART_Type * base,
                                  const lpuart_idle_line_config_t *config);

/*!
 * @brief  LPUART break character transmit length configuration
 *
 * This function configures the break char length.
 * In some LPUART instances, the user should disable the transmitter before calling
 * this function. Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer
 * @param length   LPUART break character length setting: 0 - minimum 10-bit times (default),
 *                   1 - minimum 13-bit times
 * Implements : LPUART_HAL_SetBreakCharTransmitLength_Activity
 */
static inline void LPUART_HAL_SetBreakCharTransmitLength(LPUART_Type * base,
                                                         lpuart_break_char_length_t length)
{
    base->STAT = (base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK & ~LPUART_STAT_BRK13_MASK)) | \
                 ((uint32_t)length << LPUART_STAT_BRK13_SHIFT);
}

/*!
 * @brief  LPUART break character detect length configuration
 *
 * This function sets the LPUART detectable break character length.
 *
 *
 * @param base LPUART base pointer
 * @param length  LPUART break character length setting: 0 - minimum 10-bit times (default),
 *                  1 - minimum 13-bit times
 * Implements : LPUART_HAL_SetBreakCharDetectLength_Activity
 */
static inline void LPUART_HAL_SetBreakCharDetectLength(LPUART_Type * base,
                                                       lpuart_break_char_length_t length)
{
    base->STAT = (base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK & ~LPUART_STAT_LBKDE_MASK)) | \
                 ((uint32_t)length << LPUART_STAT_LBKDE_SHIFT);
}

/*!
 * @brief  LPUART transmit sends break character configuration.
 *
 * This function sets break character transmission in queue mode.
 *
 *
 * @param base LPUART base pointer
 * Implements : LPUART_HAL_QueueBreakField_Activity
 */
static inline void LPUART_HAL_QueueBreakField(LPUART_Type * base)
{
    base->DATA = LPUART_DATA_FRETSC(1U);
}

/*!
 * @brief Configures match address mode control.
 *
 * This function configures match address mode control.
 *
 *
 * @param base LPUART base pointer
 * @param config MATCFG: Configures the match addressing mode used.
 * Implements : LPUART_HAL_SetMatchAddressMode_Activity
 */
static inline void LPUART_HAL_SetMatchAddressMode(LPUART_Type * base, lpuart_match_config_t config)
{
    uint32_t baudRegValTemp;

    baudRegValTemp = base->BAUD;
    baudRegValTemp &= ~(LPUART_BAUD_MATCFG_MASK);
    baudRegValTemp |= LPUART_BAUD_MATCFG(config);
    base->BAUD = baudRegValTemp;
}

/*!
 * @brief Configures address match register 1.
 *
 * This function configures address match register 1.
 * The MAEN bit must be cleared before configuring MA value, so the enable/disable
 * and set value must be included inside one function.
 *
 * @param base LPUART base pointer
 * @param enable Match address model enable (true)/disable (false)
 * @param value Match address value to program into match address register 1
 */
void LPUART_HAL_SetMatchAddressReg1(LPUART_Type * base, bool enable, uint8_t value);

/*!
 * @brief Configures address match register 2.
 *
 * This function configures address match register 2.
 * The MAEN bit must be cleared before configuring MA value, so the enable/disable
 * and set value must be included inside one function.
 *
 * @param base LPUART base pointer
 * @param enable Match address model enable (true)/disable (false)
 * @param value Match address value to program into match address register 2
 */
void LPUART_HAL_SetMatchAddressReg2(LPUART_Type * base, bool enable, uint8_t value);

/*!
 * @brief LPUART sends the MSB first configuration
 *
 * This function configures whether MSB is sent first.
 * Note: In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer
 * @param enable  false - LSB (default, disabled), true - MSB (enabled)
 * Implements : LPUART_HAL_SetSendMsbFirstCmd_Activity
 */
static inline void LPUART_HAL_SetSendMsbFirstCmd(LPUART_Type * base, bool enable)
{
    base->STAT = (base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK & ~LPUART_STAT_MSBF_MASK)) | \
                 ((enable ? 1UL : 0UL) << LPUART_STAT_MSBF_SHIFT);
}

/*!
 * @brief  LPUART enable/disable re-sync of received data configuration.
 *
 * This function enables or disables re-sync of received data, based on the
 * parameter received.
 *
 *
 * @param base LPUART base pointer
 * @param enable  re-sync of received data word configuration:
 *                true - re-sync of received data word (default)
 *                false - disable the re-sync
 * Implements : LPUART_HAL_SetReceiveResyncCmd_Activity
 */
static inline void LPUART_HAL_SetReceiveResyncCmd(LPUART_Type * base, bool enable)
{
    /* When disabled, the resynchronization of the received data word when a data
     * one followed by data zero transition is detected. This bit should only be
     * changed when the receiver is disabled. */
    base->BAUD = (base->BAUD & ~LPUART_BAUD_RESYNCDIS_MASK) | ((enable ? 1UL : 0UL) << LPUART_BAUD_RESYNCDIS_SHIFT);
}

#if FEATURE_LPUART_HAS_MODEM_SUPPORT
/*!
 * @brief  Transmits the CTS source configuration.
 *
 * This function transmits the CTS source configuration.
 *
 *
 * @param base LPUART base pointer
 * @param source   LPUART CTS source
 * Implements : LPUART_HAL_SetCtsSource_Activity
 */
static inline void LPUART_HAL_SetCtsSource(LPUART_Type * base,
                                           lpuart_cts_source_t source)
{
    base->MODIR = (base->MODIR & ~LPUART_MODIR_TXCTSSRC_MASK) | ((uint32_t)source << LPUART_MODIR_TXCTSSRC_SHIFT);
}

/*!
 * @brief  Transmits the CTS configuration.
 *
 * This function transmits the CTS configuration.
 * Note: configures if the CTS state is checked at the start of each character
 * or only when the transmitter is idle.
 *
 *
 * @param base LPUART base pointer
 * @param mode     LPUART CTS configuration
 * Implements : LPUART_HAL_SetCtsMode_Activity
 */
static inline void LPUART_HAL_SetCtsMode(LPUART_Type * base, lpuart_cts_config_t mode)
{
    base->MODIR = (base->MODIR & ~LPUART_MODIR_TXCTSC_MASK) | ((uint32_t)mode << LPUART_MODIR_TXCTSC_SHIFT);
}

/*!
 * @brief Enable/Disable the transmitter clear-to-send.
 *
 * This function controls the transmitter clear-to-send, based on the parameter
 * received.
 *
 *
 * @param base LPUART base pointer
 * @param enable  disable(0)/enable(1) transmitter CTS.
 * Implements : LPUART_HAL_SetTxCtsCmd_Activity
 */
static inline void LPUART_HAL_SetTxCtsCmd(LPUART_Type * base, bool enable)
{
    base->MODIR = (base->MODIR & ~LPUART_MODIR_TXCTSE_MASK) | ((enable ? 1U : 0U) << LPUART_MODIR_TXCTSE_SHIFT);
}

/*!
 * @brief  Enable/Disable the receiver request-to-send.
 *
 * This function enables  or disables the receiver request-to-send, based
 * on the parameter received.
 * Note: do not enable both Receiver RTS (RXRTSE) and Transmit RTS (TXRTSE).
 *
 *
 * @param base LPUART base pointer
 * @param enable  disable(0)/enable(1) receiver RTS.
 * Implements : LPUART_HAL_SetRxRtsCmd_Activity
 */
static inline void LPUART_HAL_SetRxRtsCmd(LPUART_Type * base, bool enable)
{
    base->MODIR = (base->MODIR & ~LPUART_MODIR_RXRTSE_MASK) | ((enable ? 1U : 0U) << LPUART_MODIR_RXRTSE_SHIFT);
}

/*!
 * @brief  Enable/Disable the transmitter request-to-send.
 *
 * This function enables  or disables the transmitter request-to-send, based
 * on the parameter received.
 * Note: do not enable both Receiver RTS (RXRTSE) and Transmit RTS (TXRTSE).
 *
 *
 * @param base LPUART base pointer
 * @param enable  disable(0)/enable(1) transmitter RTS.
 * Implements : LPUART_HAL_SetTxRtsCmd_Activity
 */
static inline void LPUART_HAL_SetTxRtsCmd(LPUART_Type * base, bool enable)
{
    base->MODIR = (base->MODIR & ~LPUART_MODIR_TXRTSE_MASK) | ((enable ? 1U : 0U) << LPUART_MODIR_TXRTSE_SHIFT);
}

/*!
 * @brief Configures the transmitter RTS polarity.
 *
 * This function configures the transmitter RTS polarity.
 *
 *
 * @param base LPUART base pointer
 * @param polarity Settings to choose RTS polarity (0=active low, 1=active high)
 * Implements : LPUART_HAL_SetTxRtsPolarityMode_Activity
 */
static inline void LPUART_HAL_SetTxRtsPolarityMode(LPUART_Type * base, bool polarity)
{
    base->MODIR = (base->MODIR & ~LPUART_MODIR_TXRTSPOL_MASK) | ((polarity ? 1U : 0U) << LPUART_MODIR_TXRTSPOL_SHIFT);
}

#endif  /* FEATURE_LPUART_HAS_MODEM_SUPPORT */

#if FEATURE_LPUART_FIFO_SIZE > 0U
/*!
 * @brief  Enable/Disable the transmitter FIFO.
 *
 * This function enables or disables the transmitter FIFO structure, based
 * on the parameter received.
 * Note: The size of the FIFO structure is indicated by TXFIFOSIZE.
 *
 *
 * @param base LPUART base pointer
 * @param enable  disable(0)/enable(1) transmitter FIFO.
 * Implements : LPUART_HAL_SetTxFIFOCmd_Activity
 */
static inline void LPUART_HAL_SetTxFIFOCmd(LPUART_Type * base, bool enable)
{
    base->FIFO = (base->FIFO & (~FEATURE_LPUART_FIFO_REG_FLAGS_MASK & ~LPUART_FIFO_TXFE_MASK)) | \
                 ((enable ? 1U : 0U) << LPUART_FIFO_TXFE_SHIFT);
}

/*!
 * @brief  Enable/Disable the receiver FIFO.
 *
 * This function enables or disables the receiver FIFO structure, based
 * on the parameter received.
 * Note: The size of the FIFO structure is indicated by RXFIFOSIZE.
 *
 *
 * @param base LPUART base pointer
 * @param enable  disable(0)/enable(1) receiver FIFO.
 * Implements : LPUART_HAL_SetRxFIFOCmd_Activity
 */
static inline void LPUART_HAL_SetRxFIFOCmd(LPUART_Type * base, bool enable)
{
    base->FIFO = (base->FIFO & (~FEATURE_LPUART_FIFO_REG_FLAGS_MASK & ~LPUART_FIFO_RXFE_MASK)) | \
                 ((enable ? 1U : 0U) << LPUART_FIFO_RXFE_SHIFT);
}

/*!
 * @brief Enables the assertion of RDRF when the receiver is idle.
 *
 * This function enables the assertion of RDRF when the receiver is idle for
 * a number of idle characters and the FIFO is not empty.
 *
 *
 * @param base LPUART base pointer.
 * @param duration  The number of characters the receiver must be empty before RDRF assertion
 *        0 - disabled, >0 - rx must be idle for 2^(duration-1) characters before RDRF assertion
 *
 * Implements : LPUART_HAL_SetRxIdleEmptyDuration_Activity
 */
static inline void LPUART_HAL_SetRxIdleEmptyDuration(LPUART_Type * base, uint8_t duration)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(duration < ((uint8_t)(1U << LPUART_FIFO_RXIDEN_WIDTH)));
#endif
    uint32_t fifoRegValTemp;

    fifoRegValTemp = base->FIFO;
    fifoRegValTemp &= (~FEATURE_LPUART_FIFO_REG_FLAGS_MASK & ~LPUART_FIFO_RXIDEN_MASK);
    fifoRegValTemp |= LPUART_FIFO_RXIDEN(duration);
    base->FIFO = fifoRegValTemp;
}

/*!
 * @brief  Flush tx FIFO buffer.
 *
 * This function flushes the tx FIFO buffer.
 * Note: This does not affect data that is in the transmit shift register.
 *
 *
 * @param base LPUART base pointer
 * Implements : LPUART_HAL_FlushTxFifoBuffer_Activity
 */
static inline void LPUART_HAL_FlushTxFifoBuffer(LPUART_Type * base)
{
    base->FIFO |= LPUART_FIFO_TXFLUSH_MASK;
}

/*!
 * @brief  Flush rx FIFO buffer.
 *
 * This function flushes the rx FIFO buffer.
 * Note: This does not affect data that is in the receive shift register.
 *
 *
 * @param base LPUART base pointer
 * Implements : LPUART_HAL_FlushRxFifoBuffer_Activity
 */
static inline void LPUART_HAL_FlushRxFifoBuffer(LPUART_Type * base)
{
    base->FIFO |= LPUART_FIFO_RXFLUSH_MASK;
}

/*!
 * @brief  Sets the tx watermark.
 *
 * This function sets the tx FIFO watermark.
 * When the number of datawords in the transmit FIFO/buffer is equal to or less than the value in
 * this register field, an interrupt or a DMA request is generated.
 * Note: For proper operation, the value in TXWATER must be set to be less than the
 * transmit FIFO/buffer size and greater than 0.
 *
 *
 * @param base LPUART base pointer
 * @param txWater Tx FIFO Watermark
 * Implements : LPUART_HAL_SetTxWatermark_Activity
 */
static inline void LPUART_HAL_SetTxWatermark(LPUART_Type * base, uint8_t txWater)
{
#ifdef DEV_ERROR_DETECT
    uint8_t txFifoSize = ((uint8_t)((base->FIFO & LPUART_FIFO_TXFIFOSIZE_MASK) >> LPUART_FIFO_TXFIFOSIZE_SHIFT));
    DEV_ASSERT(txFifoSize > 0U);
    DEV_ASSERT(txWater < ((uint8_t)(1U << (txFifoSize + 1U))));
#endif
    uint32_t waterRegValTemp;

    waterRegValTemp = base->WATER;
    waterRegValTemp &= ~(LPUART_WATER_TXWATER_MASK);
    waterRegValTemp |= LPUART_WATER_TXWATER(txWater);
    base->WATER = waterRegValTemp;
}

/*!
 * @brief  Sets the rx watermark.
 *
 * This function sets the rx FIFO watermark.
 * When the number of datawords in the receive FIFO/buffer is greater than the value in
 * this register field, an interrupt or a DMA request is generated.
 * Note: For proper operation, the value in RXWATER must be set to be less than the
 * receive FIFO/buffer size and greater than 0.
 *
 *
 * @param base LPUART base pointer
 * @param rxWater Rx FIFO Watermark
 * Implements : LPUART_HAL_SetRxWatermark_Activity
 */
static inline void LPUART_HAL_SetRxWatermark(LPUART_Type * base, uint8_t rxWater)
{
#ifdef DEV_ERROR_DETECT
    uint8_t rxFifoSize = ((uint8_t)((base->FIFO & LPUART_FIFO_RXFIFOSIZE_MASK) >> LPUART_FIFO_RXFIFOSIZE_SHIFT));
    DEV_ASSERT(rxFifoSize > 0U);
    DEV_ASSERT(rxWater < ((uint8_t)(1U << (rxFifoSize + 1U))));
#endif
    uint32_t waterRegValTemp;

    waterRegValTemp = base->WATER;
    waterRegValTemp &= ~(LPUART_WATER_RXWATER_MASK);
    waterRegValTemp |= LPUART_WATER_RXWATER(rxWater);
    base->WATER = waterRegValTemp;
}

#endif /* FEATURE_LPUART_FIFO_SIZE */

#if FEATURE_LPUART_HAS_IR_SUPPORT
/*!
 * @brief  Configures the LPUART infrared operation.
 *
 * This function configures the LPUART infrared operation.
 *
 *
 * @param base LPUART base pointer
 * @param enable    Enable (1) or disable (0) the infrared operation
 * @param pulseWidth The transmit narrow pulse width of type lpuart_ir_tx_pulsewidth_t
 */
void LPUART_HAL_SetInfrared(LPUART_Type * base, bool enable,
                            lpuart_ir_tx_pulsewidth_t pulseWidth);
#endif  /* FEATURE_LPUART_HAS_IR_SUPPORT */

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* LPUART_HAL_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/

