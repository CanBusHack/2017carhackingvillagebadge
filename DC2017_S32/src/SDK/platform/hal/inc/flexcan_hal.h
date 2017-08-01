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

/*!
 * @file flexcan_hal.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, global macro not referenced
 * There are some global macros used for accessing different fields of CAN frames
 * which might also be useful to the user.
 */

#ifndef FLEXCAN_HAL_H
#define FLEXCAN_HAL_H

#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "device_registers.h"
#include "status.h"

/*!
 * @defgroup flexcan_hal FlexCAN HAL
 * @ingroup flexcan
 * @addtogroup flexcan_hal
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief FlexCAN operation modes
 * Implements : flexcan_operation_modes_t_Class
 */
typedef enum {
    FLEXCAN_NORMAL_MODE,        /*!< Normal mode or user mode @internal gui name="Normal" */
    FLEXCAN_LISTEN_ONLY_MODE,   /*!< Listen-only mode @internal gui name="Listen-only" */
    FLEXCAN_LOOPBACK_MODE,      /*!< Loop-back mode @internal gui name="Loop back" */
    FLEXCAN_FREEZE_MODE,        /*!< Freeze mode @internal gui name="Freeze" */
    FLEXCAN_DISABLE_MODE        /*!< Module disable mode @internal gui name="Disabled" */
} flexcan_operation_modes_t;

/*! @brief FlexCAN message buffer CODE for Rx buffers*/
enum {
    FLEXCAN_RX_INACTIVE  = 0x0, /*!< MB is not active.*/
    FLEXCAN_RX_FULL      = 0x2, /*!< MB is full.*/
    FLEXCAN_RX_EMPTY     = 0x4, /*!< MB is active and empty.*/
    FLEXCAN_RX_OVERRUN   = 0x6, /*!< MB is overwritten into a full buffer.*/
    FLEXCAN_RX_BUSY      = 0x8, /*!< FlexCAN is updating the contents of the MB.*/
                                /*!  The CPU must not access the MB.*/
    FLEXCAN_RX_RANSWER   = 0xA, /*!< A frame was configured to recognize a Remote Request Frame*/
                                /*!  and transmit a Response Frame in return.*/
    FLEXCAN_RX_NOT_USED   = 0xF /*!< Not used*/
};

/*! @brief FlexCAN message buffer CODE FOR Tx buffers*/
enum {
    FLEXCAN_TX_INACTIVE  = 0x08, /*!< MB is not active.*/
    FLEXCAN_TX_ABORT     = 0x09, /*!< MB is aborted.*/
    FLEXCAN_TX_DATA      = 0x0C, /*!< MB is a TX Data Frame(MB RTR must be 0).*/
    FLEXCAN_TX_REMOTE    = 0x1C, /*!< MB is a TX Remote Request Frame (MB RTR must be 1).*/
    FLEXCAN_TX_TANSWER   = 0x0E, /*!< MB is a TX Response Request Frame from.*/
                                 /*!  an incoming Remote Request Frame.*/
    FLEXCAN_TX_NOT_USED   = 0xF  /*!< Not used*/
};

/*! @brief FlexCAN message buffer transmission types*/
enum {
    FLEXCAN_MB_STATUS_TYPE_TX,          /*!< Transmit MB*/
    FLEXCAN_MB_STATUS_TYPE_TX_REMOTE,   /*!< Transmit remote request MB*/
    FLEXCAN_MB_STATUS_TYPE_RX,          /*!< Receive MB*/
    FLEXCAN_MB_STATUS_TYPE_RX_REMOTE,   /*!< Receive remote request MB*/
    FLEXCAN_MB_STATUS_TYPE_RX_TX_REMOTE /*!< FlexCAN remote frame receives remote request and*/
                                        /*!  transmits MB.*/
};

/*! @brief FlexCAN payload sizes
 * Implements : flexcan_fd_payload_size_t_Class
 */
typedef enum {
    FLEXCAN_PAYLOAD_SIZE_8 = 0,  /*!< FlexCAN message buffer payload size in bytes*/
    FLEXCAN_PAYLOAD_SIZE_16 ,    /*!< FlexCAN message buffer payload size in bytes*/
    FLEXCAN_PAYLOAD_SIZE_32 ,    /*!< FlexCAN message buffer payload size in bytes*/
    FLEXCAN_PAYLOAD_SIZE_64      /*!< FlexCAN message buffer payload size in bytes*/
} flexcan_fd_payload_size_t;

/*! @brief ID formats for RxFIFO
 * Implements : flexcan_rx_fifo_id_element_format_t_Class
 */
typedef enum {
    FLEXCAN_RX_FIFO_ID_FORMAT_A, /*!< One full ID (standard and extended) per ID Filter Table*/
                                 /*!  element.*/
    FLEXCAN_RX_FIFO_ID_FORMAT_B, /*!< Two full standard IDs or two partial 14-bit (standard and*/
                                 /*!  extended) IDs per ID Filter Table element.*/
    FLEXCAN_RX_FIFO_ID_FORMAT_C, /*!< Four partial 8-bit Standard IDs per ID Filter Table*/
                                 /*!  element.*/
    FLEXCAN_RX_FIFO_ID_FORMAT_D  /*!< All frames rejected.*/
} flexcan_rx_fifo_id_element_format_t;

/*! @brief FlexCAN RX FIFO ID filter table structure
 * Implements : flexcan_id_table_t_Class
 */
typedef struct {
    bool isRemoteFrame;      /*!< Remote frame*/
    bool isExtendedFrame;    /*!< Extended frame*/
    uint32_t *idFilter;      /*!< Rx FIFO ID filter elements*/
} flexcan_id_table_t;

/*! @brief FlexCAN RX mask type.
 * Implements : flexcan_rx_mask_type_t_Class
 */
typedef enum {
    FLEXCAN_RX_MASK_GLOBAL,      /*!< Rx global mask*/
    FLEXCAN_RX_MASK_INDIVIDUAL   /*!< Rx individual mask*/
} flexcan_rx_mask_type_t;

/*! @brief FlexCAN Message Buffer ID type
 * Implements : flexcan_msgbuff_id_type_t_Class
 */
typedef enum {
    FLEXCAN_MSG_ID_STD,         /*!< Standard ID*/
    FLEXCAN_MSG_ID_EXT          /*!< Extended ID*/
} flexcan_msgbuff_id_type_t;

#if FEATURE_CAN_HAS_PE_CLKSRC_SELECT
/*! @brief FlexCAN clock source
 * Implements : flexcan_clk_source_t_Class
 */
typedef enum {
    FLEXCAN_CLK_SOURCE_SOSCDIV2,  /*!< Clock divider 2 for System OSC */
    FLEXCAN_CLK_SOURCE_SYS        /*!< Sys clock */
} flexcan_clk_source_t;
#endif

/*! @brief FlexCAN error interrupt types
 * Implements : flexcan_int_type_t_Class
 */
typedef enum {
    FLEXCAN_INT_RX_WARNING = CAN_CTRL1_RWRNMSK_MASK,     /*!< RX warning interrupt*/
    FLEXCAN_INT_TX_WARNING = CAN_CTRL1_TWRNMSK_MASK,     /*!< TX warning interrupt*/
    FLEXCAN_INT_ERR = CAN_CTRL1_ERRMSK_MASK,             /*!< Error interrupt*/
    FLEXCAN_INT_BUSOFF = CAN_CTRL1_BOFFMSK_MASK,         /*!< Bus off interrupt*/
} flexcan_int_type_t;

/*! @brief FlexCAN bus error counters
 * Implements : flexcan_buserr_counter_t_Class
 */
typedef struct {
    uint16_t txerr;           /*!< Transmit error counter*/
    uint16_t rxerr;           /*!< Receive error counter*/
} flexcan_buserr_counter_t;

/*! @brief FlexCAN Message Buffer code and status for transmit and receive
 * Implements : flexcan_msgbuff_code_status_t_Class
 */
typedef struct {
    uint32_t code;                        /*!< MB code for TX or RX buffers.*/
                                          /*! Defined by flexcan_mb_code_rx_t and flexcan_mb_code_tx_t */
    flexcan_msgbuff_id_type_t msgIdType;  /*!< Type of message ID (standard or extended)*/
    uint32_t dataLen;                     /*!< Length of Data in Bytes*/
    bool fd_enable;
    uint8_t fd_padding;
    bool enable_brs;                   /* Enable bit rate switch*/
} flexcan_msgbuff_code_status_t;

/*! @brief FlexCAN message buffer structure
 * Implements : flexcan_msgbuff_t_Class
 */
typedef struct {
    uint32_t cs;                        /*!< Code and Status*/
    uint32_t msgId;                     /*!< Message Buffer ID*/
    uint8_t data[64];                   /*!< Bytes of the FlexCAN message*/
    uint8_t dataLen;                    /*!< Length of data in bytes */
} flexcan_msgbuff_t;

/*! @brief FlexCAN timing related structures
 * Implements : flexcan_time_segment_t_Class
 */
typedef struct {
    uint32_t propSeg;         /*!< Propagation segment*/
    uint32_t phaseSeg1;       /*!< Phase segment 1*/
    uint32_t phaseSeg2;       /*!< Phase segment 2*/
    uint32_t preDivider;      /*!< Clock pre divider*/
    uint32_t rJumpwidth;      /*!< Resync jump width*/
} flexcan_time_segment_t;

#if FEATURE_CAN_HAS_PRETENDED_NETWORKING

/*! @brief Pretended Networking ID filter */
typedef struct {
    bool extendedId;    /*!< Specifies if the ID is standard or extended. */
    bool remoteFrame;   /*!< Specifies if the frame is standard or remote. */
    uint32_t id;        /*!< Specifies the ID value. */
} flexcan_pn_id_filter_t;

/*! @brief Pretended Networking payload filter */
typedef struct {
    uint8_t dlcLow;       /*!< Specifies the lower limit of the payload size. */
    uint8_t dlcHigh;      /*!< Specifies the upper limit of the payload size. */
    uint8_t payload1[8U]; /*!< Specifies the payload to be matched (for MATCH_EXACT), the lower limit
                              (for MATCH_GEQ and MATCH_RANGE) or the upper limit (for MATCH_LEQ). */
    uint8_t payload2[8U]; /*!< Specifies the mask (for MATCH_EXACT) or the upper limit (for MATCH_RANGE). */
} flexcan_pn_payload_filter_t;

/*! @brief Pretended Networking filtering combinations */
typedef enum {
    FLEXCAN_FILTER_ID,                  /*!< Message ID filtering only */
    FLEXCAN_FILTER_ID_PAYLOAD,          /*!< Message ID filtering and payload filtering */
    FLEXCAN_FILTER_ID_NTIMES,           /*!< Message ID filtering occurring a specified number of times */
    FLEXCAN_FILTER_ID_PAYLOAD_NTIMES    /*!< Message ID filtering and payload filtering a specified number of times */
} flexcan_pn_filter_combination_t;

/*! @brief Pretended Networking matching schemes */
typedef enum {
    FLEXCAN_FILTER_MATCH_EXACT,   /*!< Match an exact target value. */
    FLEXCAN_FILTER_MATCH_GEQ,     /*!< Match greater than or equal to a specified target value. */
    FLEXCAN_FILTER_MATCH_LEQ,     /*!< Match less than or equal to a specified target value. */
    FLEXCAN_FILTER_MATCH_RANGE    /*!< Match inside a range, greater than or equal to a specified lower limit and smaller than or
                                      equal a specified upper limit. */
} flexcan_pn_filter_selection_t;

/*! @brief Pretended Networking configuration structure
 * Implements : flexcan_pn_config_t_Class
 */
typedef struct {
    bool wakeUpTimeout;         /*!< Specifies if an wake up event is triggered on timeout. */
    bool wakeUpMatch;           /*!< Specifies if an wake up event is triggered on match. */
    uint16_t numMatches;        /*!< The number of matches needed before generating a wake up event. */
    uint16_t matchTimeout;      /*!< Defines a timeout value that generates a wake up event if wakeUpTimeout is true. */
    flexcan_pn_filter_combination_t filterComb;       /*!< Defines the filtering scheme used. */
    flexcan_pn_id_filter_t idFilter1;                   /*!< The configuration of the first ID filter (match exact / lower limit / upper limit). */
    flexcan_pn_id_filter_t idFilter2;                   /*!< The configuration of the second ID filter (mask / upper limit). */
    flexcan_pn_filter_selection_t idFilterType;       /*!< Defines the ID filtering scheme. */
    flexcan_pn_filter_selection_t payloadFilterType;  /*!< Defines the payload filtering scheme. */
    flexcan_pn_payload_filter_t payloadFilter;          /*!< The configuration of the payload filter. */
} flexcan_pn_config_t;

#endif /* FEATURE_CAN_HAS_PRETENDED_NETWORKING */

#define CAN_ID_EXT_MASK                          0x3FFFFu
#define CAN_ID_EXT_SHIFT                         0
#define CAN_ID_EXT_WIDTH                         18

#define CAN_ID_STD_MASK                          0x1FFC0000u
#define CAN_ID_STD_SHIFT                         18
#define CAN_ID_STD_WIDTH                         11

#define CAN_ID_PRIO_MASK                         0xE0000000u
#define CAN_ID_PRIO_SHIFT                        29
#define CAN_ID_PRIO_WIDTH                        3
/* CS Bit Fields */
#define CAN_CS_TIME_STAMP_MASK                   0xFFFFu
#define CAN_CS_TIME_STAMP_SHIFT                  0
#define CAN_CS_TIME_STAMP_WIDTH                  16

#define CAN_CS_DLC_MASK                          0xF0000u
#define CAN_CS_DLC_SHIFT                         16
#define CAN_CS_DLC_WIDTH                         4

#define CAN_CS_RTR_MASK                          0x100000u
#define CAN_CS_RTR_SHIFT                         20
#define CAN_CS_RTR_WIDTH                         1

#define CAN_CS_IDE_MASK                          0x200000u
#define CAN_CS_IDE_SHIFT                         21
#define CAN_CS_IDE_WIDTH                         1

#define CAN_CS_SRR_MASK                          0x400000u
#define CAN_CS_SRR_SHIFT                         22
#define CAN_CS_SRR_WIDTH                         1

#define CAN_CS_CODE_MASK                         0xF000000u
#define CAN_CS_CODE_SHIFT                        24
#define CAN_CS_CODE_WIDTH                        4

#define CAN_MB_EDL_MASK                          0x80000000u
#define CAN_MB_BRS_MASK                          0x40000000u


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Configuration
 * @{
 */

/*!
 * @brief Enables FlexCAN controller.
 *
 * @param   base    The FlexCAN base address
 */
void FLEXCAN_HAL_Enable(CAN_Type * base);

/*!
 * @brief Disables FlexCAN controller.
 *
 * @param   base    The FlexCAN base address
 */
void FLEXCAN_HAL_Disable(CAN_Type * base);

#if FEATURE_CAN_HAS_PE_CLKSRC_SELECT
/*!
 * @brief Selects the clock source for FlexCAN.
 *
 * @param   base The FlexCAN base address
 * @param   clk         The FlexCAN clock source
 */
void FLEXCAN_HAL_SelectClock(CAN_Type * base, flexcan_clk_source_t clk);

/*!
 * @brief Reads the clock source for FlexCAN Protocol Engine (PE).
 *
 * @param   base The FlexCAN base address
 * @return  0: if clock source is oscillator clock, 1: if clock source is peripheral clock
 * Implements : FLEXCAN_HAL_GetClock_Activity
 */
static inline bool FLEXCAN_HAL_GetClock(const CAN_Type * base)
{
    return (((base->CTRL1 & CAN_CTRL1_CLKSRC_MASK) >> CAN_CTRL1_CLKSRC_SHIFT) != 0U);
}
#endif

/*!
 * @brief Initializes the FlexCAN controller.
 *
 * @param   base  The FlexCAN base address
 */
void FLEXCAN_HAL_Init(CAN_Type * base);

/*!
 * @brief Sets the FlexCAN time segments for setting up bit rate.
 *
 * @param   base The FlexCAN base address
 * @param   timeSeg    FlexCAN time segments, which need to be set for the bit rate.
 */
void FLEXCAN_HAL_SetTimeSegments(CAN_Type * base, const flexcan_time_segment_t *timeSeg);


/*!
 * @brief Sets the FlexCAN time segments for setting up bit rate for FD BRS.
 *
 * @param   base The FlexCAN base address
 * @param   timeSeg    FlexCAN time segments, which need to be set for the bit rate.
 */
void FLEXCAN_HAL_SetTimeSegmentsCbt(CAN_Type * base, const flexcan_time_segment_t *timeSeg);

/*!
 * @brief Gets the  FlexCAN time segments to calculate the bit rate.
 *
 * @param   base The FlexCAN base address
 * @param   timeSeg    FlexCAN time segments read for bit rate
 */
void FLEXCAN_HAL_GetTimeSegments(const CAN_Type * base, flexcan_time_segment_t *timeSeg);

/*!
 * @brief Un freezes the FlexCAN module.
 *
 * @param   base     The FlexCAN base address
 */
void FLEXCAN_HAL_ExitFreezeMode(CAN_Type * base);

/*!
 * @brief Freezes the FlexCAN module.
 *
 * @param   base     The FlexCAN base address
 */
void FLEXCAN_HAL_EnterFreezeMode(CAN_Type * base);

/*!
 * @brief Set operation mode.
 *
 * @param   base  The FlexCAN base address
 * @param   mode  Set an operation mode
 */
void FLEXCAN_HAL_SetOperationMode(
    CAN_Type * base,
    flexcan_operation_modes_t mode);

/*!
 * @brief Exit operation mode.
 *
 * @param   base  The FlexCAN base address
 * @param   mode  Exit An operation mode
 */
void FLEXCAN_HAL_ExitOperationMode(
    CAN_Type * base,
    flexcan_operation_modes_t mode);

/*!
 * @brief Enables/Disables Flexible Data rate (if supported).
 *
 * @param   base    The FlexCAN base address
 * @param   enable  true to enable; false to disable
 */
void FLEXCAN_HAL_SetFDEnabled(CAN_Type * base, bool enable);

/*!
 * @brief Checks if the Flexible Data rate feature is enabled.
 *
 * @param   base    The FlexCAN base address
 * @return  true if enabled; false if disabled
 */
bool FLEXCAN_HAL_IsFDEnabled(const CAN_Type * base);

/*!
 * @brief Sets the payload size of the MBs.
 *
 * @param   base         The FlexCAN base address
 * @param   payloadSize  The payload size
 */
void FLEXCAN_HAL_SetPayloadSize(
    CAN_Type * base,
    flexcan_fd_payload_size_t payloadSize);

/*!
 * @brief Gets the payload size of the MBs.
 *
 * @param   base         The FlexCAN base address
 * @return  The payload size in bytes
 */
uint8_t FLEXCAN_HAL_GetPayloadSize(const CAN_Type * base);

/*@}*/

/*!
 * @name Data transfer
 * @{
 */

/*!
 * @brief Sets the FlexCAN message buffer fields for transmitting.
 *
 * @param   base  The FlexCAN base address
 * @param   msgBuffIdx       Index of the message buffer
 * @param   cs           CODE/status values (TX)
 * @param   msgId       ID of the message to transmit
 * @param   msgData      Bytes of the FlexCAN message
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_FLEXCAN_MB_OUT_OF_RANGE if the index of the
 *          message buffer is invalid
 */
status_t FLEXCAN_HAL_SetTxMsgBuff(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    const flexcan_msgbuff_code_status_t *cs,
    uint32_t msgId,
    const uint8_t *msgData);

/*!
 * @brief Sets the FlexCAN message buffer fields for receiving.
 *
 * @param   base  The FlexCAN base address
 * @param   msgBuffIdx       Index of the message buffer
 * @param   cs           CODE/status values (RX)
 * @param   msgId       ID of the message to receive
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_FLEXCAN_MB_OUT_OF_RANGE if the index of the
 *          message buffer is invalid
 */
status_t FLEXCAN_HAL_SetRxMsgBuff(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    const flexcan_msgbuff_code_status_t *cs,
    uint32_t msgId);

/*!
 * @brief Gets the FlexCAN message buffer fields.
 *
 * @param   base  The FlexCAN base address
 * @param   msgBuffIdx       Index of the message buffer
 * @param   msgBuff           The fields of the message buffer
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_FLEXCAN_MB_OUT_OF_RANGE if the index of the
 *          message buffer is invalid
 */
status_t FLEXCAN_HAL_GetMsgBuff(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    flexcan_msgbuff_t *msgBuff);

/*!
 * @brief Locks the FlexCAN Rx message buffer.
 *
 * @param   base  The FlexCAN base address
 * @param   msgBuffIdx       Index of the message buffer
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_FLEXCAN_MB_OUT_OF_RANGE if the index of the
 *          message buffer is invalid
 */
status_t FLEXCAN_HAL_LockRxMsgBuff(
    CAN_Type * base,
    uint32_t msgBuffIdx);

/*!
 * @brief Unlocks the FlexCAN Rx message buffer.
 *
 * @param   base     The FlexCAN base address
 * Implements : FLEXCAN_HAL_UnlockRxMsgBuff_Activity
 */
static inline void FLEXCAN_HAL_UnlockRxMsgBuff(const CAN_Type * base)
{
    /* Unlock the mailbox by reading the free running timer */
    (void)base->TIMER;
}

/*!
 * @brief Enables the Rx FIFO.
 *
 * @param   base     The FlexCAN base address
 * @param   numOfFilters    The number of Rx FIFO filters
 * @return  The status of the operation
 * @retval  STATUS_SUCCESS RxFIFO was successfully enabled
 * @retval  STATUS_ERROR RxFIFO could not be enabled (e.g. the FD feature
 *          was enabled, and these two features are not compatible)
 */
status_t FLEXCAN_HAL_EnableRxFifo(CAN_Type * base, uint32_t numOfFilters);

/*!
 * @brief Disables the Rx FIFO.
 *
 * @param   base     The FlexCAN base address
 */
void FLEXCAN_HAL_DisableRxFifo(CAN_Type * base);

/*!
 * @brief Checks if Rx FIFO is enabled.
 *
 * @param   base     The FlexCAN base address
 * @return  RxFifo status (true = enabled / false = disabled)
 * Implements : FLEXCAN_HAL_IsRxFifoEnabled_Activity
 */
static inline bool FLEXCAN_HAL_IsRxFifoEnabled(const CAN_Type * base)
{
    return (((base->MCR & CAN_MCR_RFEN_MASK) >> CAN_MCR_RFEN_SHIFT) != 0U);
}

/*!
 * @brief Sets the number of the Rx FIFO filters.
 *
 * @param   base  The FlexCAN base address
 * @param   number       The number of Rx FIFO filters
 */
void FLEXCAN_HAL_SetRxFifoFilterNum(CAN_Type * base, uint32_t number);

/*!
 * @brief Sets  the maximum number of Message Buffers.
 *
 * @param   base  The FlexCAN base address
 * @param   maxMsgBuffNum     Maximum number of message buffers
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_FLEXCAN_MB_OUT_OF_RANGE if the index of the
 *          message buffer is invalid
 */
status_t FLEXCAN_HAL_SetMaxMsgBuffNum(
    CAN_Type * base,
    uint32_t maxMsgBuffNum);

/*!
 * @brief Sets the FlexCAN Rx FIFO fields.
 *
 * @param   base             The FlexCAN base address
 * @param   idFormat               The format of the Rx FIFO ID Filter Table Elements
 * @param   idFilterTable         The ID filter table elements which contain RTR bit, IDE bit,
 *                                  and RX message ID.
 */
void FLEXCAN_HAL_SetRxFifoFilter(
    CAN_Type * base,
    flexcan_rx_fifo_id_element_format_t idFormat,
    const flexcan_id_table_t *idFilterTable);

/*!
 * @brief Gets the FlexCAN Rx FIFO data.
 *
 * @param   base  The FlexCAN base address
 * @param   rxFifo      The FlexCAN receive FIFO data
 */
void FLEXCAN_HAL_ReadRxFifo(
    const CAN_Type * base,
    flexcan_msgbuff_t *rxFifo);

/*@}*/

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Enables/Disables the FlexCAN Message Buffer interrupt.
 *
 * @param   base  The FlexCAN base address
 * @param   msgBuffIdx       Index of the message buffer
 * @param   enable       choose enable or disable
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_FLEXCAN_MB_OUT_OF_RANGE if the index of the
 *          message buffer is invalid
 */
status_t FLEXCAN_HAL_SetMsgBuffIntCmd(
    CAN_Type * base,
    uint32_t msgBuffIdx, bool enable);

/*!
 * @brief Enables error interrupt of the FlexCAN module.
 * @param   base     The FlexCAN base address
 * @param   errType     The interrupt type
 * @param   enable       choose enable or disable
 */
void FLEXCAN_HAL_SetErrIntCmd(CAN_Type * base, flexcan_int_type_t errType, bool enable);

/*@}*/

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Gets the value of FlexCAN freeze ACK.
 *
 * @param   base     The FlexCAN base address
 * @return  freeze ACK state (1-freeze mode, 0-not in freeze mode).
 * Implements : FLEXCAN_HAL_GetFreezeAck_Activity
 */
static inline uint32_t FLEXCAN_HAL_GetFreezeAck(const CAN_Type * base)
{
    return ((base->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT);
}

/*!
 * @brief Gets the individual FlexCAN MB interrupt flag.
 *
 * @param   base  The FlexCAN base address
 * @param   msgBuffIdx       Index of the message buffer
 * @return  the individual Message Buffer interrupt flag (0 and 1 are the flag value)
 */
uint8_t FLEXCAN_HAL_GetMsgBuffIntStatusFlag(
    const CAN_Type * base,
    uint32_t msgBuffIdx);

/*!
 * @brief Gets all FlexCAN Message Buffer interrupt flags.
 *
 * @param   base     The FlexCAN base address
 * @return  all MB interrupt flags
 * Implements : FLEXCAN_HAL_GetAllMsgBuffIntStatusFlag_Activity
 */
static inline uint32_t FLEXCAN_HAL_GetAllMsgBuffIntStatusFlag(const CAN_Type * base)
{
    uint32_t mask = base->IMASK1 & CAN_IMASK1_BUF31TO0M_MASK;

    return (base->IFLAG1 & mask);
}

/*!
 * @brief Clears the interrupt flag of the message buffers.
 *
 * @param   base  The FlexCAN base address
 * @param   flag      The value to be written to the interrupt flag1 register.
 * Implements : FLEXCAN_HAL_ClearMsgBuffIntStatusFlag_Activity
 */
static inline void FLEXCAN_HAL_ClearMsgBuffIntStatusFlag(
    CAN_Type * base,
    uint32_t flag)
{
    /* Clear the corresponding message buffer interrupt flag*/
    (base->IFLAG1) = (flag);
}

/*!
 * @brief Gets the transmit error counter and receives the error counter.
 *
 * @param   base  The FlexCAN base address
 * @param   errCount      Transmit error counter and receive error counter
 */
void FLEXCAN_HAL_GetErrCounter(
    const CAN_Type * base,
    flexcan_buserr_counter_t *errCount);

/*!
 * @brief Gets error and status.
 *
 * @param   base     The FlexCAN base address
 * @return  The current error and status
 * Implements : FLEXCAN_HAL_GetErrStatus_Activity
 */
static inline uint32_t FLEXCAN_HAL_GetErrStatus(const CAN_Type * base)
{
    return (base->ESR1);
}

/*!
 * @brief Clears all other interrupts in ERRSTAT register (Error, Busoff, Wakeup).
 *
 * @param   base     The FlexCAN base address
 */
void FLEXCAN_HAL_ClearErrIntStatusFlag(CAN_Type * base);

/*@}*/

/*!
 * @name Mask
 * @{
 */

/*!
 * @brief Sets the Rx masking type.
 *
 * @param   base  The FlexCAN base address
 * @param   type         The FlexCAN Rx mask type
 */
void FLEXCAN_HAL_SetRxMaskType(CAN_Type * base, flexcan_rx_mask_type_t type);

/*!
 * @brief Sets the FlexCAN RX FIFO global standard mask.
 *
 * @param   base  The FlexCAN base address
 * @param   stdMask     Standard mask
 */
void FLEXCAN_HAL_SetRxFifoGlobalStdMask(
    CAN_Type * base,
    uint32_t stdMask);

/*!
 * @brief Sets the FlexCAN Rx FIFO global extended mask.
 *
 * @param   base  The FlexCAN base address
 * @param   extMask     Extended mask
 */
void FLEXCAN_HAL_SetRxFifoGlobalExtMask(
    CAN_Type * base,
    uint32_t extMask);

/*!
 * @brief Sets the FlexCAN Rx individual standard mask for ID filtering in the Rx MBs and the Rx FIFO.
 *
 * @param   base  The FlexCAN base address
 * @param   msgBuffIdx       Index of the message buffer
 * @param   stdMask     Individual standard mask
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_FLEXCAN_MB_OUT_OF_RANGE if the index of the
 *          message buffer is invalid
 */
status_t FLEXCAN_HAL_SetRxIndividualStdMask(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    uint32_t stdMask);

/*!
 * @brief Sets the FlexCAN Rx individual extended mask for ID filtering in the Rx Message Buffers and the Rx FIFO.
 *
 * @param   base  The FlexCAN base address
 * @param   msgBuffIdx       Index of the message buffer
 * @param   extMask     Individual extended mask
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_FLEXCAN_MB_OUT_OF_RANGE if the index of the
 *          message buffer is invalid
 */
status_t FLEXCAN_HAL_SetRxIndividualExtMask(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    uint32_t extMask);

/*!
 * @brief Sets the FlexCAN Rx Message Buffer global standard mask.
 *
 * @param   base  The FlexCAN base address
 * @param   stdMask     Standard mask
 */
void FLEXCAN_HAL_SetRxMsgBuffGlobalStdMask(
    CAN_Type * base,
    uint32_t stdMask);

/*!
 * @brief Sets the FlexCAN RX Message Buffer BUF14 standard mask.
 *
 * @param   base  The FlexCAN base address
 * @param   stdMask     Standard mask
 */
void FLEXCAN_HAL_SetRxMsgBuff14StdMask(
    CAN_Type * base,
    uint32_t stdMask);

/*!
 * @brief Sets the FlexCAN Rx Message Buffer BUF15 standard mask.
 *
 * @param   base  The FlexCAN base address
 * @param   stdMask     Standard mask
 */
void FLEXCAN_HAL_SetRxMsgBuff15StdMask(
    CAN_Type * base,
    uint32_t stdMask);

/*!
 * @brief Sets the FlexCAN RX Message Buffer global extended mask.
 *
 * @param   base  The FlexCAN base address
 * @param   extMask     Extended mask
 */
void FLEXCAN_HAL_SetRxMsgBuffGlobalExtMask(
    CAN_Type * base,
    uint32_t extMask);

/*!
 * @brief Sets the FlexCAN RX Message Buffer BUF14 extended mask.
 *
 * @param   base  The FlexCAN base address
 * @param   extMask     Extended mask
 */
void FLEXCAN_HAL_SetRxMsgBuff14ExtMask(
    CAN_Type * base,
    uint32_t extMask);

/*!
 * @brief Sets the FlexCAN RX MB BUF15 extended mask.
 *
 * @param   base  The FlexCAN base address
 * @param   extMask     Extended mask
 */
void FLEXCAN_HAL_SetRxMsgBuff15ExtMask(
    CAN_Type * base,
    uint32_t extMask);

/*!
 * @brief Gets the FlexCAN ID acceptance filter hit indicator on Rx FIFO.
 *
 * @param   base  The FlexCAN base address
 * @return  RX FIFO information
 * Implements : FLEXCAN_HAL_GetRxFifoHitIdAcceptanceFilter_Activity
 */
static inline uint32_t  FLEXCAN_HAL_GetRxFifoHitIdAcceptanceFilter(const CAN_Type * base)
{
    return ((((base)->RXFIR) & CAN_RXFIR_IDHIT_MASK) >> CAN_RXFIR_IDHIT_SHIFT);
}

/*!
 * @brief Enables/Disables the Stuff Bit Count for CAN FD frames.
 *
 * If enabled, the modulo 8 count of variable stuff bits inserted plus the respective
 * parity bit (even parity calculated over the 3-bit modulo 8 count) are combined as
 * the 4-bit Stuff Count field and inserted before the CRC Sequence field. CRC
 * calculation extends beyond the end of Data field and takes the Stuff Count field bits
 * into account.
 *
 * @param   base  The FlexCAN base address
 * @param   enable Enable/Disable Stuff Bit Count
 */
void FLEXCAN_HAL_SetStuffBitCount(CAN_Type * base, bool enable);

/*!
 * @brief Enables/Disables the Self Reception feature.
 *
 * If enabled, FlexCAN is allowed to receive frames transmitted by itself.
 *
 * @param   base  The FlexCAN base address
 * @param   enable Enable/Disable Self Reception
 */
void FLEXCAN_HAL_SetSelfReception(CAN_Type * base, bool enable);

#if FEATURE_CAN_HAS_DMA_ENABLE
/*!
 * @brief Enables/Disables the DMA support for RxFIFO.
 *
 * @param   base  The FlexCAN base address
 * @param   enable Enable/Disable DMA support
 * @return  STATUS_SUCCESS if successfully enabled; STATUS_ERROR
 *    if not enabled (due to the RxFIFO feature being disabled).
 */
status_t FLEXCAN_HAL_SetRxFifoDMA(CAN_Type * base, bool enable);
#endif

/*!
 * @brief Enables/Disables the Transceiver Delay Compensation feature and sets
 * the Transceiver Delay Compensation Offset (offset value to be added to the
 * measured transceiver's loop delay in order to define the position of the
 * delayed comparison point when bit rate switching is active).
 *
 * @param   base  The FlexCAN base address
 * @param   enable Enable/Disable Transceiver Delay Compensation
 * @param   offset Transceiver Delay Compensation Offset
 */
void FLEXCAN_HAL_SetTDCOffset(CAN_Type * base, bool enable, uint8_t offset);

/*!
 * @brief Gets the value of the Transceiver Delay Compensation.
 *
 * @param   base  The FlexCAN base address
 * @return  The value of the transceiver loop delay measured from the transmitted
 * EDL to R0 transition edge to the respective received one added to the TDCOFF
 * value specified by FLEXCAN_HAL_SetTDCOffset.
 * Implements : FLEXCAN_HAL_GetTDCValue_Activity
 */
static inline uint8_t FLEXCAN_HAL_GetTDCValue(const CAN_Type * base)
{
    return (uint8_t)((base->FDCTRL & CAN_FDCTRL_TDCVAL_MASK) >> CAN_FDCTRL_TDCVAL_SHIFT);
}

/*!
 * @brief Gets the value of the TDC Fail flag.
 *
 * @param   base  The FlexCAN base address
 * @return  If true, indicates that the TDC mechanism is out of range, unable to
 * compensate the transceiver's loop delay and successfully compare the delayed
 * received bits to the transmitted ones.
 * Implements : FLEXCAN_HAL_GetTDCFail_Activity
 */
static inline bool FLEXCAN_HAL_GetTDCFail(const CAN_Type * base)
{
    return (((base->FDCTRL & CAN_FDCTRL_TDCFAIL_MASK) >> CAN_FDCTRL_TDCFAIL_SHIFT) != 0U);
}

/*!
 * @brief Clears the TDC Fail flag.
 *
 * @param   base  The FlexCAN base address
 * Implements : FLEXCAN_HAL_ClearTDCFail_Activity
 */
static inline void FLEXCAN_HAL_ClearTDCFail(CAN_Type * base)
{
    base->FDCTRL = base->FDCTRL | CAN_FDCTRL_TDCFAIL_MASK;
}

#if FEATURE_CAN_HAS_PRETENDED_NETWORKING

/*!
 * @brief Configures the Pretended Networking mode.
 *
 * @param   base  The FlexCAN base address
 * @param   pnConfig  The pretended networking configuration
 */
void FLEXCAN_HAL_ConfigPN(CAN_Type * base, const flexcan_pn_config_t *pnConfig);

/*!
 * @brief Extracts one of the frames which triggered the wake up event.
 *
 * @param   base  The FlexCAN base address
 * @param   wmbIndex  The index of the message buffer to be extracted.
 * @param   wmb  Pointer to the message buffer structure where the frame will be saved.
 */
void FLEXCAN_HAL_GetWMB(const CAN_Type * base, uint8_t wmbIndex, flexcan_msgbuff_t *wmb);

/*!
 * @brief Enables/Disables the Pretended Networking mode.
 *
 * @param   base  The FlexCAN base address
 * @param   enable  Enable/Disable Pretending Networking
 * Implements : FLEXCAN_HAL_SetPN_Activity
 */
static inline void FLEXCAN_HAL_SetPN(CAN_Type * base, bool enable)
{
    base->MCR = (base->MCR & ~CAN_MCR_PNET_EN_MASK) | CAN_MCR_PNET_EN(enable ? 1UL : 0UL);
}

#endif /* FEATURE_CAN_HAS_PRETENDED_NETWORKING */

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* FLEXCAN_HAL_H */

/*******************************************************************************
 * EOF
 ******************************************************************************/
