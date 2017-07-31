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
 * @file flexcan_driver.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, could define variable at block scope
 * The variable is defined in the common source file to make the update of the
 * DMA channels easier.
 */

#ifndef FLEXCAN_DRIVER_H
#define FLEXCAN_DRIVER_H

#include "flexcan_hal.h"
#include "osif.h"
#if FEATURE_CAN_HAS_DMA_ENABLE
#include "edma_driver.h"
#endif

/*!
 * @defgroup flexcan_driver FlexCAN Driver
 * @ingroup flexcan
 * @addtogroup flexcan_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Table of base addresses for FlexCAN instances. */
extern CAN_Type * const g_flexcanBase[CAN_INSTANCE_COUNT];

/*! @brief Table to save RX Warning IRQ numbers for FlexCAN instances. */
extern const IRQn_Type g_flexcanRxWarningIrqId[CAN_INSTANCE_COUNT];
/*! @brief Table to save TX Warning IRQ numbers for FlexCAN instances. */
extern const IRQn_Type g_flexcanTxWarningIrqId[CAN_INSTANCE_COUNT];
/*! @brief Table to save wakeup IRQ numbers for FlexCAN instances. */
extern const IRQn_Type g_flexcanWakeUpIrqId[CAN_INSTANCE_COUNT];
/*! @brief Table to save error IRQ numbers for FlexCAN instances. */
extern const IRQn_Type g_flexcanErrorIrqId[CAN_INSTANCE_COUNT];
/*! @brief Table to save Bus off IRQ numbers for FlexCAN instances. */
extern const IRQn_Type g_flexcanBusOffIrqId[CAN_INSTANCE_COUNT];
/*! @brief Table to save message buffer IRQ numbers for FlexCAN instances. */
extern const IRQn_Type g_flexcanOredMessageBufferIrqId[CAN_INSTANCE_COUNT][FEATURE_CAN_MB_IRQS_MAX_COUNT];

#if FEATURE_CAN_HAS_DMA_ENABLE
/*! @brief Table of DMA requests for FlexCAN instances. */
extern const dma_request_source_t g_flexcanDmaRequests[CAN_INSTANCE_COUNT];
#endif

/*! @brief The type of the RxFIFO transfer (interrupts/DMA).
 * Implements : flexcan_rxfifo_transfer_type_t_Class
 */
typedef enum {
    FLEXCAN_RXFIFO_USING_INTERRUPTS,    /*!< Use interrupts for RxFIFO. */
    FLEXCAN_RXFIFO_USING_DMA            /*!< Use DMA for RxFIFO. */
} flexcan_rxfifo_transfer_type_t;

/*! @brief The type of the event which occured when the callback was invoked.
 * Implements : flexcan_event_type_t_Class
 */
typedef enum {
    FLEXCAN_EVENT_RX_COMPLETE,     /*!< A frame was received in the configured Rx MB. */
    FLEXCAN_EVENT_RXFIFO_COMPLETE, /*!< A frame was received in the RxFIFO. */
    FLEXCAN_EVENT_TX_COMPLETE,     /*!< A frame was sent from the configured Tx MB. */
#if FEATURE_CAN_HAS_PRETENDED_NETWORKING
    FLEXCAN_EVENT_WAKEUP_TIMEOUT,  /*!< An wake up event occurred due to timeout. */
    FLEXCAN_EVENT_WAKEUP_MATCH     /*!< An wake up event occurred due to matching. */
#endif /* FEATURE_CAN_HAS_PRETENDED_NETWORKING */
} flexcan_event_type_t;


/*! @brief The state of a given MB (idle/Rx busy/Tx busy).
 * Implements : flexcan_mb_state_t_Class
 */
typedef enum {
    FLEXCAN_MB_IDLE,      /*!< The MB is not used by any transfer. */
    FLEXCAN_MB_RX_BUSY,   /*!< The MB is used for a reception. */
    FLEXCAN_MB_TX_BUSY    /*!< The MB is used for a transmission. */
} flexcan_mb_state_t;

/*! @brief Information needed for internal handling of a given MB.
 * Implements : flexcan_mb_handle_t_Class
 */
typedef struct {
    flexcan_msgbuff_t *mb_message;   /*!< The FlexCAN receive MB data */
    semaphore_t mbSema;              /*!< Semaphore used for signaling completion of a blocking transfer */
    flexcan_mb_state_t state;        /*!< The state of the current MB (idle/Rx busy/Tx busy) */
    bool isBlocking;                 /*!< True if the transfer is blocking */
    bool isRemote;                   /*!< True if the frame is a remote frame */
} flexcan_mb_handle_t;

/*!
 * @brief Internal driver state information.
 *
 * @note The contents of this structure are internal to the driver and should not be
 *      modified by users. Also, contents of the structure are subject to change in
 *      future releases.
 * Implements : flexcan_state_t_Class
 */
typedef struct FlexCANState {
    volatile flexcan_mb_handle_t mbs[FEATURE_CAN_MAX_MB_NUM]; /*!< Array containing information related to each MB */
    void (*callback)(uint8_t instance, flexcan_event_type_t eventType,
                     struct FlexCANState * state); /*!< IRQ handler callback function. */
    void *callbackParam;             /*!< Parameter used to pass user data when invoking the callback function. */
#if FEATURE_CAN_HAS_DMA_ENABLE
    uint8_t rxFifoDMAChannel;        /*!< DMA channel number used for transfers. */
#endif
    flexcan_rxfifo_transfer_type_t transferType;  /*!< Type of RxFIFO transfer. */
} flexcan_state_t;

/*! @brief FlexCAN data info from user
 * Implements : flexcan_data_info_t_Class
 */
typedef struct {
    flexcan_msgbuff_id_type_t msg_id_type;  /*!< Type of message ID (standard or extended)*/
    uint32_t data_length;                   /*!< Length of Data in Bytes*/
    bool fd_enable;                      /*!< Enable or disable FD*/
    uint8_t fd_padding;                     /*!< Set a value for padding. It will be used when payload*/
                                            /*! is bigger than data_length to fill the MB*/
    bool enable_brs;                     /*!< Enable bit rate switch*/
    bool is_remote;                      /*!< Specifies if the frame is standard or remote */
} flexcan_data_info_t;


/*! @brief FlexCAN Rx FIFO filters number
 * Implements : flexcan_rx_fifo_id_filter_num_t_Class
 */
typedef enum {
    FLEXCAN_RX_FIFO_ID_FILTERS_8   = 0x0,         /*!<   8 Rx FIFO Filters. @internal gui name="8 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_16  = 0x1,         /*!<  16 Rx FIFO Filters. @internal gui name="16 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_24  = 0x2,         /*!<  24 Rx FIFO Filters. @internal gui name="24 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_32  = 0x3,         /*!<  32 Rx FIFO Filters. @internal gui name="32 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_40  = 0x4,         /*!<  40 Rx FIFO Filters. @internal gui name="40 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_48  = 0x5,         /*!<  48 Rx FIFO Filters. @internal gui name="48 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_56  = 0x6,         /*!<  56 Rx FIFO Filters. @internal gui name="56 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_64  = 0x7,         /*!<  64 Rx FIFO Filters. @internal gui name="64 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_72  = 0x8,         /*!<  72 Rx FIFO Filters. @internal gui name="72 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_80  = 0x9,         /*!<  80 Rx FIFO Filters. @internal gui name="80 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_88  = 0xA,         /*!<  88 Rx FIFO Filters. @internal gui name="88 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_96  = 0xB,         /*!<  96 Rx FIFO Filters. @internal gui name="96 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_104 = 0xC,         /*!< 104 Rx FIFO Filters. @internal gui name="104 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_112 = 0xD,         /*!< 112 Rx FIFO Filters. @internal gui name="112 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_120 = 0xE,         /*!< 120 Rx FIFO Filters. @internal gui name="120 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_128 = 0xF          /*!< 128 Rx FIFO Filters. @internal gui name="128 Rx FIFO Filters" */
} flexcan_rx_fifo_id_filter_num_t;

/*! @brief FlexCAN configuration
 * @internal gui name="Common configuration" id="flexcanCfg"
 * Implements : flexcan_user_config_t_Class
 */
typedef struct {
    uint32_t max_num_mb;                            /*!< The maximum number of Message Buffers @internal gui name="Maximum number of message buffers" id="max_num_mb" */
    flexcan_rx_fifo_id_filter_num_t num_id_filters; /*!< The number of RX FIFO ID filters needed @internal gui name="Number of RX FIFO ID filters" id="num_id_filters" */
    bool is_rx_fifo_needed;                         /*!< 1 if needed; 0 if not. This controls whether the Rx FIFO feature is enabled or not. @internal gui name="Use rx fifo" id="is_rx_fifo_needed" */
    flexcan_operation_modes_t flexcanMode;          /*!< User configurable FlexCAN operation modes. @internal gui name="Flexcan Operation Mode" id="flexcanMode"*/
    flexcan_fd_payload_size_t payload;              /*!< The payload size of the mailboxes. */
    bool fd_enable;                                 /*!< Enable/Disable the Flexible Data Rate feature. */
#if FEATURE_CAN_HAS_PE_CLKSRC_SELECT
    flexcan_clk_source_t pe_clock;                  /*!< The clock source of the CAN Protocol Engine (PE). */
#endif
    flexcan_time_segment_t bitrate;                 /*!< The bitrate used for standard frames or for the arbitration phase of FD frames. */
    flexcan_time_segment_t bitrate_cbt;             /*!< The bitrate used for the data phase of FD frames. */
    flexcan_rxfifo_transfer_type_t transfer_type;   /*!< Specifies if the RxFIFO uses interrupts or DMA. */
    uint8_t rxFifoDMAChannel;                       /*!< Specifies the DMA channel number to be used for DMA transfers. */
} flexcan_user_config_t;

/*! @brief FlexCAN Driver callback function type
 * Implements : flexcan_callback_t_Class
 */
typedef void (*flexcan_callback_t)(uint8_t instance, flexcan_event_type_t eventType,
                                   flexcan_state_t *flexcanState);

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Bit rate
 * @{
 */

/*!
 * @brief Sets the FlexCAN bit rate.
 *
 * @param   instance    A FlexCAN instance number
 * @param   bitrate     A pointer to the FlexCAN bit rate settings.
 */
void FLEXCAN_DRV_SetBitrate(uint8_t instance, const flexcan_time_segment_t *bitrate);
/*!
 * @name Set baud rate for BRS FD
 * @{
 */

/*!
 * @brief Sets the FlexCAN bit rate for FD BRS.
 *
 * @param   instance    A FlexCAN instance number
 * @param   bitrate     A pointer to the FlexCAN bit rate settings.
 */
void FLEXCAN_DRV_SetBitrateCbt(uint8_t instance, const flexcan_time_segment_t *bitrate);
/*!
 * @brief Gets the FlexCAN bit rate.
 *
 * @param   instance    A FlexCAN instance number
 * @param   bitrate     A pointer to a variable for returning the FlexCAN bit rate settings
 */
void FLEXCAN_DRV_GetBitrate(uint8_t instance, flexcan_time_segment_t *bitrate);

/*@}*/

/*!
 * @name Global mask
 * @{
 */

/*!
 * @brief Sets the RX masking type.
 *
 * @param   instance     A FlexCAN instance number
 * @param   type         The FlexCAN RX mask type
 */
void FLEXCAN_DRV_SetRxMaskType(uint8_t instance, flexcan_rx_mask_type_t type);

/*!
 * @brief Sets the FlexCAN RX FIFO global standard or extended mask.
 *
 * @param   instance    A FlexCAN instance number
 * @param   id_type     Standard ID or extended ID
 * @param   mask        Mask value
 */
void FLEXCAN_DRV_SetRxFifoGlobalMask(
    uint8_t instance,
    flexcan_msgbuff_id_type_t id_type,
    uint32_t mask);

/*!
 * @brief Sets the FlexCAN RX MB global standard or extended mask.
 *
 * @param   instance    A FlexCAN instance number
 * @param   id_type     Standard ID or extended ID
 * @param   mask        Mask value
 */
void FLEXCAN_DRV_SetRxMbGlobalMask(
    uint8_t instance,
    flexcan_msgbuff_id_type_t id_type,
    uint32_t mask);

/*!
 * @brief Sets the FlexCAN RX individual standard or extended mask.
 *
 * @param   instance  A FlexCAN instance number
 * @param   id_type   A standard ID or an extended ID
 * @param   mb_idx    Index of the message buffer
 * @param   mask      Mask value
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_FLEXCAN_MB_OUT_OF_RANGE if the index of the
 *          message buffer is invalid
 */
status_t FLEXCAN_DRV_SetRxIndividualMask(
    uint8_t instance,
    flexcan_msgbuff_id_type_t id_type,
    uint8_t mb_idx,
    uint32_t mask);

/*@}*/

/*!
 * @name Initialization and Shutdown
 * @{
 */

/*!
 * @brief Initializes the FlexCAN peripheral.
 *
 * This function initializes
 * @param   instance                   A FlexCAN instance number
 * @param   state                      Pointer to the FlexCAN driver state structure.
 * @param   data                       The FlexCAN platform data
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_FLEXCAN_MB_OUT_OF_RANGE if the index of a message buffer is invalid;
 *          STATUS_ERROR if other error occurred
 */
status_t FLEXCAN_DRV_Init(
       uint32_t instance,
       flexcan_state_t *state,
       const flexcan_user_config_t *data);

/*!
 * @brief Shuts down a FlexCAN instance.
 *
 * @param   instance    A FlexCAN instance number
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_ERROR if failed
 */
status_t FLEXCAN_DRV_Deinit(uint8_t instance);

/*@}*/

/*!
 * @name Send configuration
 * @{
 */

/*!
 * @brief FlexCAN transmit message buffer field configuration.
 *
 * @param   instance                   A FlexCAN instance number
 * @param   mb_idx                     Index of the message buffer
 * @param   tx_info                    Data info
 * @param   msg_id                     ID of the message to transmit
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_FLEXCAN_MB_OUT_OF_RANGE if the index of the message buffer is invalid
 */
status_t FLEXCAN_DRV_ConfigTxMb(
    uint8_t instance,
    uint8_t mb_idx,
    const flexcan_data_info_t *tx_info,
    uint32_t msg_id);

/*!
 * @brief Sends a CAN frame using the specified message buffer, in a blocking manner.
 *
 * This function sends a CAN frame using a configured message buffer. The function
 * blocks until either the frame was sent, or the specified timeout expired.
 *
 * @param   instance   A FlexCAN instance number
 * @param   mb_idx     Index of the message buffer
 * @param   tx_info    Data info
 * @param   msg_id     ID of the message to transmit
 * @param   mb_data    Bytes of the FlexCAN message
 * @param   timeout_ms A timeout for the transfer in milliseconds.
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_FLEXCAN_MB_OUT_OF_RANGE if the index of a message buffer is invalid;
 *          STATUS_BUSY if a resource is busy;
 *          STATUS_TIMEOUT if the timeout is reached;
 *          STATUS_ERROR if other error occurred
 */
status_t FLEXCAN_DRV_SendBlocking(
    uint8_t instance,
    uint8_t mb_idx,
    const flexcan_data_info_t *tx_info,
    uint32_t msg_id,
    const uint8_t *mb_data,
    uint32_t timeout_ms);

/*!
 * @brief Sends a CAN frame using the specified message buffer.
 *
 * This function sends a CAN frame using a configured message buffer. The function
 * returns immediately. If a callback is installed, it will be invoked after
 * the frame was sent.
 *
 * @param   instance   A FlexCAN instance number
 * @param   mb_idx     Index of the message buffer
 * @param   tx_info    Data info
 * @param   msg_id     ID of the message to transmit
 * @param   mb_data    Bytes of the FlexCAN message.
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_FLEXCAN_MB_OUT_OF_RANGE if the index of a message buffer is invalid;
 *          STATUS_BUSY if a resource is busy;
 *          STATUS_ERROR if other error occurred
 */
status_t FLEXCAN_DRV_Send(
    uint8_t instance,
    uint8_t mb_idx,
    const flexcan_data_info_t *tx_info,
    uint32_t msg_id,
    const uint8_t *mb_data);

/*!
 * @brief Ends a non-blocking FlexCAN transfer early.
 *
 * @param   instance   A FlexCAN instance number
 * @param   mb_idx     The index of the message buffer
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_FLEXCAN_NO_TRANSFER_IN_PROGRESS if no transfer was running
 */
status_t FLEXCAN_DRV_AbortTransfer(uint32_t instance, uint8_t mb_idx);

/*@}*/

/*!
 * @name Receive configuration
 * @{
 */

/*!
 * @brief FlexCAN receive message buffer field configuration
 *
 * @param   instance                   A FlexCAN instance number
 * @param   mb_idx                     Index of the message buffer
 * @param   rx_info                    Data info
 * @param   msg_id                     ID of the message to transmit
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_FLEXCAN_MB_OUT_OF_RANGE if the index of a message buffer is invalid;
 *          STATUS_ERROR if other error occurred
 */
status_t FLEXCAN_DRV_ConfigRxMb(
    uint8_t instance,
    uint8_t mb_idx,
    const flexcan_data_info_t *rx_info,
    uint32_t msg_id);

/*!
 * @brief FlexCAN RX FIFO field configuration
 *
 * @param   instance           A FlexCAN instance number
 * @param   id_format          The format of the RX FIFO ID Filter Table Elements
 * @param   id_filter_table    The ID filter table elements which contain RTR bit, IDE bit,
 *                             and RX message ID
 */
void FLEXCAN_DRV_ConfigRxFifo(
    uint8_t instance,
    flexcan_rx_fifo_id_element_format_t id_format,
    const flexcan_id_table_t *id_filter_table);

/*!
 * @brief Receives a CAN frame using the specified message buffer, in a blocking manner.
 *
 * This function receives a CAN frame using a configured message buffer. The function
 * blocks until either a frame was received, or the specified timeout expired.
 *
 * @param   instance   A FlexCAN instance number
 * @param   mb_idx     Index of the message buffer
 * @param   data       The FlexCAN receive message buffer data.
 * @param   timeout_ms A timeout for the transfer in milliseconds.
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_FLEXCAN_MB_OUT_OF_RANGE if the index of a message buffer is invalid;
 *          STATUS_BUSY if a resource is busy;
 *          STATUS_TIMEOUT if the timeout is reached;
 *          STATUS_ERROR if other error occurred
 */
status_t FLEXCAN_DRV_ReceiveBlocking(
    uint8_t instance,
    uint8_t mb_idx,
    flexcan_msgbuff_t *data,
    uint32_t timeout_ms);

/*!
 * @brief Receives a CAN frame using the specified message buffer.
 *
 * This function receives a CAN frame using a configured message buffer. The function
 * returns immediately. If a callback is installed, it will be invoked after
 * the frame was received and read into the specified buffer.
 *
 * @param   instance   A FlexCAN instance number
 * @param   mb_idx     Index of the message buffer
 * @param   data       The FlexCAN receive message buffer data.
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_FLEXCAN_MB_OUT_OF_RANGE if the index of a message buffer is invalid;
 *          STATUS_BUSY if a resource is busy;
 *          STATUS_ERROR if other error occurred
 */
status_t FLEXCAN_DRV_Receive(
    uint8_t instance,
    uint8_t mb_idx,
    flexcan_msgbuff_t *data);

/*!
 * @brief Receives a CAN frame using the message FIFO, in a blocking manner.
 *
 * This function receives a CAN frame using the Rx FIFO. The function blocks until
 * either a frame was received, or the specified timeout expired.
 *
 * @param   instance    A FlexCAN instance number
 * @param   data        The FlexCAN receive message buffer data.
 * @param   timeout_ms  A timeout for the transfer in milliseconds.
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_BUSY if a resource is busy;
 *          STATUS_TIMEOUT if the timeout is reached;
 *          STATUS_ERROR if other error occurred
 */
status_t FLEXCAN_DRV_RxFifoBlocking(
    uint8_t instance,
    flexcan_msgbuff_t *data,
    uint32_t timeout_ms);

/*!
 * @brief Receives a CAN frame using the message FIFO.
 *
 * This function receives a CAN frame using the Rx FIFO. The function returns
 * immediately. If a callback is installed, it will be invoked after the frame
 * was received and read into the specified buffer.
 *
 * @param   instance    A FlexCAN instance number
 * @param   data        The FlexCAN receive message buffer data.
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_BUSY if a resource is busy;
 *          STATUS_ERROR if other error occurred
 */
status_t FLEXCAN_DRV_RxFifo(
    uint8_t instance,
    flexcan_msgbuff_t *data);

/*@}*/

/*!
 * @brief Interrupt handler for a FlexCAN instance.
 *
 * @param   instance    The FlexCAN instance number.
 */
void FLEXCAN_DRV_IRQHandler(uint8_t instance);

#if FEATURE_CAN_HAS_PRETENDED_NETWORKING

/*!
 * @brief Wake up handler for a FlexCAN instance.
 *
 * @param   instance    The FlexCAN instance number.
 */
void FLEXCAN_DRV_WakeUpHandler(uint8_t instance);

#endif /* FEATURE_CAN_HAS_PRETENDED_NETWORKING */

/*!
 * @brief Returns whether the previous FLEXCAN transfer has finished.
 *
 * When performing an async transfer, call this function to ascertain the state of the
 * current transfer: in progress (or busy) or complete (success).
 *
 * @param instance The FLEXCAN module base address.
 * @param mb_idx The index of the message buffer.
 * @return STATUS_SUCCESS if successful;
 *         STATUS_BUSY if a resource is busy;
 */
status_t FLEXCAN_DRV_GetTransferStatus(uint32_t instance, uint8_t mb_idx);

/*!
 * @brief Installs a callback function for the IRQ handler.
 *
 * @param instance The FLEXCAN module base address.
 * @param callback The callback function.
 * @param callbackParam User parameter passed to the callback function through the state parameter.
 */
void FLEXCAN_DRV_InstallEventCallback(uint8_t instance, flexcan_callback_t callback, void *callbackParam);

#if FEATURE_CAN_HAS_PRETENDED_NETWORKING

/*!
 * @brief Configures Pretended Networking settings.
 *
 * @param instance The FLEXCAN module base address.
 * @param enable Enable/Disable Pretended Networking mode.
 * @param pnConfig Pointer to the Pretended Networking configuration structure.
 */
void FLEXCAN_DRV_ConfigPN(uint8_t instance, bool enable, const flexcan_pn_config_t *pnConfig);

#endif /* FEATURE_CAN_HAS_PRETENDED_NETWORKING */


#ifdef __cplusplus
}
#endif

/*! @}*/

#endif /* FLEXCAN_DRIVER_H */

/*******************************************************************************
 * EOF
 ******************************************************************************/
