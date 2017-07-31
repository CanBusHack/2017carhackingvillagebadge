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

#ifndef LPUART_DRIVER_H__
#define LPUART_DRIVER_H__

#include <stddef.h>
#include "lpuart_hal.h"
#include "clock_manager.h"
#include "interrupt_manager.h"
#include "osif.h"
#if FEATURE_LPUART_HAS_DMA_ENABLE
#include "edma_driver.h"
#endif

/*!
 * @addtogroup lpuart_driver
 * @{
 */


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief LPUART receive callback function type.
 *
 * Implements : lpuart_rx_callback_t_Class
 */
typedef void (* lpuart_rx_callback_t)(uint32_t instance, void * lpuartState);

/*! @brief LPUART transmit callback function type.
 *
 * Implements : lpuart_tx_callback_t_Class
 */
typedef void (* lpuart_tx_callback_t)(uint32_t instance, void * lpuartState);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Table of base addresses for LPUART instances. */
extern LPUART_Type * const g_lpuartBase[LPUART_INSTANCE_COUNT];

/*! @brief Table to save LPUART IRQ enumeration numbers defined in the CMSIS header file */
extern const IRQn_Type g_lpuartRxTxIrqId[LPUART_INSTANCE_COUNT];

/*! @brief Table to save LPUART indexes in PCC register map for clock configuration */
extern const clock_names_t g_lpuartClkNames[LPUART_INSTANCE_COUNT];

/*! @brief Type of LPUART transfer (based on interrupts or DMA).
 *
 * Implements : lpuart_transfer_type_t_Class
 */
typedef enum
{
    LPUART_USING_DMA         = 0,    /*!< The driver will use DMA to perform UART transfer */
    LPUART_USING_INTERRUPTS          /*!< The driver will use interrupts to perform UART transfer */
} lpuart_transfer_type_t;

/*!
 * @brief Runtime state of the LPUART driver.
 *
 * Note that the caller provides memory for the driver state structures during
 * initialization because the driver does not statically allocate memory.
 *
 * Implements : lpuart_state_t_Class
 */
typedef struct
{
    const uint8_t * txBuff;              /*!< The buffer of data being sent.*/
    uint8_t * rxBuff;                    /*!< The buffer of received data.*/
    volatile uint32_t txSize;            /*!< The remaining number of bytes to be transmitted. */
    volatile uint32_t rxSize;            /*!< The remaining number of bytes to be received. */
    volatile bool isTxBusy;              /*!< True if there is an active transmit.*/
    volatile bool isRxBusy;              /*!< True if there is an active receive.*/
    volatile bool isTxBlocking;          /*!< True if transmit is blocking transaction. */
    volatile bool isRxBlocking;          /*!< True if receive is blocking transaction. */
    lpuart_bit_count_per_char_t bitCountPerChar; /*!< number of bits in a char (8/9/10) */
    lpuart_rx_callback_t rxCallback;     /*!< Callback to invoke for data receive
                                              Note: when the transmission is interrupt based, the callback
                                              is being called upon receiving a byte; when DMA transmission
                                              is used, the bytes are copied to the rx buffer by the DMA engine
                                              and the callback is called when all the bytes have been transferred. */
    void * rxCallbackParam;              /*!< Receive callback parameter pointer.*/
    lpuart_tx_callback_t txCallback;     /*!< Callback to invoke for data send
                                              Note: when the transmission is interrupt based, the callback
                                              is being called upon sending a byte; when DMA transmission
                                              is used, the bytes are copied to the tx buffer by the DMA engine
                                              and the callback is called when all the bytes have been transferred. */
    void * txCallbackParam;              /*!< Transmit callback parameter pointer.*/
    lpuart_transfer_type_t transferType; /*!< Type of LPUART transfer (interrupt/dma based) */
#if FEATURE_LPUART_HAS_DMA_ENABLE
    uint8_t rxDMAChannel;                /*!< DMA channel number for DMA-based rx. */
    uint8_t txDMAChannel;                /*!< DMA channel number for DMA-based tx. */
#endif
    semaphore_t rxComplete;              /*!< Synchronization object for blocking Rx timeout condition */
    semaphore_t txComplete;              /*!< Synchronization object for blocking Tx timeout condition */
    volatile status_t transmitStatus;    /*!< Status of last driver transmit operation */
    volatile status_t receiveStatus;     /*!< Status of last driver receive operation */
} lpuart_state_t;

/*! @brief LPUART configuration structure
 *
 * Implements : lpuart_user_config_t_Class
 */
typedef struct
{
    uint32_t baudRate;                           /*!< LPUART baud rate */
    lpuart_parity_mode_t parityMode;             /*!< parity mode, disabled (default), even, odd */
    lpuart_stop_bit_count_t stopBitCount;        /*!< number of stop bits, 1 stop bit (default) or 2 stop bits */
    lpuart_bit_count_per_char_t bitCountPerChar; /*!< number of bits in a character (8-default, 9 or 10);
                                                      for 9/10 bits chars, users must provide appropriate buffers
                                                      to the send/receive functions (bits 8/9 in subsequent bytes);
                                                      for DMA transmission only 8-bit char is supported. */
    lpuart_transfer_type_t transferType;         /*!< Type of LPUART transfer (interrupt/dma based) */
    uint8_t rxDMAChannel;                        /*!< Channel number for DMA rx channel.
                                                      If DMA mode isn't used this field will be ignored. */
    uint8_t txDMAChannel;                        /*!< Channel number for DMA tx channel.
                                                      If DMA mode isn't used this field will be ignored. */
} lpuart_user_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name LPUART Driver
 * @{
 */

/*!
 * @brief Initializes an LPUART operation instance.
 *
 * The caller provides memory for the driver state structures during initialization.
 * The user must select the LPUART clock source in the application to initialize the LPUART.
 *
 * @param instance  LPUART instance number
 * @param lpuartUserConfig user configuration structure of type #lpuart_user_config_t
 * @param lpuartStatePtr pointer to the LPUART driver state structure
 * @return STATUS_SUCCESS if successful;
 *         STATUS_ERROR if an error occured
 */
status_t LPUART_DRV_Init(uint32_t instance, lpuart_state_t * lpuartStatePtr,
                         const lpuart_user_config_t * lpuartUserConfig);

/*!
 * @brief Shuts down the LPUART by disabling interrupts and transmitter/receiver.
 *
 * @param instance  LPUART instance number
 * @return STATUS_SUCCESS if successful;
 *         STATUS_ERROR if an error occured
 */
status_t LPUART_DRV_Deinit(uint32_t instance);

/*!
 * @brief Installs callback function for the LPUART receive.
 *
 * @note After a callback is installed, it bypasses part of the LPUART IRQHandler logic.
 * Therefore, the callback needs to handle the indexes of txBuff and txSize.
 *
 * @param instance The LPUART instance number.
 * @param function The LPUART receive callback function.
 * @param rxBuff The receive buffer used inside IRQHandler. This buffer must be kept as long as the callback is alive.
 * @param callbackParam The LPUART receive callback parameter pointer.
 * @return Former LPUART receive callback function pointer.
 */
lpuart_rx_callback_t LPUART_DRV_InstallRxCallback(uint32_t instance,
                                                  lpuart_rx_callback_t function,
                                                  void * callbackParam);

/*!
 * @brief Installs callback function for the LPUART transmit.
 *
 * @note After a callback is installed, it bypasses part of the LPUART IRQHandler logic.
 * Therefore, the callback needs to handle the indexes of txBuff and txSize.
 *
 * @param instance The LPUART instance number.
 * @param function The LPUART transmit callback function.
 * @param txBuff The transmit buffer used inside IRQHandler. This buffer must be kept as long as the callback is alive.
 * @param callbackParam The LPUART transmit callback parameter pointer.
 * @return Former LPUART transmit callback function pointer.
 */
lpuart_tx_callback_t LPUART_DRV_InstallTxCallback(uint32_t instance,
                                                  lpuart_tx_callback_t function,
                                                  void * callbackParam);

/*!
 * @brief Sends data out through the LPUART module using a blocking method.
 *
 *  Blocking means that the function does not return until the transmission is complete.
 *
 * @param instance  LPUART instance number
 * @param txBuff  source buffer containing 8-bit data chars to send
 * @param txSize the number of bytes to send
 * @param timeout timeout value in milliseconds
 * @return STATUS_SUCCESS if successful;
 *         STATUS_TIMEOUT if the timeout was reached;
 *         STATUS_BUSY if a resource is busy;
 *         STATUS_ERROR if an error occured
 */
status_t LPUART_DRV_SendDataBlocking(uint32_t instance,
                                     const uint8_t * txBuff,
                                     uint32_t txSize,
                                     uint32_t timeout);

/*!
 * @brief Sends data out through the LPUART module using a non-blocking method.
 *  This enables an a-sync method for transmitting data. When used with
 *  a non-blocking receive, the LPUART can perform a full duplex operation.
 *  Non-blocking  means that the function returns immediately.
 *  The application has to get the transmit status to know when the transmit is complete.
 *
 * @param instance  LPUART instance number
 * @param txBuff  source buffer containing 8-bit data chars to send
 * @param txSize  the number of bytes to send
 * @return STATUS_SUCCESS if successful;
 *         STATUS_BUSY if a resource is busy;
 *         STATUS_ERROR if an error occured
 */
status_t LPUART_DRV_SendData(uint32_t instance,
                             const uint8_t * txBuff,
                             uint32_t txSize);

/*!
 * @brief Returns whether the previous transmit is complete.
 *
 * @param instance  LPUART instance number
 * @param bytesRemaining Pointer to value that is populated with the number of bytes that
 *      have been sent in the active transfer
 * @return The transmit status.
 * @retval STATUS_SUCCESS The transmit has completed successfully.
 * @retval STATUS_BUSY The transmit is still in progress. @a bytesTransmitted will be
 *     filled with the number of bytes that have been transmitted so far.
 * @retval STATUS_UART_ABORTED The transmit was aborted.
 * @retval STATUS_TIMEOUT A timeout was reached.
 * @retval STATUS_ERROR An error occured.
 */
status_t LPUART_DRV_GetTransmitStatus(uint32_t instance, uint32_t * bytesRemaining);

/*!
 * @brief Terminates a non-blocking transmission early.
 *
 * @param instance  LPUART instance number
 * @return Whether the aborting is successful or not.
 */
status_t LPUART_DRV_AbortSendingData(uint32_t instance);

/*!
 * @brief Gets data from the LPUART module by using a blocking method.
 *  Blocking means that the function does not return until the
 *  receive is complete.
 *
 * @param instance  LPUART instance number
 * @param rxBuff  buffer containing 8-bit read data chars received
 * @param rxSize the number of bytes to receive
 * @param timeout timeout value in milliseconds
 * @return STATUS_SUCCESS if successful;
 *         STATUS_TIMEOUT if the timeout was reached;
 *         STATUS_BUSY if a resource is busy;
 *         STATUS_ERROR if an error occured
 */
status_t LPUART_DRV_ReceiveDataBlocking(uint32_t instance,
                                        uint8_t * rxBuff,
                                        uint32_t rxSize,
                                        uint32_t timeout);

/*!
 * @brief Gets data from the LPUART module by using a non-blocking method.
 *  This enables an a-sync method for receiving data. When used with
 *  a non-blocking transmission, the LPUART can perform a full duplex operation.
 *  Non-blocking means that the function returns immediately.
 *  The application has to get the receive status to know when the receive is complete.
 *
 * @param instance  LPUART instance number
 * @param rxBuff  buffer containing 8-bit read data chars received
 * @param rxSize  the number of bytes to receive
 * @return STATUS_SUCCESS if successful;
 *         STATUS_BUSY if a resource is busy;
 *         STATUS_ERROR if an error occured
 */
status_t LPUART_DRV_ReceiveData(uint32_t instance,
                                uint8_t * rxBuff,
                                uint32_t rxSize);

/*!
 * @brief Returns whether the previous receive is complete.
 *
 * @param instance  LPUART instance number
 * @param bytesRemaining pointer to value that is filled  with the number of bytes that
 *        still need to be received in the active transfer.
 * @return The receive status.
 * @retval STATUS_SUCCESS the receive has completed successfully.
 * @retval STATUS_BUSY the receive is still in progress. @a bytesReceived will be
 *     filled with the number of bytes that have been received so far.
 * @retval STATUS_UART_ABORTED The receive was aborted.
 * @retval STATUS_TIMEOUT A timeout was reached.
 * @retval STATUS_ERROR An error occured.
 */
status_t LPUART_DRV_GetReceiveStatus(uint32_t instance, uint32_t * bytesRemaining);

/*!
 * @brief Terminates a non-blocking receive early.
 *
 * @param instance  LPUART instance number
 *
 * @return Whether the receiving was successful or not.
 */
status_t LPUART_DRV_AbortReceivingData(uint32_t instance);

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* LPUART_DRIVER_H__ */
/******************************************************************************/
/* EOF */
/******************************************************************************/
