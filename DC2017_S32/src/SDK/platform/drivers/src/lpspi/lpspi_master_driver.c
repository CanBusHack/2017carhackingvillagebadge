/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
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
 * @lpspi_master_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable. A source of error in writing
 * dynamic code is that the stack segment may be different from the data segment.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.7, Symbol 'status' not referenced
 * This parameter is not used because the DMA callback doesn't need this, but must be defined to
 * ensure the API compatibility for callback.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.1, Object/function previously declared.
 * This requirement is fulfilled since the function is declared as external in and only in
 * one configuration C file.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.2, Identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.4, Identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.5, Identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * The return statement before end of function is used for simpler code
 * structure and better readability.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, Could define variable at block scope
 * The variables are defined in the common source file and this rule can't be
 * applied.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower or different essential type.
 * The cast is required to perform a conversion between an unsigned integer and an enum type.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type.
 * This conversion is required because the converted values are the addresses used in DMA transfer.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from pointer to unsigned long.
 * The cast is required to initialize a DMA transfer. The converted value is the address of a buffer.
 * Cast from unsigned long to pointer. The cast is required to perform a conversion between a pointer
 * and an unsigned long define, representing an address or vice versa.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * The return statement before end of function is used for simpler code
 * structure and better readability.
 *
 */

#include <string.h>
#include "lpspi_master_driver.h"
#include "clock_manager.h"
#include "interrupt_manager.h"
#include "lpspi_shared_function.h"
#include "S32K144_features.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MICROSECONDS 1000000
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* This function initialize a new SPI transfer */
static status_t LPSPI_DRV_MasterStartTransfer(uint32_t instance,
                                                    const uint8_t * sendBuffer,
                                                    uint8_t * receiveBuffer,
                                                    uint16_t transferByteCount);

/* This function cleans up state structure and hardware after a transfer is complete .*/
static void LPSPI_DRV_MasterCompleteTransfer(uint32_t instance);

/* Callback for DMA transfer done.*/
static void LPSPI_DRV_MasterCompleteDMATransfer(void* parameter, edma_chn_status_t status);

/*! The main purpose of this function is to clear continuous mode. */
static void LPSPI_DRV_MasterClearCountinuous(void* parameter, edma_chn_status_t status);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_DRV_MasterInit
 * Description   : Initializes a LPSPI instance for interrupt driven master mode operation.
 *
 * This function uses an interrupt-driven method for transferring data.
 * In this function, the term "spiConfig" is used to indicate the SPI device for which the LPSPI
 * master is communicating.
 * This function initializes the run-time state structure to track the ongoing
 * transfers, un-gates the clock to the LPSPI module, resets the LPSPI module,
 * configures the IRQ state structure, enables the module-level interrupt to the core, and
 * enables the LPSPI module.
 * Implements : LPSPI_DRV_MasterInit_Activity
 *
 *END**************************************************************************/
status_t LPSPI_DRV_MasterInit(uint32_t instance, lpspi_state_t * lpspiState,
                                    const lpspi_master_config_t * spiConfig)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(lpspiState != NULL);
    DEV_ASSERT(spiConfig != NULL);
    LPSPI_Type *base = g_lpspiBase[instance];
    status_t errorCode = STATUS_SUCCESS;

    /* Save runtime structure pointers so irq handler can point to the correct state structure */
    g_lpspiStatePtr[instance] = lpspiState;
    /* Reset the LPSPI registers to their default state */
    LPSPI_HAL_Init(base);
    /* Set for master mode */
    (void)LPSPI_HAL_SetMasterSlaveMode(base, LPSPI_MASTER);
    /* Set Pin configuration such that SDO=out and SDI=in */
    (void)LPSPI_HAL_SetPinConfigMode(base, LPSPI_SDI_IN_SDO_OUT, LPSPI_DATA_OUT_RETAINED, true);
    /* Calculate the FIFO size for the LPSPI */
    LPSPI_HAL_GetFifoSizes(base, &(lpspiState->fifoSize), NULL);

    /* Configure bus for this device. If NULL is passed, we assume the caller has
     * preconfigured the bus and doesn't wish to re-configure it again for this transfer.
     * Do nothing for calculatedBaudRate. If the user wants to know the calculatedBaudRate
     * then they can call this function separately.
     */
    errorCode = LPSPI_DRV_MasterConfigureBus(instance, spiConfig, NULL);
    if (errorCode != STATUS_SUCCESS)
    {
        return errorCode;
    }
    /* Initialize the semaphore */
    errorCode = OSIF_SemaCreate(&(lpspiState->lpspiSemaphore), 0);
    DEV_ASSERT(errorCode == STATUS_SUCCESS);
    /* Enable the interrupt */
    INT_SYS_EnableIRQ(g_lpspiIrqId[instance]);
    /* Finally, enable LPSPI */
    LPSPI_HAL_Enable(base);
    return errorCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_DRV_MasterDeinit
 * Description   : Shuts down a LPSPI instance.
 *
 * This function resets the LPSPI peripheral, gates its clock, and disables the interrupt to
 * the core.  It first checks to see if a transfer is in progress and if so returns an error
 * status.
 * Implements : LPSPI_DRV_MasterDeinit_Activity
 *
 *END**************************************************************************/
status_t LPSPI_DRV_MasterDeinit(uint32_t instance)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    /* Instantiate local variable of type lpspi_state_t and point to global state */
    const lpspi_state_t * lpspiState = g_lpspiStatePtr[instance];
    LPSPI_Type *base = g_lpspiBase[instance];
    status_t errorCode = STATUS_SUCCESS;

    /* Check if a transfer is still in progress */
    DEV_ASSERT(lpspiState->isTransferInProgress == false);

    /* Reset the LPSPI registers to their default state, inlcuding disabling the LPSPI */
    LPSPI_HAL_Init(base);
    /* Disable the interrupt */
    INT_SYS_DisableIRQ(g_lpspiIrqId[instance]);
    /* Clear the state pointer. */
    g_lpspiStatePtr[instance] = NULL;

    /* Destroy the semaphore */
    errorCode = OSIF_SemaDestroy(&(lpspiState->lpspiSemaphore));
    DEV_ASSERT(errorCode == STATUS_SUCCESS);
    return errorCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_DRV_MasterSetDelay
 * Description   : Configures the LPSPI master mode bus timing delay options.
 *
 * This function involves the LPSPI module's delay options to
 * "fine tune" some of the signal timings and match the timing needs of a slower peripheral device.
 * This is an optional function that can be called after the LPSPI module has been initialized for
 * master mode. The timings are adjusted in terms of cycles of the baud rate clock.
 * The bus timing delays that can be adjusted are listed below:
 *
 * SCK to PCS Delay: Adjustable delay option between the last edge of SCK to the de-assertion
 *                   of the PCS signal.
 *
 * PCS to SCK Delay: Adjustable delay option between the assertion of the PCS signal to the
 *                   first SCK edge.
 *
 * Delay between Transfers: Adjustable delay option between the de-assertion of the PCS signal for
 *                          a frame to the assertion of the PCS signal for the next frame.
 * Implements : LPSPI_DRV_MasterSetDelay_Activity
 *
 *END**************************************************************************/
status_t LPSPI_DRV_MasterSetDelay(uint32_t instance, uint32_t delayBetwenTransfers,
                uint32_t delaySCKtoPCS, uint32_t delayPCStoSCK)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    /* Instantiate local variable of type lpspi_state_t and point to global state */
    const lpspi_state_t * lpspiState = g_lpspiStatePtr[instance];
    LPSPI_Type *base = g_lpspiBase[instance];
    status_t errorCode = STATUS_SUCCESS;
    uint32_t realDelayBetwenTransfers, realDelaySCKtoPCS, realDelayPCStoSCK;
    lpspi_prescaler_t prescaler;
    /* Disable module */
    errorCode = LPSPI_HAL_Disable(base);
    if (errorCode != STATUS_SUCCESS)
    {
        return errorCode;
    }
    prescaler = LPSPI_HAL_GetClockPrescaler(base);
    realDelayBetwenTransfers = delayBetwenTransfers * lpspiState->lpspiSrcClk / s_baudratePrescaler[prescaler] / (uint32_t)MICROSECONDS;
    realDelaySCKtoPCS = delaySCKtoPCS * lpspiState->lpspiSrcClk / s_baudratePrescaler[prescaler] / (uint32_t)MICROSECONDS;
    realDelayPCStoSCK = delayPCStoSCK * lpspiState->lpspiSrcClk/ s_baudratePrescaler[prescaler] / (uint32_t)MICROSECONDS;
    /* Verify if current prescaler can be used for this configuration of delays
     * If a delay is out of range, it will be adjusted to min or max value */
    if (realDelayBetwenTransfers > (uint32_t)257U)
    {
        realDelayBetwenTransfers = (uint32_t)257U;
    }
    if(realDelaySCKtoPCS > (uint32_t)256U)
    {
        realDelaySCKtoPCS = (uint32_t)256U;
    }
    if(realDelayPCStoSCK > (uint32_t)256U)
    {
        realDelayPCStoSCK = (uint32_t)256U;
    }
    if (realDelayBetwenTransfers < (uint32_t)2U)
    {
        realDelayBetwenTransfers = (uint32_t)2U;
    }
    if(realDelaySCKtoPCS == (uint32_t)0)
    {
        realDelaySCKtoPCS = (uint32_t)1U;
    }
    if(realDelayPCStoSCK == (uint32_t)0U)
    {
        realDelayPCStoSCK = (uint32_t)1U;
    }

    (void)LPSPI_HAL_SetDelay(base, LPSPI_SCK_TO_PCS, realDelaySCKtoPCS-1U);
    (void)LPSPI_HAL_SetDelay(base, LPSPI_PCS_TO_SCK, realDelayPCStoSCK-1U);
    (void)LPSPI_HAL_SetDelay(base, LPSPI_BETWEEN_TRANSFER, realDelayBetwenTransfers-2U);
    /* Enable module */
    LPSPI_HAL_Enable(base);
    return errorCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_DRV_MasterConfigureBus
 * Description   : Configures the LPSPI port physical parameters to access a device on the bus when
 *                 the LSPI instance is configured for interrupt operation.
 *
 * In this function, the term "spiConfig" is used to indicate the SPI device for which the LPSPI
 * master is communicating. This is an optional function as the spiConfig parameters are
 * normally configured in the initialization function or the transfer functions, where these various
 * functions would call the configure bus function.
 * The user can pass in a different spiConfig structure to the transfer function which contains
 * the parameters for the SPI bus to allow for communication to a different SPI device
 * (the transfer function then calls this function). However, the user also has the option to call
 * this function directly especially to get the calculated baud rate, at which point they may pass
 * in NULL for the spiConfig structure in the transfer function (assuming they have called this
 * configure bus function first).
 * Implements : LPSPI_DRV_MasterConfigureBus_Activity
 *
 *END**************************************************************************/
status_t LPSPI_DRV_MasterConfigureBus(uint32_t instance,
                                            const lpspi_master_config_t * spiConfig,
                                            uint32_t * calculatedBaudRate)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(spiConfig != NULL);
    /* Instantiate local variable of type lpspi_state_t and point to global state */
    lpspi_state_t * lpspiState = g_lpspiStatePtr[instance];
    LPSPI_Type *base = g_lpspiBase[instance];
    uint32_t baudRate;

    /* The Transmit Command Register (TCR) Prescale value is calculated as part of the baud rate
       calculation. The value is stored in the run-time state structure for later programming
       in the TCR. */
    uint32_t tcrPrescaleValue;

    /* Check the bitcount to make sure it falls within the boundary conditions */
    if ((spiConfig->bitcount < 8U) || (spiConfig->bitcount > 4096U))
    {
        return STATUS_ERROR;
    }
    /* First, per the spec, we need to disable the LPSPI module before setting the delay */

    if (LPSPI_HAL_Disable(base) != STATUS_SUCCESS)
    {
        /* If error is returned, the LPSPI is busy */
        return STATUS_ERROR;
    }
    /* Configure internal state structure for LPSPI */
    lpspiState->bitsPerFrame = spiConfig->bitcount;
    lpspiState->lpspiSrcClk = spiConfig->lpspiSrcClk;
    lpspiState->isPcsContinuous = spiConfig->isPcsContinuous;
    lpspiState->lsb = spiConfig->lsbFirst;
    /* Save transfer type DMA/Interrupt */
    lpspiState->transferType = spiConfig->transferType;
    /* Update transfer status */
    lpspiState->isTransferInProgress = false;
    lpspiState->isBlocking = false;
    /* Calculate the bytes/frame for lpspiState->bytesPerFrame. */
    lpspiState->bytesPerFrame = (uint16_t)((lpspiState->bitsPerFrame + 7U) / 8U);
    /* For DMA transfers bytes per frame must be equal to 1, 2 or multiple of 4 */
    if ((lpspiState->transferType == LPSPI_USING_DMA) && (!(((lpspiState->bytesPerFrame % 4U) == (uint16_t)0) ||
            (lpspiState->bytesPerFrame<=2U))))
    {
        return STATUS_ERROR;
    }
    /* Store DMA channel number used in transfer */
    lpspiState->rxDMAChannel = spiConfig->rxDMAChannel;
    lpspiState->txDMAChannel = spiConfig->txDMAChannel;

    /* Configure the desired PCS polarity */
    (void)LPSPI_HAL_SetPcsPolarityMode(base, spiConfig->whichPcs, spiConfig->pcsPolarity);


    /* Set up the baud rate */
    baudRate = LPSPI_HAL_SetBaudRate(base, spiConfig->bitsPerSec, spiConfig->lpspiSrcClk,
                                     &tcrPrescaleValue);

    /* Now, re-enable the LPSPI module */
    LPSPI_HAL_Enable(base);

    /* If the baud rate return is "0", it means there was an error */
    if (baudRate == (uint32_t)0)
    {
        return STATUS_ERROR;
    }

    /* If the user wishes to know the calculated baud rate, then pass it back */
    if (calculatedBaudRate != NULL)
    {
        *calculatedBaudRate = baudRate;
    }
    /* Write the TCR for this transfer. */
    lpspi_tx_cmd_config_t txCmdCfg =
    {
        .frameSize = lpspiState->bitsPerFrame,
        .width = LPSPI_SINGLE_BIT_XFER,
        .txMask = false,
        .rxMask = false,
        .contCmd = false,
        .contTransfer = spiConfig->isPcsContinuous,
        .byteSwap = false,
        .lsbFirst = spiConfig->lsbFirst,
        .whichPcs = spiConfig->whichPcs,
        .preDiv = tcrPrescaleValue,
        .clkPhase = spiConfig->clkPhase,
        .clkPolarity = spiConfig->clkPolarity
     };
    LPSPI_HAL_SetTxCommandReg(base, &txCmdCfg);
    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_DRV_MasterTransferBlocking
 * Description   : Performs an interrupt driven blocking SPI master mode transfer.
 *
 * This function simultaneously sends and receives data on the SPI bus, as SPI is naturally
 * a full-duplex bus. The function does not return until the transfer is complete.
 * This function allows the user to optionally pass in a SPI configuration structure which
 * allows the user to change the SPI bus attributes in conjunction with initiating a SPI transfer.
 * The difference between passing in the SPI configuration structure here as opposed to the
 * configure bus function is that the configure bus function returns the calculated baud rate where
 * this function does not. The user can also call the configure bus function prior to the transfer
 * in which case the user would simply pass in a NULL to the transfer function's device structure
 * parameter.
 * Implements : LPSPI_DRV_MasterTransferBlocking_Activity
 *
 *END**************************************************************************/
status_t LPSPI_DRV_MasterTransferBlocking(uint32_t instance,
                                                const uint8_t * sendBuffer,
                                                uint8_t * receiveBuffer,
                                                uint16_t transferByteCount,
                                                uint32_t timeout)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    /* Instantiate local variable of type lpspi_state_t and point to global state */
    lpspi_state_t * lpspiState = g_lpspiStatePtr[instance];
    LPSPI_Type *base = g_lpspiBase[instance];
    status_t error = STATUS_SUCCESS;
    status_t osifError;
    /* If the transfer count is zero, then return immediately.*/
    if (transferByteCount == (uint16_t)0)
    {
        return error;
    }
    lpspiState->isBlocking = true;
    /* start the transfer process, if it returns an error code, return this back to user */
    error = LPSPI_DRV_MasterStartTransfer(instance, sendBuffer, receiveBuffer,
                                          transferByteCount);
    if (error != STATUS_SUCCESS)
    {
        /* The transfer is complete.*/
        lpspiState->isTransferInProgress = false;
        /* Disable interrupt requests*/
        /*LPSPI_CLR_IER(base, LPSPI_IER_TDIE_MASK|LPSPI_IER_RDIE_MASK);*/
        LPSPI_HAL_SetIntMode(base, LPSPI_TX_DATA_FLAG, false);
        LPSPI_HAL_SetIntMode(base, LPSPI_RX_DATA_FLAG, false);

        LPSPI_DRV_DisableTEIEInterrupts(instance);
        LPSPI_HAL_SetIntMode(base, LPSPI_TRANSFER_COMPLETE, false);
        (void)LPSPI_HAL_ClearStatusFlag(base, LPSPI_TRANSFER_COMPLETE);

        return error;
    }

    /* As this is a synchronous transfer, wait until the transfer is complete.*/
    osifError = OSIF_SemaWait(&(lpspiState->lpspiSemaphore), timeout);

    /* If a timeout occurs, stop the transfer by setting the isTransferInProgress to false and
     * disabling interrupts, then return the timeout error status.
     */
    if (osifError == STATUS_TIMEOUT)
    {
        /* Set isBlocking variable to false to avoid dummy semaphore post. */
        lpspiState->isBlocking = false;
        /* Complete transfer. */
        LPSPI_DRV_MasterCompleteTransfer(instance);
        return(STATUS_TIMEOUT);
    }

    LPSPI_DRV_DisableTEIEInterrupts(instance);
    LPSPI_HAL_SetIntMode(base, LPSPI_TRANSFER_COMPLETE, false);
    (void)LPSPI_HAL_ClearStatusFlag(base, LPSPI_TRANSFER_COMPLETE);

    return error;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_DRV_MasterTransfer
 * Description   : Performs an interrupt driven non-blocking SPI master mode transfer.
 *
 * This function simultaneously sends and receives data on the SPI bus, as SPI is naturally
 * a full-duplex bus. The function returns immediately after initiating the transfer. The user
 * needs to check whether the transfer is complete using the LPSPI_DRV_MasterGetTransferStatus
 * function.
 * This function allows the user to optionally pass in a SPI configuration structure which
 * allows the user to change the SPI bus attributes in conjunction with initiating a SPI transfer.
 * The difference between passing in the SPI configuration structure here as opposed to the
 * configure bus function is that the configure bus function returns the calculated baud rate where
 * this function does not. The user can also call the configure bus function prior to the transfer
 * in which case the user would simply pass in a NULL to the transfer function's device structure
 * parameter.
 * Implements : LPSPI_DRV_MasterTransfer_Activity
 *
 *END**************************************************************************/
status_t LPSPI_DRV_MasterTransfer(uint32_t instance,
                                        const uint8_t * sendBuffer,
                                        uint8_t * receiveBuffer,
                                        uint16_t transferByteCount)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    status_t error = STATUS_SUCCESS;
    /* If the transfer count is zero, then return immediately.*/
    if (transferByteCount == (uint16_t)0)
    {
        return STATUS_SUCCESS;
    }

    /* Start the transfer process, if it returns an error code, return this back to user */
    error = LPSPI_DRV_MasterStartTransfer(instance, sendBuffer, receiveBuffer,
                                          transferByteCount);
    if (error != STATUS_SUCCESS)
    {
        return error;
    }

    /* Else, return immediately as this is an async transfer */
    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_DRV_MasterGetTransferStatus
 * Description   : Returns whether the previous interrupt driven transfer is completed.
 *
 * When performing an a-sync (non-blocking) transfer, the user can call this function to ascertain
 * the state of the current transfer: in progress (or busy) or complete (success).
 * In addition, if the transfer is still in progress, the user can get the number of words that
 * should be receive.
 * Implements : LPSPI_DRV_MasterGetTransferStatus_Activity
 *
 *END**************************************************************************/
status_t LPSPI_DRV_MasterGetTransferStatus(uint32_t instance, uint32_t * bytesRemained)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    /* Instantiate local variable of type lpspi_state_t and point to global state */
    const lpspi_state_t * lpspiState = g_lpspiStatePtr[instance];
    /* Fill in the bytes transferred.*/
    if (bytesRemained != NULL)
    {
        *bytesRemained = lpspiState->rxCount;
    }
    if (lpspiState->status == LPSPI_TRANSFER_OK)
    {
        return (status_t)(lpspiState->isTransferInProgress ? STATUS_BUSY : STATUS_SUCCESS);
    }
    else
    {
        return STATUS_ERROR;
    }
}
/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_DRV_MasterAbortTransfer
 * Description   : Terminates an interrupt driven asynchronous transfer early.
 *
 * During an a-sync (non-blocking) transfer, the user has the option to terminate the transfer early
 * if the transfer is still in progress.
 * Implements : LPSPI_DRV_MasterAbortTransfer_Activity
 *END**************************************************************************/
status_t LPSPI_DRV_MasterAbortTransfer(uint32_t instance)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    LPSPI_Type *base = g_lpspiBase[instance];
    /* Stop the running transfer. */
    LPSPI_DRV_MasterCompleteTransfer(instance);
    LPSPI_HAL_SetFlushFifoCmd(base, true, true);
    /* The second flush command is used to avoid the case when one word is still in shifter. */
    LPSPI_HAL_SetFlushFifoCmd(base, true, true);
    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_DRV_MasterStartTransfer
 * Description   : Configure a non-blocking transfer.
 *
 * The number of transferByteCount must be divided by number of bytes/frame.
 * The sendBuffer must be not NULL, but receiveBuffer can be NULL.
 *
 *END**************************************************************************/
static status_t LPSPI_DRV_MasterStartTransfer(uint32_t instance,
                                                    const uint8_t * sendBuffer,
                                                    uint8_t * receiveBuffer,
                                                    uint16_t transferByteCount)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(sendBuffer != NULL);
    /* Instantiate local variable of type dspi_master_state_t and point to global state */
    lpspi_state_t * lpspiState = g_lpspiStatePtr[instance];
    LPSPI_Type *base = g_lpspiBase[instance];
    DMA_Type *baseDma = g_edmaBase[LPSPI_DMA_INSTANCE];
    edma_transfer_size_t dmaTransferSize = EDMA_TRANSFER_SIZE_1B;

    /* Check that we're not busy. */
    if (LPSPI_HAL_GetStatusFlag(base, LPSPI_MODULE_BUSY))
    {
        return STATUS_BUSY;
    }
    #ifdef ERRATA_E10655
    else
    {
        /* Double check to fix errata e10655. */
        if (LPSPI_HAL_GetStatusFlag(base, LPSPI_MODULE_BUSY))
        {
            return STATUS_BUSY;
        }
    }
    #endif

    /* Verify if the number of bytes is divided by number of bytes/frame. */
    if ((transferByteCount % lpspiState->bytesPerFrame) != (uint16_t)0)
    {
        return STATUS_ERROR;
    }
    if(lpspiState->isPcsContinuous == true)
    {
        LPSPI_HAL_SetContCBit(base);
    }

    /* Configure watermarks */
    LPSPI_HAL_SetRxWatermarks(base, 0U);
    LPSPI_HAL_SetTxWatermarks(base, 2U);

    lpspiState->status = LPSPI_TRANSFER_OK;
    /* Clear all interrupts sources */
    (void)LPSPI_HAL_ClearStatusFlag(base, LPSPI_ALL_STATUS);
    /* Enable fault interrupts sources */
    LPSPI_HAL_SetIntMode(base,LPSPI_TRANSMIT_ERROR , true);
    LPSPI_HAL_SetIntMode(base,LPSPI_RECEIVE_ERROR , true);

    if (lpspiState->transferType == LPSPI_USING_INTERRUPTS)
    {
        /* Fill out the other members of the run-time state structure. */
        lpspiState->txBuff = (const uint8_t *)sendBuffer;
        lpspiState->rxBuff = (uint8_t *)receiveBuffer;
        lpspiState->txFrameCnt = 0;
        lpspiState->rxFrameCnt = 0;
        lpspiState->txCount = transferByteCount;
        /*For continuous mode an extra word must be written to negate the PCS */
        if (lpspiState->isPcsContinuous == true)
        {
            lpspiState->txCount++;
        }
        /* Clean RX and TX buffers */
        LPSPI_HAL_SetFlushFifoCmd(base, true, true);
        /* The second flush command is used to avoid the case when one word is still in shifter. */
        LPSPI_HAL_SetFlushFifoCmd(base, true, true);
        /* Update transfer status */
        lpspiState->isTransferInProgress = true;
        /* Mask the RX if no buffer is passed in. */
        if (lpspiState->rxBuff == NULL)
        {
            /* Since we're not receiving, set rxCount to 0. */
            lpspiState->rxCount = 0;
        }
        else
        {
            /* Enable RDF interrupt if RX buffer is not NULL. */
            LPSPI_HAL_SetIntMode(base, LPSPI_RX_DATA_FLAG, true);
            lpspiState->rxCount = transferByteCount;
        }
        /* Enable the TDF and RDF interrupt. */
        LPSPI_HAL_SetIntMode(base, LPSPI_TX_DATA_FLAG, true);
    }
    else
    {
        /* Configure rxCount depending on transfer type.*/
        if (receiveBuffer == NULL)
        {
            lpspiState->rxCount = 0;
        }
        else
        {
            lpspiState->rxCount = transferByteCount;
        }

        /* When LPSPI use DMA frames with 3 bytes size are not accepted. */
        switch(lpspiState->bytesPerFrame)
        {
            case 1: dmaTransferSize = EDMA_TRANSFER_SIZE_1B; break;
            case 2: dmaTransferSize = EDMA_TRANSFER_SIZE_2B; break;
            case 4: dmaTransferSize = EDMA_TRANSFER_SIZE_4B; break;
            default : dmaTransferSize = EDMA_TRANSFER_SIZE_1B; break;
        }
        /* Configure TX DMA channel */
        (void)EDMA_DRV_ConfigSingleBlockTransfer(lpspiState->txDMAChannel, EDMA_TRANSFER_MEM2PERIPH,
                (uint32_t)sendBuffer, (uint32_t)(&(base->TDR)), dmaTransferSize, (uint32_t)1U<<(uint8_t)(dmaTransferSize));
        EDMA_HAL_TCDSetMajorCount(baseDma, lpspiState->txDMAChannel, (uint32_t)transferByteCount/(uint32_t)((uint32_t)1U <<(uint8_t)(dmaTransferSize)));
        /* Disable DMA requests for TX channel when transfer is done. */
        EDMA_HAL_TCDSetDisableDmaRequestAfterTCDDoneCmd(baseDma, lpspiState->txDMAChannel, true);
        /* Start TX channel */
        (void)EDMA_DRV_StartChannel(lpspiState->txDMAChannel);
        /* Configure RX DMA channel if is used in current transfer. */
        if(receiveBuffer != NULL)
        {
            (void)EDMA_DRV_ConfigSingleBlockTransfer(lpspiState->rxDMAChannel, EDMA_TRANSFER_PERIPH2MEM,
                                (uint32_t)(&(base->RDR)),(uint32_t)receiveBuffer, dmaTransferSize, (uint32_t)1U<<(uint8_t)(dmaTransferSize));
            EDMA_HAL_TCDSetMajorCount(baseDma, lpspiState->rxDMAChannel, (uint32_t)transferByteCount/(uint32_t)((uint32_t)1U <<(uint8_t)(dmaTransferSize)));
            /* Transfer is done when all bytes were received. */

            (void)EDMA_DRV_InstallCallback(lpspiState->rxDMAChannel, (LPSPI_DRV_MasterCompleteDMATransfer),(void*)(instance));
            if (lpspiState->isPcsContinuous == true)
            {
                (void)EDMA_DRV_InstallCallback(lpspiState->txDMAChannel, (LPSPI_DRV_MasterClearCountinuous),(void*)(instance));
            }
            /* Disable DMA requests for RX channel when transfer is done. */
            EDMA_HAL_TCDSetDisableDmaRequestAfterTCDDoneCmd(baseDma, lpspiState->rxDMAChannel, true);
            /* Start RX channel */
            (void)EDMA_DRV_StartChannel(lpspiState->rxDMAChannel);
        }
        else
        {
            /* If RX buffer is null the transfer is done when all bytes were sent. */
            (void)EDMA_DRV_InstallCallback(lpspiState->txDMAChannel, (LPSPI_DRV_MasterCompleteDMATransfer),(void*)(instance));
        }
        /* Update transfer status */
        lpspiState->isTransferInProgress = true;
        /* Enable LPSPI DMA request */
        if (receiveBuffer!=NULL)
        {
            LPSPI_HAL_SetRxDmaCmd(base, true);
        }
        LPSPI_HAL_SetTxDmaCmd(base, true);
    }
    return STATUS_SUCCESS;
}

/*!
 * @brief Finish up a transfer.
 * Cleans up after a transfer is complete. Interrupts are disabled, and the LPSPI module
 * is disabled. This is not a public API as it is called from other driver functions.
 */
static void LPSPI_DRV_MasterCompleteTransfer(uint32_t instance)
{
    /* instantiate local variable of type dspi_master_state_t and point to global state */
    lpspi_state_t * lpspiState = g_lpspiStatePtr[instance];
    LPSPI_Type *base = g_lpspiBase[instance];
    /* The transfer is complete.*/
    lpspiState->isTransferInProgress = false;
    if(lpspiState->transferType == LPSPI_USING_DMA)
    {
        /* Disable LPSPI DMA request */
        LPSPI_HAL_SetRxDmaCmd(base, false);
        LPSPI_HAL_SetTxDmaCmd(base, false);
    }
    else
    {
        /* Disable (clear) interrupt requests */
        LPSPI_HAL_SetIntMode(base, LPSPI_RX_DATA_FLAG, false);
        LPSPI_HAL_SetIntMode(base, LPSPI_TX_DATA_FLAG, false);
    }

    LPSPI_DRV_DisableTEIEInterrupts(instance);
    LPSPI_HAL_SetIntMode(base, LPSPI_TRANSFER_COMPLETE, false);
    (void)LPSPI_HAL_ClearStatusFlag(base, LPSPI_TRANSFER_COMPLETE);
    if (lpspiState->isBlocking == true)
    {
        (void)OSIF_SemaPost(&(lpspiState->lpspiSemaphore));
        lpspiState->isBlocking = false;
    }

}

/*!
 * @brief Finish up a transfer DMA.
 * The main purpose of this function is to create a function compatible with DMA callback type
 */
static void LPSPI_DRV_MasterCompleteDMATransfer(void* parameter, edma_chn_status_t status)
{
    uint32_t instance = (uint32_t)parameter;

    (void)status;
    /* Disable continuous PCS */
    LPSPI_DRV_MasterCompleteTransfer(instance);
}

/*!
 * @brief Clear the continuous mode.
 * The main purpose of this function is to clear continuous mode
 */
static void LPSPI_DRV_MasterClearCountinuous(void* parameter, edma_chn_status_t status)
{
    uint32_t instance = (uint32_t)parameter;
    LPSPI_Type *base = g_lpspiBase[instance];

    (void)status;
    /* Disable continuous PCS */
    LPSPI_HAL_ClearContCBit(base);
}

/*!
 * @brief Interrupt handler for LPSPI master mode.
 * This handler uses the buffers stored in the lpspi_state_t structs to transfer data.
 * This is not a public API as it is called whenever an interrupt occurs.
 */
void LPSPI_DRV_MasterIRQHandler(uint32_t instance)
{
    /* Instantiate local variable of type dspi_master_state_t and point to global state */
    lpspi_state_t * lpspiState = g_lpspiStatePtr[instance];
    LPSPI_Type *base = g_lpspiBase[instance];
    uint16_t txCount, rxCount;
    txCount = lpspiState->txCount;
    rxCount = lpspiState->rxCount;

    /* If an error is detected the transfer will be aborted */
    if ((bool)LPSPI_HAL_GetStatusFlag(base, LPSPI_TRANSMIT_ERROR))
    {
        lpspiState->status = LPSPI_TRANSMIT_FAIL;
        (void)LPSPI_DRV_MasterAbortTransfer(instance);
        (void)LPSPI_HAL_ClearStatusFlag(base, LPSPI_TRANSMIT_ERROR);
        return;
    }
    if (LPSPI_HAL_GetStatusFlag(base, LPSPI_RECEIVE_ERROR))
    {
        lpspiState->status = LPSPI_RECEIVE_FAIL;
        (void)LPSPI_DRV_MasterAbortTransfer(instance);
        (void)LPSPI_HAL_ClearStatusFlag(base, LPSPI_RECEIVE_ERROR);
        return;
    }

    /* RECEIVE IRQ handler: Check read buffer only if there are remaining bytes to read. */
    if(LPSPI_HAL_GetStatusFlag(base,LPSPI_RX_DATA_FLAG) && (rxCount != (uint16_t)0))
    {
        LPSPI_DRV_ReadRXBuffer(instance);
    }
    /* Transmit data */
    if (LPSPI_HAL_GetStatusFlag(base,LPSPI_TX_DATA_FLAG) && ((txCount != (uint16_t)0)))
    {
        LPSPI_DRV_FillupTxBuffer(instance);
    }
    txCount = lpspiState->txCount;
    rxCount = lpspiState->rxCount;
    if (txCount == (uint16_t)0)
    {
        /* Disable TX flag. Software buffer is empty.*/
        LPSPI_HAL_SetIntMode(base, LPSPI_TX_DATA_FLAG, false);
        LPSPI_HAL_SetIntMode(base, LPSPI_TRANSFER_COMPLETE, true);
        /* Check if we're done with this transfer.*/
        if (rxCount == (uint16_t)0)
        {
            if (LPSPI_HAL_GetStatusFlag(base, LPSPI_TRANSFER_COMPLETE) == true)
            {
                LPSPI_DRV_MasterCompleteTransfer(instance);
            }
        }
    }
}


/*******************************************************************************
 * EOF
 ******************************************************************************/
