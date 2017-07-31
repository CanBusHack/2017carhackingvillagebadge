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
 * @lpspi_slave_driver.c
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
 * Violates MISRA 2012 Advisory Rule 2.7, Symbol 'status' (line 783) not referenced.
 * This parameter is not used because the DMA callback doesn't need this, but must be defined to
 * ensure the API compatibility for callback.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.1, Object/function previously declared.
 * This requirement is fulfilled since the function is declared as external in and only in
 * one configuration C file.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.2, Identifier clash.
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.4, Identifier clash.
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.5, Identifier clash.
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, Could define variable at block scope.
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
#include "lpspi_slave_driver.h"
#include "clock_manager.h"
#include "interrupt_manager.h"
#include "lpspi_shared_function.h"
#include "S32K144_features.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Callback for DMA transfer done.*/
static void LPSPI_DRV_SlaveCompleteDMATransfer(void* parameter, edma_chn_status_t status);

/*******************************************************************************
 * Code
 ******************************************************************************/
 /*
  * Implements : LPSPI_DRV_SlaveInit_Activity
  */
status_t LPSPI_DRV_SlaveInit(uint32_t instance,
                               lpspi_state_t * lpspiState,
                               const lpspi_slave_config_t * slaveConfig)
{
    DEV_ASSERT(lpspiState != NULL);
    DEV_ASSERT(slaveConfig != NULL);
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    LPSPI_Type * base = g_lpspiBase[instance];
    status_t errorCode = STATUS_SUCCESS;

    lpspiState->lsb = slaveConfig->lsbFirst;
    lpspiState->bitsPerFrame = slaveConfig->bitcount;
    lpspiState->transferType = slaveConfig->transferType;
    lpspiState->isBlocking = false;
    /* Store DMA channels numbers used for DMA transfer */
    lpspiState->rxDMAChannel = slaveConfig->rxDMAChannel;
    lpspiState->txDMAChannel = slaveConfig->txDMAChannel;
    /* Calculate the bytes/frame for lpspiState->bytesPerFrame. */
    lpspiState->bytesPerFrame = (uint16_t)((lpspiState->bitsPerFrame + 7U) / 8U);
    /* For DMA transfers bytes per frame must be equal to 1, 2 or multiple of 4. */
    if ((lpspiState->transferType == LPSPI_USING_DMA) && (!(((lpspiState->bytesPerFrame % 4U) == 0U) ||
            (lpspiState->bytesPerFrame <= 2U))))
    {
        return STATUS_ERROR;
    }
    lpspiState->isTransferInProgress = false;
    /* Initialize the semaphore */
    errorCode = OSIF_SemaCreate(&(lpspiState->lpspiSemaphore), 0);
    DEV_ASSERT(errorCode == STATUS_SUCCESS);
    g_lpspiStatePtr[instance] = lpspiState;

    /* Configure registers */
    LPSPI_HAL_Init(base);

    /* Configure lpspi to slave mode */
    (void)LPSPI_HAL_SetMasterSlaveMode(base, LPSPI_SLAVE);
    /* Set Pin settings */
    (void)LPSPI_HAL_SetPinConfigMode(base, LPSPI_SDI_IN_SDO_OUT, LPSPI_DATA_OUT_RETAINED, true);
    /* Calculate the FIFO size for the LPSPI */
    LPSPI_HAL_GetFifoSizes(base, &(lpspiState->fifoSize), NULL);

    /* Set polarity */
    (void)LPSPI_HAL_SetPcsPolarityMode(base, LPSPI_PCS0,slaveConfig->pcsPolarity);

     /* Write the TCR for this transfer */
    lpspi_tx_cmd_config_t txCmdCfg = {
        .frameSize = lpspiState->bitsPerFrame,
        .width = LPSPI_SINGLE_BIT_XFER,
        .txMask = false,
        .rxMask = false,
        .byteSwap = false,
        .lsbFirst = slaveConfig->lsbFirst,
        .clkPhase = slaveConfig->clkPhase,
        .clkPolarity = slaveConfig->clkPolarity,
        .whichPcs = slaveConfig->whichPcs
    };

    /* Write to the TX CMD register */
    LPSPI_HAL_SetTxCommandReg(base, &txCmdCfg);
    LPSPI_HAL_Enable(base);
    /* Enable the interrupt source */
    INT_SYS_EnableIRQ(g_lpspiIrqId[instance]);

    return errorCode;
}

 /*
  * Implements : LPSPI_DRV_SlaveDeinit_Activity
  */
status_t LPSPI_DRV_SlaveDeinit(uint32_t instance)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    /* Instantiate local variable of type lpspi_master_state_t and point to global state */
    const lpspi_state_t * lpspiState = (lpspi_state_t *)g_lpspiStatePtr[instance];
    LPSPI_Type *base = g_lpspiBase[instance];
    status_t errorCode = STATUS_SUCCESS;

    /* Check if a transfer is still in progress */
    DEV_ASSERT(lpspiState->isTransferInProgress == false);
    /* Destroy the semaphore */
    errorCode = OSIF_SemaDestroy(&(lpspiState->lpspiSemaphore));
    DEV_ASSERT(errorCode == STATUS_SUCCESS);
    /* Reset the LPSPI registers to their default state, including disabling the LPSPI */
    LPSPI_HAL_Init(base);

    /* Disable the interrupt*/
    INT_SYS_DisableIRQ(g_lpspiIrqId[instance]);

    /* Clear the state pointer. */
    g_lpspiStatePtr[instance] = NULL;

    return errorCode;
}

 /*
  * Implements : LPSPI_DRV_SlaveTransferBlocking_Activity
  */
status_t LPSPI_DRV_SlaveTransferBlocking(uint32_t instance,
                                           const uint8_t *sendBuffer,
                                           uint8_t *receiveBuffer,
                                           uint16_t transferByteCount,
                                           uint32_t timeout)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    lpspi_state_t * state = (lpspi_state_t *)g_lpspiStatePtr[instance];
    status_t error;
    status_t osifError;
    state->isBlocking = true;
    error = LPSPI_DRV_SlaveTransfer(instance, sendBuffer, receiveBuffer, transferByteCount);
    if(error != STATUS_SUCCESS)
    {
        LPSPI_DRV_DisableTEIEInterrupts(instance);
        return error;
    }

    /* As this is a synchronous transfer, wait until the transfer is complete.*/
    osifError = OSIF_SemaWait(&(state->lpspiSemaphore), timeout);

    if (osifError == STATUS_TIMEOUT)
    {
        /* Set isBlocking variable to false to avoid dummy semaphore post. */
        state->isBlocking = false;
        /* Complete transfer. */
        (void)LPSPI_DRV_SlaveAbortTransfer(instance);
        return(STATUS_TIMEOUT);
    }

    LPSPI_DRV_DisableTEIEInterrupts(instance);

    return STATUS_SUCCESS;
}

 /*
  * Implements : LPSPI_DRV_SlaveTransfer_Activity
  */
status_t LPSPI_DRV_SlaveTransfer(uint32_t instance,
                                   const uint8_t *sendBuffer,
                                   uint8_t *receiveBuffer,
                                   uint16_t transferByteCount)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(!((sendBuffer == NULL) && (receiveBuffer == NULL)));
    LPSPI_Type * base = g_lpspiBase[instance];
    DMA_Type *baseDma = g_edmaBase[LPSPI_DMA_INSTANCE];
    lpspi_state_t * state = (lpspi_state_t *)g_lpspiStatePtr[instance];
    edma_transfer_size_t dmaTransferSize = EDMA_TRANSFER_SIZE_1B;
    /* Check if a transfer is still in progress */
    DEV_ASSERT(state->isTransferInProgress == false);
    /* The number of transferred bytes should be divisible by frame size */
    if ((uint16_t)(transferByteCount % state->bytesPerFrame) != (uint16_t)0)
    {
        return STATUS_ERROR;
    }
    /* Check if LPSPI module isn't busy */
    if (LPSPI_HAL_GetStatusFlag(base, LPSPI_MODULE_BUSY))
    {
        return STATUS_BUSY;
    }
    /* Initialize the status of the current transfer */
    state->status = LPSPI_TRANSFER_OK;
    /* Clear all interrupts sources */
    (void)LPSPI_HAL_ClearStatusFlag(base, LPSPI_ALL_STATUS);
    /* Enable fault interrupts sources */
    LPSPI_HAL_SetIntMode(base,LPSPI_TRANSMIT_ERROR , true);
    LPSPI_HAL_SetIntMode(base,LPSPI_RECEIVE_ERROR , true);
    if (state->transferType == LPSPI_USING_INTERRUPTS)
    {
        state->rxBuff = receiveBuffer;
        state->txBuff = sendBuffer;
        state->txCount = transferByteCount;
        state->rxCount = transferByteCount;
        state->txFrameCnt = 0;
        state->rxFrameCnt = 0;
        state->isPcsContinuous = false;
        /* Configure watermarks */
        LPSPI_HAL_SetRxWatermarks(base, 0U);
        LPSPI_HAL_SetTxWatermarks(base, 2U);

        /* Clean RX and TX buffers */
        LPSPI_HAL_SetFlushFifoCmd(base, true, true);
        /* The second flush command is used to avoid the case when one word is still in shifter. */
        LPSPI_HAL_SetFlushFifoCmd(base, true, true);
        state->isTransferInProgress = true;
        /* Enable interrupts for RX and TX only if it's necessary */
        if(state->txBuff != NULL)
        {
            LPSPI_HAL_SetIntMode(base,LPSPI_TX_DATA_FLAG , true);
        }
        else
        {
            state->txCount = 0;
        }
        if(state->rxBuff != NULL)
        {
            LPSPI_HAL_SetIntMode(base, LPSPI_RX_DATA_FLAG, true);
        }
        else
        {
            state->rxCount = 0;
        }
    }
    else
    {
        /* Configure watermarks */
        LPSPI_HAL_SetRxWatermarks(base, 0U);
        LPSPI_HAL_SetTxWatermarks(base, 3U);
        /* When LPSPI use DMA frames with 3 bytes size are not accepted. */
        switch(state->bytesPerFrame)
        {
            case 1: dmaTransferSize = EDMA_TRANSFER_SIZE_1B; break;
            case 2: dmaTransferSize = EDMA_TRANSFER_SIZE_2B; break;
            case 4: dmaTransferSize = EDMA_TRANSFER_SIZE_4B; break;
            default: dmaTransferSize = EDMA_TRANSFER_SIZE_1B; break;
        }
        if(receiveBuffer != NULL)
        {
            (void)EDMA_DRV_ConfigSingleBlockTransfer(state->rxDMAChannel, EDMA_TRANSFER_PERIPH2MEM,
                    (uint32_t)(&(base->RDR)),(uint32_t)receiveBuffer, dmaTransferSize, (uint32_t)1U<<(uint8_t)(dmaTransferSize));
            EDMA_HAL_TCDSetMajorCount(baseDma, state->rxDMAChannel, (uint32_t)transferByteCount/(uint32_t)((uint32_t)1U <<(uint8_t)(dmaTransferSize)));
            /* Disable DMA requests for RX channel when transfer is done. */
            EDMA_HAL_TCDSetDisableDmaRequestAfterTCDDoneCmd(baseDma, state->rxDMAChannel, true);
            state->rxCount = transferByteCount;
            /* Start RX channel */
            (void)EDMA_DRV_StartChannel(state->rxDMAChannel);
        }
        else
        {
            state->rxCount = 0;
        }
        if(sendBuffer != NULL)
        {
            (void)EDMA_DRV_ConfigSingleBlockTransfer(state->txDMAChannel, EDMA_TRANSFER_MEM2PERIPH,
                            (uint32_t)sendBuffer, (uint32_t)(&(base->TDR)), dmaTransferSize, (uint32_t)1U<<(uint8_t)(dmaTransferSize));
            EDMA_HAL_TCDSetMajorCount(baseDma, state->txDMAChannel, (uint32_t)transferByteCount/(uint32_t)((uint32_t)1U <<(uint8_t)(dmaTransferSize)));
            /* Disable DMA requests for RX channel when transfer is done. */
            EDMA_HAL_TCDSetDisableDmaRequestAfterTCDDoneCmd(baseDma, state->txDMAChannel, true);
            state->txCount = transferByteCount;
            /* Start TX channel */
            (void)EDMA_DRV_StartChannel(state->txDMAChannel);
        }
        else
        {
            state->txCount = 0;
        }
        /* Configure which channel will generate transfer complete */
        /* If current transfer uses both buffers (RX and TX) RX transfer done will generate transfer complete
         * interrupt. Otherwise transfer complete will be generate by available channel(RX or TX).
         */
        if(receiveBuffer != NULL)
        {
            (void)EDMA_DRV_InstallCallback(state->rxDMAChannel, (LPSPI_DRV_SlaveCompleteDMATransfer),(void*)(instance));
        }
        else
        {
            (void)EDMA_DRV_InstallCallback(state->txDMAChannel, (LPSPI_DRV_SlaveCompleteDMATransfer),(void*)(instance));
        }
        state->isTransferInProgress = true;
        /* Enable LPSPI DMA request */
        if (receiveBuffer != NULL)
        {
            LPSPI_HAL_SetRxDmaCmd(base, true);
        }
        if (sendBuffer != NULL)
        {
            LPSPI_HAL_SetTxDmaCmd(base, true);
        }
    }
    return STATUS_SUCCESS;
}

void LPSPI_DRV_SlaveIRQHandler(uint32_t instance)
{
    LPSPI_Type * base = g_lpspiBase[instance];
    lpspi_state_t * lpspiState = (lpspi_state_t *)g_lpspiStatePtr[instance];
    uint16_t txCount, rxCount;

    /* If an error is detected the transfer will be aborted */
    if ((bool)LPSPI_HAL_GetStatusFlag(base, LPSPI_TRANSMIT_ERROR))
    {
        lpspiState->status = LPSPI_TRANSMIT_FAIL;
        (void)LPSPI_DRV_SlaveAbortTransfer(instance);
        return;
    }
    if (LPSPI_HAL_GetStatusFlag(base, LPSPI_RECEIVE_ERROR))
    {
        lpspiState->status = LPSPI_RECEIVE_FAIL;
        (void)LPSPI_DRV_SlaveAbortTransfer(instance);
        return;
    }

    /* Receive data */
    if(LPSPI_HAL_GetStatusFlag(base,LPSPI_RX_DATA_FLAG))
    {
        LPSPI_DRV_ReadRXBuffer(instance);
    }
    /* Transmit data */
    txCount = lpspiState->txCount;
    if (LPSPI_HAL_GetStatusFlag(base,LPSPI_TX_DATA_FLAG) && ((txCount != (uint8_t)0)))
    {
        LPSPI_DRV_FillupTxBuffer(instance);
    }
    txCount = lpspiState->txCount;
    rxCount = lpspiState->rxCount;
    /* If all bytes are sent disable interrupt TDF */
    if (txCount == (uint8_t)0)
    {
        LPSPI_HAL_SetIntMode(base, LPSPI_TX_DATA_FLAG, false);
    }
    /* If all bytes are received disable interrupt RDF */
    if (rxCount == (uint8_t)0)
    {
        LPSPI_HAL_SetIntMode(base, LPSPI_RX_DATA_FLAG, false);
    }
    if ((rxCount == (uint8_t)0) && (txCount == (uint8_t)0))
    {
        lpspiState->isTransferInProgress = false;
        if(lpspiState->isBlocking == true)
        {
            (void)OSIF_SemaPost(&(lpspiState->lpspiSemaphore));
            lpspiState->isBlocking = false;
        }
    }
}

 /*
  * Implements : LPSPI_DRV_SlaveAbortTransfer_Activity
  */
status_t LPSPI_DRV_SlaveAbortTransfer(uint32_t instance)
{
    LPSPI_Type * base = g_lpspiBase[instance];
    lpspi_state_t * state = (lpspi_state_t *)g_lpspiStatePtr[instance];

    if (state->transferType == LPSPI_USING_INTERRUPTS)
    {
        /* Disable interrupts */
        LPSPI_HAL_SetIntMode(base, LPSPI_TX_DATA_FLAG, false);
        LPSPI_HAL_SetIntMode(base, LPSPI_RX_DATA_FLAG, false);
    }
    else
    {
        /* Disable LPSPI DMA request */
        LPSPI_HAL_SetRxDmaCmd(base, false);
        LPSPI_HAL_SetTxDmaCmd(base, false);
    }

    LPSPI_DRV_DisableTEIEInterrupts(instance);

    state->isTransferInProgress = false;
    /* Clean RX and TX buffers */
    LPSPI_HAL_SetFlushFifoCmd(base, true, true);
    /* The second flush command is used to avoid the case when one word is still in shifter. */
    LPSPI_HAL_SetFlushFifoCmd(base, true, true);
    if(state->isBlocking == true)
    {
        (void)OSIF_SemaPost(&(state->lpspiSemaphore));
        state->isBlocking = false;
    }
    return STATUS_SUCCESS;
}

 /*
  * Implements : LPSPI_DRV_SlaveGetTransferStatus_Activity
  */
status_t LPSPI_DRV_SlaveGetTransferStatus(uint32_t instance,uint32_t * bytesRemained)
{
    const lpspi_state_t * lpspiState = (lpspi_state_t *)g_lpspiStatePtr[instance];

    /* Fill in the bytes transferred.*/
    if (bytesRemained != NULL)
    {
        *bytesRemained = lpspiState->txCount;
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

/*!
 * @brief Finish up a transfer DMA.
 * The main purpose of this function is to create a function compatible with DMA callback type
 */
static void LPSPI_DRV_SlaveCompleteDMATransfer(void* parameter, edma_chn_status_t status)
{
    uint32_t instance = (uint32_t)parameter;

    (void)status;
    (void)LPSPI_DRV_SlaveAbortTransfer(instance);
}
