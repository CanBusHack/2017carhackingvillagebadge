/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
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
 * @file edma_hal.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 */

#include "edma_hal.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_Init
 * Description   : Initializes eDMA module to known state.
 *
 * Implements    : EDMA_HAL_Init_Activity
 *END**************************************************************************/
void EDMA_HAL_Init(DMA_Type * base)
{
    uint32_t i;

    /* Clear the bit of CR register */
    uint32_t regValTemp;
    regValTemp = base->CR;
    regValTemp &= ~(DMA_CR_CLM_MASK);
    regValTemp &= ~(DMA_CR_CX_MASK);
    regValTemp &= ~(DMA_CR_ECX_MASK);
    regValTemp &= ~(DMA_CR_EDBG_MASK);
    regValTemp &= ~(DMA_CR_EMLM_MASK);
    regValTemp &= ~(DMA_CR_ERCA_MASK);
    base->CR = regValTemp;

    for (i = 0; i < FEATURE_EDMA_MODULE_CHANNELS; i++)
    {
        EDMA_HAL_TCDClearReg(base, i);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_CancelTransfer
 * Description   : Cancels the remaining data transfer.
 *
 * Implements    : EDMA_HAL_CancelTransfer_Activity
 *END**************************************************************************/
void EDMA_HAL_CancelTransfer(DMA_Type * base)
{
    uint32_t regValTemp;
    regValTemp = base->CR;
    regValTemp &= ~(DMA_CR_CX_MASK);
    regValTemp |= DMA_CR_CX(1U);
    base->CR = regValTemp;
    while (((base->CR & DMA_CR_CX_MASK) >> DMA_CR_CX_SHIFT) != 0UL)
    {}
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_CancelTransferWithError
 * Description   : Cancels the remaining data transfer and treat it as error.
 *
 * Implements    : EDMA_HAL_CancelTransferWithError_Activity
 *END**************************************************************************/
void EDMA_HAL_CancelTransferWithError(DMA_Type * base)
{
    uint32_t regValTemp;
    regValTemp = base->CR;
    regValTemp &= ~(DMA_CR_ECX_MASK);
    regValTemp |= DMA_CR_ECX(1U);
    base->CR = regValTemp;
    while (((base->CR & DMA_CR_ECX_MASK) >> DMA_CR_ECX_SHIFT) != 0UL)
    {}
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_SetErrorIntCmd
 * Description   : Enable/Disable error interrupt for channels.
 *
 * Implements    : EDMA_HAL_SetErrorIntCmd_Activity
 *END**************************************************************************/
void EDMA_HAL_SetErrorIntCmd(DMA_Type * base, uint8_t channel, bool enable)
{
    if (enable)
    {
        base->SEEI = channel;
    }
    else
    {
        base->CEEI = channel;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_SetDmaRequestCmd
 * Description   : Enable/Disable dma request for channel or all channels.
 *
 * Implements    : EDMA_HAL_SetDmaRequestCmd_Activity
 *END**************************************************************************/
void EDMA_HAL_SetDmaRequestCmd(DMA_Type * base, uint8_t channel,bool enable)
{

    if (enable)
    {
        base->SERQ = channel;
    }
    else
    {
        base->CERQ = channel;
    }
}

#if FEATURE_EDMA_CHANNEL_GROUP_COUNT > 0x1U
/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_SetGroupPriority
 * Description   : Configures DMA group priorities.
 *
 * Implements    : EDMA_HAL_SetGroupPriority_Activity
 *END**************************************************************************/
void EDMA_HAL_SetGroupPriority(DMA_Type * base, edma_group_priority_t priority)
{
    uint32_t regValTemp;
    uint32_t mask = DMA_CR_GRP0PRI_MASK | DMA_CR_GRP1PRI_MASK;
    regValTemp = base->CR;
    regValTemp &= ~mask;
    if (priority == EDMA_GRP0_PRIO_HIGH_GRP1_PRIO_LOW)
    {
        regValTemp |= DMA_CR_GRP0PRI_MASK;
    }
    else
    {
        regValTemp |= DMA_CR_GRP1PRI_MASK;
    }

    base->CR = regValTemp;
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDClearReg
 * Description   : Set registers to 0 for hardware TCD of eDMA channel.
 *
 * Implements    : EDMA_HAL_TCDClearReg_Activity
 *END**************************************************************************/
void EDMA_HAL_TCDClearReg(DMA_Type * base,uint32_t channel)
{
    base->TCD[channel].SADDR = 0U;
    base->TCD[channel].SOFF = 0;
    base->TCD[channel].ATTR = 0U;
    base->TCD[channel].NBYTES.MLNO = 0U;
    base->TCD[channel].SLAST = 0;
    base->TCD[channel].DADDR = 0U;
    base->TCD[channel].DOFF = 0;
    base->TCD[channel].CITER.ELINKNO = 0U;
    base->TCD[channel].DLASTSGA = 0;
    base->TCD[channel].CSR = 0U;
    base->TCD[channel].BITER.ELINKNO = 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDSetAttribute
 * Description   : Configures the transfer attribute for eDMA channel.
 *
 * Implements    : EDMA_HAL_TCDSetAttribute_Activity
 *END**************************************************************************/
void EDMA_HAL_TCDSetAttribute(
                DMA_Type * base, uint32_t channel,
                edma_modulo_t srcModulo, edma_modulo_t destModulo,
                edma_transfer_size_t srcTransferSize, edma_transfer_size_t destTransferSize)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FEATURE_EDMA_MODULE_CHANNELS);
#endif
    uint16_t regValTemp;

    regValTemp = (uint16_t)(((uint16_t)srcModulo << DMA_TCD_ATTR_SMOD_SHIFT) | ((uint16_t)srcTransferSize << DMA_TCD_ATTR_SSIZE_SHIFT));
    regValTemp |= (uint16_t)(((uint16_t)destModulo << DMA_TCD_ATTR_DMOD_SHIFT) | ((uint16_t)destTransferSize << DMA_TCD_ATTR_DSIZE_SHIFT));
    base->TCD[channel].ATTR = regValTemp;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDSetNbytes
 * Description   : Configures the nbytes for eDMA channel.
 *
 * Implements    : EDMA_HAL_TCDSetNbytes_Activity
 *END**************************************************************************/
void EDMA_HAL_TCDSetNbytes(DMA_Type * base, uint32_t channel, uint32_t nbytes)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FEATURE_EDMA_MODULE_CHANNELS);
#endif

    if (((base->CR & DMA_CR_EMLM_MASK) >> DMA_CR_EMLM_SHIFT) != 0UL)
    {
        bool mlOffNo = false;
        if (((base->TCD[channel].NBYTES.MLOFFNO & DMA_TCD_NBYTES_MLOFFNO_SMLOE_MASK) >> DMA_TCD_NBYTES_MLOFFNO_SMLOE_SHIFT) == 0UL)
        {
            if (((base->TCD[channel].NBYTES.MLOFFNO & DMA_TCD_NBYTES_MLOFFNO_DMLOE_MASK) >> DMA_TCD_NBYTES_MLOFFNO_DMLOE_SHIFT) == 0UL)
            {
                base->TCD[channel].NBYTES.MLOFFNO = (nbytes & DMA_TCD_NBYTES_MLOFFNO_NBYTES_MASK);
                mlOffNo = true;
            }
        }
        if (!mlOffNo)
        {
            uint32_t regValTemp;

            regValTemp = base->TCD[channel].NBYTES.MLOFFYES;
            regValTemp &= ~(DMA_TCD_NBYTES_MLOFFYES_NBYTES_MASK);
            regValTemp |= DMA_TCD_NBYTES_MLOFFYES_NBYTES(nbytes);
            base->TCD[channel].NBYTES.MLOFFYES = regValTemp;
        }

    }
    else
    {
        base->TCD[channel].NBYTES.MLNO = nbytes;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDGetNbytes
 * Description   : Get nbytes configuration data.
 *
 * Implements    : EDMA_HAL_TCDGetNbytes_Activity
 *END**************************************************************************/
uint32_t EDMA_HAL_TCDGetNbytes(const DMA_Type * base, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FEATURE_EDMA_MODULE_CHANNELS);
#endif
    uint32_t result = 0UL;

    if (((base->CR & DMA_CR_EMLM_MASK) >> DMA_CR_EMLM_SHIFT) != 0UL)
    {
        bool mlOffNo = false;
        if (((base->TCD[channel].NBYTES.MLOFFNO & DMA_TCD_NBYTES_MLOFFNO_SMLOE_MASK) >> DMA_TCD_NBYTES_MLOFFNO_SMLOE_SHIFT) != 0UL)
        {
            mlOffNo = true;
        }
        if (((base->TCD[channel].NBYTES.MLOFFNO & DMA_TCD_NBYTES_MLOFFNO_DMLOE_MASK) >> DMA_TCD_NBYTES_MLOFFNO_DMLOE_SHIFT) != 0UL)
        {
            mlOffNo = true;
        }
        if (mlOffNo)
        {
            result = ((base->TCD[channel].NBYTES.MLOFFYES & DMA_TCD_NBYTES_MLOFFYES_NBYTES_MASK) >> DMA_TCD_NBYTES_MLOFFYES_NBYTES_SHIFT);
        }
        else
        {
            result = ((base->TCD[channel].NBYTES.MLOFFNO & DMA_TCD_NBYTES_MLOFFNO_NBYTES_MASK) >> DMA_TCD_NBYTES_MLOFFNO_NBYTES_SHIFT);
        }
    }
    else
    {
        result = ((base->TCD[channel].NBYTES.MLNO & DMA_TCD_NBYTES_MLNO_NBYTES_MASK) >> DMA_TCD_NBYTES_MLNO_NBYTES_SHIFT);
    }
    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDSetMinorLoopOffset
 * Description   : Configures the minor loop offset for the TCD.
 *
 * Implements    : EDMA_HAL_TCDSetMinorLoopOffset_Activity
 *END**************************************************************************/
void EDMA_HAL_TCDSetMinorLoopOffset(DMA_Type * base, uint32_t channel, int32_t offset)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FEATURE_EDMA_MODULE_CHANNELS);
#endif

    if (((base->CR & DMA_CR_EMLM_MASK) >> DMA_CR_EMLM_SHIFT) != 0UL)
    {
        bool mlOffNo = false;
        if (((base->TCD[channel].NBYTES.MLOFFNO & DMA_TCD_NBYTES_MLOFFNO_SMLOE_MASK) >> DMA_TCD_NBYTES_MLOFFNO_SMLOE_SHIFT) != 0UL)
        {
            mlOffNo = true;
        }
        if (((base->TCD[channel].NBYTES.MLOFFNO & DMA_TCD_NBYTES_MLOFFNO_DMLOE_MASK) >> DMA_TCD_NBYTES_MLOFFNO_DMLOE_SHIFT) != 0UL)
        {
            mlOffNo = true;
        }
        if (mlOffNo)
        {
            uint32_t regValTemp;

            regValTemp = base->TCD[channel].NBYTES.MLOFFYES;
            regValTemp &= ~(DMA_TCD_NBYTES_MLOFFYES_MLOFF_MASK);
            regValTemp |= DMA_TCD_NBYTES_MLOFFYES_MLOFF(offset);
            base->TCD[channel].NBYTES.MLOFFYES = regValTemp;
        }
    }
}
/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDSetScatterGatherLink
 * Description   : Configures the memory address of the next TCD, in Scatter/Gather mode.
 *
 * Implements    : EDMA_HAL_TCDSetScatterGatherLink_Activity
 *END**************************************************************************/
void EDMA_HAL_TCDSetScatterGatherLink(DMA_Type * base, uint32_t channel, uint32_t nextTCDAddr)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FEATURE_EDMA_MODULE_CHANNELS);
#endif
    base->TCD[channel].DLASTSGA = nextTCDAddr;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDSetChannelMinorLink
 * Description   : Set Channel minor link for hardware TCD.
 *
 * Implements    : EDMA_HAL_TCDSetChannelMinorLink_Activity
 *END**************************************************************************/
void EDMA_HAL_TCDSetChannelMinorLink(
                DMA_Type * base, uint32_t channel, uint32_t linkChannel, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FEATURE_EDMA_MODULE_CHANNELS);
    DEV_ASSERT(linkChannel < FEATURE_EDMA_MODULE_CHANNELS);
#endif

    uint16_t regValTemp;
    regValTemp = base->TCD[channel].BITER.ELINKYES;
    regValTemp &= (uint16_t)~(DMA_TCD_BITER_ELINKYES_ELINK_MASK);
    regValTemp |= (uint16_t)DMA_TCD_BITER_ELINKYES_ELINK(enable ? 1UL : 0UL);
    base->TCD[channel].BITER.ELINKYES = regValTemp;
    regValTemp = base->TCD[channel].CITER.ELINKYES;
    regValTemp &= (uint16_t)~(DMA_TCD_CITER_ELINKYES_ELINK_MASK);
    regValTemp |= (uint16_t)DMA_TCD_CITER_ELINKYES_ELINK(enable ? 1UL : 0UL);
    base->TCD[channel].CITER.ELINKYES = regValTemp;

    if (enable)
    {

        regValTemp = base->TCD[channel].BITER.ELINKYES;
        regValTemp &= (uint16_t)~(DMA_TCD_BITER_ELINKYES_LINKCH_MASK);
        regValTemp |= (uint16_t)DMA_TCD_BITER_ELINKYES_LINKCH(linkChannel);
        base->TCD[channel].BITER.ELINKYES = regValTemp;

        regValTemp = base->TCD[channel].CITER.ELINKYES;
        regValTemp &= (uint16_t)~(DMA_TCD_CITER_ELINKYES_LINKCH_MASK);
        regValTemp |= (uint16_t)DMA_TCD_CITER_ELINKYES_LINKCH(linkChannel);
        base->TCD[channel].CITER.ELINKYES = regValTemp;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCD_TCDSetMajorCount
 * Description   : Sets the major iteration count according to minor loop
 * channel link setting.
 *
 * Implements    : EDMA_HAL_TCDSetMajorCount_Activity
 *END**************************************************************************/
void EDMA_HAL_TCDSetMajorCount(DMA_Type * base, uint32_t channel, uint32_t count)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FEATURE_EDMA_MODULE_CHANNELS);
#endif

    uint16_t regValTemp;
    if (((base->TCD[channel].BITER.ELINKNO & DMA_TCD_BITER_ELINKNO_ELINK_MASK) >> DMA_TCD_BITER_ELINKNO_ELINK_SHIFT) != 0U)
    {
        regValTemp = base->TCD[channel].BITER.ELINKYES;
        regValTemp &= (uint16_t)~(DMA_TCD_BITER_ELINKYES_BITER_MASK);
        regValTemp |= (uint16_t)DMA_TCD_BITER_ELINKYES_BITER(count);
        base->TCD[channel].BITER.ELINKYES = regValTemp;

        regValTemp = base->TCD[channel].CITER.ELINKYES;
        regValTemp &= (uint16_t)~(DMA_TCD_CITER_ELINKYES_CITER_LE_MASK);
        regValTemp |= (uint16_t)DMA_TCD_CITER_ELINKYES_CITER_LE(count);
        base->TCD[channel].CITER.ELINKYES = regValTemp;
    }
    else
    {
        regValTemp = base->TCD[channel].BITER.ELINKNO;
        regValTemp &= (uint16_t)~(DMA_TCD_BITER_ELINKNO_BITER_MASK);
        regValTemp |= (uint16_t)DMA_TCD_BITER_ELINKNO_BITER(count);
        base->TCD[channel].BITER.ELINKNO = regValTemp;

        regValTemp = base->TCD[channel].CITER.ELINKNO;
        regValTemp &= (uint16_t)~(DMA_TCD_CITER_ELINKNO_CITER_MASK);
        regValTemp |= (uint16_t)DMA_TCD_CITER_ELINKNO_CITER(count);
        base->TCD[channel].CITER.ELINKNO = regValTemp;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDGetBeginMajorCount
 * Description   : Gets the begin major iteration count according to minor loop
 * channel link setting.
 *
 * Implements    : EDMA_HAL_TCDGetBeginMajorCount_Activity
 *END**************************************************************************/
uint32_t EDMA_HAL_TCDGetBeginMajorCount(const DMA_Type * base, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FEATURE_EDMA_MODULE_CHANNELS);
#endif
    uint16_t result = 0U;
    if (((base->TCD[channel].BITER.ELINKNO & DMA_TCD_BITER_ELINKNO_ELINK_MASK) >> DMA_TCD_BITER_ELINKNO_ELINK_SHIFT) != 0U)
    {
        result = (uint16_t)((base->TCD[channel].BITER.ELINKYES & DMA_TCD_BITER_ELINKYES_BITER_MASK) >> DMA_TCD_BITER_ELINKYES_BITER_SHIFT);
    }
    else
    {
        result = (uint16_t)((base->TCD[channel].BITER.ELINKNO & DMA_TCD_BITER_ELINKNO_BITER_MASK) >> DMA_TCD_BITER_ELINKNO_BITER_SHIFT);
    }
    return (uint32_t) result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDGetCurrentMajorCount
 * Description   : Gets the current major iteration count according to minor
 * loop channel link setting.
 *
 * Implements    : EDMA_HAL_TCDGetCurrentMajorCount_Activity
 *END**************************************************************************/
uint32_t EDMA_HAL_TCDGetCurrentMajorCount(const DMA_Type * base, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FEATURE_EDMA_MODULE_CHANNELS);
#endif

    uint16_t result = 0U;
    if (((base->TCD[channel].BITER.ELINKNO & DMA_TCD_BITER_ELINKNO_ELINK_MASK) >> DMA_TCD_BITER_ELINKNO_ELINK_SHIFT) != 0U)
    {
        result = (uint16_t)((base->TCD[channel].CITER.ELINKYES & DMA_TCD_CITER_ELINKYES_CITER_LE_MASK) >> DMA_TCD_CITER_ELINKYES_CITER_LE_SHIFT);
    }
    else
    {
        result = (uint16_t)((base->TCD[channel].CITER.ELINKNO & DMA_TCD_CITER_ELINKNO_CITER_MASK) >> DMA_TCD_CITER_ELINKNO_CITER_SHIFT);
    }
    return (uint32_t) result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDGetUnfinishedBytes
 * Description   : Get the bytes number of bytes haven't been transferred for
 * this hardware TCD.
 *
 * Implements    : EDMA_HAL_TCDGetUnfinishedBytes_Activity
 *END**************************************************************************/
uint32_t EDMA_HAL_TCDGetUnfinishedBytes(const DMA_Type * base, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FEATURE_EDMA_MODULE_CHANNELS);
#endif

    uint32_t nbytes;
    uint16_t result = 0U;

    nbytes = EDMA_HAL_TCDGetNbytes(base, channel);

    if (((base->TCD[channel].BITER.ELINKNO & DMA_TCD_BITER_ELINKNO_ELINK_MASK) >> DMA_TCD_BITER_ELINKNO_ELINK_SHIFT) != 0U)
    {
        result = (uint16_t)((base->TCD[channel].CITER.ELINKYES & DMA_TCD_CITER_ELINKYES_CITER_LE_MASK) >> DMA_TCD_CITER_ELINKYES_CITER_LE_SHIFT);
    }
    else
    {
        result = (uint16_t)((base->TCD[channel].CITER.ELINKNO & DMA_TCD_CITER_ELINKNO_CITER_MASK) >> DMA_TCD_CITER_ELINKNO_CITER_SHIFT);
    }
    return (((uint32_t) result) * nbytes);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDGetFinishedBytes
 * Description   : Get the bytes number of bytes already be transferred for this
 * hardware TCD.
 *
 * Implements    : EDMA_HAL_TCDGetFinishedBytes_Activity
 *END**************************************************************************/
uint32_t EDMA_HAL_TCDGetFinishedBytes(const DMA_Type * base, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FEATURE_EDMA_MODULE_CHANNELS);
#endif

    uint32_t nbytes, begin_majorcount, current_majorcount;

    nbytes = EDMA_HAL_TCDGetNbytes(base, channel);
    begin_majorcount = EDMA_HAL_TCDGetBeginMajorCount(base,channel);
    current_majorcount = EDMA_HAL_TCDGetCurrentMajorCount(base,channel);

    return ((begin_majorcount - current_majorcount) * nbytes);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
