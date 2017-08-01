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
 * @file lpuart_hal.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a
 * narrower or different essential type.
 * The assign operations are safe as the baud rate calculation algorithm cannot
 * overflow the result.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.7, Composite expression with smaller
 * essential type than other operand.
 * The expression is safe as the baud rate calculation algorithm cannot overflow
 * the result.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of
 * function .
 * The return statement before end of function is used for simpler code structure
 * and better readability.
 */


#include "lpuart_hal.h"


/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_Init
 * Description   : Initializes the LPUART controller to known state, using
 *                 register reset values defined in the reference manual.
 *
 * Implements    : LPUART_HAL_Init_Activity
 *END**************************************************************************/
void LPUART_HAL_Init(LPUART_Type * base)
{
    /* Set the default oversampling ratio (16) and baud-rate divider (4) */
    base->BAUD = ((uint32_t)((FEATURE_LPUART_DEFAULT_OSR << LPUART_BAUD_OSR_SHIFT) | \
                 (FEATURE_LPUART_DEFAULT_SBR << LPUART_BAUD_SBR_SHIFT)));
    /* Clear the error/interrupt flags */
    base->STAT = FEATURE_LPUART_STAT_REG_FLAGS_MASK;
    /* Reset all features/interrupts by default */
    base->CTRL = 0x00000000;
    /* Reset match addresses */
    base->MATCH = 0x00000000;
#if FEATURE_LPUART_HAS_MODEM_SUPPORT
    /* Reset IrDA modem features */
    base->MODIR = 0x00000000;
#endif
#if FEATURE_LPUART_FIFO_SIZE > 0U
    /* Reset FIFO feature */
    base->FIFO = FEATURE_LPUART_FIFO_REG_FLAGS_MASK;
    /* Reset FIFO Watermark values */
    base->WATER = 0x00000000;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetBaudRate
 * Description   : Configures the LPUART baud rate.
 * In some LPUART instances the user must disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * Implements    : LPUART_HAL_SetBaudRate_Activity
 *END**************************************************************************/
status_t LPUART_HAL_SetBaudRate(LPUART_Type * base,
                                uint32_t sourceClockInHz,
                                uint32_t desiredBaudRate)
{
    uint16_t sbr, sbrTemp, i;
    uint32_t osr, tempDiff, calculatedBaud, baudDiff, baudRegValTemp;

    /* This lpuart instantiation uses a slightly different baud rate calculation
     * The idea is to use the best OSR (over-sampling rate) possible
     * Note, osr is typically hard-set to 16 in other lpuart instantiations
     * First calculate the baud rate using the minimum OSR possible (4) */
    osr = 4;
    sbr = (uint16_t)(sourceClockInHz / (desiredBaudRate * osr));
    calculatedBaud = (sourceClockInHz / (osr * sbr));

    if (calculatedBaud > desiredBaudRate)
    {
        baudDiff = calculatedBaud - desiredBaudRate;
    }
    else
    {
        baudDiff = desiredBaudRate - calculatedBaud;
    }

    /* loop to find the best osr value possible, one that generates minimum baudDiff
     * iterate through the rest of the supported values of osr */
    for (i = 5U; i <= 32U; i++)
    {
        /* calculate the temporary sbr value   */
        sbrTemp = (uint16_t)(sourceClockInHz / (desiredBaudRate * i));
        /* calculate the baud rate based on the temporary osr and sbr values */
        calculatedBaud = (sourceClockInHz / (i * sbrTemp));

        if (calculatedBaud > desiredBaudRate)
        {
            tempDiff = calculatedBaud - desiredBaudRate;
        }
        else
        {
            tempDiff = desiredBaudRate - calculatedBaud;
        }

        if (tempDiff <= baudDiff)
        {
            baudDiff = tempDiff;
            osr = i;  /* update and store the best osr value calculated */
            sbr = sbrTemp;  /* update store the best sbr value calculated */
        }
    }

    /* Check to see if actual baud rate is within 3% of desired baud rate
     * based on the best calculate osr value */
    if (baudDiff < ((desiredBaudRate / 100U) * 3U))
    {
        /* Acceptable baud rate, check if osr is between 4x and 7x oversampling.
         * If so, then "BOTHEDGE" sampling must be turned on */
        if ((osr > 3U) && (osr < 8U))
        {
            base->BAUD = (base->BAUD & ~LPUART_BAUD_BOTHEDGE_MASK) | ((uint32_t)1U << LPUART_BAUD_BOTHEDGE_SHIFT);
        }

        /* program the osr value (bit value is one less than actual value) */
        baudRegValTemp = base->BAUD;
        baudRegValTemp &= ~(LPUART_BAUD_OSR_MASK);
        baudRegValTemp |= LPUART_BAUD_OSR(osr - 1U);
        base->BAUD = baudRegValTemp;

        /* write the sbr value to the BAUD registers */
        baudRegValTemp = base->BAUD;
        baudRegValTemp &= ~(LPUART_BAUD_SBR_MASK);
        baudRegValTemp |= LPUART_BAUD_SBR(sbr);
        base->BAUD = baudRegValTemp;
    }
    else
    {
        /* Unacceptable baud rate difference of more than 3% */
        return STATUS_ERROR;
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetBitCountPerChar
 * Description   : Configures the number of bits per char in LPUART controller.
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * Implements    : LPUART_HAL_SetBitCountPerChar_Activity
 *END**************************************************************************/
void LPUART_HAL_SetBitCountPerChar(LPUART_Type * base,
                                   lpuart_bit_count_per_char_t bitCountPerChar)
{
    if (bitCountPerChar == LPUART_10_BITS_PER_CHAR)
    {
        base->BAUD = (base->BAUD & ~LPUART_BAUD_M10_MASK) | ((uint32_t)1U << LPUART_BAUD_M10_SHIFT);
    }
    else
    {
        /* config 8-bit (M=0) or 9-bits (M=1) */
        base->CTRL = (base->CTRL & ~LPUART_CTRL_M_MASK) | ((uint32_t)bitCountPerChar << LPUART_CTRL_M_SHIFT);
        /* clear M10 to make sure not 10-bit mode */
        base->BAUD &= ~LPUART_BAUD_M10_MASK;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetParityMode
 * Description   : Configures parity mode in the LPUART controller.
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * Implements    : LPUART_HAL_SetParityMode_Activity
 *END**************************************************************************/
void LPUART_HAL_SetParityMode(LPUART_Type * base, lpuart_parity_mode_t parityModeType)
{
    base->CTRL = (base->CTRL & ~LPUART_CTRL_PE_MASK) | (((uint32_t)parityModeType >> 1U) << LPUART_CTRL_PE_SHIFT);
    base->CTRL = (base->CTRL & ~LPUART_CTRL_PT_MASK) | (((uint32_t)parityModeType & 1U) << LPUART_CTRL_PT_SHIFT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_Putchar9
 * Description   : Sends the LPUART 9-bit character.
 *
 * Implements    : LPUART_HAL_Putchar9_Activity
 *END**************************************************************************/
void LPUART_HAL_Putchar9(LPUART_Type * base, uint16_t data)
{
    uint8_t ninthDataBit;
    volatile uint8_t * dataRegBytes = (volatile uint8_t *)(&(base->DATA));


    ninthDataBit = (uint8_t)((data >> 8U) & 0x1U);

    /* write to ninth data bit T8(where T[0:7]=8-bits, T8=9th bit) */
    base->CTRL = (base->CTRL & ~LPUART_CTRL_R9T8_MASK) | ((uint32_t)(ninthDataBit) << LPUART_CTRL_R9T8_SHIFT);

    /* write 8-bits to the data register*/
    dataRegBytes[0] = (uint8_t)data;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_Putchar10
 * Description   : Sends the LPUART 10-bit character.
 *
 * Implements    : LPUART_HAL_Putchar10_Activity
 *END**************************************************************************/
void LPUART_HAL_Putchar10(LPUART_Type * base, uint16_t data)
{
    uint8_t ninthDataBit, tenthDataBit;
    uint32_t ctrlRegVal;
    volatile uint8_t * dataRegBytes = (volatile uint8_t *)(&(base->DATA));

    ninthDataBit = (uint8_t)((data >> 8U) & 0x1U);
    tenthDataBit = (uint8_t)((data >> 9U) & 0x1U);

    /* write to ninth/tenth data bit (T[0:7]=8-bits, T8=9th bit, T9=10th bit) */
    ctrlRegVal = base->CTRL;
    ctrlRegVal = (ctrlRegVal & ~LPUART_CTRL_R9T8_MASK) | ((uint32_t)ninthDataBit << LPUART_CTRL_R9T8_SHIFT);
    ctrlRegVal = (ctrlRegVal & ~LPUART_CTRL_R8T9_MASK) | ((uint32_t)tenthDataBit << LPUART_CTRL_R8T9_SHIFT);
    base->CTRL = ctrlRegVal;

    /* write to 8-bits to the data register */
    dataRegBytes[0] = (uint8_t)data;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_Getchar9
 * Description   : Gets the LPUART 9-bit character.
 *
 * Implements    : LPUART_HAL_Getchar9_Activity
 *END**************************************************************************/
void LPUART_HAL_Getchar9(const LPUART_Type * base, uint16_t *readData)
{
    DEV_ASSERT(readData != NULL);

    /* get ninth bit from lpuart data register */
    *readData = (uint16_t)(((base->CTRL >> LPUART_CTRL_R8T9_SHIFT) & 1U) << 8);

    /* get 8-bit data from the lpuart data register */
    *readData |= (uint8_t)base->DATA;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_Getchar10
 * Description   : Gets the LPUART 10-bit character, available only on
 *                 supported lpuarts
 *
 * Implements    : LPUART_HAL_Getchar10_Activity
 *END**************************************************************************/
void LPUART_HAL_Getchar10(const LPUART_Type * base, uint16_t *readData)
{
    DEV_ASSERT(readData != NULL);

    /* read tenth data bit */
    *readData = (uint16_t)(((base->CTRL >> LPUART_CTRL_R9T8_SHIFT) & 1U) << 9);
    /* read ninth data bit */
    *readData |= (uint16_t)(((base->CTRL >> LPUART_CTRL_R8T9_SHIFT) & 1U) << 8);

    /* get 8-bit data */
    *readData |= (uint8_t)base->DATA;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SendDataPolling
 * Description   : Send out multiple bytes of data using polling method.
 * This function only supports 8-bit transaction.
 *
 * Implements    : LPUART_HAL_SendDataPolling_Activity
 *END**************************************************************************/
void LPUART_HAL_SendDataPolling(LPUART_Type * base,
                                const uint8_t *txBuff,
                                uint32_t txSize)
{
    DEV_ASSERT(txBuff != NULL);

    while (txSize-- > 0U)
    {
        while (((base->STAT >> LPUART_STAT_TDRE_SHIFT) & 1U) == 0U)
        {}

        LPUART_HAL_Putchar(base, *txBuff++);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_ReceiveDataPolling
 * Description   : Receive multiple bytes of data using polling method.
 * This function only supports 8-bit transaction.
 *
 * Implements    : LPUART_HAL_ReceiveDataPolling_Activity
 *END**************************************************************************/
status_t LPUART_HAL_ReceiveDataPolling(LPUART_Type * base,
                                       uint8_t *rxBuff,
                                       uint32_t rxSize)
{
    DEV_ASSERT(rxBuff != NULL);

    status_t retVal = STATUS_SUCCESS;

    while (rxSize-- > 0U)
    {
        while (((base->STAT >> LPUART_STAT_RDRF_SHIFT) & 1U) == 0U)
        {}

        LPUART_HAL_Getchar(base, rxBuff++);

        /* Clear the Overrun flag since it will block receiving */
        if (((base->STAT >> LPUART_STAT_OR_SHIFT) & 1U) > 0U)
        {
            base->STAT = (base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK)) | LPUART_STAT_OR_MASK;
            retVal = STATUS_UART_RX_OVERRUN;
        }
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetIntMode
 * Description   : Configures the LPUART module interrupts to enable/disable
 * various interrupt sources.
 *
 * Implements    : LPUART_HAL_SetIntMode_Activity
 *END**************************************************************************/
void LPUART_HAL_SetIntMode(LPUART_Type * base, lpuart_interrupt_t intSrc, bool enable)
{
    uint32_t reg = (uint32_t)(intSrc) >> LPUART_SHIFT;
    uint32_t intRegOffset = (uint16_t)(intSrc);

    switch (reg)
    {
        case LPUART_BAUD_REG_ID:
            base->BAUD = (base->BAUD & ~(1UL << intRegOffset)) | ((enable ? 1U : 0U) << intRegOffset);
            break;
        case LPUART_STAT_REG_ID:
            base->STAT = (base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK & ~(1UL << intRegOffset))) | \
                         ((enable ? 1U : 0U) << intRegOffset);
            break;
        case LPUART_CTRL_REG_ID:
            base->CTRL = (base->CTRL & ~(1UL << intRegOffset)) | ((enable ? 1U : 0U) << intRegOffset);
            break;
        case LPUART_DATA_REG_ID:
            base->DATA = (base->DATA & ~(1UL << intRegOffset)) | ((enable ? 1U : 0U) << intRegOffset);
            break;
        case LPUART_MATCH_REG_ID:
            base->MATCH = (base->MATCH & ~(1UL << intRegOffset)) | ((enable ? 1U : 0U) << intRegOffset);
            break;
#if FEATURE_LPUART_HAS_MODEM_SUPPORT
        case LPUART_MODIR_REG_ID:
            base->MODIR = (base->MODIR & ~(1UL << intRegOffset)) | ((enable ? 1U : 0U) << intRegOffset);
            break;
#endif
#if FEATURE_LPUART_FIFO_SIZE > 0U
        case LPUART_FIFO_REG_ID:
            base->FIFO = (base->FIFO & (~FEATURE_LPUART_FIFO_REG_FLAGS_MASK & ~(1UL << intRegOffset))) | \
                         ((enable ? 1U : 0U) << intRegOffset);
            break;
#endif
        default :
            /* Invalid parameter: return */
            break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_GetIntMode
 * Description   : Returns whether LPUART module interrupt is enabled/disabled.
 *
 * Implements    : LPUART_HAL_GetIntMode_Activity
 *END**************************************************************************/
bool LPUART_HAL_GetIntMode(const LPUART_Type * base, lpuart_interrupt_t intSrc)
{
    uint32_t reg = (uint32_t)(intSrc) >> LPUART_SHIFT;
    bool retVal = false;

    switch ( reg )
    {
        case LPUART_BAUD_REG_ID:
            retVal = (((base->BAUD >> (uint16_t)(intSrc)) & 1U) > 0U);
            break;
        case LPUART_STAT_REG_ID:
            retVal = (((base->STAT >> (uint16_t)(intSrc)) & 1U) > 0U);
            break;
        case LPUART_CTRL_REG_ID:
            retVal = (((base->CTRL >> (uint16_t)(intSrc)) & 1U) > 0U);
            break;
        case LPUART_DATA_REG_ID:
            retVal = (((base->DATA >> (uint16_t)(intSrc)) & 1U) > 0U);
            break;
        case LPUART_MATCH_REG_ID:
            retVal = (((base->MATCH >> (uint16_t)(intSrc)) & 1U) > 0U);
            break;
#if FEATURE_LPUART_HAS_MODEM_SUPPORT
        case LPUART_MODIR_REG_ID:
            retVal = (((base->MODIR >> (uint16_t)(intSrc)) & 1U) > 0U);
            break;
#endif
#if FEATURE_LPUART_FIFO_SIZE > 0U
        case LPUART_FIFO_REG_ID:
            retVal = (((base->FIFO >> (uint16_t)(intSrc)) & 1U) > 0U);
            break;
#endif
        default :
            /* Invalid parameter: return */
            break;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetLoopbackCmd
 * Description   : Configures the LPUART loopback operation (enable/disable
 *                 loopback operation)
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * Implements    : LPUART_HAL_SetLoopbackCmd_Activity
 *END**************************************************************************/
void LPUART_HAL_SetLoopbackCmd(LPUART_Type * base, bool enable)
{
    /* configure LOOPS bit to enable(1)/disable(0) loopback mode, but also need
     * to clear RSRC */
    base->CTRL = (base->CTRL & ~LPUART_CTRL_LOOPS_MASK) | ((enable ? 1U : 0U) << LPUART_CTRL_LOOPS_SHIFT);

    /* clear RSRC for loopback mode, and if loopback disabled, */
    /* this bit has no meaning but clear anyway */
    /* to set it back to default value */
    base->CTRL &= ~LPUART_CTRL_RSRC_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetSingleWireCmd
 * Description   : Configures the LPUART single-wire operation (enable/disable
 *                 single-wire mode)
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * Implements    : LPUART_HAL_SetSingleWireCmd_Activity
 *END**************************************************************************/
void LPUART_HAL_SetSingleWireCmd(LPUART_Type * base, bool enable)
{
    /* to enable single-wire mode, need both LOOPS and RSRC set,
     * to enable or clear both */
    base->CTRL = (base->CTRL & ~LPUART_CTRL_LOOPS_MASK) | ((enable ? 1U : 0U) << LPUART_CTRL_LOOPS_SHIFT);
    base->CTRL = (base->CTRL & ~LPUART_CTRL_RSRC_MASK) | ((enable ? 1U : 0U) << LPUART_CTRL_RSRC_SHIFT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetReceiverInStandbyMode
 * Description   : Places the LPUART receiver in standby mode.
 * In some LPUART instances, before placing LPUART in standby mode, first
 * determine whether the receiver is set to wake on idle or whether it is
 * already in idle state.
 *
 * Implements    : LPUART_HAL_SetReceiverInStandbyMode_Activity
 *END**************************************************************************/
status_t LPUART_HAL_SetReceiverInStandbyMode(LPUART_Type * base)
{
    lpuart_wakeup_method_t rxWakeMethod;
    bool lpuart_current_rx_state;

    rxWakeMethod = LPUART_HAL_GetReceiverWakeupMode(base);
    lpuart_current_rx_state = LPUART_HAL_GetStatusFlag(base, LPUART_RX_ACTIVE);

    /* if both rxWakeMethod is set for idle and current rx state is idle,
     * don't put in standby */
    if ((rxWakeMethod == LPUART_IDLE_LINE_WAKE) && (lpuart_current_rx_state == false))
    {
        return STATUS_ERROR;
    }
    else
    {
        /* set the RWU bit to place receiver into standby mode */
        base->CTRL = (base->CTRL & ~LPUART_CTRL_RWU_MASK) | ((uint32_t)1U << LPUART_CTRL_RWU_SHIFT);
        return STATUS_SUCCESS;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetIdleLineDetect
 * Description   : LPUART idle-line detect operation configuration (idle line
 * bit-count start and wake up affect on IDLE status bit).
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * Implements    : LPUART_HAL_SetIdleLineDetect_Activity
 *END**************************************************************************/
void LPUART_HAL_SetIdleLineDetect(LPUART_Type * base,
                                  const lpuart_idle_line_config_t *config)
{
    DEV_ASSERT(config != NULL);

    /* Configure the idle line detection configuration as follows:
     * configure the ILT to bit count after start bit or stop bit
     * configure RWUID to set or not set IDLE status bit upon detection of
     * an idle character when receiver in standby */
    base->CTRL = (base->CTRL & ~LPUART_CTRL_ILT_MASK) | ((uint32_t)config->idleLineType << LPUART_CTRL_ILT_SHIFT);
    base->STAT = (base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK & ~LPUART_STAT_RWUID_MASK)) | \
                 ((uint32_t)config->rxWakeIdleDetect << LPUART_STAT_RWUID_SHIFT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetMatchAddressReg1
 * Description   : Configures match address register 1
 *
 * Implements    : LPUART_HAL_SetMatchAddressReg1_Activity
 *END**************************************************************************/
void LPUART_HAL_SetMatchAddressReg1(LPUART_Type * base, bool enable, uint8_t value)
{
    uint32_t matchRegValTemp;

    /* The MAEN bit must be cleared before configuring MA value */
    base->BAUD &= ~LPUART_BAUD_MAEN1_MASK;
    if (enable)
    {
        matchRegValTemp = base->MATCH;
        matchRegValTemp &= ~(LPUART_MATCH_MA1_MASK);
        matchRegValTemp |= LPUART_MATCH_MA1(value);
        base->MATCH = matchRegValTemp;

        base->BAUD = (base->BAUD & ~LPUART_BAUD_MAEN1_MASK) | ((uint32_t)1U << LPUART_BAUD_MAEN1_SHIFT);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetMatchAddressReg2
 * Description   : Configures match address register 2
 *
 * Implements    : LPUART_HAL_SetMatchAddressReg2_Activity
 *END**************************************************************************/
void LPUART_HAL_SetMatchAddressReg2(LPUART_Type * base, bool enable, uint8_t value)
{
    uint32_t matchRegValTemp;

    /* The MAEN bit must be cleared before configuring MA value */
    base->BAUD &= ~LPUART_BAUD_MAEN2_MASK;
    if (enable)
    {
        matchRegValTemp = base->MATCH;
        matchRegValTemp &= ~(LPUART_MATCH_MA2_MASK);
        matchRegValTemp |= LPUART_MATCH_MA2(value);
        base->MATCH = matchRegValTemp;

        base->BAUD = (base->BAUD & ~LPUART_BAUD_MAEN2_MASK) | ((uint32_t)1U << LPUART_BAUD_MAEN2_SHIFT);
    }
}

#if FEATURE_LPUART_HAS_IR_SUPPORT
/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetInfrared
 * Description   : Configures the LPUART infrared operation.
 *
 * Implements    : LPUART_HAL_SetInfrared_Activity
 *END**************************************************************************/
void LPUART_HAL_SetInfrared(LPUART_Type * base, bool enable,
                            lpuart_ir_tx_pulsewidth_t pulseWidth)
{
    uint32_t modirRegValTemp;

    /* enable or disable infrared */
    base->MODIR = (base->MODIR & ~LPUART_MODIR_IREN_MASK) | ((enable ? 1UL : 0UL) << LPUART_MODIR_IREN_SHIFT);

    /* configure the narrow pulse width of the IR pulse */
    modirRegValTemp = base->MODIR;
    modirRegValTemp &= ~(LPUART_MODIR_TNP_MASK);
    modirRegValTemp |= LPUART_MODIR_TNP(pulseWidth);
    base->MODIR = modirRegValTemp;
}
#endif  /* FEATURE_LPUART_HAS_IR_SUPPORT */

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_GetStatusFlag
 * Description   : LPUART get status flag by passing flag enum.
 *
 * Implements    : LPUART_HAL_GetStatusFlag_Activity
 *END**************************************************************************/
bool LPUART_HAL_GetStatusFlag(const LPUART_Type * base, lpuart_status_flag_t statusFlag)
{
    uint32_t reg = (uint32_t)(statusFlag) >> LPUART_SHIFT;
    bool retVal = false;

    switch ( reg )
    {
        case LPUART_BAUD_REG_ID:
            retVal = (((base->BAUD >> (uint16_t)(statusFlag)) & 1U) > 0U);
            break;
        case LPUART_STAT_REG_ID:
            retVal = (((base->STAT >> (uint16_t)(statusFlag)) & 1U) > 0U);
            break;
        case LPUART_CTRL_REG_ID:
            retVal = (((base->CTRL >> (uint16_t)(statusFlag)) & 1U) > 0U);
            break;
        case LPUART_DATA_REG_ID:
            retVal = (((base->DATA >> (uint16_t)(statusFlag)) & 1U) > 0U);
            break;
        case LPUART_MATCH_REG_ID:
            retVal = (((base->MATCH >> (uint16_t)(statusFlag)) & 1U) > 0U);
            break;
#if FEATURE_LPUART_HAS_MODEM_SUPPORT
        case LPUART_MODIR_REG_ID:
            retVal = (((base->MODIR >> (uint16_t)(statusFlag)) & 1U) > 0U);
            break;
#endif
#if FEATURE_LPUART_FIFO_SIZE > 0U
        case LPUART_FIFO_REG_ID:
            retVal = (((base->FIFO >> (uint16_t)(statusFlag)) & 1U) > 0U);
            break;
#endif
        default:
            /* Invalid parameter: return */
            break;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_ClearStatusFlag
 * Description   : LPUART clears an individual status flag
 * (see lpuart_status_flag_t for list of status bits).
 *
 * Implements    : LPUART_HAL_ClearStatusFlag_Activity
 *END**************************************************************************/
status_t LPUART_HAL_ClearStatusFlag(LPUART_Type * base,
                                    lpuart_status_flag_t statusFlag)
{
    status_t returnCode = STATUS_SUCCESS;

    switch(statusFlag)
    {
        /* These flags are cleared automatically by other lpuart operations
         * and cannot be manually cleared, return error code */
        case LPUART_TX_DATA_REG_EMPTY:
        case LPUART_TX_COMPLETE:
        case LPUART_RX_DATA_REG_FULL:
        case LPUART_RX_ACTIVE:
#if FEATURE_LPUART_HAS_EXTENDED_DATA_REGISTER_FLAGS
        case LPUART_NOISE_IN_CURRENT_WORD:
        case LPUART_PARITY_ERR_IN_CURRENT_WORD:
#endif
            returnCode = STATUS_ERROR;
            break;

        case LPUART_IDLE_LINE_DETECT:
            base->STAT = (base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK)) | LPUART_STAT_IDLE_MASK;
            break;

        case LPUART_RX_OVERRUN:
            base->STAT = (base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK)) | LPUART_STAT_OR_MASK;
            break;

        case LPUART_NOISE_DETECT:
            base->STAT = (base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK)) | LPUART_STAT_NF_MASK;
            break;

        case LPUART_FRAME_ERR:
            base->STAT = (base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK)) | LPUART_STAT_FE_MASK;
            break;

        case LPUART_PARITY_ERR:
            base->STAT = (base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK)) | LPUART_STAT_PF_MASK;
            break;

        case LPUART_LIN_BREAK_DETECT:
            base->STAT = (base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK)) | LPUART_STAT_LBKDIF_MASK;
            break;

        case LPUART_RX_ACTIVE_EDGE_DETECT:
            base->STAT = (base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK)) | LPUART_STAT_RXEDGIF_MASK;
            break;

#if FEATURE_LPUART_HAS_ADDRESS_MATCHING
        case LPUART_MATCH_ADDR_ONE:
            base->STAT = (base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK)) | LPUART_STAT_MA1F_MASK;
            break;
        case LPUART_MATCH_ADDR_TWO:
            base->STAT = (base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK)) | LPUART_STAT_MA2F_MASK;
            break;
#endif
#if FEATURE_LPUART_FIFO_SIZE > 0U
        case LPUART_FIFO_TX_OF:
            base->FIFO = (base->FIFO & (~FEATURE_LPUART_FIFO_REG_FLAGS_MASK)) | LPUART_FIFO_TXOF_MASK;
            break;
        case LPUART_FIFO_RX_UF:
            base->FIFO = (base->FIFO & (~FEATURE_LPUART_FIFO_REG_FLAGS_MASK)) | LPUART_FIFO_RXUF_MASK;
            break;
#endif
        default:
            returnCode = STATUS_ERROR;
            break;
    }

    return (returnCode);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

