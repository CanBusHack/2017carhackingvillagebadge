/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
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
 * @file flexcan_hal.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * The return statement before end of function is used for simpler code structure
 * and better readability.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.1, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.2, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.4, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.5, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Directive 4.9, Function-like macro
 * Function-like macros are used instead of inline functions in order to ensure
 * that the performance will not be decreased if the functions will not be
 * inlined by the compiler.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.3, cast performed between a pointer to
 * object type and a pointer to a different object type
 * The cast is used for casting a bytes buffer into an words buffer in order to
 * optimize copying data to/from the message buffer.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 2.2, Last value assigned to variable not
 * used
 * The values of the reported variables are changed by the REV_BYTES_32 macro,
 * which uses inline assembly.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, A conversion should not be
 * performed between a pointer to object and an integer type.
 * The cast is required as CAN instance base addresses are defined as unsigned
 * integers in the header file, but the registers are accessed via pointers to
 * structures.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, A conversion should not be
 * performed between a pointer to object and an integer type.
 * The cast is required as CAN instance base addresses are defined as unsigned
 * integers in the header file, but the registers are accessed via pointers to
 * structures.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * The function is defined for use by application code.
 */

#include "flexcan_hal.h"


/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATAB_RTR_SHIFT  (31U)          /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format A&B RTR mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATAB_IDE_SHIFT  (30U)          /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format A&B IDE mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_RTR_SHIFT  (15U)           /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format B RTR-2 mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_IDE_SHIFT  (14U)           /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format B IDE-2 mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_EXT_MASK    (0x3FFFFFFFU)  /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format A extended mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_EXT_SHIFT   (1U)           /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format A extended shift.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_STD_MASK    (0x3FF80000U)  /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format A standard mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_STD_SHIFT   (19U)          /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format A standard shift.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_MASK    (0x3FFFU)      /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format B extended mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_SHIFT1  (16U)          /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format B extended mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_SHIFT2  (0U)           /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format B extended mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_MASK    (0x7FFU)       /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format B standard mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_SHIFT1  (19U)          /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format B standard shift1.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_SHIFT2  (3U)           /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format B standard shift2.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK        (0xFFU)        /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format C mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT1      (24U)          /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format C shift1.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT2      (16U)          /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format C shift2.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT3      (8U)           /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format C shift3.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT4      (0U)           /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format C shift4.*/
#define FLEXCAN_ALL_INT                               (0x0007U)      /*!< Masks for wakeup, error, bus off*/
                                                                     /*! interrupts*/

/* CAN FD extended data length DLC encoding */
#define CAN_DLC_VALUE_12_BYTES                   9U
#define CAN_DLC_VALUE_16_BYTES                   10U
#define CAN_DLC_VALUE_20_BYTES                   11U
#define CAN_DLC_VALUE_24_BYTES                   12U
#define CAN_DLC_VALUE_32_BYTES                   13U
#define CAN_DLC_VALUE_48_BYTES                   14U
#define CAN_DLC_VALUE_64_BYTES                   15U

#define RxFifoFilterTableOffset         0x18U

#define FlexCanRxFifoAcceptRemoteFrame   1UL
#define FlexCanRxFifoAcceptExtFrame      1UL

#define FLEXCAN_8_BYTE_PAYLOAD_MB_SIZE   16U
#define FLEXCAN_ARBITRATION_FIELD_SIZE   8U

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

static volatile uint32_t* FLEXCAN_HAL_GetMsgBuffRegion(CAN_Type * base, uint32_t msgBuffIdx);
static uint8_t FLEXCAN_HAL_ComputeDLCValue(uint8_t payloadSize);
static uint8_t FLEXCAN_HAL_ComputePayloadSize(uint8_t dlcValue);
static uint32_t FLEXCAN_HAL_MaxMbRAMSize(const CAN_Type * base);
static void FLEXCAN_HAL_ClearRAM(CAN_Type * base);

static inline uint32_t RxFifoOcuppiedLastMsgBuff(uint32_t x)
{
    return (5U + ((((x) + 1U) * 8U) / 4U));
}

/* Converts the index in an array of bytes to an index in an array of bytes with
 * reversed endianness. */
#define FlexcanSwapBytesInWordIndex(index) (((index) & ~3U) + (3U - ((index) & 3U)))

/* Determines the RxFIFO Filter element number */
#define RxFifoFilterElementNum(x) (((x) + 1U) * 8U)



/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_GetMsgBuffRegion
 * Description   : Returns the start of a MB area, based on its index.
 *
 *END**************************************************************************/
static volatile uint32_t* FLEXCAN_HAL_GetMsgBuffRegion(
        CAN_Type * base,
        uint32_t msgBuffIdx)
{
    uint8_t payload_size = FLEXCAN_HAL_GetPayloadSize(base);
    uint8_t arbitration_field_size = 8U;

    uint8_t mb_size = (uint8_t)(payload_size + arbitration_field_size);
    /* Multiply the MB index by the MB size (in words) */
    uint32_t mb_index = msgBuffIdx * ((uint32_t)mb_size >> 2U);

    return &(base->RAMn[mb_index]);
}

/*FUNCTION**********************************************************************
 *
 * Function Name: FLEXCAN_HAL_ComputeDLCValue
 * Description  : Computes the DLC field value, given a payload size (in bytes).
 *
 *END**************************************************************************/
static uint8_t FLEXCAN_HAL_ComputeDLCValue(
        uint8_t payloadSize)
{
    uint8_t ret;

    if (payloadSize <= 8U)
    {
        ret = payloadSize;
    }
    else if ((payloadSize > 8U) && (payloadSize <= 12U))
    {
        ret = CAN_DLC_VALUE_12_BYTES;
    }
    else if ((payloadSize > 12U) && (payloadSize <= 16U))
    {
        ret = CAN_DLC_VALUE_16_BYTES;
    }
    else if ((payloadSize > 16U) && (payloadSize <= 20U))
    {
        ret = CAN_DLC_VALUE_20_BYTES;
    }
    else if ((payloadSize > 20U) && (payloadSize <= 24U))
    {
        ret = CAN_DLC_VALUE_24_BYTES;
    }
    else if ((payloadSize > 24U) && (payloadSize <= 32U))
    {
        ret = CAN_DLC_VALUE_32_BYTES;
    }
    else if ((payloadSize > 32U) && (payloadSize <= 48U))
    {
        ret = CAN_DLC_VALUE_48_BYTES;
    }
    else if ((payloadSize > 48U) && (payloadSize <= 64U))
    {
        ret = CAN_DLC_VALUE_64_BYTES;
    }
    else
    {
        /* The argument is not a valid payload size */
        ret = 0xFFU;
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_ComputePayloadSize
 * Description   : Computes the maximum payload size (in bytes), given a DLC
 * field value.
 *
 *END**************************************************************************/
static uint8_t FLEXCAN_HAL_ComputePayloadSize(
        uint8_t dlcValue)
{
    uint8_t ret;

    if (dlcValue <= 8U)
    {
        ret = dlcValue;
    }
    else
    {
        switch (dlcValue) {
        case CAN_DLC_VALUE_12_BYTES:
            ret = 12U;
            break;
        case CAN_DLC_VALUE_16_BYTES:
            ret = 16U;
            break;
        case CAN_DLC_VALUE_20_BYTES:
            ret = 20U;
            break;
        case CAN_DLC_VALUE_24_BYTES:
            ret = 24U;
            break;
        case CAN_DLC_VALUE_32_BYTES:
            ret = 32U;
            break;
        case CAN_DLC_VALUE_48_BYTES:
            ret = 48U;
            break;
        case CAN_DLC_VALUE_64_BYTES:
            ret = 64U;
            break;
        default:
            /* The argument is not a valid DLC size */
            ret = 0xFFU;
            break;
        }
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_MaxMbRAMSize
 * Description   : Computes the maximum RAM size occupied by MBs.
 *
 *END**************************************************************************/
static uint32_t FLEXCAN_HAL_MaxMbRAMSize(const CAN_Type * base)
{
    uint32_t ret = 0;

    if (base == CAN0)
    {
        ret = FEATURE_CAN0_MAX_MB_NUM * FLEXCAN_8_BYTE_PAYLOAD_MB_SIZE;
    }

#if (CAN_INSTANCE_COUNT > 1U)
    if (base == CAN1)
    {
        ret = FEATURE_CAN1_MAX_MB_NUM * FLEXCAN_8_BYTE_PAYLOAD_MB_SIZE;
    }
#endif

#if (CAN_INSTANCE_COUNT > 2U)
    if (base == CAN2)
    {
        ret = FEATURE_CAN2_MAX_MB_NUM * FLEXCAN_8_BYTE_PAYLOAD_MB_SIZE;
    }
#endif

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_ClearRAM
 * Description   : Clears FlexCAN memory positions that require initialization.
 *
 *END**************************************************************************/
static void FLEXCAN_HAL_ClearRAM(CAN_Type * base)
{
    uint32_t databyte;
    uint32_t RAM_size = FLEXCAN_HAL_MaxMbRAMSize(base) / 4U;
    volatile uint32_t *RAM = base->RAMn;

    /* Clear MB region */
    for (databyte = 0; databyte < RAM_size; databyte++) {
        RAM[databyte] = 0;
    }

    RAM = base->RXIMR;

    FLEXCAN_HAL_EnterFreezeMode(base);

    /* Clear RXIMR region */
    for (databyte = 0; databyte < CAN_RXIMR_COUNT; databyte++) {
        RAM[databyte] = 0;
    }

#if defined(CPU_S32V234)

    /* Set WRMFRZ bit in CTRL2 Register to grant write access to memory */
    base->CTRL2 = (base->CTRL2 & ~CAN_CTRL2_WRMFRZ_MASK) | CAN_CTRL2_WRMFRZ(1U);

    RAM = base->RAMn;

    /* Clear SMB FD region */
    for (databyte = FEATURE_CAN_SMB_FD_START_ADDRESS_OFFSET; databyte < FEATURE_CAN_SMB_FD_END_ADDRESS_OFFSET; databyte++) {
            RAM[databyte] = 0;
    }

    /* Clear WRMFRZ bit in CTRL2 Register to restrict write access to memory */
    base->CTRL2 = (base->CTRL2 & ~CAN_CTRL2_WRMFRZ_MASK) | CAN_CTRL2_WRMFRZ(0U);

#endif

    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_Enable
 * Description   : Enable FlexCAN module.
 * This function will enable FlexCAN module.
 *
 * Implements    : FLEXCAN_HAL_Enable_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_Enable(CAN_Type * base)
{
    /* Check for low power mode */
    if(((base->MCR & CAN_MCR_LPMACK_MASK) >> CAN_MCR_LPMACK_SHIFT) == 1U)
    {
        /* Enable clock */
        base->MCR = (base->MCR & ~CAN_MCR_MDIS_MASK) | CAN_MCR_MDIS(0U);
        base->MCR = (base->MCR & ~CAN_MCR_FRZ_MASK) | CAN_MCR_FRZ(0U);
        base->MCR = (base->MCR & ~CAN_MCR_HALT_MASK) | CAN_MCR_HALT(0U);
        /* Wait until enabled */
        while (((base->MCR & CAN_MCR_LPMACK_MASK) >> CAN_MCR_LPMACK_SHIFT) != 0U) {}
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_Disable
 * Description   : Disable FlexCAN module.
 * This function will disable FlexCAN module.
 *
 * Implements : FLEXCAN_HAL_Disable_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_Disable(CAN_Type * base)
{
    /* To access the memory mapped registers */
    /* Entre disable mode (hard reset). */
    if(((base->MCR & CAN_MCR_MDIS_MASK) >> CAN_MCR_MDIS_SHIFT) == 0U)
    {
        /* Clock disable (module) */
        base->MCR = (base->MCR & ~CAN_MCR_MDIS_MASK) | CAN_MCR_MDIS(1U);

        /* Wait until disable mode acknowledged */
        while (((base->MCR & CAN_MCR_LPMACK_MASK) >> CAN_MCR_LPMACK_SHIFT) == 0U) {}
    }
}

#if FEATURE_CAN_HAS_PE_CLKSRC_SELECT
/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SelectClock
 * Description   : Select FlexCAN clock source.
 * This function will select either internal sys clock or external clock as
 * FlexCAN clock source.
 *
 * Implements    : FLEXCAN_HAL_SelectClock_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SelectClock(
    CAN_Type * base,
    flexcan_clk_source_t clk)
{
    base->CTRL1 = (base->CTRL1 & ~CAN_CTRL1_CLKSRC_MASK) | CAN_CTRL1_CLKSRC(clk);
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_Init
 * Description   : Initialize FlexCAN module.
 * This function will reset FlexCAN module, set maximum number of message
 * buffers, initialize all message buffers as inactive, enable RX FIFO
 * if needed, mask all mask bits, and disable all MB interrupts.
 *
 * Implements    : FLEXCAN_HAL_Init_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_Init(CAN_Type * base)
{
    /* Reset the FLEXCAN */
    base->MCR = (base->MCR & ~CAN_MCR_SOFTRST_MASK) | CAN_MCR_SOFTRST(1U);

    /* Wait for reset cycle to complete */
    while (((base->MCR & CAN_MCR_SOFTRST_MASK) >> CAN_MCR_SOFTRST_SHIFT) != 0U) {}

    /* Clear FlexCAN memory */
    FLEXCAN_HAL_ClearRAM(base);

    /* Set Freeze, Halt*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* Rx global mask*/
    (base->RXMGMASK) = (((uint32_t)(((uint32_t)(CAN_RXMGMASK_MG_MASK)) << CAN_ID_EXT_SHIFT)) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

    /* Rx reg 14 mask*/
    (base->RX14MASK) =  (((uint32_t)(((uint32_t)(CAN_RX14MASK_RX14M_MASK)) << CAN_ID_EXT_SHIFT)) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

    /* Rx reg 15 mask*/
    (base->RX15MASK) = (((uint32_t)(((uint32_t)(CAN_RX15MASK_RX15M_MASK)) << CAN_ID_EXT_SHIFT)) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);

    /* Disable all MB interrupts*/
    (base->IMASK1) = 0x0;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetTimeSegments
 * Description   : Set FlexCAN time segments.
 * This function will set all FlexCAN time segments which define the length of
 * Propagation Segment in the bit time, the length of Phase Buffer Segment 2 in
 * the bit time, the length of Phase Buffer Segment 1 in the bit time, the ratio
 * between the PE clock frequency and the Serial Clock (Sclock) frequency, and
 * the maximum number of time quanta that a bit time can be changed by one
 * resynchronization. (One time quantum is equal to the Sclock period.)
 *
 * Implements    : FLEXCAN_HAL_SetTimeSegments_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetTimeSegments(
    CAN_Type * base,
    const flexcan_time_segment_t *timeSeg)
{
    DEV_ASSERT(timeSeg != NULL);

    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* Set FlexCAN time segments*/
    if ((base->CBT & CAN_CBT_BTF_MASK) != 0U)
    {
        /* If extended bit time definitions are enabled, use CBT register */
        (base->CBT) = ((base->CBT) & ~((CAN_CBT_EPROPSEG_MASK | CAN_CBT_EPSEG2_MASK |
                                    CAN_CBT_EPSEG1_MASK | CAN_CBT_EPRESDIV_MASK) |
                                    CAN_CBT_ERJW_MASK));

        (base->CBT) = ((base->CBT) | (CAN_CBT_EPROPSEG(timeSeg->propSeg) |
                                    CAN_CBT_EPSEG2(timeSeg->phaseSeg2) |
                                    CAN_CBT_EPSEG1(timeSeg->phaseSeg1) |
                                    CAN_CBT_EPRESDIV(timeSeg->preDivider) |
                                    CAN_CBT_ERJW(timeSeg->rJumpwidth)));
    }
    else
    {
        /* If extended bit time definitions are disabled, use CTRL1 register */
        (base->CTRL1) = ((base->CTRL1) & ~((CAN_CTRL1_PROPSEG_MASK | CAN_CTRL1_PSEG2_MASK |
                                    CAN_CTRL1_PSEG1_MASK | CAN_CTRL1_PRESDIV_MASK) |
                                    CAN_CTRL1_RJW_MASK));

        (base->CTRL1) = ((base->CTRL1) | (CAN_CTRL1_PROPSEG(timeSeg->propSeg) |
                                    CAN_CTRL1_PSEG2(timeSeg->phaseSeg2) |
                                    CAN_CTRL1_PSEG1(timeSeg->phaseSeg1) |
                                    CAN_CTRL1_PRESDIV(timeSeg->preDivider) |
                                    CAN_CTRL1_RJW(timeSeg->rJumpwidth)));
    }

    /* De-assert Freeze mode*/
     FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetTimeSegmentsCbt
 * Description   : Set FlexCAN time segments for the data phase of FD frames.
 * This function will set all FlexCAN time segments which define the length of
 * Propagation Segment in the bit time, the length of Phase Buffer Segment 2 in
 * the bit time, the length of Phase Buffer Segment 1 in the bit time, the ratio
 * between the PE clock frequency and the Serial Clock (Sclock) frequency, and
 * the maximum number of time quanta that a bit time can be changed by one
 * resynchronization. (One time quantum is equal to the Sclock period.)
 *
 * Implements    : FLEXCAN_HAL_SetTimeSegmentsCbt_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetTimeSegmentsCbt(
    CAN_Type * base,
    const flexcan_time_segment_t *timeSeg)
{
    DEV_ASSERT(timeSeg != NULL);

    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);
    /* Set FlexCAN time segments*/
    (base->FDCBT) = ((base->FDCBT) & ~((CAN_FDCBT_FPROPSEG_MASK | CAN_FDCBT_FPSEG2_MASK |
                                CAN_FDCBT_FPSEG1_MASK | CAN_FDCBT_FPRESDIV_MASK) |
                                CAN_FDCBT_FRJW_MASK));

    (base->FDCBT) = ((base->FDCBT) | (CAN_FDCBT_FPROPSEG(timeSeg->propSeg) |
                                CAN_FDCBT_FPSEG2(timeSeg->phaseSeg2) |
                                CAN_FDCBT_FPSEG1(timeSeg->phaseSeg1) |
                                CAN_FDCBT_FPRESDIV(timeSeg->preDivider) |
                                CAN_FDCBT_FRJW(timeSeg->rJumpwidth)));

    /* De-assert Freeze mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_GetTimeSegments
 * Description   : Get FlexCAN time segments.
 * This function will get all FlexCAN time segments defined.
 *
 * Implements    : FLEXCAN_HAL_GetTimeSegments_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_GetTimeSegments(
    const CAN_Type * base,
    flexcan_time_segment_t *timeSeg)
{
    DEV_ASSERT(timeSeg != NULL);

    timeSeg->preDivider = ( (base->CTRL1) & CAN_CTRL1_PRESDIV_MASK) >> CAN_CTRL1_PRESDIV_SHIFT;
    timeSeg->propSeg = ((base->CTRL1) & CAN_CTRL1_PROPSEG_MASK) >> CAN_CTRL1_PROPSEG_SHIFT;
    timeSeg->phaseSeg1 = ((base->CTRL1) & CAN_CTRL1_PSEG1_MASK) >> CAN_CTRL1_PSEG1_SHIFT;
    timeSeg->phaseSeg2 = ((base->CTRL1) & CAN_CTRL1_PSEG2_MASK) >> CAN_CTRL1_PSEG2_SHIFT;
    timeSeg->rJumpwidth = ((base->CTRL1) & CAN_CTRL1_RJW_MASK) >> CAN_CTRL1_RJW_SHIFT;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetTxMsgBuff
 * Description   : Configure a message buffer for transmission.
 * This function will first check if RX FIFO is enabled. If RX FIFO is enabled,
 * the function will make sure if the MB requested is not occupied by RX FIFO
 * and ID filter table. Then this function will copy user's buffer into the
 * message buffer data area and configure the message buffer as required for
 * transmission.
 *
 * Implements    : FLEXCAN_HAL_SetTxMsgBuff_Activity
 *END**************************************************************************/
status_t FLEXCAN_HAL_SetTxMsgBuff(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    const flexcan_msgbuff_code_status_t *cs,
    uint32_t msgId,
    const uint8_t *msgData)
{
    DEV_ASSERT(cs != NULL);

    uint32_t val1, val2 = 1;
    uint32_t flexcan_mb_config = 0;
    uint32_t databyte;
    uint8_t dlc_value;
    status_t stat = STATUS_SUCCESS;

    volatile uint32_t *flexcan_mb = FLEXCAN_HAL_GetMsgBuffRegion(base, msgBuffIdx);

    volatile uint32_t *flexcan_mb_id   = &flexcan_mb[1];
    volatile uint8_t  *flexcan_mb_data = (volatile uint8_t *)(&flexcan_mb[2]);
    volatile uint32_t *flexcan_mb_data_32 = &flexcan_mb[2];
    const uint32_t *msgData_32 = (const uint32_t *)msgData;

    if (msgBuffIdx >= (((base->MCR) & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT) )
    {
        stat = STATUS_FLEXCAN_MB_OUT_OF_RANGE;
    }

    /* Check if RX FIFO is enabled*/
    if (((base->MCR & CAN_MCR_RFEN_MASK) >> CAN_MCR_RFEN_SHIFT) != 0U)
    {
        /* Get the number of RX FIFO Filters*/
        val1 = (((base->CTRL2) & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);
        /* Get the number if MBs occupied by RX FIFO and ID filter table*/
        /* the Rx FIFO occupies the memory space originally reserved for MB0-5*/
        /* Every number of RFFN means 8 number of RX FIFO filters*/
        /* and every 4 number of RX FIFO filters occupied one MB*/
        val2 = RxFifoOcuppiedLastMsgBuff(val1);

        if (msgBuffIdx <= val2) {
            return STATUS_FLEXCAN_MB_OUT_OF_RANGE;
        }
    }

    if (stat == STATUS_SUCCESS)
    {
        /* Make sure the BRS bit will not be ignored */
        if (FLEXCAN_HAL_IsFDEnabled(base) && cs->enable_brs)
        {
            base->FDCTRL = (base->FDCTRL & ~CAN_FDCTRL_FDRATE_MASK) | CAN_FDCTRL_FDRATE(1U);
        }

        /* Compute the value of the DLC field */
        dlc_value = FLEXCAN_HAL_ComputeDLCValue((uint8_t)cs->dataLen);

        /* Copy user's buffer into the message buffer data area */
        if (msgData != NULL)
        {
            uint8_t payload_size = FLEXCAN_HAL_ComputePayloadSize(dlc_value);
            for (databyte = 0; databyte < cs->dataLen; databyte += 4U)
            {
                REV_BYTES_32(msgData_32[databyte >> 2U], flexcan_mb_data_32[databyte >> 2U]);
            }
            for ( ; databyte < cs->dataLen; databyte++)
            {
                flexcan_mb_data[FlexcanSwapBytesInWordIndex(databyte)] =  msgData[databyte];
            }
            /* Add padding, if needed */
            for (databyte = cs->dataLen; databyte < payload_size; databyte++)
            {
                flexcan_mb_data[FlexcanSwapBytesInWordIndex(databyte)] = cs->fd_padding;
            }
        }

        /* Clean up the arbitration field area */
        *flexcan_mb = 0;
        *flexcan_mb_id = 0;

        /* Set the ID according the format structure */
        if (cs->msgIdType == FLEXCAN_MSG_ID_EXT)
        {
            /* ID [28-0] */
            *flexcan_mb_id &= ~(CAN_ID_STD_MASK | CAN_ID_EXT_MASK);
            *flexcan_mb_id |= (msgId & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

            /* Set IDE */
            flexcan_mb_config |= CAN_CS_IDE_MASK;

            /* Clear SRR bit */
            flexcan_mb_config &= ~CAN_CS_SRR_MASK;
        }
        if(cs->msgIdType == FLEXCAN_MSG_ID_STD)
        {
            /* ID[28-18] */
            *flexcan_mb_id &= ~CAN_ID_STD_MASK;
            *flexcan_mb_id |= (msgId << CAN_ID_STD_SHIFT) & CAN_ID_STD_MASK;

            /* make sure IDE and SRR are not set */
            flexcan_mb_config &= ~(CAN_CS_IDE_MASK | CAN_CS_SRR_MASK);
        }

        /* Set the length of data in bytes */
        flexcan_mb_config &= ~CAN_CS_DLC_MASK;
        flexcan_mb_config |= ((uint32_t)dlc_value << CAN_CS_DLC_SHIFT) & CAN_CS_DLC_MASK;

        /* Set MB CODE */
        if (cs->code != (uint32_t)FLEXCAN_TX_NOT_USED)
        {
            if (cs->code == (uint32_t)FLEXCAN_TX_REMOTE)
            {
                /* Set RTR bit */
                flexcan_mb_config |= CAN_CS_RTR_MASK;
            }

            /* Reset the code */
            flexcan_mb_config &= ~CAN_CS_CODE_MASK;

            /* Set the code */
            if (cs->fd_enable)
            {
                flexcan_mb_config |= ((cs->code << CAN_CS_CODE_SHIFT) & CAN_CS_CODE_MASK) | CAN_MB_EDL_MASK;
            }
            else
            {
                flexcan_mb_config |= (cs->code << CAN_CS_CODE_SHIFT) & CAN_CS_CODE_MASK;
            }

            if (cs->enable_brs)
            {
                flexcan_mb_config |= CAN_MB_BRS_MASK;
            }

            *flexcan_mb |= flexcan_mb_config;
        }
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMsgBuff
 * Description   : Configure a message buffer for receiving.
 * This function will first check if RX FIFO is enabled. If RX FIFO is enabled,
 * the function will make sure if the MB requested is not occupied by RX FIFO
 * and ID filter table. Then this function will configure the message buffer as
 * required for receiving.
 *
 * Implements    : FLEXCAN_HAL_SetRxMsgBuff_Activity
 *END**************************************************************************/
status_t FLEXCAN_HAL_SetRxMsgBuff(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    const flexcan_msgbuff_code_status_t *cs,
    uint32_t msgId)
{
    DEV_ASSERT(cs != NULL);

    uint32_t val1, val2 = 1;

    volatile uint32_t *flexcan_mb = FLEXCAN_HAL_GetMsgBuffRegion(base, msgBuffIdx);
    volatile uint32_t *flexcan_mb_id = &flexcan_mb[1];
    status_t stat = STATUS_SUCCESS;

    if (msgBuffIdx >= (((base->MCR) & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT))
    {
        stat = STATUS_FLEXCAN_MB_OUT_OF_RANGE;
    }

    /* Check if RX FIFO is enabled */
    if (((base->MCR & CAN_MCR_RFEN_MASK) >> CAN_MCR_RFEN_SHIFT) != 0U)
    {
        /* Get the number of RX FIFO Filters*/
        val1 = (((base->CTRL2) & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);
        /* Get the number if MBs occupied by RX FIFO and ID filter table*/
        /* the Rx FIFO occupies the memory space originally reserved for MB0-5*/
        /* Every number of RFFN means 8 number of RX FIFO filters*/
        /* and every 4 number of RX FIFO filters occupied one MB*/
        val2 = RxFifoOcuppiedLastMsgBuff(val1);

        if (msgBuffIdx <= val2) {
            return STATUS_FLEXCAN_MB_OUT_OF_RANGE;
        }
    }

    if (stat == STATUS_SUCCESS)
    {
        /* Clean up the arbitration field area */
        *flexcan_mb = 0;
        *flexcan_mb_id = 0;

        /* Set the ID according the format structure */
        if (cs->msgIdType == FLEXCAN_MSG_ID_EXT)
        {
            /* Set IDE */
            *flexcan_mb |= CAN_CS_IDE_MASK;

            /* Clear SRR bit */
            *flexcan_mb &= ~CAN_CS_SRR_MASK;

            /* ID [28-0] */
            *flexcan_mb_id &= ~(CAN_ID_STD_MASK | CAN_ID_EXT_MASK);
            *flexcan_mb_id |= (msgId & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));
        }
        if (cs->msgIdType == FLEXCAN_MSG_ID_STD)
        {
            /* Make sure IDE and SRR are not set */
            *flexcan_mb &= ~(CAN_CS_IDE_MASK | CAN_CS_SRR_MASK);

            /* ID[28-18] */
            *flexcan_mb_id &= ~CAN_ID_STD_MASK;
            *flexcan_mb_id |= (msgId << CAN_ID_STD_SHIFT) & CAN_ID_STD_MASK;
        }

        /* Set MB CODE */
        if (cs->code != (uint32_t)FLEXCAN_RX_NOT_USED)
        {
             *flexcan_mb &= ~CAN_CS_CODE_MASK;
             *flexcan_mb |= (cs->code << CAN_CS_CODE_SHIFT) & CAN_CS_CODE_MASK;
        }
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_GetMsgBuff
 * Description   : Get a message buffer field values.
 * This function will first check if RX FIFO is enabled. If RX FIFO is enabled,
 * the function will make sure if the MB requested is not occupied by RX FIFO
 * and ID filter table. Then this function will get the message buffer field
 * values and copy the MB data field into user's buffer.
 *
 * Implements    : FLEXCAN_HAL_GetMsgBuff_Activity
 *END**************************************************************************/
status_t FLEXCAN_HAL_GetMsgBuff(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    flexcan_msgbuff_t *msgBuff)
{
    DEV_ASSERT(msgBuff != NULL);

    uint32_t i;
    uint32_t val1, val2 = 1;
    status_t stat = STATUS_SUCCESS;

    volatile const uint32_t *flexcan_mb = FLEXCAN_HAL_GetMsgBuffRegion(base, msgBuffIdx);
    volatile const uint32_t *flexcan_mb_id   = &flexcan_mb[1];
    volatile const uint8_t  *flexcan_mb_data = (volatile const uint8_t *)(&flexcan_mb[2]);
    volatile const uint32_t *flexcan_mb_data_32 = &flexcan_mb[2];
    uint32_t *msgBuff_data_32 = (uint32_t *)(msgBuff->data);
    uint32_t mbWord;

    uint8_t flexcan_mb_dlc_value = (uint8_t)(((*flexcan_mb) & CAN_CS_DLC_MASK) >> 16);
    uint8_t payload_size = FLEXCAN_HAL_ComputePayloadSize(flexcan_mb_dlc_value);

    msgBuff->dataLen = payload_size;

    if (msgBuffIdx >= (((base->MCR) & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT))
    {
        stat = STATUS_FLEXCAN_MB_OUT_OF_RANGE;
    }

    /* Check if RX FIFO is enabled */
    if (((base->MCR & CAN_MCR_RFEN_MASK) >> CAN_MCR_RFEN_SHIFT) != 0U)
    {
        /* Get the number of RX FIFO Filters*/
        val1 = (((base->CTRL2) & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);
        /* Get the number if MBs occupied by RX FIFO and ID filter table*/
        /* the Rx FIFO occupies the memory space originally reserved for MB0-5*/
        /* Every number of RFFN means 8 number of RX FIFO filters*/
        /* and every 4 number of RX FIFO filters occupied one MB*/
        val2 = RxFifoOcuppiedLastMsgBuff(val1);

        if (msgBuffIdx <= val2) {
            return STATUS_FLEXCAN_MB_OUT_OF_RANGE;
        }
    }

    if (stat == STATUS_SUCCESS)
    {
        /* Get a MB field values */
        msgBuff->cs = *flexcan_mb;
        if ((msgBuff->cs & CAN_CS_IDE_MASK) != 0U)
        {
            msgBuff->msgId = (*flexcan_mb_id);
        }
        else
        {
            msgBuff->msgId = (*flexcan_mb_id) >> CAN_ID_STD_SHIFT;
        }

        /* Copy MB data field into user's buffer */
        for (i = 0 ; i < payload_size ; i += 4U)
        {
            mbWord = flexcan_mb_data_32[i >> 2U];
            REV_BYTES_32(mbWord, msgBuff_data_32[i >> 2U]);
        }
        for ( ; i < payload_size ; i++)
        {
            msgBuff->data[i] = flexcan_mb_data[FlexcanSwapBytesInWordIndex(i)];
        }
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_LockRxMsgBuff
 * Description   : Lock the RX message buffer.
 * This function will lock the RX message buffer.
 *
 * Implements    : FLEXCAN_HAL_LockRxMsgBuff_Activity
 *END**************************************************************************/
status_t FLEXCAN_HAL_LockRxMsgBuff(
    CAN_Type * base,
    uint32_t msgBuffIdx)
{
    volatile const uint32_t *flexcan_mb = FLEXCAN_HAL_GetMsgBuffRegion(base, msgBuffIdx);
    status_t stat = STATUS_SUCCESS;

    if (msgBuffIdx >= (((base->MCR) & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT))
    {
        stat = STATUS_FLEXCAN_MB_OUT_OF_RANGE;
    }

    if (stat == STATUS_SUCCESS)
    {
        /* Lock the mailbox by reading it */
        (void)*flexcan_mb;
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_EnableRxFifo
 * Description   : Enable Rx FIFO feature.
 * This function will enable the Rx FIFO feature.
 *
 * Implements    : FLEXCAN_HAL_EnableRxFifo_Activity
 *END**************************************************************************/
status_t FLEXCAN_HAL_EnableRxFifo(CAN_Type * base, uint32_t numOfFilters)
{
    uint32_t i;
    status_t stat = STATUS_SUCCESS;

    /* RxFIFO cannot be enabled if FD is enabled */
    if (FLEXCAN_HAL_IsFDEnabled(base))
    {
        stat = STATUS_ERROR;
    }

    if (stat == STATUS_SUCCESS)
    {
        /* Set Freeze mode */
        FLEXCAN_HAL_EnterFreezeMode(base);

        /* Enable RX FIFO */
        base->MCR = (base->MCR & ~CAN_MCR_RFEN_MASK) | CAN_MCR_RFEN(1U);
        /* Set the number of the RX FIFO filters needed */
        base->CTRL2 = (base->CTRL2 & ~CAN_CTRL2_RFFN_MASK) | ((numOfFilters << CAN_CTRL2_RFFN_SHIFT) & CAN_CTRL2_RFFN_MASK);
        /* RX FIFO global mask */
        (base->RXFGMASK) = (CAN_RXFGMASK_FGM_MASK << CAN_ID_EXT_SHIFT) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK);

        for (i = 0; i < CAN_RXIMR_COUNT; i++)
        {
            /* RX individual mask */
            base->RXIMR[i] = (CAN_RXIMR_MI_MASK << CAN_ID_EXT_SHIFT) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK);
        }

        /* De-assert Freeze Mode */
        FLEXCAN_HAL_ExitFreezeMode(base);
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_DisableRxFifo
 * Description   : Disable Rx FIFO feature.
 * This function will disable the Rx FIFO feature.
 *
 * Implements    : FLEXCAN_HAL_DisableRxFifo_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_DisableRxFifo(CAN_Type * base)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* Disable RX FIFO */
    base->MCR = (base->MCR & ~CAN_MCR_RFEN_MASK) | CAN_MCR_RFEN(0U);

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxFifoFilterNum
 * Description   : Set the number of Rx FIFO filters.
 * This function will define the number of Rx FIFO filters.
 *
 * Implements    : FLEXCAN_HAL_SetRxFifoFilterNum_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxFifoFilterNum(
    CAN_Type * base,
    uint32_t number)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* Set the number of RX FIFO ID filters*/
    (base->CTRL2) = (((base->CTRL2) & ~(CAN_CTRL2_RFFN_MASK)) | (((uint32_t)(((uint32_t)(number))<<CAN_CTRL2_RFFN_SHIFT))&CAN_CTRL2_RFFN_MASK));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetMaxMsgBuffNum
 * Description   : Set the number of the last Message Buffers.
 * This function will define the number of the last Message Buffers
 *
 * Implements    : FLEXCAN_HAL_SetMaxMsgBuffNum_Activity
 *END**************************************************************************/
status_t FLEXCAN_HAL_SetMaxMsgBuffNum(
    CAN_Type * base,
    uint32_t maxMsgBuffNum)
{
    uint8_t msgBuffIdx;
    uint32_t databyte;
    uint8_t can_real_payload = FLEXCAN_HAL_GetPayloadSize(base);
    uint8_t max_mb_num = (uint8_t)(FLEXCAN_HAL_MaxMbRAMSize(base) /
                         ((uint32_t)FLEXCAN_ARBITRATION_FIELD_SIZE + can_real_payload));
    status_t status = STATUS_SUCCESS;

    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* Check that the number of MBs is supported */
    if (maxMsgBuffNum > max_mb_num)
    {
        status = STATUS_FLEXCAN_MB_OUT_OF_RANGE;
    }

    if (status == STATUS_SUCCESS)
    {
        /* Set the maximum number of MBs*/
        base->MCR = (base->MCR & ~CAN_MCR_MAXMB_MASK) | ((maxMsgBuffNum << CAN_MCR_MAXMB_SHIFT) & CAN_MCR_MAXMB_MASK);

        if (!FLEXCAN_HAL_IsRxFifoEnabled(base))
        {
            /* Initialize all message buffers as inactive */
            for (msgBuffIdx = 0; msgBuffIdx < maxMsgBuffNum; msgBuffIdx++)
            {
                volatile uint32_t *flexcan_mb = FLEXCAN_HAL_GetMsgBuffRegion(base, msgBuffIdx);
                volatile uint32_t *flexcan_mb_id   = &flexcan_mb[1];
                volatile uint8_t  *flexcan_mb_data = (volatile uint8_t *)(&flexcan_mb[2]);

                *flexcan_mb = 0x0;
                *flexcan_mb_id = 0x0;
                for (databyte = 0; databyte < can_real_payload; databyte++)
                {
                   flexcan_mb_data[databyte] = 0x0;
                }
            }
        }
    }

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxFifoFilter
 * Description   : Confgure RX FIFO ID filter table elements.
 *
 * Implements    : FLEXCAN_HAL_SetRxFifoFilter_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxFifoFilter(
    CAN_Type * base,
    flexcan_rx_fifo_id_element_format_t idFormat,
    const flexcan_id_table_t *idFilterTable)
{
    DEV_ASSERT(idFilterTable != NULL);

    /* Set RX FIFO ID filter table elements*/
    uint32_t i, j, numOfFilters;
    uint32_t val1 = 0, val2 = 0, val = 0;

    volatile uint32_t *filterTable = &base->RAMn[RxFifoFilterTableOffset];

    numOfFilters = (((base->CTRL2) & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);

    switch(idFormat)
    {
        case (FLEXCAN_RX_FIFO_ID_FORMAT_A):
            /* One full ID (standard and extended) per ID Filter Table element.*/
            (base->MCR) = (((base->MCR) & ~(CAN_MCR_IDAM_MASK)) | ( (((uint32_t)(((uint32_t)(FLEXCAN_RX_FIFO_ID_FORMAT_A))<<CAN_MCR_IDAM_SHIFT))&CAN_MCR_IDAM_MASK)));
            if (idFilterTable->isRemoteFrame)
            {
                val = FlexCanRxFifoAcceptRemoteFrame << FLEXCAN_RX_FIFO_ID_FILTER_FORMATAB_RTR_SHIFT;
            }
            if (idFilterTable->isExtendedFrame)
            {
                val |= FlexCanRxFifoAcceptExtFrame << FLEXCAN_RX_FIFO_ID_FILTER_FORMATAB_IDE_SHIFT;
            }
            for (i = 0; i < RxFifoFilterElementNum(numOfFilters); i++)
            {
                if(idFilterTable->isExtendedFrame)
                {
                    filterTable[i] = val + ((idFilterTable->idFilter[i] <<
                                             FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_EXT_SHIFT) &
                                             FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_EXT_MASK);
                }
                else
                {
                    filterTable[i] = val + ((idFilterTable->idFilter[i] <<
                                             FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_STD_SHIFT) &
                                             FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_STD_MASK);
                }
            }
            break;
        case (FLEXCAN_RX_FIFO_ID_FORMAT_B):
            /* Two full standard IDs or two partial 14-bit (standard and extended) IDs*/
            /* per ID Filter Table element.*/
           (base->MCR) = (((base->MCR) & ~(CAN_MCR_IDAM_MASK)) | ( (((uint32_t)(((uint32_t)(FLEXCAN_RX_FIFO_ID_FORMAT_B))<<CAN_MCR_IDAM_SHIFT))&CAN_MCR_IDAM_MASK)));
            if (idFilterTable->isRemoteFrame)
            {
                val1 = FlexCanRxFifoAcceptRemoteFrame << FLEXCAN_RX_FIFO_ID_FILTER_FORMATAB_RTR_SHIFT;
                val2 = FlexCanRxFifoAcceptRemoteFrame << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_RTR_SHIFT;
            }
            if (idFilterTable->isExtendedFrame)
            {
                val1 |= FlexCanRxFifoAcceptExtFrame << FLEXCAN_RX_FIFO_ID_FILTER_FORMATAB_IDE_SHIFT;
                val2 |= FlexCanRxFifoAcceptExtFrame << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_IDE_SHIFT;
            }
            j = 0;
            for (i = 0; i < RxFifoFilterElementNum(numOfFilters); i++)
            {
                if (idFilterTable->isExtendedFrame)
                {
                    filterTable[i] = val1 + ((idFilterTable->idFilter[j] &
                                              FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_MASK) <<
                                              FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_SHIFT1);
                    filterTable[i] |= val2 + ((idFilterTable->idFilter[j + 1U] &
                                              FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_MASK) <<
                                              FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_SHIFT2);
                }
                else
                {
                    filterTable[i] = val1 + ((idFilterTable->idFilter[j] &
                                              FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_MASK) <<
                                              FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_SHIFT1);
                    filterTable[i] |= val2 + ((idFilterTable->idFilter[j + 1U] &
                                              FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_MASK) <<
                                              FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_SHIFT2);
                }
                j = j + 2U;
            }
            break;
        case (FLEXCAN_RX_FIFO_ID_FORMAT_C):
            /* Four partial 8-bit Standard IDs per ID Filter Table element.*/
            (base->MCR) = (((base->MCR) & ~(CAN_MCR_IDAM_MASK)) | ( (((uint32_t)(((uint32_t)(FLEXCAN_RX_FIFO_ID_FORMAT_C))<<CAN_MCR_IDAM_SHIFT))&CAN_MCR_IDAM_MASK)));
            j = 0;
            for (i = 0; i < RxFifoFilterElementNum(numOfFilters); i++)
            {
                filterTable[i] = ((idFilterTable->idFilter[j] &
                                  FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK) <<
                                  FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT1);
                filterTable[i] = ((idFilterTable->idFilter[j + 1U] &
                                  FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK) <<
                                  FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT2);
                filterTable[i] = ((idFilterTable->idFilter[j + 2U] &
                                  FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK) <<
                                  FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT3);
                filterTable[i] = ((idFilterTable->idFilter[j + 3U] &
                                  FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK) <<
                                  FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT4);
                j = j + 4U;
            }
            break;
        case (FLEXCAN_RX_FIFO_ID_FORMAT_D):
            /* All frames rejected.*/
            (base->MCR) = (((base->MCR) & ~(CAN_MCR_IDAM_MASK)) | ( (((uint32_t)(((uint32_t)(FLEXCAN_RX_FIFO_ID_FORMAT_D))<<CAN_MCR_IDAM_SHIFT))&CAN_MCR_IDAM_MASK)));
            break;
        default:
            /* Should not get here */
            break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetMsgBuffIntCmd
 * Description   : Enable/Disable the corresponding Message Buffer interrupt.
 *
 * Implements    : FLEXCAN_HAL_SetMsgBuffIntCmd_Activity
 *END**************************************************************************/
status_t FLEXCAN_HAL_SetMsgBuffIntCmd(
    CAN_Type * base,
    uint32_t msgBuffIdx, bool enable)
{
    uint32_t temp;
    status_t stat = STATUS_SUCCESS;

    if (msgBuffIdx >= (((base->MCR) & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT))
    {
        stat = STATUS_FLEXCAN_MB_OUT_OF_RANGE;
    }

    if (stat == STATUS_SUCCESS)
    {
        /* Enable the corresponding message buffer Interrupt */
        temp = 1UL << msgBuffIdx;
        if(enable)
        {
            (base->IMASK1) = ((base ->IMASK1) |  (temp));
        }
        else
        {
            (base->IMASK1) = ((base->IMASK1) & ~(temp));
        }
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetErrIntCmd
 * Description   : Enable the error interrupts.
 * This function will enable Error interrupt.
 *
 * Implements    : FLEXCAN_HAL_SetErrIntCmd_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetErrIntCmd(CAN_Type * base, flexcan_int_type_t errType, bool enable)
{
    uint32_t temp = (uint32_t)errType;
    if (enable)
    {
        if((errType == FLEXCAN_INT_RX_WARNING)||(errType == FLEXCAN_INT_TX_WARNING))
        {
           base->MCR = (base->MCR & ~CAN_MCR_WRNEN_MASK) | CAN_MCR_WRNEN(1U);
        }
        (base->CTRL1) = ((base->CTRL1) |  (temp));
    }
    else
    {
        (base->CTRL1) = ((base->CTRL1) & ~(temp));
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_ExitFreezeMode
 * Description   : Exit of freeze mode.
 *
 * Implements    : FLEXCAN_HAL_ExitFreezeMode_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_ExitFreezeMode(CAN_Type * base)
{
    base->MCR = (base->MCR & ~CAN_MCR_HALT_MASK) | CAN_MCR_HALT(0U);
    base->MCR = (base->MCR & ~CAN_MCR_FRZ_MASK) | CAN_MCR_FRZ(0U);

    /* Wait till exit freeze mode */
    while (((base->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT) != 0U) {}
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_EnterFreezeMode
 * Description   : Enter the freeze mode.
 *
 * Implements    : FLEXCAN_HAL_EnterFreezeMode_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_EnterFreezeMode(CAN_Type * base)
{
    base->MCR = (base->MCR & ~CAN_MCR_FRZ_MASK) | CAN_MCR_FRZ(1U);
    base->MCR = (base->MCR & ~CAN_MCR_HALT_MASK) | CAN_MCR_HALT(1U);

    /* Wait for entering the freeze mode */
    while (((base->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT) == 0U) {}
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_GetMsgBuffIntStatusFlag
 * Description   : Get the corresponding message buffer interrupt flag.
 *
 * Implements    : FLEXCAN_HAL_GetMsgBuffIntStatusFlag_Activity
 *END**************************************************************************/
uint8_t FLEXCAN_HAL_GetMsgBuffIntStatusFlag(
    const CAN_Type * base,
    uint32_t msgBuffIdx)
{
    /* Get the corresponding message buffer interrupt flag */
    return (uint8_t)((base->IFLAG1 >> msgBuffIdx) & 1U);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_GetErrCounter
 * Description   : Get transmit error counter and receive error counter.
 *
 * Implements    : FLEXCAN_HAL_GetErrCounter_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_GetErrCounter(
    const CAN_Type * base,
    flexcan_buserr_counter_t *errCount)
{
    DEV_ASSERT(errCount != NULL);

    /* Get transmit error counter and receive error counter */
    errCount->rxerr = (uint16_t)(((base->ECR) & CAN_ECR_RXERRCNT_MASK) >> CAN_ECR_RXERRCNT_SHIFT);
    errCount->txerr = (uint16_t)(((base->ECR) & CAN_ECR_TXERRCNT_MASK) >> CAN_ECR_TXERRCNT_SHIFT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_ClearErrIntStatusFlag
 * Description   : Clear all error interrupt status.
 *
 * Implements    : FLEXCAN_HAL_ClearErrIntStatusFlag_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_ClearErrIntStatusFlag(CAN_Type * base)
{
    if((base->ESR1 & FLEXCAN_ALL_INT) != 0U)
    {
        (base->ESR1) = FLEXCAN_ALL_INT;
#ifdef ERRATA_E9005
        /* Dummy read as a workaround for errata e9005 to ensure the flags are
        cleared before continuing. */
        (void)(base->ESR1);
#endif
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_ReadRxFifo
 * Description   : Read Rx FIFO data.
 * This function will copy MB[0] data field into user's buffer.
 *
 * Implements    : FLEXCAN_HAL_ReadRxFifo_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_ReadRxFifo(
    const CAN_Type * base,
    flexcan_msgbuff_t *rxFifo)
{
    DEV_ASSERT(rxFifo != NULL);

    uint32_t databyte;
    uint32_t mbWord;

    volatile const uint32_t *flexcan_mb = base->RAMn;
    volatile const uint32_t *flexcan_mb_id = &base->RAMn[1];
    volatile const uint32_t *flexcan_mb_data_32 = &flexcan_mb[2];
    uint32_t *msgData_32 = (uint32_t *)(rxFifo->data);

    uint8_t flexcan_mb_dlc_value = (uint8_t)(((*flexcan_mb) & CAN_CS_DLC_MASK) >> 16);
    uint8_t can_real_payload = FLEXCAN_HAL_ComputePayloadSize(flexcan_mb_dlc_value);

    rxFifo->dataLen = can_real_payload;
    rxFifo->cs = *flexcan_mb;

    if ((rxFifo->cs & CAN_CS_IDE_MASK) != 0U)
    {
        rxFifo->msgId = *flexcan_mb_id;
    }
    else
    {
        rxFifo->msgId = (*flexcan_mb_id) >> CAN_ID_STD_SHIFT;
    }

    /* Copy MB[0] data field into user's buffer */
    for (databyte = 0; databyte < can_real_payload; databyte += 4U)
    {
    	mbWord = flexcan_mb_data_32[databyte >> 2U];
        REV_BYTES_32(mbWord, msgData_32[databyte >> 2U]);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetMaskType
 * Description   : Set RX masking type.
 * This function will set RX masking type as RX global mask or RX individual
 * mask.
 *
 * Implements    : FLEXCAN_HAL_SetRxMaskType_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMaskType(
    CAN_Type * base,
    flexcan_rx_mask_type_t type)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* Set RX masking type (RX global mask or RX individual mask)*/
    if (type == FLEXCAN_RX_MASK_GLOBAL)
    {
        /* Enable Global RX masking */
        base->MCR = (base->MCR & ~CAN_MCR_IRMQ_MASK) | CAN_MCR_IRMQ(0U);
    }
    else
    {
        /* Enable Individual Rx Masking and Queue */
        base->MCR = (base->MCR & ~CAN_MCR_IRMQ_MASK) | CAN_MCR_IRMQ(1U);
    }

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxFifoGlobalStdMask
 * Description   : Set Rx FIFO global mask as the 11-bit standard mask.
 *
 * Implements    : FLEXCAN_HAL_SetRxFifoGlobalStdMask_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxFifoGlobalStdMask(
    CAN_Type * base,
    uint32_t stdMask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* 11 bit standard mask*/
    (base->RXFGMASK) = (((uint32_t)(((uint32_t)(stdMask))<<CAN_ID_STD_SHIFT))&CAN_ID_STD_MASK);

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxFifoGlobalExtMask
 * Description   : Set Rx FIFO global mask as the 29-bit extended mask.
 *
 * Implements    : FLEXCAN_HAL_SetRxFifoGlobalExtMask_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxFifoGlobalExtMask(
    CAN_Type * base,
    uint32_t extMask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* 29-bit extended mask*/
    (base->RXFGMASK) = (((uint32_t)(((uint32_t)(extMask)) << CAN_ID_EXT_SHIFT)) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxIndividualStdMask
 * Description   : Set Rx individual mask as the 11-bit standard mask.
 *
 * Implements    : FLEXCAN_HAL_SetRxIndividualStdMask_Activity
 *END**************************************************************************/
status_t FLEXCAN_HAL_SetRxIndividualStdMask(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    uint32_t stdMask)
{
    status_t stat = STATUS_SUCCESS;

    if (msgBuffIdx >= ((((base->MCR) & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT)))
    {
        stat = STATUS_FLEXCAN_MB_OUT_OF_RANGE;
    }

    if (stat == STATUS_SUCCESS)
    {
        /* Set Freeze mode */
        FLEXCAN_HAL_EnterFreezeMode(base);

        /* 11 bit standard mask*/
        (base->RXIMR[msgBuffIdx]) = (stdMask << CAN_ID_STD_SHIFT) & CAN_ID_STD_MASK;

        /* De-assert Freeze Mode */
        FLEXCAN_HAL_ExitFreezeMode(base);
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxIndividualExtMask
 * Description   : Set Rx individual mask as the 29-bit extended mask.
 *
 * Implements    : FLEXCAN_HAL_SetRxIndividualExtMask_Activity
 *END**************************************************************************/
status_t FLEXCAN_HAL_SetRxIndividualExtMask(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    uint32_t extMask)
{
    status_t stat = STATUS_SUCCESS;

    if (msgBuffIdx >= (((base->MCR) & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT))
    {
        stat = STATUS_FLEXCAN_MB_OUT_OF_RANGE;
    }

    if (stat == STATUS_SUCCESS)
    {
        /* Set Freeze mode */
        FLEXCAN_HAL_EnterFreezeMode(base);

        /* 29-bit extended mask */
        base->RXIMR[msgBuffIdx] = (extMask << CAN_ID_EXT_SHIFT) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK);
        /* De-assert Freeze Mode */
        FLEXCAN_HAL_ExitFreezeMode(base);
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMsgBuffGlobalStdMask
 * Description   : Set Rx Message Buffer global mask as the 11-bit standard mask.
 *
 * Implements    : FLEXCAN_HAL_SetRxMsgBuffGlobalStdMask_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMsgBuffGlobalStdMask(
    CAN_Type * base,
    uint32_t stdMask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* 11 bit standard mask*/

    (base->RXMGMASK) = (((uint32_t)(((uint32_t)(stdMask))<<CAN_ID_STD_SHIFT))&CAN_ID_STD_MASK);

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMsgBuff14StdMask
 * Description   : Set Rx Message Buffer 14 mask as the 11-bit standard mask.
 *
 * Implements    : FLEXCAN_HAL_SetRxMsgBuff14StdMask_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMsgBuff14StdMask(
    CAN_Type * base,
    uint32_t stdMask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* 11 bit standard mask*/
    (base->RX14MASK) = (((uint32_t)(((uint32_t)(stdMask))<<CAN_ID_STD_SHIFT))&CAN_ID_STD_MASK);

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMsgBuff15StdMask
 * Description   : Set Rx Message Buffer 15 mask as the 11-bit standard mask.
 *
 * Implements    : FLEXCAN_HAL_SetRxMsgBuff15StdMask_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMsgBuff15StdMask(
    CAN_Type * base,
    uint32_t stdMask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* 11 bit standard mask*/
    (base->RX15MASK) = (((uint32_t)(((uint32_t)(stdMask))<<CAN_ID_STD_SHIFT))&CAN_ID_STD_MASK);

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMsgBuffGlobalExtMask
 * Description   : Set Rx Message Buffer global mask as the 29-bit extended mask.
 *
 * Implements    : FLEXCAN_HAL_SetRxMsgBuffGlobalExtMask_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMsgBuffGlobalExtMask(
    CAN_Type * base,
    uint32_t extMask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* 29-bit extended mask*/
    (base->RXMGMASK) = (((uint32_t)(((uint32_t)(extMask)) << CAN_ID_EXT_SHIFT)) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMsgBuff14ExtMask
 * Description   : Set Rx Message Buffer 14 mask as the 29-bit extended mask.
 *
 * Implements    : FLEXCAN_HAL_SetRxMsgBuff14ExtMask_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMsgBuff14ExtMask(
    CAN_Type * base,
    uint32_t extMask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* 29-bit extended mask*/
    (base->RX14MASK) = (((uint32_t)(((uint32_t)(extMask)) << CAN_ID_EXT_SHIFT)) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMsgBuff15ExtMask
 * Description   : Set Rx Message Buffer 15 mask as the 29-bit extended mask.
 *
 * Implements    : FLEXCAN_HAL_SetRxMsgBuff15ExtMask_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMsgBuff15ExtMask(
    CAN_Type * base,
    uint32_t extMask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* 29-bit extended mask*/
    (base->RX15MASK) = (((uint32_t)(((uint32_t)(extMask)) << CAN_ID_EXT_SHIFT)) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetOperationMode
 * Description   : Enable a FlexCAN operation mode.
 * This function will enable one of the modes listed in flexcan_operation_modes_t.
 *
 * Implements    : FLEXCAN_HAL_SetOperationMode_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetOperationMode(
    CAN_Type * base,
    flexcan_operation_modes_t mode)
{

    if (mode == FLEXCAN_FREEZE_MODE)
    {
        /* Debug mode, Halt and Freeze*/
        FLEXCAN_HAL_EnterFreezeMode(base);
    }
    else if (mode == FLEXCAN_DISABLE_MODE)
    {
        /* Debug mode, Halt and Freeze */
        base->MCR = (base->MCR & ~CAN_MCR_MDIS_MASK) | CAN_MCR_MDIS(1U);
    }
    else {
        /* Set Freeze mode */
        FLEXCAN_HAL_EnterFreezeMode(base);

        if (mode == FLEXCAN_NORMAL_MODE)
        {
            base->MCR = (base->MCR & ~CAN_MCR_SUPV_MASK) | CAN_MCR_SUPV(0U);
            base->CTRL1 = (base->CTRL1 & ~CAN_CTRL1_LOM_MASK) | CAN_CTRL1_LOM(0U);
            base->CTRL1 = (base->CTRL1 & ~CAN_CTRL1_LPB_MASK) | CAN_CTRL1_LPB(0U);
        }
        else if (mode == FLEXCAN_LISTEN_ONLY_MODE)
        {
            base->CTRL1 = (base->CTRL1 & ~CAN_CTRL1_LOM_MASK) | CAN_CTRL1_LOM(1U);
        }
        else if (mode == FLEXCAN_LOOPBACK_MODE)
        {
             base->CTRL1 = (base->CTRL1 & ~CAN_CTRL1_LPB_MASK) | CAN_CTRL1_LPB(1U);
             base->CTRL1 = (base->CTRL1 & ~CAN_CTRL1_LOM_MASK) | CAN_CTRL1_LOM(0U);
        }
        else {
            /* Should not get here */
        }

        /* De-assert Freeze Mode */
        FLEXCAN_HAL_ExitFreezeMode(base);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_ExitOperationMode
 * Description   : Disable a FlexCAN operation mode.
 * This function will disable one of the modes listed in flexcan_operation_modes_t.
 *
 * Implements    : FLEXCAN_HAL_ExitOperationMode_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_ExitOperationMode(
    CAN_Type * base,
    flexcan_operation_modes_t mode)
{

    if (mode == FLEXCAN_FREEZE_MODE)
    {
        /* De-assert Freeze Mode*/
        FLEXCAN_HAL_ExitFreezeMode(base);
    }
    else if (mode == FLEXCAN_DISABLE_MODE)
    {
        /* Disable module mode*/
        base->MCR = (base->MCR & ~CAN_MCR_MDIS_MASK) | CAN_MCR_MDIS(0U);
    }
    else
    {
        /* Set Freeze mode */
        FLEXCAN_HAL_EnterFreezeMode(base);

        if (mode == FLEXCAN_NORMAL_MODE)
        {
            base->MCR = (base->MCR & ~CAN_MCR_SUPV_MASK) | CAN_MCR_SUPV(1U);
        }
        else if (mode == FLEXCAN_LISTEN_ONLY_MODE)
        {
            base->CTRL1 = (base->CTRL1 & ~CAN_CTRL1_LOM_MASK) | CAN_CTRL1_LOM(0U);
        }
        else if (mode == FLEXCAN_LOOPBACK_MODE)
        {
            base->CTRL1 = (base->CTRL1 & ~CAN_CTRL1_LPB_MASK) | CAN_CTRL1_LPB(0U);
        }
        else
        {
            /* Should not get here */
        }

        /* De-assert Freeze Mode */
        FLEXCAN_HAL_ExitFreezeMode(base);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetFDEnabled
 * Description   : Enables/Disables Flexible Data rate (if supported).
 *
 * Implements    : FLEXCAN_HAL_SetFDEnabled_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetFDEnabled(CAN_Type * base, bool enable)
{
    FLEXCAN_HAL_EnterFreezeMode(base);

    base->MCR = (base->MCR & ~CAN_MCR_FDEN_MASK) | CAN_MCR_FDEN(enable? 1UL : 0UL);

    /* Enable the use of extended bit time definitions */
    base->CBT = base->CBT | CAN_CBT_BTF(1U);

    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_IsFDEnabled
 * Description   : Checks if the Flexible Data rate feature is enabled.
 *
 * Implements    : FLEXCAN_HAL_IsFDEnabled_Activity
 *END**************************************************************************/
bool FLEXCAN_HAL_IsFDEnabled(const CAN_Type * base)
{
    return (((base->MCR & CAN_MCR_FDEN_MASK) >> CAN_MCR_FDEN_SHIFT) != 0U);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetPayloadSize
 * Description   : Sets the payload size of the MBs.
 *
 * Implements    : FLEXCAN_HAL_SetPayloadSize_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetPayloadSize(
    CAN_Type * base,
    flexcan_fd_payload_size_t payloadSize)
{
    uint32_t tmp;

    DEV_ASSERT(FLEXCAN_HAL_IsFDEnabled(base) || (payloadSize == FLEXCAN_PAYLOAD_SIZE_8));

    /* If FD is not enabled, only 8 bytes payload is supported */
    if (FLEXCAN_HAL_IsFDEnabled(base))
    {
        FLEXCAN_HAL_EnterFreezeMode(base);

        tmp = base->FDCTRL;
        tmp &= ~(CAN_FDCTRL_MBDSR0_MASK);
        tmp |= ((uint32_t)payloadSize) << CAN_FDCTRL_MBDSR0_SHIFT;
#if FEATURE_CAN_HAS_MBDSR1
        tmp &= ~(CAN_FDCTRL_MBDSR1_MASK);
        tmp |= ((uint32_t)payloadSize) << CAN_FDCTRL_MBDSR1_SHIFT;
#endif

        base->FDCTRL = tmp;

        FLEXCAN_HAL_ExitFreezeMode(base);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_GetPayloadSize
 * Description   : Returns the payload size of the MBs (in bytes).
 *
 * Implements    : FLEXCAN_HAL_GetPayloadSize_Activity
 *END**************************************************************************/
uint8_t FLEXCAN_HAL_GetPayloadSize(const CAN_Type * base)
{
    uint32_t payloadSize;

    /* The standard payload size is 8 bytes */
    if (!FLEXCAN_HAL_IsFDEnabled(base))
    {
        payloadSize = 8U;
    }
    else
    {
        payloadSize = 1UL << (((base->FDCTRL & CAN_FDCTRL_MBDSR0_MASK) >> CAN_FDCTRL_MBDSR0_SHIFT) + 3U);
    }

    return (uint8_t)payloadSize;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetStuffBitCount
 * Description   : Enables/Disables the Stuff Bit Count for CAN FD frames.
 *
 * Implements    : FLEXCAN_HAL_SetStuffBitCount_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetStuffBitCount(CAN_Type * base, bool enable)
{
    FLEXCAN_HAL_EnterFreezeMode(base);

#if FEATURE_CAN_HAS_STFCNTEN_ENABLE
    base->CTRL2 = (base->CTRL2 & ~CAN_CTRL2_STFCNTEN_MASK) | CAN_CTRL2_STFCNTEN(enable? 1UL : 0UL);
#elif FEATURE_CAN_HAS_ISOCANFDEN_ENABLE
    base->CTRL2 = (base->CTRL2 & ~CAN_CTRL2_ISOCANFDEN_MASK) |
                   CAN_CTRL2_ISOCANFDEN(enable? 1UL : 0UL);
#endif

    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetSelfReception
 * Description   : Enables/Disables the Self Reception feature.
 *
 * Implements    : FLEXCAN_HAL_SetSelfReception_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetSelfReception(CAN_Type * base, bool enable)
{
    FLEXCAN_HAL_EnterFreezeMode(base);

    base->MCR = (base->MCR & ~CAN_MCR_SRXDIS_MASK) | CAN_MCR_SRXDIS(enable? 0UL : 1UL);

    FLEXCAN_HAL_ExitFreezeMode(base);
}

#if FEATURE_CAN_HAS_DMA_ENABLE
/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxFifoDMA
 * Description   : Enables/Disables the DMA support for RxFIFO.
 *
 * Implements    : FLEXCAN_HAL_SetRxFifoDMA_Activity
 *END**************************************************************************/
status_t FLEXCAN_HAL_SetRxFifoDMA(CAN_Type * base, bool enable)
{
    uint32_t rxFifoEnabled = (base->MCR & CAN_MCR_RFEN_MASK);
    if (enable && (rxFifoEnabled == 0U))
    {
        return STATUS_ERROR;
    }

    FLEXCAN_HAL_EnterFreezeMode(base);

    base->MCR = (base->MCR & ~CAN_MCR_DMA_MASK) | CAN_MCR_DMA(enable? 1UL : 0UL);

    FLEXCAN_HAL_ExitFreezeMode(base);

    return STATUS_SUCCESS;
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetTDCOffset
 * Description   : Enables/Disables the Transceiver Delay Compensation feature
 * and sets the Transceiver Delay Compensation Offset.
 *
 * Implements    : FLEXCAN_HAL_SetTDCOffset_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_SetTDCOffset(CAN_Type * base, bool enable, uint8_t offset)
{
    uint32_t tmp;

    FLEXCAN_HAL_EnterFreezeMode(base);

    tmp = base->FDCTRL;

    if (enable)
    {
        tmp = tmp | CAN_FDCTRL_TDCEN_MASK;
        tmp = tmp & ~CAN_FDCTRL_TDCOFF_MASK;
        tmp = tmp | CAN_FDCTRL_TDCOFF(offset);
    }
    else
    {
        tmp = tmp & ~CAN_FDCTRL_TDCEN_MASK;
    }

    base->FDCTRL = tmp;

    FLEXCAN_HAL_ExitFreezeMode(base);
}

#if FEATURE_CAN_HAS_PRETENDED_NETWORKING

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_ConfigPN
 * Description   : Configures the Pretended Networking mode.
 *
 * Implements    : FLEXCAN_HAL_ConfigPN_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_ConfigPN(CAN_Type * base, const flexcan_pn_config_t *pnConfig)
{
    DEV_ASSERT(pnConfig != NULL);

    uint32_t tmp;

    /* Configure specific pretended networking settings */
    tmp = base->CTRL1_PN;
    tmp &= ~(CAN_CTRL1_PN_WTOF_MSK_MASK | CAN_CTRL1_PN_WUMF_MSK_MASK | CAN_CTRL1_PN_NMATCH_MASK |
            CAN_CTRL1_PN_PLFS_MASK | CAN_CTRL1_PN_IDFS_MASK | CAN_CTRL1_PN_FCS_MASK);
    tmp |= CAN_CTRL1_PN_WTOF_MSK(pnConfig->wakeUpTimeout ? 1UL : 0UL);
    tmp |= CAN_CTRL1_PN_WUMF_MSK(pnConfig->wakeUpMatch ? 1UL : 0UL);
    tmp |= CAN_CTRL1_PN_NMATCH(pnConfig->numMatches);
    tmp |= CAN_CTRL1_PN_FCS(pnConfig->filterComb);
    tmp |= CAN_CTRL1_PN_IDFS(pnConfig->idFilterType);
    tmp |= CAN_CTRL1_PN_PLFS(pnConfig->payloadFilterType);
    base->CTRL1_PN = tmp;

    tmp = base->CTRL2_PN;
    tmp &= ~CAN_CTRL2_PN_MATCHTO_MASK;
    tmp |= CAN_CTRL2_PN_MATCHTO(pnConfig->matchTimeout);
    base->CTRL2_PN = tmp;

    /* Configure ID filtering */
    tmp = base->FLT_ID1;
    tmp &= ~(CAN_FLT_ID1_FLT_IDE_MASK | CAN_FLT_ID1_FLT_RTR_MASK | CAN_FLT_ID1_FLT_ID1_MASK);
    tmp |= CAN_FLT_ID1_FLT_IDE(pnConfig->idFilter1.extendedId ? 1UL : 0UL);
    tmp |= CAN_FLT_ID1_FLT_RTR(pnConfig->idFilter1.remoteFrame ? 1UL : 0UL);
    if (pnConfig->idFilter1.extendedId)
    {
        tmp |= CAN_FLT_ID1_FLT_ID1(pnConfig->idFilter1.id);
    }
    else
    {
        tmp |= CAN_FLT_ID1_FLT_ID1(pnConfig->idFilter1.id << CAN_ID_STD_SHIFT);
    }
    base->FLT_ID1 = tmp;

    /* Configure the second ID, if needed (as mask for exact matching or higher limit for range matching) */
    if ((pnConfig->idFilterType == FLEXCAN_FILTER_MATCH_EXACT) || (pnConfig->idFilterType == FLEXCAN_FILTER_MATCH_RANGE))
    {
        tmp = base->FLT_ID2_IDMASK;
        tmp &= ~(CAN_FLT_ID2_IDMASK_IDE_MSK_MASK | CAN_FLT_ID2_IDMASK_RTR_MSK_MASK | CAN_FLT_ID2_IDMASK_FLT_ID2_IDMASK_MASK);
        tmp |= CAN_FLT_ID2_IDMASK_IDE_MSK(pnConfig->idFilter2.extendedId ? 1UL : 0UL);
        tmp |= CAN_FLT_ID2_IDMASK_RTR_MSK(pnConfig->idFilter2.remoteFrame ? 1UL : 0UL);
        if (pnConfig->idFilter2.extendedId)
        {
            tmp |= CAN_FLT_ID2_IDMASK_FLT_ID2_IDMASK(pnConfig->idFilter2.id);
        }
        else
        {
            tmp |= CAN_FLT_ID2_IDMASK_FLT_ID2_IDMASK(pnConfig->idFilter2.id << CAN_ID_STD_SHIFT);
        }
        base->FLT_ID2_IDMASK = tmp;
    }

    /* Configure payload filtering, if requested */
    if ((pnConfig->filterComb == FLEXCAN_FILTER_ID_PAYLOAD) || (pnConfig->filterComb == FLEXCAN_FILTER_ID_PAYLOAD_NTIMES))
    {
        tmp = base->FLT_DLC;
        tmp &= ~(CAN_FLT_DLC_FLT_DLC_HI_MASK | CAN_FLT_DLC_FLT_DLC_HI_MASK);
        tmp |= CAN_FLT_DLC_FLT_DLC_HI(pnConfig->payloadFilter.dlcHigh);
        tmp |= CAN_FLT_DLC_FLT_DLC_LO(pnConfig->payloadFilter.dlcLow);
        base->FLT_DLC = tmp;

        tmp = CAN_PL1_HI_Data_byte_4(pnConfig->payloadFilter.payload1[4]);
        tmp |= CAN_PL1_HI_Data_byte_5(pnConfig->payloadFilter.payload1[5]);
        tmp |= CAN_PL1_HI_Data_byte_6(pnConfig->payloadFilter.payload1[6]);
        tmp |= CAN_PL1_HI_Data_byte_7(pnConfig->payloadFilter.payload1[7]);
        base->PL1_HI = tmp;

        tmp = CAN_PL1_LO_Data_byte_0(pnConfig->payloadFilter.payload1[0]);
        tmp |= CAN_PL1_LO_Data_byte_1(pnConfig->payloadFilter.payload1[1]);
        tmp |= CAN_PL1_LO_Data_byte_2(pnConfig->payloadFilter.payload1[2]);
        tmp |= CAN_PL1_LO_Data_byte_3(pnConfig->payloadFilter.payload1[3]);
        base->PL1_LO = tmp;

        /* Configure the second payload, if needed (as mask for exact matching or higher limit for range matching) */
        if ((pnConfig->payloadFilterType == FLEXCAN_FILTER_MATCH_EXACT) || (pnConfig->payloadFilterType == FLEXCAN_FILTER_MATCH_RANGE))
        {
            tmp = CAN_PL2_PLMASK_HI_Data_byte_4(pnConfig->payloadFilter.payload2[4]);
            tmp |= CAN_PL2_PLMASK_HI_Data_byte_5(pnConfig->payloadFilter.payload2[5]);
            tmp |= CAN_PL2_PLMASK_HI_Data_byte_6(pnConfig->payloadFilter.payload2[6]);
            tmp |= CAN_PL2_PLMASK_HI_Data_byte_7(pnConfig->payloadFilter.payload2[7]);
            base->PL2_PLMASK_HI = tmp;

            tmp = CAN_PL2_PLMASK_LO_Data_byte_0(pnConfig->payloadFilter.payload2[0]);
            tmp |= CAN_PL2_PLMASK_LO_Data_byte_1(pnConfig->payloadFilter.payload2[1]);
            tmp |= CAN_PL2_PLMASK_LO_Data_byte_2(pnConfig->payloadFilter.payload2[2]);
            tmp |= CAN_PL2_PLMASK_LO_Data_byte_3(pnConfig->payloadFilter.payload2[3]);
            base->PL2_PLMASK_LO = tmp;
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_GetWMB
 * Description   : Extracts one of the frames which triggered the wake up event.
 *
 * Implements    : FLEXCAN_HAL_GetWMB_Activity
 *END**************************************************************************/
void FLEXCAN_HAL_GetWMB(const CAN_Type * base, uint8_t wmbIndex, flexcan_msgbuff_t *wmb)
{
    DEV_ASSERT(wmb != NULL);

    uint32_t *tmp, wmbData;

    tmp = (uint32_t *)&wmb->data[0];
    wmbData = base->WMB[wmbIndex].WMBn_D03;
    REV_BYTES_32(wmbData, *tmp);

    tmp = (uint32_t *)&wmb->data[4];
    wmbData = base->WMB[wmbIndex].WMBn_D47;
    REV_BYTES_32(wmbData, *tmp);

    wmb->cs = base->WMB[wmbIndex].WMBn_CS;

    if ((wmb->cs & CAN_CS_IDE_MASK) != 0U)
    {
        wmb->msgId = base->WMB[wmbIndex].WMBn_ID;
    }
    else
    {
        wmb->msgId = base->WMB[wmbIndex].WMBn_ID >> CAN_ID_STD_SHIFT;
    }

    wmb->dataLen = (uint8_t)((wmb->cs & CAN_CS_DLC_MASK) >> 16);
}

#endif /* FEATURE_CAN_HAS_PRETENDED_NETWORKING */

/*******************************************************************************
 * EOF
 ******************************************************************************/
