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

 /*!
 * @file flexcan_irq.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, Function not defined with external linkage.
 * The functions are not defined static because they are referenced in .s startup files.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, could define variable at block scope
 * The variable is defined in the common source file to make transition to other
 * platforms easier.
 */

#include "flexcan_driver.h"

#if (defined(CPU_S32K144HFT0VLLT) || defined(CPU_S32K144LFT0MLLT))

/*******************************************************************************
 * Default interrupt handlers signatures
 ******************************************************************************/
void CAN0_ORed_IRQHandler(void);
void CAN0_Error_IRQHandler(void);
void CAN0_Wake_Up_IRQHandler(void);
void CAN0_ORed_0_15_MB_IRQHandler(void);
void CAN0_ORed_16_31_MB_IRQHandler(void);
#if (CAN_INSTANCE_COUNT > 1U)
void CAN1_ORed_IRQHandler(void);
void CAN1_Error_IRQHandler(void);
void CAN1_Wake_Up_IRQHandler(void);
void CAN1_ORed_0_15_MB_IRQHandler(void);
void CAN1_ORed_16_31_MB_IRQHandler(void);
#endif
#if (CAN_INSTANCE_COUNT > 2U)
void CAN2_ORed_IRQHandler(void);
void CAN2_Error_IRQHandler(void);
void CAN2_Wake_Up_IRQHandler(void);
void CAN2_ORed_0_15_MB_IRQHandler(void);
void CAN2_ORed_16_31_MB_IRQHandler(void);
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
#if (CAN_INSTANCE_COUNT > 0U)
/* Implementation of CAN0 IRQ handler for OR'ed interrupts (Bus Off,
Transmit Warning, Receive Warning). */
void CAN0_ORed_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(0);
}

/* Implementation of CAN0 IRQ handler for interrupts indicating that errors were
detected on the CAN bus. */
void CAN0_Error_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(0);
}

/* Implementation of CAN0 IRQ handler for interrupts indicating a wake up
event. */
void CAN0_Wake_Up_IRQHandler(void)
{
    FLEXCAN_DRV_WakeUpHandler(0);
}

/* Implementation of CAN0 IRQ handler for interrupts indicating a successful
transmission or reception for Message Buffers 0-15. */
void CAN0_ORed_0_15_MB_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(0);
}

/* Implementation of CAN0 IRQ handler for interrupts indicating a successful
transmission or reception for Message Buffers 16-31. */
void CAN0_ORed_16_31_MB_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(0);
}
#endif /* (CAN_INSTANCE_COUNT > 0U) */

#if (CAN_INSTANCE_COUNT > 1U)
/* Implementation of CAN1 IRQ handler for OR'ed interrupts (Bus Off,
Transmit Warning, Receive Warning). */
void CAN1_ORed_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(1);
}

/* Implementation of CAN1 IRQ handler for interrupts indicating that errors were
detected on the CAN bus. */
void CAN1_Error_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(1);
}

/* Implementation of CAN1 IRQ handler for interrupts indicating a wake up
event. */
void CAN1_Wake_Up_IRQHandler(void)
{
    FLEXCAN_DRV_WakeUpHandler(1);
}

/* Implementation of CAN1 IRQ handler for interrupts indicating a successful
transmission or reception for Message Buffers 0-15. */
void CAN1_ORed_0_15_MB_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(1);
}

/* Implementation of CAN1 IRQ handler for interrupts indicating a successful
transmission or reception for Message Buffers 16-31. */
void CAN1_ORed_16_31_MB_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(1);
}
#endif /* (CAN_INSTANCE_COUNT > 1U) */

#if (CAN_INSTANCE_COUNT > 2U)
/* Implementation of CAN2 IRQ handler for OR'ed interrupts (Bus Off,
Transmit Warning, Receive Warning). */
void CAN2_ORed_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(2);
}

/* Implementation of CAN2 IRQ handler for interrupts indicating that errors were
detected on the CAN bus. */
void CAN2_Error_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(2);
}

/* Implementation of CAN2 IRQ handler for interrupts indicating a wake up
event. */
void CAN2_Wake_Up_IRQHandler(void)
{
    FLEXCAN_DRV_WakeUpHandler(2);
}

/* Implementation of CAN2 IRQ handler for interrupts indicating a successful
transmission or reception for Message Buffers 0-15. */
void CAN2_ORed_0_15_MB_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(2);
}

/* Implementation of CAN2 IRQ handler for interrupts indicating a successful
transmission or reception for Message Buffers 16-31. */
void CAN2_ORed_16_31_MB_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(2);
}
#endif /* (CAN_INSTANCE_COUNT > 2U) */

#elif defined(CPU_S32V234)

/*******************************************************************************
 * Default interrupt handlers signatures
 ******************************************************************************/
void CAN0_IRQHandler(void);
void CAN0_Buff_IRQHandler(void);
#if (CAN_INSTANCE_COUNT > 1U)
void CAN1_IRQHandler(void);
void CAN1_Buff_IRQHandler(void);
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
#if (CAN_INSTANCE_COUNT > 0U)
/* Implementation of CAN0 handler named in startup code. */
void CAN0_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(0);
}

/* Implementation of CAN0 handler named in startup code. */
void CAN0_Buff_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(0);
}
#endif /* (CAN_INSTANCE_COUNT > 0U) */

#if (CAN_INSTANCE_COUNT > 1U)
/* Implementation of CAN1 handler named in startup code. */
void CAN1_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(1);
}

/* Implementation of CAN1 handler named in startup code. */
void CAN1_Buff_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(1);
}
#endif /* (CAN_INSTANCE_COUNT > 1U) */

#else
    #error "No valid CPU defined!"
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/

