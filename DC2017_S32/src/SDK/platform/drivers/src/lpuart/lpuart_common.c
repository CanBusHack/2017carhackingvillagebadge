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
 * @file lpuart_common.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, A conversion should not be
 * performed between a pointer to object and an integer type.
 * The cast is required as LPUART instance base addresses are defined as unsigned
 * integers in the header file, but the registers are accessed via pointers to
 * structures.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, external symbol defined without a prior
 * declaration.
 * The symbols are declared in the driver header as external; the header is not included
 * by this file.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, could define variable at block scope
 * The variables are defined in the common source file to make transition to other
 * platforms easier.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, A conversion should not be
 * performed between a pointer to object and an integer type.
 * The cast is required as LPUART instance base addresses are defined as unsigned
 * integers in the header file, but the registers are accessed via pointers to
 * structures.
 */


#include "device_registers.h"
#include "clock_manager.h"
#include "interrupt_manager.h"
#if FEATURE_LPUART_HAS_DMA_ENABLE
#include "edma_driver.h"
#endif

/*******************************************************************************
 *  Default interrupt handlers signatures
 ******************************************************************************/

#if (LPUART_INSTANCE_COUNT > 0U)
/*! @brief LPUART0 interrupt handler. */
extern void LPUART0_IrqHandler(void);
#endif

#if (LPUART_INSTANCE_COUNT > 1U)
/*! @brief LPUART1 interrupt handler. */
extern void LPUART1_IrqHandler(void);
#endif

#if (LPUART_INSTANCE_COUNT > 2U)
/*! @brief LPUART2 interrupt handler. */
extern void LPUART2_IrqHandler(void);
#endif

#if (LPUART_INSTANCE_COUNT > 3U)
/*! @brief LPUART3 interrupt handler. */
extern void LPUART3_IrqHandler(void);
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of base addresses for lpuart instances. */
LPUART_Type * const g_lpuartBase[LPUART_INSTANCE_COUNT] = LPUART_BASE_PTRS;

/* Table to save LPUART enum numbers defined in CMSIS files. */
const IRQn_Type g_lpuartRxTxIrqId[LPUART_INSTANCE_COUNT] = LPUART_RX_TX_IRQS;

/* Table to save LPUART clock names as defined in clock manager. */
const clock_names_t g_lpuartClkNames[LPUART_INSTANCE_COUNT] = {PCC_LPUART0_CLOCK, PCC_LPUART1_CLOCK,
                                                               PCC_LPUART2_CLOCK};

/* Table to save LPUART ISRs - to be used for interrupt service routing
 * at runtime, parameter for INT_SYS_InstallHandler */
const isr_t g_lpuartIsr[LPUART_INSTANCE_COUNT] = {LPUART0_IrqHandler, LPUART1_IrqHandler, LPUART2_IrqHandler};

/*******************************************************************************
 * EOF
 ******************************************************************************/
