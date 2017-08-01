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
 * @file edma_common.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, could define variable at block scope
 * The variables are defined in the common source file to make transition to other
 * platforms easier.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 */

#include <stdint.h>
#include "device_registers.h"
#include "clock_manager.h"
#include "edma_driver.h"

/* Array of base addresses for DMA instances. */
DMA_Type * const g_edmaBase[DMA_INSTANCE_COUNT] = DMA_BASE_PTRS;

/* Array of base addresses for DMAMUX instances. */
DMAMUX_Type * const  g_dmamuxBase[DMAMUX_INSTANCE_COUNT] = DMAMUX_BASE_PTRS;

/* Array of default DMA channel interrupt handlers. */
const IRQn_Type g_edmaIrqId[FEATURE_CHANNEL_INTERRUPT_LINES] = DMA_CHN_IRQS;

/* Array of default DMA error interrupt handlers. */
#if defined FEATURE_EDMA_HAS_ERROR_IRQ
const IRQn_Type g_edmaErrIrqId[FEATURE_ERROR_INTERRUPT_LINES] = DMA_ERROR_IRQS;
#endif

/*! Array for eDMA & DMAMUX clock sources. */
const clock_names_t g_edmaClockNames[DMA_INSTANCE_COUNT] = EDMA_CLOCK_NAMES;
const clock_names_t g_dmamuxClockNames[DMAMUX_INSTANCE_COUNT] = DMAMUX_CLOCK_NAMES;

/*******************************************************************************
* EOF
******************************************************************************/

