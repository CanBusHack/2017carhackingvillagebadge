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
 * @file edma_irq.c
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, external symbol defined without a prior
 * declaration.
 * The symbols are declared in the driver common file as external; they are needed
 * at driver initialization to install the correct interrupt handler, but are not
 * a part of the public API.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, Function not defined with external linkage.
 * The functions are not defined static because they are referenced in .s startup files.
 */

#include "device_registers.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/* External declaration of interrupt handlers, implemented in the driver c file */
void EDMA_DRV_IRQHandler(uint8_t channel);
void EDMA_DRV_ErrorIRQHandler(void);

#ifdef FEATURE_EDMA_ORED_IRQ_LINES_16_CHN
/*! @brief EDMA IRQ handler with the same name in the startup code*/
void DMA0_15_IRQHandler(void)
{
    /* Read the status flags register */
    uint32_t mask = 0xFFFF;
    uint32_t flags = DMA->INT;
    uint8_t i = 0U;
    flags &= mask;
    /* Check all the flags from 0 to 15 and call the handler for the appropriate channel */
    while (flags > 0U)
    {
       if ((flags & 1U) > 0U)
       {
           EDMA_DRV_IRQHandler(i);
       }
       i++;
       flags >>= 1U;
    }
}

/*! @brief DMA16_31_IRQn IRQ handler with the same name in the startup code*/
void DMA16_31_IRQHandler(void)
{
    /* Read the status flags register */
    uint32_t flags = DMA->INT;
    uint8_t i = 16U;
    flags >>= 16U;
    /* Check all the flags from 16 to 31 and call the handler for the appropriate channel */
    while (flags > 0)
    {
       if ((flags & 1U) > 0)
       {
           EDMA_DRV_IRQHandler(i);
       }
       i++;
       flags >>= 1;
    }
}
#endif

#ifdef FEATURE_EDMA_SEPARATE_IRQ_LINES_PER_CHN
/*! @brief EDMA IRQ handler with the same name in the startup code*/
void DMA0_IRQHandler(void)
{
    EDMA_DRV_IRQHandler(0);
}

/*! @brief EDMA IRQ handler with the same name in the startup code*/
void DMA1_IRQHandler(void)
{
    EDMA_DRV_IRQHandler(1);
}

/*! @brief EDMA IRQ handler with the same name in the startup code*/
void DMA2_IRQHandler(void)
{
    EDMA_DRV_IRQHandler(2);
}

/*! @brief EDMA IRQ handler with the same name in the startup code*/
void DMA3_IRQHandler(void)
{
    EDMA_DRV_IRQHandler(3);
}

/*! @brief EDMA IRQ handler with the same name in the startup code*/
void DMA4_IRQHandler(void)
{
    EDMA_DRV_IRQHandler(4);
}

/*! @brief EDMA IRQ handler with the same name in the startup code*/
void DMA5_IRQHandler(void)
{
    EDMA_DRV_IRQHandler(5);
}

/*! @brief EDMA IRQ handler with the same name in the startup code*/
void DMA6_IRQHandler(void)
{
    EDMA_DRV_IRQHandler(6);
}

/*! @brief EDMA IRQ handler with the same name in the startup code*/
void DMA7_IRQHandler(void)
{
    EDMA_DRV_IRQHandler(7);
}

/*! @brief EDMA IRQ handler with the same name in the startup code*/
void DMA8_IRQHandler(void)
{
    EDMA_DRV_IRQHandler(8);
}

/*! @brief EDMA IRQ handler with the same name in the startup code*/
void DMA9_IRQHandler(void)
{
    EDMA_DRV_IRQHandler(9);
}

/*! @brief EDMA IRQ handler with the same name in the startup code*/
void DMA10_IRQHandler(void)
{
    EDMA_DRV_IRQHandler(10);
}

/*! @brief EDMA IRQ handler with the same name in the startup code*/
void DMA11_IRQHandler(void)
{
    EDMA_DRV_IRQHandler(11);
}

/*! @brief EDMA IRQ handler with the same name in the startup code*/
void DMA12_IRQHandler(void)
{
    EDMA_DRV_IRQHandler(12);
}

/*! @brief EDMA IRQ handler with the same name in the startup code*/
void DMA13_IRQHandler(void)
{
    EDMA_DRV_IRQHandler(13);
}

/*! @brief EDMA IRQ handler with the same name in the startup code*/
void DMA14_IRQHandler(void)
{
    EDMA_DRV_IRQHandler(14);
}

/*! @brief EDMA IRQ handler with the same name in the startup code*/
void DMA15_IRQHandler(void)
{
    EDMA_DRV_IRQHandler(15);
}
#endif

#ifdef FEATURE_EDMA_HAS_ERROR_IRQ
/*! @brief EDMA ERROR IRQ handler with the same name in the startup code*/
void DMA_Error_IRQHandler(void)
{
    EDMA_DRV_ErrorIRQHandler();
}
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/

