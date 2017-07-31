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

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.1, Conversions shall not be performed
 * between a pointer to a function and any other type.
 * This is required in order to read/write from vector table memory.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, A cast shall not be performed
 * between pointer to void and an arithmetic type.
 * The address of hardware modules is provided as integer so
 * it needs to be cast to pointer.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, A conversion should not be performed
 * between a pointer to object and an integer type.
 * The address of hardware modules is provided as integer so
 * a conversion between a pointer and an integer has to be performed.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, An object should be defined at block scope
 * if its identifier only appears in a single function.
 * __VECTOR_RAM variable is not an object with static storage duration, it needs to be
 * declared as extern.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 */

 /*! @file interrupt_manager.c */

#include "interrupt_manager.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Counter to manage the nested callings of global disable/enable interrupt.
 */
static int32_t g_interruptDisableCount = 0;

/*!
 * @brief Declaration of vector table.
 * FEATURE_INTERRUPT_IRQ_MAX is the highest interrupt request number.
 * 16 is the maximum number of exceptions
 */
extern uint32_t __VECTOR_RAM[((uint32_t)(FEATURE_INTERRUPT_IRQ_MAX)) + 16U + 1U];

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : INT_SYS_InstallHandler
 * Description   : Install an interrupt handler routine for a given IRQ number
 * This function will let application register/replace the interrupt
 * handler for specified IRQ number. IRQ number is different than Vector
 * number. IRQ 0 will start from Vector 16 address. Refer to reference
 * manual for details. Also refer to startup_<CPU>.s file for each chip
 * family to find out the default interrupt handler for each device. This
 * function will convert the IRQ number to vector number by adding 16 to
 * it.
 *
 * Note          : This method is applicable only if interrupt vector is
 *                 copied in RAM, __flash_vector_table__ symbol is used to
 *                 control this from linker options.
 * Implements INT_SYS_InstallHandler_Activity
 *
 *END**************************************************************************/
void INT_SYS_InstallHandler(IRQn_Type irqNumber,
                            const isr_t newHandler,
                            isr_t* const oldHandler)
{
    /* Check IRQ number */
    DEV_ASSERT(FEATURE_INTERRUPT_IRQ_MIN <= irqNumber);
    DEV_ASSERT(irqNumber <= FEATURE_INTERRUPT_IRQ_MAX);
    DEV_ASSERT(__VECTOR_RAM != 0U);
    /* Check whether there is vector table in RAM */
    DEV_ASSERT((uint32_t)__VECTOR_RAM == S32_SCB->VTOR);

    /* Save the former handler pointer */
    if (oldHandler != (isr_t *) 0)
    {
        *oldHandler = (isr_t)__VECTOR_RAM[((int32_t)irqNumber) + 16];
    }

#if FEATURE_MSCM_HAS_INTERRUPT_ROUTER

    DEV_ASSERT((uint32_t)irqNumber < MSCM_IRSPRC_COUNT);
    /* Check routing is not read-only in case it needs to be written */
    uint16_t cpu_enable = (uint16_t)(1UL << (MSCM->CPXNUM));
    if ((MSCM->IRSPRC[irqNumber] & cpu_enable) == 0U)
    {
        DEV_ASSERT((MSCM->IRSPRC[irqNumber] & (uint16_t)(MSCM_IRSPRC_RO_MASK)) == (uint16_t)MSCM_IRSPRC_RO(0));
    }

#endif /* FEATURE_MSCM_HAS_INTERRUPT_ROUTER */

    /* Set handler into vector table */
    __VECTOR_RAM[((int32_t)irqNumber) + 16] = (uint32_t)newHandler;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : INT_SYS_EnableIRQ
 * Description   : Enables an interrupt for a given IRQ number.
 * It calls the system NVIC API to access the interrupt control
 * register and MSCM (if available) API for interrupt routing.
 * The input IRQ number does not include the core interrupt, only
 * the peripheral interrupt, from 0 to a maximum supported IRQ.
 * Implements INT_SYS_EnableIRQ_Activity
 *END**************************************************************************/
void INT_SYS_EnableIRQ(IRQn_Type irqNumber)
{
    /* Check IRQ number */
    DEV_ASSERT(0 <= (int32_t)irqNumber);
    DEV_ASSERT(irqNumber <= FEATURE_INTERRUPT_IRQ_MAX);

    /* Enable interrupt */
    S32_NVIC->ISER[(uint32_t)(irqNumber) >> 5U] = (uint32_t)(1UL << ((uint32_t)(irqNumber) & (uint32_t)0x1FU));

#if FEATURE_MSCM_HAS_INTERRUPT_ROUTER

    /* Enable routing to current CPU */
    uint16_t cpu_enable = (uint16_t)(1UL << (MSCM->CPXNUM));
    MSCM->IRSPRC[irqNumber] |= cpu_enable;

#endif /* FEATURE_MSCM_HAS_INTERRUPT_ROUTER */
}

/*FUNCTION**********************************************************************
 *
 * Function Name : INT_SYS_DisableIRQ
 * Description   : Disable individual interrupt for a specified IRQ
 * It  calls the system NVIC API to access the interrupt control register
 * and MSCM (if available) API for interrupt routing.
 * Implements INT_SYS_DisableIRQ_Activity
 *
 *END**************************************************************************/
void INT_SYS_DisableIRQ(IRQn_Type irqNumber)
{
    /* Check IRQ number */
    DEV_ASSERT(0 <= (int32_t)irqNumber);
    DEV_ASSERT(irqNumber <= FEATURE_INTERRUPT_IRQ_MAX);

    /* Disable interrupt */
    S32_NVIC->ICER[((uint32_t)(irqNumber) >> 5U)] = (uint32_t)(1UL << ((uint32_t)(irqNumber) & (uint32_t)0x1FU));

#if FEATURE_MSCM_HAS_INTERRUPT_ROUTER

    /* Disable routing to current CPU */
    uint16_t cpu_enable = (uint16_t)(1UL << (MSCM->CPXNUM));
    MSCM->IRSPRC[irqNumber] &= (uint16_t)~(cpu_enable);

#endif /* FEATURE_MSCM_HAS_INTERRUPT_ROUTER */

}

/*FUNCTION**********************************************************************
 *
 * Function Name : INT_SYS_EnableIRQGlobal
 * Description   : Enable system interrupt
 * This function will enable the global interrupt by calling the core API
 * Implements INT_SYS_EnableIRQGlobal_Activity
 *
 *END**************************************************************************/
void INT_SYS_EnableIRQGlobal(void)
{
    /* Check and update */
    if (g_interruptDisableCount > 0)
    {
        g_interruptDisableCount--;

        if (g_interruptDisableCount <= 0)
        {
            /* Enable the global interrupt*/
            ENABLE_INTERRUPTS();
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : INT_SYS_DisableIRQGlobal
 * Description   : Disable system interrupt
 * This function will disable the global interrupt by calling the core API
 * Implements INT_SYS_DisableIRQGlobal_Activity
 *
 *END**************************************************************************/
void INT_SYS_DisableIRQGlobal(void)
{
    /* Disable the global interrupt */
    DISABLE_INTERRUPTS();

    /* Update counter*/
    g_interruptDisableCount++;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
