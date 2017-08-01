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
#if !defined(INTERRUPT_MANAGER_H)
#define INTERRUPT_MANAGER_H

#include "device_registers.h"


/**
 * @page misra_violations MISRA-C:2012 violations
 *
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
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * The return statement before end of function is used for simpler code structure
 * and better readability.
 */

 /*! @file interrupt_manager.h */

/*! @addtogroup interrupt_manager*/
/*! @{*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if FEATURE_MSCM_HAS_CPU_INTERRUPT_ROUTER

/*! @brief The target for directed CPU interrupts */
typedef enum
{
    INTERRUPT_MANAGER_TARGET_SELF       = -2,
    INTERRUPT_MANAGER_TARGET_OTHERS     = -1,
    INTERRUPT_MANAGER_TARGET_NONE       =  0,
    INTERRUPT_MANAGER_TARGET_CP0        =  1,
    INTERRUPT_MANAGER_TARGET_CP1        =  2,
    INTERRUPT_MANAGER_TARGET_CP0_CP1    =  3
} interrupt_manager_cpu_targets_t;

#endif /* FEATURE_MSCM_HAS_CPU_INTERRUPT_ROUTER */

/*! @brief Interrupt handler type */
typedef void (* isr_t)(void);

/*******************************************************************************
 * Default interrupt handler - implemented in startup.s
 ******************************************************************************/
/*! @brief Default ISR. */
void DefaultISR(void);

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*! @name Interrupt manager APIs*/
/*@{*/

/*!
 * @brief Installs an interrupt handler routine for a given IRQ number.
 *
 * This function lets the application register/replace the interrupt
 * handler for a specified IRQ number. The IRQ number is different than the vector
 * number. IRQ 0  starts from the vector 16 address. See a chip-specific reference
 * manual for details and the  startup_<SoC>.s file for each chip
 * family to find out the default interrupt handler for each device. This
 * function converts the IRQ number to the vector number by adding 16 to
 * it.
 *
 * @note This method is applicable only if interrupt vector is copied in RAM,
 *       __flash_vector_table__ symbol is used to control this from linker options.
 *
 * @param irqNumber   IRQ number
 * @param newHandler  New interrupt handler routine address pointer
 * @param oldHandler  Pointer to a location to store current interrupt handler
 */
void INT_SYS_InstallHandler(IRQn_Type irqNumber,
                            const isr_t newHandler,
                            isr_t* const oldHandler);

/*!
 * @brief Enables an interrupt for a given IRQ number.
 *
 * This function  enables the individual interrupt for a specified IRQ
 * number. It calls the system NVIC API to access the interrupt control
 * register and MSCM (if available) API for interrupt routing.
 * The input IRQ number does not include the core interrupts, only
 * the peripheral interrupts and directed CPU interrupts (if available),
 * from 0 to a maximum supported IRQ.
 *
 * @param irqNumber IRQ number
 */
void INT_SYS_EnableIRQ(IRQn_Type irqNumber);

/*!
 * @brief Disables an interrupt for a given IRQ number.
 *
 * This function disables the individual interrupt for a specified IRQ
 * number. It calls the system NVIC API to access the interrupt control
 * register and MSCM (if available) API for interrupt routing.
 *
 * @param irqNumber IRQ number
 */
void INT_SYS_DisableIRQ(IRQn_Type irqNumber);

/*!
 * @brief Enables system interrupt.
 *
 * This function enables the global interrupt by calling the core API.
 *
 */
void INT_SYS_EnableIRQGlobal(void);

/*!
 * @brief Disable system interrupt.
 *
 * This function disables the global interrupt by calling the core API.
 *
 */
void INT_SYS_DisableIRQGlobal(void);

/*! @brief  Set Interrupt Priority
 *
 *   The function sets the priority of an interrupt.
 *
 *   Note: The priority cannot be set for every core interrupt.
 *   Implements INT_SYS_SetPriority_Activity
 *
 *   @param  irqNumber  Interrupt number.
 *   @param  priority  Priority to set.
 */
static inline void INT_SYS_SetPriority(IRQn_Type irqNumber, uint8_t priority)
{
    /* Check IRQ number and priority. */
    DEV_ASSERT(irqNumber <= FEATURE_INTERRUPT_IRQ_MAX);
    DEV_ASSERT(priority < (uint8_t)(1U << FEATURE_NVIC_PRIO_BITS));

    uint8_t shift = (uint8_t) (8U - FEATURE_NVIC_PRIO_BITS);

    if ((int32_t)irqNumber < 0)
    {
        uint32_t intVectorId = ((uint32_t)(irqNumber) & 0xFU);
        uint32_t regId = intVectorId / 4U;
        /* Compute pointer to SHPR register - avoid MISRA violation. */
        volatile uint8_t * shpr_reg_ptr = ((regId == 1U) ? (volatile uint8_t *)&S32_SCB->SHPR1 : ((regId == 2U) ? (volatile uint8_t *)&S32_SCB->SHPR2 : (volatile uint8_t *)&S32_SCB->SHPR3));
        /* Set Priority for Cortex-M  System Interrupts */
        shpr_reg_ptr[intVectorId % 4U] = (uint8_t)(((((uint32_t)priority) << shift)) & 0xffUL);
    }
    else
    {
        /* Set Priority for device specific Interrupts */
        S32_NVIC->IP[(uint32_t)(irqNumber)] =  (uint8_t)(((((uint32_t)priority) << shift)) & 0xFFUL);
    }
}

/*! @brief  Get Interrupt Priority
 *
 *   The function gets the priority of an interrupt.
 *
 *   Note: The priority cannot be obtained for every core interrupt.
 *   Implements INT_SYS_GetPriority_Activity
 *
 *   @param  irqNumber  Interrupt number.
 *   @return priority   Priority of the interrupt.
 */
static inline uint8_t INT_SYS_GetPriority(IRQn_Type irqNumber)
{
    /* Check IRQ number. */
    DEV_ASSERT(irqNumber <= FEATURE_INTERRUPT_IRQ_MAX);

    uint8_t priority = 0U;
    uint8_t shift = (uint8_t) (8U - FEATURE_NVIC_PRIO_BITS);

    if ((int32_t)irqNumber < 0)
    {
        uint32_t intVectorId = ((uint32_t)(irqNumber) & 0xFU);
        uint32_t regId = intVectorId / 4U;

        /* Compute pointer to SHPR register - avoid MISRA violation. */
        volatile const uint8_t * shpr_reg_ptr = ((regId == 1U) ? (volatile uint8_t *)&S32_SCB->SHPR1 : ((regId == 2U) ? (volatile uint8_t *)&S32_SCB->SHPR2 : (volatile uint8_t *)&S32_SCB->SHPR3));
        /* Get Priority from Cortex-M  System Interrupts */
        priority = (uint8_t)(shpr_reg_ptr[intVectorId % 4U] >> (shift));
    }
    else
    {
        /* Get Priority for device specific Interrupts  */
        priority = (uint8_t)(S32_NVIC->IP[(uint32_t)(irqNumber)] >> shift);
    }

    return priority;
}

/*!
 * @brief Clear Pending Interrupt
 *
 * The function clears the pending bit of a peripheral interrupt
 * or a directed interrupt to this CPU (if available).
 * Implements INT_SYS_ClearPending_Activity
 *
 * @param irqNumber IRQ number
 */
static inline void INT_SYS_ClearPending(IRQn_Type irqNumber)
{
    /* Check IRQ number */
    DEV_ASSERT(0 <= (int32_t)irqNumber);
    DEV_ASSERT(irqNumber <= FEATURE_INTERRUPT_IRQ_MAX);

#if FEATURE_MSCM_HAS_CPU_INTERRUPT_ROUTER

    if ((FEATURE_DIRECTED_CPU_INT_MIN <= irqNumber) && (irqNumber <= FEATURE_DIRECTED_CPU_INT_MAX))
    {
        /* Clear Directed CPU Pending Interrupt */
        switch (MSCM->CPXNUM)
        {
            case 0:
                MSCM->IRCP0IR |= (1UL << ((uint32_t)irqNumber - (uint32_t)FEATURE_DIRECTED_CPU_INT_MIN));
                break;
            default:
                MSCM->IRCP1IR |= (1UL << ((uint32_t)irqNumber - (uint32_t)FEATURE_DIRECTED_CPU_INT_MIN));
                break;
        }
        return;
    }
#endif /* FEATURE_MSCM_HAS_CPU_INTERRUPT_ROUTER */

    /* Clear Pending Interrupt */
    S32_NVIC->ICPR[(uint32_t)(irqNumber) >> 5U] = (uint32_t)(1UL << ((uint32_t)(irqNumber) & (uint32_t)0x1FU));
}

/*!
 * @brief Set Pending Interrupt
 *
 * The function configures the pending bit of a peripheral interrupt.
 * Implements INT_SYS_SetPending_Activity
 *
 * @param irqNumber IRQ number
 */
static inline void INT_SYS_SetPending(IRQn_Type irqNumber)
{
    /* Check IRQ number */
    DEV_ASSERT(0 <= (int32_t)irqNumber);
    DEV_ASSERT(irqNumber <= FEATURE_INTERRUPT_IRQ_MAX);

    /* Set Pending Interrupt */
    S32_NVIC->ISPR[(uint32_t)(irqNumber) >> 5U] = (uint32_t)(1UL << ((uint32_t)(irqNumber) & (uint32_t)0x1FU));
}

/*!
 * @brief Get Pending Interrupt
 *
 * The function gets the pending bit of a peripheral interrupt
 * or a directed interrupt to this CPU (if available).
 * Implements INT_SYS_GetPending_Activity
 *
 * @param irqNumber IRQ number
 * @return pending  Pending status 0/1
 */
static inline uint32_t INT_SYS_GetPending(IRQn_Type irqNumber)
{
    /* Check IRQ number */
    DEV_ASSERT(0 <= (int32_t)irqNumber);
    DEV_ASSERT(irqNumber <= FEATURE_INTERRUPT_IRQ_MAX);

#if FEATURE_MSCM_HAS_CPU_INTERRUPT_ROUTER
    /* Get Directed CPU Pending Interrupt */
    if ((FEATURE_DIRECTED_CPU_INT_MIN <= irqNumber) && (irqNumber <= FEATURE_DIRECTED_CPU_INT_MAX))
    {
        return (((((MSCM->CPXNUM != 0UL) ? MSCM->IRCP1IR : MSCM->IRCP0IR) &
                  (1UL << ((uint32_t)irqNumber - (uint32_t)FEATURE_DIRECTED_CPU_INT_MIN))) != 0UL) ? 1UL : 0UL);
    }
#endif /* FEATURE_MSCM_HAS_CPU_INTERRUPT_ROUTER */

    /* Get Pending Interrupt */
    return ((uint32_t)(((S32_NVIC->ISPR[(((uint32_t)irqNumber) >> 5UL)] & (1UL << (((uint32_t)irqNumber) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}

/*!
 * @brief Get Active Interrupt
 *
 * The function gets the active state of a peripheral interrupt.
 * Implements INT_SYS_GetActive_Activity
 *
 * @param irqNumber IRQ number
 * @return active   Active status 0/1
 */
static inline uint32_t INT_SYS_GetActive(IRQn_Type irqNumber)
{
    /* Check IRQ number */
    DEV_ASSERT(0 <= (int32_t)irqNumber);
    DEV_ASSERT(irqNumber <= FEATURE_INTERRUPT_IRQ_MAX);

    /* Get Active Interrupt */
    return ((uint32_t)(((S32_NVIC->IABR[(((uint32_t)irqNumber) >> 5UL)] & (1UL << (((uint32_t)irqNumber) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}

#if FEATURE_MSCM_HAS_CPU_INTERRUPT_ROUTER

/*!
 * @brief Generate Directed CPU Interrupt
 *
 * The function generates a directed interrupt to (one or more) CPUs defined by target.
 * Implements INT_SYS_GenerateDirectedCpuInterrupt_Activity
 *
 * @param irqNumber  IRQ number
 * @param cpu_target Target CPUs for the directed interrupt
 */
static inline void INT_SYS_GenerateDirectedCpuInterrupt(IRQn_Type irqNumber, interrupt_manager_cpu_targets_t cpu_target)
{
    /* Check IRQ number */
    DEV_ASSERT(FEATURE_DIRECTED_CPU_INT_MIN <= irqNumber);
    DEV_ASSERT(irqNumber <= FEATURE_DIRECTED_CPU_INT_MAX);

    uint32_t reg_val = MSCM_IRCPGIR_INTID((uint32_t)irqNumber - (uint32_t)FEATURE_DIRECTED_CPU_INT_MIN);

    switch (cpu_target)
    {
        case INTERRUPT_MANAGER_TARGET_SELF:
            reg_val |= MSCM_IRCPGIR_TLF(2);
            break;
        case INTERRUPT_MANAGER_TARGET_OTHERS:
            reg_val |= MSCM_IRCPGIR_TLF(1);
            break;
        case INTERRUPT_MANAGER_TARGET_NONE:
        case INTERRUPT_MANAGER_TARGET_CP0:
        case INTERRUPT_MANAGER_TARGET_CP1:
        case INTERRUPT_MANAGER_TARGET_CP0_CP1:
            reg_val |= (MSCM_IRCPGIR_TLF(0) | MSCM_IRCPGIR_CPUTL(cpu_target));
            break;
        default:
            /* Not treated case ? */
            break;
    }

    /* Generate Directed CPU Interrupt */
    MSCM->IRCPGIR = reg_val;
}

#endif /* FEATURE_MSCM_HAS_CPU_INTERRUPT_ROUTER */

/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* INTERRUPT_MANAGER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
