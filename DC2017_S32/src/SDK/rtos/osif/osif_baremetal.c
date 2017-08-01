/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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
 * @file osif_baremetal.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable. A source of error in
 * writing dynamic code is that the stack segment may be different from the data
 * segment.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Directive 4.9, Function-like macro defined.
 * The macros are used to validate input parameters to driver functions.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * This is required for initializing pointers to the module's memory map, which is located at a
 * fixed address.
 *
 */

#include "osif.h"
#include "devassert.h"
#include <stddef.h>

#include "interrupt_manager.h"
#include "clock_manager.h"

#if defined (USING_OS_FREERTOS)
#error "Wrong OSIF selected. Please define symbol USING_OS_BAREMETAL (or no OS define) in project settings or change the OSIF variant"
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/*! @cond DRIVER_INTERNAL_USE_ONLY */

/*! @brief Converts milliseconds to ticks - in this case, one tick = one millisecond */
#define MSEC_TO_TICK(msec) (msec)

static volatile uint32_t s_osif_tick_cnt = 0u;

void SysTick_Handler(void);

void SysTick_Handler(void)
{
    s_osif_tick_cnt++;
}

static inline uint32_t osif_GetCurrentTickCount(void)
{
    return s_osif_tick_cnt;
}

static inline void osif_UpdateSystickConfig(void)
{
    uint32_t core_freq = 0u;
    status_t clk_status = CLOCK_SYS_GetFreq(CORE_CLOCK, &core_freq);
    DEV_ASSERT(clk_status == STATUS_SUCCESS);
    DEV_ASSERT(core_freq > 0u);
    (void)clk_status;

    S32_SysTick->RVR = S32_SysTick_RVR_RELOAD(core_freq / 1000u);
    S32_SysTick->CSR = S32_SysTick_CSR_ENABLE(1u) | S32_SysTick_CSR_TICKINT(1u);
}

/*! @endcond */

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : OSIF_TimeDelay
 * Description   : This function blocks execution for a number of milliseconds.
 *
 * Implements : OSIF_TimeDelay_baremetal_Activity
 *END**************************************************************************/
void OSIF_TimeDelay(const uint32_t delay)
{
    osif_UpdateSystickConfig();
    uint32_t start = osif_GetCurrentTickCount();
    uint32_t crt_ticks = osif_GetCurrentTickCount();
    uint32_t delta = crt_ticks - start;
    uint32_t delay_ticks = MSEC_TO_TICK(delay);
    while (delta < delay_ticks)
    {
        crt_ticks = osif_GetCurrentTickCount();
        delta = crt_ticks - start;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSIF_GetMilliseconds
 * Description   : This function returns the number of miliseconds elapsed since
 *                  starting the internal timer. To initialize the internal timer
 *                  (Systick) in bare-metal, call either OSIF_TimeDelay or
 *                  OSIF_SemaWait functions. Calling OSIF_TimeDelay(0) will initialize
 *                  the timer without any side-effects (no delay).
 *
 * Implements : OSIF_GetMilliseconds_baremetal_Activity
 *END**************************************************************************/
uint32_t OSIF_GetMilliseconds(void)
{
    /*
     * Please make sure the timer is initialized before calling this function.
     * For example, calling OSIF_TimeDelay(0) ensures that the timer is initialized
     * without any other side-effects. If OSIF_TimeDelay or OSIF_SemaWait functions
     * have been called, the timer is already initialized.
     */
    return osif_GetCurrentTickCount(); /* This assumes that 1 tick = 1 millisecond */
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSIF_MutexLock
 * Description   : This function locks a mutex (mock operation in baremetal case).
 *
 * Implements : OSIF_MutexLock_baremetal_Activity
 *END**************************************************************************/
status_t OSIF_MutexLock(const mutex_t * const pMutex,
                        const uint32_t timeout)
{
    (void)pMutex;
    (void)timeout;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSIF_MutexUnlock
 * Description   : This function unlocks a mutex (mock operation in baremetal case).
 *
 * Implements : OSIF_MutexUnlock_baremetal_Activity
 *END**************************************************************************/
status_t OSIF_MutexUnlock(const mutex_t * const pMutex)
{
    (void)pMutex;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSIF_MutexCreate
 * Description   : This function creates a mutex (mock operation in baremetal case).
 *
 * Implements : OSIF_MutexCreate_baremetal_Activity
 *END**************************************************************************/
status_t OSIF_MutexCreate(mutex_t * const pMutex)
{
    (void)pMutex;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSIF_MutexDestroy
 * Description   : This function destroys a mutex (mock operation in baremetal case).
 *
 * Implements : OSIF_MutexDestroy_baremetal_Activity
 *END**************************************************************************/
status_t OSIF_MutexDestroy(const mutex_t * const pMutex)
{
    (void)pMutex;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSIF_SemaWait
 * Description   : This function performs the 'wait' (decrement) operation on a semaphore.
 *
 * Implements : OSIF_SemaWait_baremetal_Activity
 *END**************************************************************************/
status_t OSIF_SemaWait(semaphore_t * const pSem,
                       const uint32_t timeout)
{
    DEV_ASSERT(pSem != NULL);

    uint32_t timeoutTicks;
    status_t osif_ret_code = STATUS_SUCCESS;

    osif_UpdateSystickConfig();
    /* Convert timeout from milliseconds to ticks. */
    if (timeout == OSIF_WAIT_FOREVER)
    {
        timeoutTicks = OSIF_WAIT_FOREVER;
    }
    else
    {
        timeoutTicks = MSEC_TO_TICK(timeout);
    }

    uint32_t start = osif_GetCurrentTickCount();
    uint32_t end = (uint32_t)(start + timeoutTicks);
    uint32_t max = end - start;
    while (*pSem == 0u)
    {
        uint32_t crt_ticks = osif_GetCurrentTickCount();
        uint32_t delta = crt_ticks - start;
        if ((timeoutTicks != OSIF_WAIT_FOREVER) && (delta > max))
        {
            /* Timeout occured, stop waiting and return fail code */
            osif_ret_code = STATUS_TIMEOUT;
            break;
        }
    }

    if (osif_ret_code == STATUS_SUCCESS)
    {
        INT_SYS_DisableIRQGlobal();
        --(*pSem);
        INT_SYS_EnableIRQGlobal();
    }

    return osif_ret_code;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSIF_SemaPost
 * Description   : This function performs the 'post' (increment) operation on a semaphore.
 *
 * Implements : OSIF_SemaPost_baremetal_Activity
 *END**************************************************************************/
status_t OSIF_SemaPost(semaphore_t * const pSem)
{
    DEV_ASSERT(pSem != NULL);

    status_t osif_ret_code = STATUS_SUCCESS;
    INT_SYS_DisableIRQGlobal();
    if (*pSem != 255u)
    {
        ++(*pSem);
    }
    else
    {
        osif_ret_code = STATUS_ERROR;
    }

    INT_SYS_EnableIRQGlobal();

    return osif_ret_code;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSIF_SemaCreate
 * Description   : This function creates (initializes) a semaphore.
 *
 * Implements : OSIF_SemaCreate_baremetal_Activity
 *END**************************************************************************/
status_t OSIF_SemaCreate(semaphore_t * const pSem,
                         const uint8_t initValue)
{
    DEV_ASSERT(pSem != NULL);

    INT_SYS_DisableIRQGlobal();
    *pSem = initValue;
    INT_SYS_EnableIRQGlobal();

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSIF_SemaDestroy
 * Description   : This function destroys a semaphore object (mock operation in baremetal case).
 *
 * Implements : OSIF_SemaDestroy_baremetal_Activity
 *END**************************************************************************/
status_t OSIF_SemaDestroy(const semaphore_t * const pSem)
{
    DEV_ASSERT(pSem != NULL);

    (void)pSem;

    return STATUS_SUCCESS;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
