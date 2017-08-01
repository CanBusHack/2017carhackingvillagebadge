/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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

#if !defined(SIM_HAL_H)
#define SIM_HAL_H


/*! @file sim_hal.h */
/*! @brief Common SIM_HAL API */

/*!
 * @addtogroup sim_hal System Integration Module (SIM) HAL
 * @ingroup clock_manager
 * @brief System Integration Module Hardware Abstraction Layer
 * @{
 */

/*******************************************************************************
* Definitions
******************************************************************************/

/*******************************************************************************
* API
******************************************************************************/

/*
* Include the CPU-specific clock API header files.
*/
#if (defined(S32K144_SERIES))
    /* SIM API header file */
    #include "../src/sim/S32K144/sim_hal_S32K144.h"

#else
#error "No valid CPU defined!"
#endif
/* @} */
#endif /* SIM_HAL_H */
/*******************************************************************************
* EOF
******************************************************************************/

