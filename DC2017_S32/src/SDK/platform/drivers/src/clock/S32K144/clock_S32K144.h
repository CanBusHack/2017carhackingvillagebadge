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

#if !defined(CLOCK_S32K144_H)
#define CLOCK_S32K144_H

#include "scg_hal.h"
#include "pcc_hal.h"
#include "sim_hal.h"
#include "smc_hal.h"  /* Required for SCG alternative clock usage */
#include "pmc_hal.h"  /* Required for LPO settings */

/*! @file clock_manager_S32K144.h */

/*!
 * @ingroup clock_manager
 * @defgroup clock_manager_s32k144
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @brief Configures SCG module.
 *
 * This function configures the SCG module according to the
 * configuration.
 *
 * @param[in] scgConfig Pointer to the configuration structure.
 * @return Status of module initialization
 */
status_t CLOCK_SYS_SetScgConfiguration(const scg_config_t *scgConfig);

/*!
 * @brief Configures PCC module.
 *
 * This function configures the PCC module according to the
 * configuration.
 *
 * @param[in] peripheralClockConfig   Pointer to the configuration structure.
 */
void CLOCK_SYS_SetPccConfiguration(const pcc_config_t *peripheralClockConfig);

/*!
 * @brief Configures SIM module.
 *
 * This function configures the SIM module according to the
 * configuration.
 *
 * @param[in] simClockConfig   Pointer to the configuration structure.
 */
void CLOCK_SYS_SetSimConfiguration(const sim_clock_config_t *simClockConfig);

/*!
 * @brief Configures PMC module.
 *
 * This function configures the PMC module according to the
 * configuration.
 *
 * @param[in] pmcConfig   Pointer to the configuration structure.
 */
void CLOCK_SYS_SetPmcConfiguration(const pmc_config_t *pmcConfig);

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* CLOCK_S32K144_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/

