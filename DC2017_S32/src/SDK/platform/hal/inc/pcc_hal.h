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
#ifndef PCC_HAL_H
#define PCC_HAL_H

#include "device_registers.h"
#include <stdbool.h>

/*! @file pcc_hal.h */

/*!
 * @defgroup pcc_hal Peripheral Clock Control (PCC)
 * @ingroup clock_manager
 * @brief This module covers Peripheral Clock Control  module functionality.
 * <p>
 *  PCC HAL provides the API for reading and writing register bit-fields belonging to the PCC module.
 *  Clock selection for most modules is controlled by PCC.
 *  The clock to each module can be individually gated on and off using PCC.
 * </p>
 * <p>
 *  The PCC hal component allows application to configure these peripheral clocking options:
 *    - Clock gating
 *    - Clock source selection
 *    - Fractional clock divider values
 * </p>
 * <p>
 * This is an example to write a peripheral clock configuration:
 * @code

    Count of peripheral clock user configurations
    #define NUM_OF_CONFIGURED_PERIPHERAL_CLOCKS_0 5U

    Peripheral clocking configuration 0
    peripheral_clock_config_t peripheralClockConfig0[NUM_OF_CONFIGURED_PERIPHERAL_CLOCKS_0] = {
        {
            .peripheral       = PCC_FLEXTMR3_CLOCK,
            .clkGate          = true,
            .clkSrc           = CLK_SRC_FIRC,
            .frac             = 0,                                       N/A
            .divider          = 0,                                       N/A
        },
        {
            .peripheral       = PCC_LPTMR0_CLOCK,
            .clkGate          = true,
            .clkSrc           = CLK_SRC_SOSC,
            .frac             = MULTIPLY_BY_TWO,
            .divider          = DIVIDE_BY_FOUR,
        },
        {
            .peripheral       = PCC_SIM0_CLOCK,
            .clkGate          = true,
            .clkSrc           = 0,                                       N/A
            .frac             = 0,                                       N/A
            .divider          = 0,                                       N/A
        },
        {
            .peripheral       = PCC_PORTA_CLOCK,
            .clkGate          = true,
            .clkSrc           = 0,                                       N/A
            .frac             = 0,                                       N/A
            .divider          = 0,                                       N/A
        },
        {
            .peripheral       = PCC_SCG0_CLOCK,
            .clkGate          = true,
            .clkSrc           = 0,                                       N/A
            .frac             = 0,                                       N/A
            .divider          = 0,                                       N/A
        },
    };

    pcc_config_t pccConfig =
    {
        .peripheralClocks = peripheralClockConfig0,                  Peripheral clock control configurations
        .count = NUM_OF_CONFIGURED_PERIPHERAL_CLOCKS_0,              Number of the peripheral clock control configurations
    };

    write peripheral clocking configuration
    PCC_HAL_SetPeripheralClockConfig(PCC, &pccConfig);
 * @endcode
 * </p>
 * <p>
 *  For higher-level functionality, use the Clock Manager driver.
 * </p>
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Clock name mappings
 *         Constant array storing the mappings between clock names and peripheral clock control indexes.
 *         If there is no peripheral clock control index for a clock name, then the corresponding value is
 *         PCC_INVALID_INDEX.
 */
extern const uint16_t clockNameMappings[CLOCK_NAME_COUNT];


/*! @brief PCC clock source select
 *  Implements peripheral_clock_source_t_Class
 */
typedef enum
{
    CLK_SRC_OFF   = 0x00U,             /* Clock is off */
    CLK_SRC_SOSC  = 0x01U,             /* OSCCLK - System Oscillator Bus Clock */
    CLK_SRC_SIRC  = 0x02U,             /* SCGIRCLK - Slow IRC Clock */
    CLK_SRC_FIRC  = 0x03U,             /* SCGFIRCLK - Fast IRC Clock */
    CLK_SRC_SPLL  = 0x06U              /* SCGPCLK System PLL clock */

} peripheral_clock_source_t;

/*! @brief PCC fractional value select
 *  Implements peripheral_clock_frac_t_Class
 */
typedef enum
{
    MULTIPLY_BY_ONE   = 0x00U,             /* Fractional value is zero */
    MULTIPLY_BY_TWO   = 0x01U              /* Fractional value is one */

} peripheral_clock_frac_t;

/*! @brief PCC divider value select
 *  Implements peripheral_clock_divider_t_Class
 */
typedef enum
{
    DIVIDE_BY_ONE     = 0x00U,             /* Divide by 1 (pass-through, no clock divide) */
    DIVIDE_BY_TWO     = 0x01U,             /* Divide by 2 */
    DIVIDE_BY_THREE   = 0x02U,             /* Divide by 3 */
    DIVIDE_BY_FOUR    = 0x03U,             /* Divide by 4 */
    DIVIDE_BY_FIVE    = 0x04U,             /* Divide by 5 */
    DIVIDE_BY_SIX     = 0x05U,             /* Divide by 6 */
    DIVIDE_BY_SEVEN   = 0x06U,             /* Divide by 7 */
    DIVIDE_BY_EIGTH   = 0x07U              /* Divide by 8 */

}peripheral_clock_divider_t;

/*! @brief PCC peripheral instance clock configuration.
 *  Implements peripheral_clock_config_t_Class
 */
typedef struct
{
    /* clockName   is the name of the peripheral clock
     *    must be one of the following values (see the clock_names_t type from S32K144_clock_names.h)
     *    PCC_DMA0_CLOCK
     *    PCC_MPU0_CLOCK
     *    ...
     *    PCC_LPUART3_CLOCK
     */
    clock_names_t clockName;
    bool clkGate;                                      /*!< Peripheral clock gate.                     */
    peripheral_clock_source_t clkSrc;                  /*!< Peripheral clock source.                   */
    peripheral_clock_frac_t frac;                      /*!< Peripheral clock fractional value.         */
    peripheral_clock_divider_t divider;                /*!< Peripheral clock divider value.            */
}peripheral_clock_config_t;

/*! @brief PCC configuration.
 *  Implements pcc_config_t_Class
 */
typedef struct
{
    uint32_t count;                                    /*!< Number of peripherals to be configured.               */
    peripheral_clock_config_t *peripheralClocks;       /*!< Pointer to the peripheral clock configurations array. */

} pcc_config_t;


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Configuration
 * @{
 */

/*!
 * @name System Clock.
 * @{
 */


/*!
 * @brief Set the peripheral clock configuration.
 *
 * This function sets the peripheral configuration.
 *
 * @param[in] base   Register base address for the PCC instance.
 * @param[in] config Pointer to the configuration.
 */
void PCC_HAL_SetPeripheralClockConfig(PCC_Type* const base,
                                                                                    const pcc_config_t* const config);
/*!
 * @brief Enables/disables the clock for a given peripheral.
 * For example, to enable the ADC0 clock, use like this:
 * @code
 *  PCC_HAL_SetClockMode(PCC, PCC_ADC0_CLOCK, true);
 * @endcode
 *
 * @param[in] base        pcc base pointer
 * @param[in] clockName   is the name of the peripheral clock
 * must be one of the following values (see the clock_names_t type from S32K144_clock_names.h)
 *    PCC_DMA0_CLOCK
 *    PCC_MPU0_CLOCK
 *    ...
 *    PCC_LPUART3_CLOCK
 * @param[in] isClockEnabled  is the value of the command that enables/disables the clock
 * Implements PCC_HAL_SetClockMode_Activity
 */
static inline void PCC_HAL_SetClockMode(PCC_Type* const base,
                                        const clock_names_t clockName,
                                        const bool isClockEnabled)
{
    uint32_t regValue = (uint32_t)base->PCCn[clockNameMappings[clockName]];
    regValue &= (uint32_t)(~(PCC_PCCn_CGC_MASK));
    regValue |= PCC_PCCn_CGC((isClockEnabled == true) ? 1UL : 0UL);
    base->PCCn[clockNameMappings[clockName]] = (uint32_t)regValue;
}

/*!
 * @brief Selects the clock source for a given peripheral
 * For example, to select the FIRC source for ADC clock, use like this:
 * @code
 *  PCC_HAL_SetClockSourceSel(PCC, PCC_ADC0_CLOCK, CLK_SRC_FIRC);
 * @endcode
 *
 * @param[in] base        pcc base pointer
 * @param[in] clockName   is the name of the peripheral clock
 * must be one of the following values (see the clock_names_t type from S32K144_clock_names.h)
 *    PCC_DMA0_CLOCK
 *    PCC_MPU0_CLOCK
 *    ...
 *    PCC_LPUART3_CLOCK
 * @param[in] clockSource is the clock source to use for a given peripheral
 * Implements PCC_HAL_SetClockSourceSel_Activity
 */
static inline void PCC_HAL_SetClockSourceSel(PCC_Type* const base,
                                             const clock_names_t clockName,
                                             const peripheral_clock_source_t clockSource)
{
    uint32_t clkSource, regValue;

    switch(clockSource)
    {
        case CLK_SRC_SPLL:
            clkSource = 6U;
            break;
        case CLK_SRC_FIRC:
            clkSource = 3U;
            break;
        case CLK_SRC_SIRC:
            clkSource = 2U;
            break;
        case CLK_SRC_SOSC:
            clkSource = 1U;
            break;
        case CLK_SRC_OFF:
        /* Pass-thourgh */
        default:
            clkSource = 0U;
            break;
    }

    regValue = base->PCCn[clockNameMappings[clockName]];
    regValue &= ~(PCC_PCCn_PCS_MASK);
    regValue |= PCC_PCCn_PCS(clkSource);
    base->PCCn[clockNameMappings[clockName]] = regValue;
}

/*!
 * @brief Selects the fractional value for a given peripheral
 * For example, to configure MULTIPLY_BY_ONE as fractional value for WDOG, use like this:
 * @code
 *  PCC_HAL_SetFracValueSel(PCC, PCC_WDOG_CLOCK, MULTIPLY_BY_ONE);
 * @endcode
 *
 * @param[in] base        pcc base pointer
 * @param[in] clockName   is the name of the peripheral clock
 * must be one of the following values (see the clock_names_t type from S32K144_clock_names.h)
 *    PCC_DMA0_CLOCK
 *    PCC_MPU0_CLOCK
 *    ...
 *    PCC_LPUART3_CLOCK
 * @param[in] fracValue is the fractional value to use for a given peripheral
 * Implements PCC_HAL_SetFracValueSel_Activity
 */
static inline void PCC_HAL_SetFracValueSel(PCC_Type* const base,
                                           const clock_names_t clockName,
                                           const peripheral_clock_frac_t fracValue)
{
    uint32_t fractionalValue, regValue;

    switch (fracValue)
    {
        case MULTIPLY_BY_TWO:
            fractionalValue = 1U;
            break;
        case MULTIPLY_BY_ONE:
        /* Pass-through */
        default:
            fractionalValue = 0U;
            break;
    }

    regValue = (uint32_t)base->PCCn[clockNameMappings[clockName]];
    regValue &= (uint32_t)(~(PCC_PCCn_FRAC_MASK));
    regValue |= PCC_PCCn_FRAC(fractionalValue);
    base->PCCn[clockNameMappings[clockName]] = regValue;
}

/*!
 * @brief Selects the divider value for a given peripheral
 * For example, to configure DIVIDE_BY_ONE as divider value for WDOG, use like this:
 * @code
 *  PCC_HAL_SetDividerValueSel(PCC, PCC_WDOG_CLOCK, DIVIDE_BY_ONE);
 * @endcode
 *
 * @param[in] base        pcc base pointer
 * @param[in] clockName   is the name of the peripheral clock
 * must be one of the following values (see the clock_names_t type from S32K144_clock_names.h)
 *    PCC_DMA0_CLOCK
 *    PCC_MPU0_CLOCK
 *    ...
 *    PCC_LPUART3_CLOCK
 * @param[in] divValue is the divider value to use for a given peripheral
 * Implements PCC_HAL_SetDividerValueSel_Activity
 */
static inline void PCC_HAL_SetDividerValueSel(PCC_Type* const base,
                                              const clock_names_t clockName,
                                              const peripheral_clock_divider_t divValue)
{
    uint32_t dividerValue, regValue;

    switch(divValue)
    {
        case DIVIDE_BY_EIGTH:
            dividerValue = 7U;
            break;
        case DIVIDE_BY_SEVEN:
            dividerValue = 6U;
            break;
        case DIVIDE_BY_SIX:
            dividerValue = 5U;
            break;
        case DIVIDE_BY_FIVE:
            dividerValue = 4U;
            break;
        case DIVIDE_BY_FOUR :
            dividerValue = 3U;
            break;
        case DIVIDE_BY_THREE:
            dividerValue = 2U;
            break;
        case DIVIDE_BY_TWO:
            dividerValue = 1U;
            break;
        case DIVIDE_BY_ONE:
        /* Pass-thourgh */
        default:
            dividerValue = 0U;
            break;
    }

    regValue = base->PCCn[clockNameMappings[clockName]];
    regValue &= ~(PCC_PCCn_PCD_MASK);
    regValue |= PCC_PCCn_PCD(dividerValue);
    base->PCCn[clockNameMappings[clockName]] = regValue;
}

/*!
 * @brief Gets the clock gate control mode.
 *
 * @param[in] base        pcc base pointer
 * @param[in] clockName   is the name of the peripheral clock
 * must be one of the following values (see the clock_names_t type from S32K144_clock_names.h)
 *    PCC_DMA0_CLOCK
 *    PCC_MPU0_CLOCK
 *    ...
 *    PCC_LPUART3_CLOCK
 * @return  the clock gate control mode
 *        - false : Clock is disabled
 *        - true : Clock is enabled
 * Implements PCC_HAL_GetClockMode_Activity
 */
static inline bool PCC_HAL_GetClockMode(const PCC_Type* const base,
                                        const clock_names_t clockName)
{
    uint32_t regValue = (uint32_t)base->PCCn[clockNameMappings[clockName]];
    regValue = (regValue & PCC_PCCn_CGC_MASK) >> PCC_PCCn_CGC_SHIFT;
    return (regValue == 0U) ? false : true;
}

/*!
 * @brief Gets the selection of a clock source for a specific peripheral
 *
 * @param[in] base        pcc base pointer
 * @param[in] clockName   is the name of the peripheral clock
 * must be one of the following values (see the clock_names_t type from S32K144_clock_names.h)
 *    PCC_DMA0_CLOCK
 *    PCC_MPU0_CLOCK
 *    ...
 *    PCC_LPUART3_CLOCK
 * @return  the clock gate control mode
 *        - PCC_CLK_SRC_OFF : Clock is disabled
 *        - PCC_CLK_SRC_SOSC : OSCCLK - System Oscillator Bus Clock
 *        - PCC_CLK_SRC_SIRC : SCGIRCLK - Slow IRC Clock
 *        - PCC_CLK_SRC_FIRC : SCGFIRCLK - Fast IRC Clock
 *        - PCC_CLK_SRC_SPLL : SCGPCLK - System PLL clock
 * Implements PCC_HAL_GetClockSourceSel_Activity
 */
static inline peripheral_clock_source_t PCC_HAL_GetClockSourceSel(const PCC_Type* const base,
                                                                  const clock_names_t clockName)
{
    peripheral_clock_source_t retValue;
    uint32_t regValue = base->PCCn[clockNameMappings[clockName]];
    regValue = (regValue & PCC_PCCn_PCS_MASK) >> PCC_PCCn_PCS_SHIFT;

    switch(regValue)
    {
        case 6U:
            retValue = CLK_SRC_SPLL;
            break;
        case 3U:
            retValue = CLK_SRC_FIRC;
            break;
        case 2U:
            retValue = CLK_SRC_SIRC;
            break;
        case 1U:
            retValue = CLK_SRC_SOSC;
            break;
        case 0U:
        /* Pass-thourgh */
        default:
            retValue = CLK_SRC_OFF;
            break;
    }
    return retValue;
}

/*!
 * @brief Gets the selection of the fractional value for a specific peripheral
 *
 * @param[in] base        pcc base pointer
 * @param[in] clockName   is the name of the peripheral clock
 * must be one of the following values (see the clock_names_t type from S32K144_clock_names.h)
 *    PCC_DMA0_CLOCK
 *    PCC_MPU0_CLOCK
 *    ...
 *    PCC_LPUART3_CLOCK
 * @return  the fractional value
 *        - PCC_MULTPCCnLY_BY_ONE : Fractional value is zero
 *        - PCC_MULTPCCnLY_BY_TWO : Fractional value is one
 * Implements PCC_HAL_GetFracValueSel_Activity
 */
static inline peripheral_clock_frac_t PCC_HAL_GetFracValueSel(const PCC_Type* const base,
                                                              const clock_names_t clockName)
{
    peripheral_clock_frac_t retValue;
    uint32_t regValue = (uint32_t)base->PCCn[clockNameMappings[clockName]];
    regValue = (regValue & PCC_PCCn_FRAC_MASK) >> PCC_PCCn_FRAC_SHIFT;

    switch(regValue)
    {
        case 1U:
            retValue = MULTIPLY_BY_TWO;
            break;
        case 0U:
        /* Pass-thourgh */
        default:
            retValue = MULTIPLY_BY_ONE;
            break;
    }
    return retValue;
}

/*!
 * @brief Gets the selection of the divider value for a specific peripheral
 *
 * @param[in] base        pcc base pointer
 * @param[in] clockName   is the name of the peripheral clock
 * must be one of the following values (see the clock_names_t type from S32K144_clock_names.h)
 *    PCC_DMA0_CLOCK
 *    PCC_MPU0_CLOCK
 *    ...
 *    PCC_LPUART3_CLOCK
 * @return  the divider value
 *        - PCC_DIVIDE_BY_ONE   : Divide by 1
 *        - PCC_DIVIDE_BY_TWO   : Divide by 2
 *        - PCC_DIVIDE_BY_THREE : Divide by 3
 *        - PCC_DIVIDE_BY_FOUR  : Divide by 4
 *        - PCC_DIVIDE_BY_FIVE  : Divide by 5
 *        - PCC_DIVIDE_BY_SIX   : Divide by 6
 *        - PCC_DIVIDE_BY_SEVEN : Divide by 7
 *        - PCC_DIVIDE_BY_EIGTH : Divide by 8
 * Implements PCC_HAL_GetDividerSel_Activity
 */
static inline peripheral_clock_divider_t PCC_HAL_GetDividerSel(const PCC_Type* const base,
                                                               const clock_names_t clockName)
{
    peripheral_clock_divider_t retValue;
    uint32_t regValue = base->PCCn[clockNameMappings[clockName]];
    regValue = (regValue & PCC_PCCn_PCD_MASK) >> PCC_PCCn_PCD_SHIFT;

    switch(regValue)
    {
        case 7U:
            retValue = DIVIDE_BY_EIGTH;
            break;
        case 6U:
            retValue = DIVIDE_BY_SEVEN;
            break;
        case 5U:
            retValue = DIVIDE_BY_SIX;
            break;
        case 4U:
            retValue = DIVIDE_BY_FIVE;
            break;
        case 3U:
            retValue = DIVIDE_BY_FOUR;
            break;
        case 2U:
            retValue = DIVIDE_BY_THREE;
            break;
        case 1U:
            retValue = DIVIDE_BY_TWO;
            break;
        case 0U:
        /* Pass-thourgh */
        default:
            retValue = DIVIDE_BY_ONE;
            break;
    }
    return retValue;
}

/*!
 * @brief Tells whether a given peripheral is present or not
 *
 * @param[in] base        pcc base pointer
 * @param[in] clockName   is the name of the peripheral clock
 * must be one of the following values (see the clock_names_t type from S32K144_clock_names.h)
 *    PCC_DMA0_CLOCK
 *    PCC_MPU0_CLOCK
 *    ...
 *    PCC_LPUART3_CLOCK
 * @return  is the value that tells whether the peripheral is present or not
 *        - false : Peripheral is not present
 *        - true : Peripheral is present
 * Implements PCC_HAL_GetPeripheralMode_Activity
 */
static inline bool PCC_HAL_GetPeripheralMode(const PCC_Type* const base,
                                             const clock_names_t clockName)
{
    uint32_t regValue = (uint32_t)base->PCCn[clockNameMappings[clockName]];
    regValue = (regValue & PCC_PCCn_PR_MASK) >> PCC_PCCn_PR_SHIFT;
    return (regValue == 0U) ? false : true;
}

#if FEATURE_PCC_HAS_IN_USE_FEATURE

/*!
 * @brief Gets the value of the InUse bit for a given peripheral
 *
 * @param[in] base        pcc base pointer
 * @param[in] clockName   is the name of the peripheral clock
 * must be one of the following values (see the clock_names_t type from S32K144_clock_names.h)
 *    PCC_DMA0_CLOCK
 *    PCC_MPU0_CLOCK
 *    ...
 *    PCC_LPUART3_CLOCK
 * @return  the value of the InUse bit
 *        - false : Peripheral is not in use
 *        - true : Peripheral is in use
 */
static inline bool PCC_HAL_GetPeripheralInUseMode(const PCC_Type* const base,
                                                  const clock_names_t clockName)
{
    uint32_t regValue = (uint32_t)base->PCCn[clockNameMappings[clockName]];
    regValue = (regValue & PCC_PCCn_INUSE_MASK) >> PCC_PCCn_INUSE_SHIFT;
    return (regValue == 0U) ? false : true;
}
#endif /* FEATURE_PCC_HAS_IN_USE_FEATURE */


/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* PCC_HAL_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/

