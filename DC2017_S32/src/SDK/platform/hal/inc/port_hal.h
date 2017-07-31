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
#ifndef PORT_HAL_H
#define PORT_HAL_H

#include "device_registers.h"
#include <stdbool.h>

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, The value of an expression
 * shall not be assigned to an object with a narrower essential type or of a
 * different essential type category.
 * The cast is required to perform a conversion between an unsigned integer
 * and an enum type with many values.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from
 * 'essentially unsigned' type to 'essentially enum<i>'.
 * This is required by the conversion of a bit-field of a register into a enum type.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from
 * 'essentially unsigned' type to 'essentially Boolean'.
 * This is required by the conversion of a bit into a bool type.
 *
 */

/*!
 * @defgroup port_hal Port Control and Interrupts (PORT)
 * @ingroup pins_driver
 * @brief This module covers the functionality of the PORT peripheral.
 * <p>
 *  PORT HAL provides the API for reading and writing register bit-fields belonging to the PORT module.
 * </p>
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if FEATURE_PORT_HAS_PULL_SELECTION
/*!
 * @brief Internal resistor pull feature selection
 * Implements : port_pull_config_t_Class
 */
typedef enum {
    PORT_INTERNAL_PULL_NOT_ENABLED   = 0U,   /*!< internal pull-down or pull-up resistor is not enabled. */
    PORT_INTERNAL_PULL_DOWN_ENABLED  = 1U,   /*!< internal pull-down resistor is enabled. @internal gui name="Down"*/
    PORT_INTERNAL_PULL_UP_ENABLED    = 2U    /*!< internal pull-up resistor is enabled. @internal gui name="Up"*/
} port_pull_config_t;
#endif

#if FEATURE_PORT_HAS_SLEW_RATE
/*!
 * @brief Slew rate selection
 * Implements : port_slew_rate_t_Class
 */
typedef enum {
    PORT_FAST_SLEW_RATE = 0U,  /*!< fast slew rate is configured. @internal gui name="Fast"*/
    PORT_SLOW_SLEW_RATE = 1U   /*!< slow slew rate is configured. @internal gui name="Slow" */
} port_slew_rate_t;
#endif

#if FEATURE_PORT_HAS_DRIVE_STRENGTH
/*!
 * @brief Configures the drive strength.
 * Implements : port_drive_strength_t_Class
 */
typedef enum {
    PORT_LOW_DRIVE_STRENGTH  = 0U, /*!< low drive strength is configured. @internal gui name="Low"*/
    PORT_HIGH_DRIVE_STRENGTH = 1U  /*!< high drive strength is configured. @internal gui name="High"*/
} port_drive_strength_t;
#endif

/*!
 * @brief Pin mux selection
 * Implements : port_mux_t_Class
 */
typedef enum {
    PORT_PIN_DISABLED = 0U,   /*!< corresponding pin is disabled, but is used as an analog pin.*/
    PORT_MUX_AS_GPIO  = 1U,   /*!< corresponding pin is configured as GPIO.*/
    PORT_MUX_ALT2     = 2U,   /*!< chip-specific*/
    PORT_MUX_ALT3     = 3U,   /*!< chip-specific*/
    PORT_MUX_ALT4     = 4U,   /*!< chip-specific*/
    PORT_MUX_ALT5     = 5U,   /*!< chip-specific*/
    PORT_MUX_ALT6     = 6U,   /*!< chip-specific*/
    PORT_MUX_ALT7     = 7U    /*!< chip-specific*/
} port_mux_t;

#if FEATURE_PORT_HAS_DIGITAL_FILTER
/*!
 * @brief Digital filter clock source selection
 * Implements : port_digital_filter_clock_source_t_Class
 */
typedef enum {
    PORT_BUS_CLOCK = 0U,  /*!< Digital filters are clocked by the bus clock.*/
    PORT_LPO_CLOCK = 1U   /*!< Digital filters are clocked by the 1 kHz LPO clock.*/
} port_digital_filter_clock_source_t;
#endif

/*!
 * @brief Configures the interrupt generation condition.
 * Implements : port_interrupt_config_t_Class
 */
typedef enum {
    PORT_DMA_INT_DISABLED     = 0x0U,  /*!< Interrupt/DMA request is disabled.*/
    #if FEATURE_PORT_HAS_DMA_REQUEST
    PORT_DMA_RISING_EDGE  = 0x1U,  /*!< DMA request on rising edge.*/
    PORT_DMA_FALLING_EDGE = 0x2U,  /*!< DMA request on falling edge.*/
    PORT_DMA_EITHER_EDGE  = 0x3U,  /*!< DMA request on either edge.*/
    #endif
    PORT_INT_LOGIC_ZERO   = 0x8U,  /*!< Interrupt when logic zero. */
    PORT_INT_RISING_EDGE  = 0x9U,  /*!< Interrupt on rising edge. */
    PORT_INT_FALLING_EDGE = 0xAU,  /*!< Interrupt on falling edge. */
    PORT_INT_EITHER_EDGE  = 0xBU,  /*!< Interrupt on either edge. */
    PORT_INT_LOGIC_ONE    = 0xCU   /*!< Interrupt when logic one. */
} port_interrupt_config_t;

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

#if FEATURE_PORT_HAS_PULL_SELECTION
/*!
 * @brief Configures the internal resistor.
 *
 * Pull configuration is valid in all digital pin muxing modes.
 *
 * @param[in] base        port base pointer.
 * @param[in] pin         port pin number
 * @param[in] pullConfig  internal resistor pull feature selection
 *        - PORT_PULL_NOT_ENABLED: internal pull-down or pull-up resistor is not enabled.
 *        - PORT_PULL_DOWN_ENABLED: internal pull-down resistor is enabled.
 *        - PORT_PULL_UP_ENABLED: internal pull-up resistor is enabled.
 * Implements : PORT_HAL_SetPullSel_Activity
 */
static inline void PORT_HAL_SetPullSel(PORT_Type* const base,
                                        const uint32_t pin,
                                        const port_pull_config_t pullConfig)
{
    DEV_ASSERT(pin < PORT_PCR_COUNT);
    switch(pullConfig)
    {
      case PORT_INTERNAL_PULL_NOT_ENABLED:
      {
        base->PCR[pin] &= ~(PORT_PCR_PE_MASK);
      }
      break;
      case PORT_INTERNAL_PULL_DOWN_ENABLED:
      {
        uint32_t regValue = base->PCR[pin];
        regValue &= ~(PORT_PCR_PS_MASK);
        regValue |= PORT_PCR_PE(1U);
        base->PCR[pin] = regValue;
      }
      break;
      case PORT_INTERNAL_PULL_UP_ENABLED:
      {
        uint32_t regValue = base->PCR[pin];
        regValue |= PORT_PCR_PE(1U);
        regValue |= PORT_PCR_PS(1U);
        base->PCR[pin] = regValue;
      }
      break;
      default:
      /* invalid command */
      DEV_ASSERT(false);
      break;
    }
}
#endif

#if FEATURE_PORT_HAS_SLEW_RATE
/*!
 * @brief Configures the fast/slow slew rate if the pin is used as a digital output.
 *
 * @param[in] base  port base pointer
 * @param[in] pin   port pin number
 * @param rateSelect  slew rate selection
 *        - PORT_FAST_SLEW_RATE: fast slew rate is configured.
 *        - PORT_SLOW_SLEW_RATE: slow slew rate is configured.
 */
static inline void PORT_HAL_SetSlewRateMode(PORT_Type* const base,
                                            const uint32_t pin,
                                            const port_slew_rate_t rateSelect)
{
    DEV_ASSERT(pin < PORT_PCR_COUNT);
    /* Rate select value */
    uint32_t rateSelectValue = (uint32_t)rateSelect;
    uint32_t regValue = base->PCR[pin];
    regValue &= ~(PORT_PCR_SRE_MASK);
    regValue |= PORT_PCR_SRE(rateSelectValue);
    base->PCR[pin] = regValue;
}
#endif

#if FEATURE_PORT_HAS_PASSIVE_FILTER
/*!
 * @brief Configures the passive filter if the pin is used as a digital input.
 *
 * If enabled, a low pass filter (10 MHz to 30 MHz bandwidth)  is enabled
 * on the digital input path. Disable the Passive Input Filter when supporting
 * high speed interfaces (> 2 MHz) on the pin.
 *
 * @param[in] base  port base pointer
 * @param[in] pin   port pin number
 * @param[in] isPassiveFilterEnabled  passive filter configuration
 *        - false: passive filter is disabled.
 *        - true : passive filter is enabled.
 * Implements : PORT_HAL_SetPassiveFilterMode_Activity
 */
static inline void PORT_HAL_SetPassiveFilterMode(PORT_Type* const base,
                                                 const uint32_t pin,
                                                 const bool isPassiveFilterEnabled)
{
    DEV_ASSERT(pin < PORT_PCR_COUNT);
    uint32_t regValue = base->PCR[pin];
    regValue &= ~(PORT_PCR_PFE_MASK);
    regValue |= PORT_PCR_PFE(isPassiveFilterEnabled);
    base->PCR[pin] = regValue;
}
#endif

#if FEATURE_PORT_HAS_OPEN_DRAIN
/*!
 * @brief Enables or disables the open drain.
 *
 * @param[in] base port base pointer
 * @param[in] pin  port pin number
 * @param[in] isOpenDrainEnabled  enable open drain or not
 *        - false: Open Drain output is disabled on the corresponding pin.
 *        - true : Open Drain output is disabled on the corresponding pin.
 */
static inline void PORT_HAL_SetOpenDrainMode(PORT_Type* const base,
                                             const uint32_t pin,
                                             const bool isOpenDrainEnabled)
{
    DEV_ASSERT(pin < PORT_PCR_COUNT);
    uint32_t regValue = base->PCR[pin];
    regValue &= ~(PORT_PCR_ODE_MASK);
    regValue |= PORT_PCR_ODE(isOpenDrainEnabled);
    base->PCR[pin] = regValue;
}
#endif

#if FEATURE_PORT_HAS_DRIVE_STRENGTH
/*!
 * @brief Configures the drive strength if the pin is used as a digital output.
 *
 * @param[in] base  port base pointer
 * @param[in] pin  port pin number
 * @param[in] driveSelect  drive strength selection
 *        - PORT_LOW_DRIVE_STRENGTH : low drive strength is configured.
 *        - PORT_HIGH_DRIVE_STRENGTH: high drive strength is configured.
 * Implements : PORT_HAL_SetDriveStrengthMode_Activity
 */
static inline void PORT_HAL_SetDriveStrengthMode(PORT_Type* const base,
                                                 const uint32_t pin,
                                                 const port_drive_strength_t driveSelect)
{
	DEV_ASSERT(pin < PORT_PCR_COUNT);
    /* Drive select value */
    uint32_t driveSelectValue = (uint32_t)driveSelect;
    uint32_t regValue = base->PCR[pin];
    regValue &= ~(PORT_PCR_DSE_MASK);
    regValue |= PORT_PCR_DSE(driveSelectValue);
    base->PCR[pin] = regValue;
}
#endif

/*!
 * @brief Configures the pin muxing.
 *
 * @param[in] base  port base pointer
 * @param[in] pin  port pin number
 * @param[in] mux  pin muxing slot selection
 *        - PORT_PIN_DISABLED: Pin disabled.
 *        - PORT_MUX_AS_GPIO  : Set as GPIO.
 *        - others          : chip-specific.
 * Implements : PORT_HAL_SetMuxModeSel_Activity
 */
static inline void PORT_HAL_SetMuxModeSel(PORT_Type* const base,
                                          const uint32_t pin,
                                          const port_mux_t mux)
{
    DEV_ASSERT(pin < PORT_PCR_COUNT);
    uint32_t regValue = base->PCR[pin];
    regValue &= ~(PORT_PCR_MUX_MASK);
    regValue |= PORT_PCR_MUX(mux);
    base->PCR[pin] = regValue;
}

#if FEATURE_PORT_HAS_PIN_CONTROL_LOCK
/*!
 * @brief Locks or unlocks the pin control register bits[15:0].
 *
 * @param[in] base  port base pointer
 * @param[in] pin  port pin number
 * @param[in] isPinLockEnabled  lock pin control register or not
 *        - false: pin control register bit[15:0] are not locked.
 *        - true : pin control register bit[15:0] are locked, cannot be updated till system reset.
 * Implements : PORT_HAL_SetPinCtrlLockMode_Activity
 */
static inline void PORT_HAL_SetPinCtrlLockMode(PORT_Type* const base,
                                               const uint32_t pin,
                                               const bool isPinLockEnabled)
{
    DEV_ASSERT(pin < PORT_PCR_COUNT);
    uint32_t regValue = base->PCR[pin];
    regValue &= ~(PORT_PCR_LK_MASK);
    regValue |= PORT_PCR_LK(isPinLockEnabled);
    base->PCR[pin] = regValue;
}
#endif

#if FEATURE_PORT_HAS_DIGITAL_FILTER
/*!
 * @brief Enables or disables the digital filter in one single port.
 *        Each bit of the 32-bit register represents one pin.
 *
 * @param[in] base  port base pointer
 * @param[in] pin   port pin number
 * @param[in] isDigitalFilterEnabled  digital filter enable/disable
 *        - false: digital filter is disabled on the corresponding pin.
 *        - true : digital filter is enabled on the corresponding pin.
 * Implements : PORT_HAL_SetDigitalFilterMode_Activity
 */
static inline void PORT_HAL_SetDigitalFilterMode(PORT_Type* const base,
                                                 const uint32_t pin,
                                                 const bool isDigitalFilterEnabled)
{
    DEV_ASSERT(pin < PORT_PCR_COUNT);
    /* Digital filter value */
    uint32_t digitalFilterValue = (isDigitalFilterEnabled == true) ? 1UL : 0UL;
    uint32_t regValue = base->DFER;
    regValue &= ~(1U << pin);
    regValue |=  (digitalFilterValue << pin);
    base->DFER = regValue;
}

/*!
 * @brief Configures the clock source for the digital input filters. Changing the filter clock source should
 *        only be done after disabling all enabled filters. Every pin in one port uses the same
 *        clock source.
 *
 * @param[in] base  port base pointer
 * @param[in] clockSource  chose which clock source to use for current port
 *        - PORT_BUS_CLOCK: digital filters are clocked by the bus clock.
 *        - PORT_LPO_CLOCK: digital filters are clocked by the 1 kHz LPO clock.
 * Implements : PORT_HAL_SetDigitalFilterClock_Activity
 */
static inline void PORT_HAL_SetDigitalFilterClock(PORT_Type* const base,
                                                  const port_digital_filter_clock_source_t clockSource)
{
    base->DFCR = (uint32_t)clockSource;
}

/*!
 * @brief Configures the maximum size of the glitches (in clock cycles) that the digital filter absorbs
 *        for enabled digital filters. Glitches that are longer than this register setting
 *        (in clock cycles)  pass through the digital filter, while glitches that are equal
 *        to or less than this register setting (in clock cycles)  are filtered. Changing the
 *        filter length should only be done after disabling all enabled filters.
 *
 * @param[in] base   port base pointer
 * @param[in] width  configure digital filter width (should be less than 5 bits).
 * Implements : PORT_HAL_SetDigitalFilterWidth_Activity
 */
static inline void PORT_HAL_SetDigitalFilterWidth(PORT_Type* const base, const uint8_t width)
{
    base->DFWR = width;
}
#endif /* FEATURE_PORT_HAS_DIGITAL_FILTER*/

/*!
 * @brief Configures the low half of the pin control register for the same settings.
 *        This function operates pin 0 -15 of one specific port.
 *
 * @param[in] base  port base pointer
 * @param[in] lowPinSelect  update corresponding pin control register or not. For a specific bit:
 *        - 0: corresponding low half of pin control register won't be updated according to configuration.
 *        - 1: corresponding low half of pin control register will be updated according to configuration.
 * @param[in] config  value  is written to a low half port control register bits[15:0].
 */
void PORT_HAL_SetLowGlobalPinCtrlCmd(PORT_Type* const base, const uint16_t lowPinSelect, const uint16_t config);

/*!
 * @brief Configures the high half of pin control register for the same settings.
 *        This function operates pin 16 -31 of one specific port.
 *
 * @param[in] base  port base pointer
 * @param[in] highPinSelect  update corresponding pin control register or not. For a specific bit:
 *        - 0: corresponding high half of pin control register won't be updated according to configuration.
 *        - 1: corresponding high half of pin control register will be updated according to configuration.
 * @param[in] config  value is  written to a high half port control register bits[15:0].
 */
void PORT_HAL_SetHighGlobalPinCtrlCmd(PORT_Type* const base, const uint16_t highPinSelect, const uint16_t config);

/*@}*/

/*!
 * @name Interrupt
 * @{
 */

/*!
 * @brief Configures the port pin interrupt/DMA request.
 *
 * @param[in] base       port base pointer.
 * @param[in] pin        port pin number
 * @param[in] intConfig  interrupt configuration
 *        - PORT_INT_DISABLED   : Interrupt/DMA request disabled.
 *        - PORT_DMA_RISING_EDGE : DMA request on rising edge.
 *        - PORT_DMA_FALLING_EDGE: DMA request on falling edge.
 *        - PORT_DMA_EITHER_EDGE : DMA request on either edge.
 *        - PORT_INT_LOGIC_ZERO  : Interrupt when logic zero.
 *        - PORT_INT_RISING_EDGE : Interrupt on rising edge.
 *        - PORT_INT_FALLING_EDGE: Interrupt on falling edge.
 *        - PORT_INT_EITHER_EDGE : Interrupt on either edge.
 *        - PORT_INT_LOGIC_ONE   : Interrupt when logic one.
 * Implements : PORT_HAL_SetPinIntSel_Activity
 */
static inline void PORT_HAL_SetPinIntSel(PORT_Type* const base,
                                         const uint32_t pin,
                                         const port_interrupt_config_t intConfig)
{
    DEV_ASSERT(pin < PORT_PCR_COUNT);
    uint32_t regValue = base->PCR[pin];
    regValue &= ~(PORT_PCR_IRQC_MASK);
    regValue |= PORT_PCR_IRQC(intConfig);
    base->PCR[pin] = regValue;
}

/*!
 * @brief Gets the current port pin interrupt/DMA request configuration.
 *
 * @param[in] base  port base pointer
 * @param[in] pin   port pin number
 * @return  interrupt configuration
 *        - PORT_INT_DISABLED   : Interrupt/DMA request disabled.
 *        - PORT_DMA_RISING_EDGE : DMA request on rising edge.
 *        - PORT_DMA_FALLING_EDGE: DMA request on falling edge.
 *        - PORT_DMA_EITHER_EDGE : DMA request on either edge.
 *        - PORT_INT_LOGIC_ZERO  : Interrupt when logic zero.
 *        - PORT_INT_RISING_EDGE : Interrupt on rising edge.
 *        - PORT_INT_FALLING_EDGE: Interrupt on falling edge.
 *        - PORT_INT_EITHER_EDGE : Interrupt on either edge.
 *        - PORT_INT_LOGIC_ONE   : Interrupt when logic one.
 * Implements : PORT_HAL_GetPinIntSel_Activity
 */
static inline port_interrupt_config_t PORT_HAL_GetPinIntSel(const PORT_Type* const base, const uint32_t pin)
{
    DEV_ASSERT(pin < PORT_PCR_COUNT);
    uint32_t regValue = base->PCR[pin];
    regValue = (regValue & PORT_PCR_IRQC_MASK) >> PORT_PCR_IRQC_SHIFT;
    return (port_interrupt_config_t)regValue;
}

/*!
 * @brief Reads the individual pin-interrupt status flag.
 *
 * If a pin is configured to generate the DMA request,  the corresponding flag
 * is cleared automatically at the completion of the requested DMA transfer.
 * Otherwise, the flag remains set until a logic one is written to that flag.
 * If configured for a level sensitive interrupt that remains asserted, the flag
 * is set again immediately.
 *
 * @param[in] base  port base pointer
 * @param[in] pin   port pin number
 * @return current pin interrupt status flag
 *         - 0: interrupt is not detected.
 *         - 1: interrupt is detected.
 * Implements : PORT_HAL_GetPinIntMode_Activity
 */
static inline bool PORT_HAL_GetPinIntMode(const PORT_Type* const base, const uint32_t pin)
{
	DEV_ASSERT(pin < PORT_PCR_COUNT);
    uint32_t regValue = base->PCR[pin];
    regValue = (regValue & PORT_PCR_ISF_MASK) >> PORT_PCR_ISF_SHIFT;
    return (bool)regValue;
}

/*!
 * @brief Clears the individual pin-interrupt status flag.
 *
 * @param[in] base  port base pointer
 * @param[in] pin   port pin number
 * Implements : PORT_HAL_ClearPinIntFlagCmd_Activity
 */
static inline void PORT_HAL_ClearPinIntFlagCmd(PORT_Type* const base, const uint32_t pin)
{
	DEV_ASSERT(pin < PORT_PCR_COUNT);
    uint32_t regValue = base->PCR[pin];
    regValue &= ~(PORT_PCR_ISF_MASK);
    regValue |= PORT_PCR_ISF(1U);
    base->PCR[pin] = regValue;
}

/*!
 * @brief Reads the entire port interrupt status flag.
 *
 * @param[in] base  port base pointer
 * @return all 32 pin interrupt status flags. For specific bit:
 *         - 0: interrupt is not detected.
 *         - 1: interrupt is detected.
 * Implements : PORT_HAL_GetPortIntFlag_Activity
 */
static inline uint32_t PORT_HAL_GetPortIntFlag(const PORT_Type* const base)
{
    uint32_t regValue = base->ISFR;
    return regValue;
}

/*!
 * @brief Clears the entire port interrupt status flag.
 *
 * @param[in] base  port base pointer
 * Implements : PORT_HAL_ClearPortIntFlagCmd_Activity
 */
static inline void PORT_HAL_ClearPortIntFlagCmd(PORT_Type* const base)
{
    base->ISFR = ~0U;
}

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* PORT_HAL_H*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

