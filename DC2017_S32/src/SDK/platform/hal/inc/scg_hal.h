/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
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

#ifndef SCG_HAL_H
#define SCG_HAL_H

#include "device_registers.h"
#include "status.h"
#include <stdbool.h>
#include <stddef.h>

/*!
 * @file scg_hal.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.3, Global typedef not referenced.
 * sim_clock_config_t is referenced from clock manager.
 */

#if FEATURE_SOC_SCG_COUNT

/*!
 * @addtogroup scg_hal System Clock Generator (SCG) HAL
 * @ingroup clock_manager
 * @brief System Clock Generator Hardware Abstraction Layer
 * @{
 */

/* @brief EXTAL0 clock frequency. */
extern uint32_t g_xtal0ClkFreq;

/* @brief RTC_CLKIN clock frequency. */
extern uint32_t g_RtcClkInFreq;


/*!
 * @name System Clock.
 * @{
 */

/*!
 * @brief SCG system clock type.
 * Implements scg_system_clock_type_t_Class
 */
typedef enum
{
    SCG_SYSTEM_CLOCK_CORE,  /*!< Core clock.        */
    SCG_SYSTEM_CLOCK_BUS,   /*!< BUS clock.         */
    SCG_SYSTEM_CLOCK_SLOW,  /*!< System slow clock. */
    SCG_SYSTEM_CLOCK_MAX,   /*!< Max value.         */
} scg_system_clock_type_t;

/*!
 * @brief SCG system clock source.
 * Implements scg_system_clock_src_t_Class
 */
typedef enum
{
    SCG_SYSTEM_CLOCK_SRC_SYS_OSC = 1U,  /*!< System OSC. */
    SCG_SYSTEM_CLOCK_SRC_SIRC    = 2U,  /*!< Slow IRC.   */
    SCG_SYSTEM_CLOCK_SRC_FIRC    = 3U,  /*!< Fast IRC.   */
    SCG_SYSTEM_CLOCK_SRC_SYS_PLL = 6U,  /*!< System PLL. */
    SCG_SYSTEM_CLOCK_SRC_NONE           /*!< MAX value.  */
} scg_system_clock_src_t;

/*!
 * @brief SCG system clock modes.
 * Implements scg_system_clock_mode_t_Class
 */
typedef enum
{
    SCG_SYSTEM_CLOCK_MODE_CURRENT = 0U,  /*!< Current mode.            */
    SCG_SYSTEM_CLOCK_MODE_RUN     = 1U,  /*!< Run mode.                */
    SCG_SYSTEM_CLOCK_MODE_VLPR    = 2U,  /*!< Very Low Power Run mode. */
    SCG_SYSTEM_CLOCK_MODE_HSRUN   = 3U,  /*!< High Speed Run mode.     */
    SCG_SYSTEM_CLOCK_MODE_NONE           /*!< MAX value.               */
} scg_system_clock_mode_t;

/*!
 * @brief SCG system clock divider value.
 * Implements scg_system_clock_div_t_Class
 */
typedef enum
{
    SCG_SYSTEM_CLOCK_DIV_BY_1   = 0U,   /*!< Divided by 1. */
    SCG_SYSTEM_CLOCK_DIV_BY_2   = 1U,   /*!< Divided by 2. */
    SCG_SYSTEM_CLOCK_DIV_BY_3   = 2U,   /*!< Divided by 3. */
    SCG_SYSTEM_CLOCK_DIV_BY_4   = 3U,   /*!< Divided by 4. */
    SCG_SYSTEM_CLOCK_DIV_BY_5   = 4U,   /*!< Divided by 5. */
    SCG_SYSTEM_CLOCK_DIV_BY_6   = 5U,   /*!< Divided by 6. */
    SCG_SYSTEM_CLOCK_DIV_BY_7   = 6U,   /*!< Divided by 7. */
    SCG_SYSTEM_CLOCK_DIV_BY_8   = 7U,   /*!< Divided by 8. */
    SCG_SYSTEM_CLOCK_DIV_BY_9   = 8U,   /*!< Divided by 9. */
    SCG_SYSTEM_CLOCK_DIV_BY_10  = 9U,   /*!< Divided by 10. */
    SCG_SYSTEM_CLOCK_DIV_BY_11  = 10U,  /*!< Divided by 11. */
    SCG_SYSTEM_CLOCK_DIV_BY_12  = 11U,  /*!< Divided by 12. */
    SCG_SYSTEM_CLOCK_DIV_BY_13  = 12U,  /*!< Divided by 13. */
    SCG_SYSTEM_CLOCK_DIV_BY_14  = 13U,  /*!< Divided by 14. */
    SCG_SYSTEM_CLOCK_DIV_BY_15  = 14U,  /*!< Divided by 15. */
    SCG_SYSTEM_CLOCK_DIV_BY_16  = 15U,  /*!< Divided by 16. */
} scg_system_clock_div_t;

/*!
 * @brief SCG system clock configuration.
 * Implements scg_system_clock_config_t_Class
 */
typedef struct
{
    scg_system_clock_div_t divSlow;  /*!< Slow clock divider.      */
    scg_system_clock_div_t divBus;   /*!< BUS clock divider.       */
    scg_system_clock_div_t divCore;  /*!< Core clock divider.      */
    scg_system_clock_src_t src;      /*!< System clock source.     */
} scg_system_clock_config_t;

/* @} */
/*!
 * @name SCG Clockout.
 * @{
 */

/*!
 * @brief SCG ClockOut type.
 * Implements scg_clockout_src_t_Class
 */
typedef enum
{
    SCG_CLOCKOUT_SRC_SCG_SLOW = 0U,   /*!< SCG SLOW.   */
    SCG_CLOCKOUT_SRC_SOSC     = 1U,   /*!< System OSC. */
    SCG_CLOCKOUT_SRC_SIRC     = 2U,   /*!< Slow IRC.   */
    SCG_CLOCKOUT_SRC_FIRC     = 3U,   /*!< Fast IRC.   */
    SCG_CLOCKOUT_SRC_SPLL     = 6U,   /*!< System PLL. */
} scg_clockout_src_t;
/* @} */

/*!
 * @brief SCG asynchronous clock type.
 * Implements scg_async_clock_type_t_Class
 */
typedef enum
{
    SCG_ASYNC_CLOCK_DIV1,   /*!< Clock divider 1  */
    SCG_ASYNC_CLOCK_DIV2,   /*!< Clock divider 2  */
    SCG_ASYNC_CLOCK_MAX,    /*!< Max value.       */
} scg_async_clock_type_t;

/*!
 * @brief SCG asynchronous clock divider value.
 * Implements scg_async_clock_div_t_Class
 */
typedef enum
{
    SCG_ASYNC_CLOCK_DISABLE     = 0U,  /*!< Clock output is disabled.  */
    SCG_ASYNC_CLOCK_DIV_BY_1    = 1U,  /*!< Divided by 1.              */
    SCG_ASYNC_CLOCK_DIV_BY_2    = 2U,  /*!< Divided by 2.              */
    SCG_ASYNC_CLOCK_DIV_BY_4    = 3U,  /*!< Divided by 4.              */
    SCG_ASYNC_CLOCK_DIV_BY_8    = 4U,  /*!< Divided by 8.              */
    SCG_ASYNC_CLOCK_DIV_BY_16   = 5U,  /*!< Divided by 16.             */
    SCG_ASYNC_CLOCK_DIV_BY_32   = 6U,  /*!< Divided by 32.             */
    SCG_ASYNC_CLOCK_DIV_BY_64   = 7U,  /*!< Divided by 64.             */
} scg_async_clock_div_t;

/*!
 * @brief SCG system OSC monitor mode.
 * Implements scg_sosc_monitor_mode_t_Class
 */
typedef enum
{
    SCG_SOSC_MONITOR_DISABLE = 0U,                         /*!< Monitor disable.                          */
    SCG_SOSC_MONITOR_INT     = SCG_SOSCCSR_SOSCCM_MASK,    /*!< Interrupt when system OSC error detected. */
    SCG_SOSC_MONITOR_RESET   = SCG_SOSCCSR_SOSCCM_MASK |
                               SCG_SOSCCSR_SOSCCMRE_MASK,  /*!< Reset when system OSC error detected.     */
} scg_sosc_monitor_mode_t;

/*!
 * @brief SCG OSC frequency range select
 * Implements scg_sosc_range_t_Class
 */
typedef enum
{
    SCG_SOSC_RANGE_LOW    = 1U,  /*!< Low frequency range selected for the crystal OSC (32 kHz to 40 kHz).   */
    SCG_SOSC_RANGE_MID    = 2U,  /*!< Medium frequency range selected for the crystal OSC (1 Mhz to 8 Mhz).  */
    SCG_SOSC_RANGE_HIGH   = 3U,  /*!< High frequency range selected for the crystal OSC (8 Mhz to 32 Mhz).   */
} scg_sosc_range_t;

/*!
 * @brief SCG OSC high gain oscillator select.
 * Implements scg_sosc_gain_t_Class
 */
typedef enum
{
    SCG_SOSC_GAIN_LOW,  /* Configure crystal oscillator for low-power operation */
    SCG_SOSC_GAIN_HIGH  /* Configure crystal oscillator for high-gain operation */
} scg_sosc_gain_t;

/*!
 * @brief SCG OSC external reference clock select.
 * Implements scg_sosc_ext_ref_t_Class
 */
typedef enum
{
    SCG_SOSC_REF_EXT,    /* External reference clock requested    */
    SCG_SOSC_REF_OSC     /* Internal oscillator of OSC requested. */
} scg_sosc_ext_ref_t;

/*!
 * @brief SCG system OSC configuration.
 * Implements scg_sosc_config_t_Class
 */
typedef struct
{
    uint32_t  freq;                       /*!< System OSC frequency.                                 */

    scg_sosc_monitor_mode_t monitorMode;   /*!< System OSC Clock monitor mode.                       */

    scg_sosc_ext_ref_t extRef;             /*!< System OSC External Reference Select.                */
    scg_sosc_gain_t    gain;               /*!< System OSC high-gain operation.                      */

    scg_sosc_range_t   range;              /*!< System OSC frequency range.                          */

    scg_async_clock_div_t div1;            /*!< Divider for platform asynchronous clock.             */
    scg_async_clock_div_t div2;            /*!< Divider for bus asynchronous clock.                  */

    bool enableInStop;                     /*!< System OSC is enable or not in stop mode.            */
    bool enableInLowPower;                 /*!< System OSC is enable or not in low power mode.       */

    bool locked;                           /*!< System OSC Control Register can be written.          */

    bool initialize;                       /*!< Initialize or not the System OSC module.             */
} scg_sosc_config_t;

/*!
 * @brief SCG slow IRC clock frequency range.
 * Implements scg_sirc_range_t_Class
 */
typedef enum
{
    SCG_SIRC_RANGE_LOW,   /*!< Slow IRC low range clock (2 MHz).  */
    SCG_SIRC_RANGE_HIGH,  /*!< Slow IRC high range clock (8 MHz). */
} scg_sirc_range_t;

/*!
 * @brief SCG slow IRC clock configuration.
 * Implements scg_sirc_config_t_Class
 */
typedef struct
{
    scg_sirc_range_t range;         /*!< Slow IRC frequency range.                 */

    scg_async_clock_div_t div1;     /*!< Divider for platform asynchronous clock.  */
    scg_async_clock_div_t div2;     /*!< Divider for bus asynchronous clock.       */

    bool initialize;                /*!< Initialize or not the SIRC module.        */
    bool enableInStop;              /*!< SIRC is enable or not in stop mode.       */
    bool enableInLowPower;          /*!< SIRC is enable or not in low power mode.  */

    bool locked;                    /*!< SIRC Control Register can be written.     */
} scg_sirc_config_t;

/*!
 * @brief SCG fast IRC clock frequency range.
 * Implements scg_firc_range_t_Class
 */
typedef enum
{
    SCG_FIRC_RANGE_48M,   /*!< Fast IRC is trimmed to 48MHz.  */
    SCG_FIRC_RANGE_52M,   /*!< Fast IRC is trimmed to 52MHz.  */
    SCG_FIRC_RANGE_56M,   /*!< Fast IRC is trimmed to 56MHz.  */
    SCG_FIRC_RANGE_60M,   /*!< Fast IRC is trimmed to 60MHz.  */
} scg_firc_range_t;

/*!
 * @brief SCG fast IRC clock configuration.
 * Implements scg_firc_config_t_Class
 */
typedef struct
{
    scg_firc_range_t range;            /*!< Fast IRC frequency range.                 */

    scg_async_clock_div_t div1;        /*!< Divider for platform asynchronous clock.  */
    scg_async_clock_div_t div2;        /*!< Divider for bus asynchronous clock.       */

    bool enableInStop;                 /*!< FIRC is enable or not in stop mode.       */
    bool enableInLowPower;             /*!< FIRC is enable or not in lowpower mode.   */
    bool regulator;                    /*!< FIRC regulator is enable or not.          */
    bool locked;                       /*!< FIRC Control Register can be written.     */

    bool initialize;                   /*!< Initialize or not the FIRC module.        */
} scg_firc_config_t;

/*!
 * @brief SCG system PLL monitor mode.
 * Implements scg_spll_monitor_mode_t_Class
 */
typedef enum
{
    SCG_SPLL_MONITOR_DISABLE = 0U,                         /*!< Monitor disable.                          */
    SCG_SPLL_MONITOR_INT     = SCG_SPLLCSR_SPLLCM_MASK,    /*!< Interrupt when system PLL error detected. */
    SCG_SPLL_MONITOR_RESET   = SCG_SPLLCSR_SPLLCM_MASK |
                               SCG_SPLLCSR_SPLLCMRE_MASK,  /*!< Reset when system PLL error detected.     */
} scg_spll_monitor_mode_t;


/*!
 * @brief SCG system PLL configuration.
 * Implements scg_spll_config_t_Class
 */
typedef struct
{
    scg_spll_monitor_mode_t monitorMode; /*!< Clock monitor mode selected.                    */

    uint8_t        prediv;               /*!< PLL reference clock divider.                    */
    uint8_t        mult;                 /*!< System PLL multiplier.                          */
    uint8_t        src;                  /*!< System PLL source.                              */

    scg_async_clock_div_t div1;          /*!< Divider for platform asynchronous clock.        */
    scg_async_clock_div_t div2;          /*!< Divider for bus asynchronous clock.             */

    bool enableInStop;                   /*!< System PLL clock is enable or not in stop mode. */

    bool locked;                         /*!< System PLL Control Register can be written.     */
    bool initialize;                     /*!< Initialize or not the System PLL module.        */
} scg_spll_config_t;

/*!
 * @brief SCG RTC configuration.
 * Implements scg_rtc_config_t_Class
 */
typedef struct
{
    uint32_t  rtcClkInFreq;              /*!< RTC_CLKIN frequency.                            */
    bool      initialize;                /*!< Initialize or not the RTC.                      */
} scg_rtc_config_t;

/*!
 * @brief SCG Clock Mode Configuration structure.
 * Implements scg_clock_mode_config_t_Class
 */
typedef struct
{
    scg_system_clock_config_t rccrConfig;      /*!< Run Clock Control configuration.                 */
    scg_system_clock_config_t vccrConfig;      /*!< VLPR Clock Control configuration.                */
    scg_system_clock_config_t hccrConfig;      /*!< HSRUN Clock Control configuration.               */
    scg_system_clock_src_t    alternateClock;  /*!< Alternate clock used during initialization       */
    bool                      initialize;      /*!< Initialize or not the Clock Mode Configuration.  */
} scg_clock_mode_config_t;

/*!
 * @brief SCG ClockOut Configuration structure.
 * Implements scg_clockout_config_t_Class
 */
typedef struct
{
    scg_clockout_src_t        source;          /*!< ClockOut source select.                          */
    bool                      initialize;      /*!< Initialize or not the ClockOut.                  */
} scg_clockout_config_t;

/*!
 * @brief SCG configure structure.
 * Implements scg_config_t_Class
 */
typedef struct
{
    scg_sirc_config_t         sircConfig;      /*!< Slow internal reference clock configuration.     */
    scg_firc_config_t         fircConfig;      /*!< Fast internal reference clock configuration.     */
    scg_sosc_config_t         soscConfig;      /*!< System oscillator configuration.                 */
    scg_spll_config_t         spllConfig;      /*!< System Phase locked loop configuration.          */
    scg_rtc_config_t          rtcConfig;       /*!< Real Time Clock configuration.                   */
    scg_clockout_config_t     clockOutConfig;  /*!< SCG ClockOut Configuration.                      */
    scg_clock_mode_config_t   clockModeConfig; /*!< SCG Clock Mode Configuration.                    */
} scg_config_t;

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @name System Clock.
 * @{
 */

/*!
 * @brief Set the system clock configuration in specified mode.
 *
 * This function sets the system configuration in specified mode.
 *
 * @param[in] base Register base address for the SCG instance.
 * @param[in] mode specifies the mode.
 * @param[in] config Pointer to the configuration.
 * @return Status of the system clock source setting
 * @retval STATUS_SUCCESS the new system clock source has been set
 * @retval STATUS_ERROR configuration for the new system clock source is not valid
 */
status_t SCG_HAL_SetSystemClockConfig(SCG_Type * base,
                                      scg_system_clock_mode_t mode,
                                      scg_system_clock_config_t const *config);

/*!
 * @brief Get the system clock configuration for specified mode.
 *
 * This function gets the system configuration for specified mode.
 *
 * @param[in] base Register base address for the SCG instance.
 * @param[in] mode specifies the mode.
 * @param[out] config Pointer to the configuration.
 */
void SCG_HAL_GetSystemClockConfig(const SCG_Type * base,
                                  scg_system_clock_mode_t mode,
                                  scg_system_clock_config_t *config);

/* @} */

/*!
 * @name SCG Clockout Configuration
 * @{
 */

/*!
 * @brief Get SCG ClockOut source select
 *
 * This function gets the SCG clockOut source
 *
 * @param[in] base Register base address for the SCG instance.
 * @return ClockOut source.
 * Implements SCG_HAL_GetClockoutSourceSel_Activity
 */
static inline scg_clockout_src_t SCG_HAL_GetClockoutSourceSel(const SCG_Type * base)
{
    scg_clockout_src_t retValue;
    uint32_t regValue = base->CLKOUTCNFG;
    regValue = (regValue & SCG_CLKOUTCNFG_CLKOUTSEL_MASK) >> SCG_CLKOUTCNFG_CLKOUTSEL_SHIFT;

    switch(regValue)
    {
        case 6U:
            retValue = SCG_CLOCKOUT_SRC_SPLL;
            break;
        case 3U:
            retValue = SCG_CLOCKOUT_SRC_FIRC;
            break;
        case 2U:
            retValue = SCG_CLOCKOUT_SRC_SIRC;
            break;
        case 1U:
            retValue = SCG_CLOCKOUT_SRC_SOSC;
            break;
        case 0U:
        /* pass-through */
        default:
            retValue = SCG_CLOCKOUT_SRC_SCG_SLOW;
            break;
    }
    return retValue;
}

/*!
 * @brief Set SCG ClockOut source select
 *
 * This function sets the SCG ClockOut source
 *
 * @param[in] base Register base address for the SCG instance.
 * @param[in] source used for ClockOut
 * Implements SCG_HAL_SetClockoutSourceSel_Activity
 */
static inline void SCG_HAL_SetClockoutSourceSel(SCG_Type * base, scg_clockout_src_t source)
{
    uint32_t src, regValue;

    switch(source)
    {
        case SCG_CLOCKOUT_SRC_SPLL:
            src = 6UL;
            break;
        case SCG_CLOCKOUT_SRC_FIRC:
            src = 3UL;
            break;
        case SCG_CLOCKOUT_SRC_SIRC:
            src = 2UL;
            break;
        case SCG_CLOCKOUT_SRC_SOSC:
            src = 1UL;
            break;
        case SCG_CLOCKOUT_SRC_SCG_SLOW:
        /* Pass-thourgh */
        default:
            src = 0UL;
            break;
    }

    regValue = base->CLKOUTCNFG;
    regValue &= ~(SCG_CLKOUTCNFG_CLKOUTSEL_MASK);
    regValue |= SCG_CLKOUTCNFG_CLKOUTSEL(src);
    base->CLKOUTCNFG = regValue;
}

/* @} */

/*!
 * @name System OSC Clock.
 * @{
 */

/*!
 * @brief Get the default system OSC configuration.
 *
 * This function gets the default SCG system OSC configuration.
 *
 * @param[out] config Pointer to the configuration structure.
 */
void SCG_HAL_GetSysOscDefaultConfig(scg_sosc_config_t *config);

/*!
 * @brief Initialize SCG system OSC.
 *
 * This function enables the SCG system OSC clock according to the
 * configuration.
 *
 * @param[in] base Register base address for the SCG instance.
 * @param[in] config   Pointer to the configuration structure.
 * @return Status of module initialization
 * @retval STATUS_SUCCESS System OSC is initialized.
 * @retval STATUS_BUSY System OSC has been enabled and used by system clock.
 *
 * @note This function can not detect whether system OSC has been enabled and
 * used by some IPs.
 */
status_t SCG_HAL_InitSysOsc(SCG_Type * base,
                                scg_sosc_config_t const *config);

/*!
 * @brief De-initialize SCG system OSC.
 *
 * This function disables the SCG system OSC clock.
 *
 * @param[in] base Register base address for the SCG instance.
 * @return Status of module de-initialization
 * @retval STATUS_SUCCESS System OSC is deinitialized.
 * @retval STATUS_BUSY System OSC is used by system clock.
 *
 * @note This function can not detect whether system OSC is used by some IPs.
 */
status_t SCG_HAL_DeinitSysOsc(SCG_Type * base);

/*!
 * @brief Get SCG system OSC clock frequency (SYSOSC).
 *
 * @param[in] base Register base address for the SCG instance.
 * @return Clock frequency, if clock is invalid, return 0.
 */
uint32_t SCG_HAL_GetSysOscFreq(const SCG_Type * base);

/*!
 * @brief Get SCG asynchronous clock frequency from system OSC.
 *
 * @param[in] base Register base address for the SCG instance.
 * @param[in] type The asynchronous clock type.
 * @return Clock frequency, if clock is invalid, return 0.
 */
uint32_t SCG_HAL_GetSysOscAsyncFreq(const SCG_Type * base,
                                    scg_async_clock_type_t type);

/* @} */

/*!
 * @name Slow IRC Clock.
 * @{
 */

/*!
 * @brief Get the default slow IRC clock configuration.
 *
 * This function gets the default slow IRC clock configuration (SIRC).
 *
 * @param[out] config Pointer to the configuration structure.
 */
void SCG_HAL_GetSircDefaultConfig(scg_sirc_config_t *config);

/*!
 * @brief Initialize SCG slow IRC clock.
 *
 * This function enables the SCG slow IRC clock according to the
 * configuration.
 *
 * @param[in] base Register base address for the SCG instance.
 * @param[in] config   Pointer to the configuration structure.
 * @return Status of module initialization
 * @retval STATUS_SUCCESS SIRC is initialized.
 * @retval STATUS_BUSY SIRC has been enabled and used by system clock.
 *
 * @note This function can not detect whether SIRC is used by some IPs.
 */
status_t SCG_HAL_InitSirc(SCG_Type * base,
                              const scg_sirc_config_t *config);

/*!
 * @brief De-initialize SCG slow IRC.
 *
 * This function disables the SCG slow IRC.
 *
 * @param[in] base Register base address for the SCG instance.
 * @return Status of module de-initialization
 * @retval STATUS_SUCCESS SIRC is deinitialized.
 * @retval STATUS_BUSY SIRC is used by system clock.
 *
 * @note This function can not detect whether SIRC is used by some IPs.
 */
status_t SCG_HAL_DeinitSirc(SCG_Type * base);

/*!
 * @brief Get SCG SIRC clock frequency.
 *
 * @param[in] base Register base address for the SCG instance.
 * @return Clock frequency, if clock is invalid, return 0.
 */
uint32_t SCG_HAL_GetSircFreq(const SCG_Type * base);

/*!
 * @brief Get SCG asynchronous clock frequency from SIRC.
 *
 * @param[in] base Register base address for the SCG instance.
 * @param[in] type The asynchronous clock type.
 * @return Clock frequency, if clock is invalid, return 0.
 */
uint32_t SCG_HAL_GetSircAsyncFreq(const SCG_Type * base,
                                  scg_async_clock_type_t type);

/* @} */

/*!
 * @name Fast IRC Clock.
 * @{
 */

/*!
 * @brief Get the default fast IRC clock configuration.
 *
 * This function gets the default fast IRC clock configuration (FIRC).
 * The default trim coarse value and trim fine value are all 0. If updateTrim
 * is false, then trimCoar and trimFine must be set. If updateTrim is ture,
 * then it is not necessary to set trimCoar and trimFine.
 *
 * @param[out] config Pointer to the configuration structure.
 */
void SCG_HAL_GetFircDefaultConfig(scg_firc_config_t *config);

/*!
 * @brief Initialize SCG fast IRC clock.
 *
 * This function enables the SCG fast IRC clock according to the
 * configuration.
 *
 * @param[in] base Register base address for the SCG instance.
 * @param[in] config Pointer to the configuration structure.
 * @return Status of module initialization
 * @retval STATUS_SUCCESS FIRC is initialized.
 * @retval STATUS_BUSY FIRC has been enabled and used by system clock.
 * @retval STATUS_ERROR FIRC initialization routine detected an error
 *
 * @note This function can not detect whether system OSC has been enabled and
 * used by some IPs.
 */
status_t SCG_HAL_InitFirc(SCG_Type * base,
                              const scg_firc_config_t *config);

/*!
 * @brief De-initialize SCG fast IRC.
 *
 * This function disables the SCG fast IRC.
 *
 * @param[in] base Register base address for the SCG instance.
 * @return Status of module de-initialization
 * @retval STATUS_SUCCESS FIRC is deinitialized.
 * @retval STATUS_BUSY FIRC is used by system clock.
 *
 * @note This function can not detect whether FIRC is used by some IPs.
 */
status_t SCG_HAL_DeinitFirc(SCG_Type * base);

/*!
 * @brief Get SCG FIRC clock frequency.
 *
 * @param[in] base Register base address for the SCG instance.
 * @return  Clock frequency, if clock is invalid, return 0.
 */
uint32_t SCG_HAL_GetFircFreq(const SCG_Type * base);

/*!
 * @brief Get SCG asynchronous clock frequency from FIRC.
 *
 * @param[in] base Register base address for the SCG instance.
 * @param[in] type The asynchronous clock type.
 * @return  Clock frequency, if clock is invalid, return 0.
 */
uint32_t SCG_HAL_GetFircAsyncFreq(const SCG_Type * base,
                                 scg_async_clock_type_t type);

/*!
 * @brief Get SCG system clock source.
 *
 * This function gets the SCG system clock source, these clocks are used for
 * core, platform, external and bus clock domains.
 *
 * @param[in] base Register base address for the SCG instance.
 * @return Clock source.
 * Implements SCG_HAL_GetSystemClockSrc_Activity
 */
static inline scg_system_clock_src_t SCG_HAL_GetSystemClockSrc(const SCG_Type * base)
{
    scg_system_clock_src_t retValue;
    uint32_t regValue = base->CSR;
    regValue = (regValue & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT;

    switch(regValue)
    {
        case 1U:
            retValue = SCG_SYSTEM_CLOCK_SRC_SYS_OSC;
            break;
        case 2U:
            retValue = SCG_SYSTEM_CLOCK_SRC_SIRC;
            break;
        case 3U:
            retValue = SCG_SYSTEM_CLOCK_SRC_FIRC;
            break;
        case 6U:
            retValue = SCG_SYSTEM_CLOCK_SRC_SYS_PLL;
            break;
        default:
            retValue = SCG_SYSTEM_CLOCK_SRC_NONE;
            break;
    }
    return retValue;
}

/*!
 * @brief Get SCG system clock frequency.
 *
 * This function gets the SCG system clock frequency, these clocks are used for
 * core, platform, external and bus clock domains.
 *
 * @param[in] base Register base address for the SCG instance.
 * @param[in] type Which type of clock to get, core clock or slow clock.
 * @return Clock frequency.
 */
uint32_t SCG_HAL_GetSystemClockFreq(const SCG_Type * base,
                                    scg_system_clock_type_t type);

/* @} */


/*!
 * @name System PLL Clock.
 * @{
 */

/*!
 * @brief Get the default system PLL configuration.
 *
 * This function gets the default SCG system PLL configuration.
 *
 * @param[out] config Pointer to the configuration structure.
 */
void SCG_HAL_GetSysPllDefaultConfig(scg_spll_config_t *config);

/*!
 * @brief Initialize SCG system PLL.
 *
 * This function enables the SCG system PLL clock according to the
 * configuration. The system PLL could use system OSC or FIRC as
 * the clock source, please make sure the source clock is valid before
 * this function.
 *
 * @param[in] base Register base address for the SCG instance.
 * @param[in] config Pointer to the configuration structure.
 * @return Status of module initialization
 * @retval STATUS_SUCCESS System PLL is initialized.
 * @retval STATUS_BUSY System PLL has been enabled and used by system clock.
 *
 * @note This function can not detect whether system PLL has been enabled and
 * used by some IPs.
 */
status_t SCG_HAL_InitSysPll(SCG_Type * base,
                                scg_spll_config_t const *config);

/*!
 * @brief De-initialize SCG system PLL.
 *
 * This function disables the SCG system PLL.
 *
 * @param[in] base Register base address for the SCG instance.
 * @return Status of module de-initialization
 * @retval STATUS_SUCCESS system PLL is deinitialized.
 * @retval STATUS_BUSY system PLL is used by system clock.
 *
 * @note This function can not detect whether system PLL is used by some IPs.
 */
status_t SCG_HAL_DeinitSysPll(SCG_Type * base);

/*!
 * @brief Get SCG system PLL clock frequency.
 *
 * @param[in] base Register base address for the SCG instance.
 * @return  Clock frequency, if clock is invalid, return 0.
 */
uint32_t SCG_HAL_GetSysPllFreq(const SCG_Type * base);

/*!
 * @brief Get SCG asynchronous clock frequency from system PLL.
 *
 * @param[in] base Register base address for the SCG instance.
 * @param[in] type The asynchronous clock type.
 * @return  Clock frequency, if clock is invalid, return 0.
 */
uint32_t SCG_HAL_GetSysPllAsyncFreq(const SCG_Type * base,
                                    scg_async_clock_type_t type);

/* @} */

/*!
 * @name RTC Clock.
 * @{
 */

/*!
 * @brief Set SCG RTC CLKIN clock frequency.
 *
 * @param[in] base Register base address for the SCG instance.
 * @param[in] frequency The frequency of the RTC_CLKIN
 */
void SCG_HAL_SetRtcClkInFreq(SCG_Type * base, uint32_t frequency);

/*!
 * @brief Get SCG RTC CLKIN clock frequency.
 *
 * @param[in] base Register base address for the SCG instance.
 * @return  Clock frequency
 */
uint32_t SCG_HAL_GetRtcClkInFreq(SCG_Type * base);

/* @} */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */

#endif /* FEATURE_SOC_SCG_COUNT */
#endif /* SCG_HAL_H */

/******************************************************************************
 * EOF
 *****************************************************************************/

