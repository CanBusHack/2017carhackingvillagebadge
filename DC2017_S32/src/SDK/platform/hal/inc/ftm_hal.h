/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 -2017 NXP
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
 * @file ftm_hal.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * The macros defined are used to define features for each driver, so this might be reported
 * when the analysis is made only on one driver.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Directive 4.9, Function-like macro defined.
 * This macro is needed in creating a common name for any IP.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from
 * 'essentially Boolean' type to 'essentially unsigned'.This is required by the
 * conversion of a bit-field of a bool type into a bit-field of a register type.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.7, Composite expression with smaller
 * essential type than other operand.
 * The expression is safe as the baud rate calculation algorithm cannot overflow
 * the result.
 */

#ifndef FTM_HAL_H
#define FTM_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include "device_registers.h"

/*!
 * @defgroup ftm_hal FTM HAL
 * @ingroup ftm
 * @brief FlexTimer Module Hardware Abstraction Level.
 * FTM HAL provides low level APIs for reading and writing to all hardware features
 * of the FlexTimer module.
 * @addtogroup ftm_hal
 * @{
 */

/*
 * S32K144 FTM
 *
 * FlexTimer Module
 *
 * Registers defined in this header file:
 * - FTM_SC - Status And Control
 * - FTM_CNT - Counter
 * - FTM_MOD - Modulo
 * - FTM_C0SC - Channel (n) Status And Control
 * - FTM_C0V - Channel (n) Value
 * - FTM_C1SC - Channel (n) Status And Control
 * - FTM_C1V - Channel (n) Value
 * - FTM_C2SC - Channel (n) Status And Control
 * - FTM_C2V - Channel (n) Value
 * - FTM_C3SC - Channel (n) Status And Control
 * - FTM_C3V - Channel (n) Value
 * - FTM_C4SC - Channel (n) Status And Control
 * - FTM_C4V - Channel (n) Value
 * - FTM_C5SC - Channel (n) Status And Control
 * - FTM_C5V - Channel (n) Value
 * - FTM_C6SC - Channel (n) Status And Control
 * - FTM_C6V - Channel (n) Value
 * - FTM_C7SC - Channel (n) Status And Control
 * - FTM_C7V - Channel (n) Value
 * - FTM_CNTIN - Counter Initial Value
 * - FTM_STATUS - Capture And Compare Status
 * - FTM_MODE - Features Mode Selection
 * - FTM_SYNC - Synchronization
 * - FTM_OUTINIT - Initial State For Channels Output
 * - FTM_OUTMASK - Output Mask
 * - FTM_COMBINE - Function For Linked Channels
 * - FTM_DEADTIME - Dead-time Insertion Control
 * - FTM_EXTTRIG - FTM External Trigger
 * - FTM_POL - Channels Polarity
 * - FTM_FMS - Fault Mode Status
 * - FTM_FILTER - Input Capture Filter Control
 * - FTM_FLTCTRL - Fault Control
 * - FTM_QDCTRL - Quadrature Decoder Control And Status
 * - FTM_CONF - Configuration
 * - FTM_FLTPOL - FTM Fault Input Polarity
 * - FTM_SYNCONF - Synchronization Configuration
 * - FTM_INVCTRL - FTM Inverting Control
 * - FTM_SWOCTRL - FTM Software Output Control
 * - FTM_PWMLOAD - FTM PWM Load
 * - FTM_HCR - Half Cycle Register
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @name FTM instance number
 * @{
 */
/*!< @brief Instance number for FTM0. */
#define FTM0_IDX (0U)
/*!< @brief Instance number for FTM1. */
#define FTM1_IDX (1U)
/*!< @brief Instance number for FTM2. */
#define FTM2_IDX (2U)
/*!< @brief Instance number for FTM3. */
#define FTM3_IDX (3U)
/*@}*/

/*!
 * @brief FTM_SC - Read and modify and write to Status And Control (RW)
 */
#define FTM_RMW_SC(base, mask, value) (((base)->SC) = ((((base)->SC) & ~(mask)) | (value)))

/*!
 * @brief FTM_CNT - Read and modify and write to Counter (RW)
 */
#define FTM_RMW_CNT(base, mask, value) (((base)->CNT) = ((((base)->CNT) & ~(mask)) | (value)))

/*!
 * @brief FTM_MOD - Read and modify and write Modulo (RW)
 */
#define FTM_RMW_MOD(base, mask, value) (((base)->MOD) = ((((base)->MOD) & ~(mask)) | (value)))

/*!
 * @brief FTM_CNTIN - Read and modify and write Counter Initial Value (RW)
 */
#define FTM_RMW_CNTIN(base, mask, value) (((base)->CNTIN) = ((((base)->CNTIN) & ~(mask)) | (value)))

/*!
 * @brief FTM_STATUS - Read and modify and write Capture And Compare Status (RW)
 */
#define FTM_RMW_STATUS(base, mask, value) (((base)->STATUS) = ((((base)->STATUS) & ~(mask)) | (value)))

/*!
 * @brief FTM_MODE -  Read and modify and write Counter Features Mode Selection (RW)
 */
#define FTM_RMW_MODE(base, mask, value) (((base)->MODE) = ((((base)->MODE) & ~(mask)) | (value)))

/*!
 * @brief FTM_CnSCV -  Read and modify and write Channel (n) Status And Control (RW)
 */
#define FTM_RMW_CnSCV_REG(base, channel, mask, value) (((base)->CONTROLS[channel].CnSC) = ((((base)->CONTROLS[channel].CnSC) & ~(mask)) | (value)))

/*!
 * @brief FTM_DEADTIME - Read and modify and write Dead-time Insertion Control (RW)
 */
#define FTM_RMW_DEADTIME(base, mask, value) (((base)->DEADTIME) = ((((base)->DEADTIME) & ~(mask)) | (value)))
/*!
 * @brief FTM_EXTTRIG - Read and modify and write External Trigger Control (RW)
 */
#define FTM_RMW_EXTTRIG_REG(base, mask, value) (((base)->EXTTRIG) = ((((base)->EXTTRIG) & ~(mask)) | (value)))

/*!
 * @brief FTM_FLTCTRL -  Read and modify and write Fault Control (RW)
 */
#define FTM_RMW_FLTCTRL(base, mask, value) (((base)->FLTCTRL) = ((((base)->FLTCTRL) & ~(mask)) | (value)))

/*!
 * @brief FTM_FMS -  Read and modify and write Fault Mode Status (RW)
 */
#define FTM_RMW_FMS(base, mask, value) (((base)->FMS) = ((((base)->FMS) & ~(mask)) | (value)))

/*!
 * @brief FTM_CONF -  Read and modify and write Configuration (RW)
 */
#define FTM_RMW_CONF(base, mask, value) (((base)->CONF) = ((((base)->CONF) & ~(mask)) | (value)))

/*!
 * @brief POL -  Read and modify and write Polarity (RW)
 */
#define FTM_RMW_POL(base, mask, value) (((base)->POL) = ((((base)->POL) & ~(mask)) | (value)))

/*!
 * @brief FILTER -  Read and modify and write Filter (RW)
 */
#define FTM_RMW_FILTER(base, mask, value) (((base)->FILTER) = ((((base)->FILTER) & ~(mask)) | (value)))

/*!
 * @brief SYNC -  Read and modify and write Synchronization (RW)
 */
#define FTM_RMW_SYNC(base, mask, value) (((base)->SYNC) = ((((base)->SYNC) & ~(mask)) | (value)))

/*!
 * @brief QDCTRL -  Read and modify and write Quadrature Decoder Control And Status (RW)
 */
#define FTM_RMW_QDCTRL(base, mask, value) (((base)->QDCTRL) = ((((base)->QDCTRL) & ~(mask)) | (value)))

/*!
 * @brief FTM_PAIR0DEADTIME - Read and modify and write Dead-time Insertion Control for the pair 0 (RW)
 */
#define FTM_RMW_PAIR0DEADTIME(base, mask, value) (((base)->PAIR0DEADTIME) = ((((base)->PAIR0DEADTIME) & ~(mask)) | (value)))

/*!
 * @brief FTM_PAIR1DEADTIME - Read and modify and write Dead-time Insertion Control for the pair 1 (RW)
 */
#define FTM_RMW_PAIR1DEADTIME(base, mask, value) (((base)->PAIR1DEADTIME) = ((((base)->PAIR1DEADTIME) & ~(mask)) | (value)))

/*!
 * @brief FTM_PAIR2DEADTIME - Read and modify and write Dead-time Insertion Control for the pair 2 (RW)
 */
#define FTM_RMW_PAIR2DEADTIME(base, mask, value) (((base)->PAIR2DEADTIME) = ((((base)->PAIR2DEADTIME) & ~(mask)) | (value)))

/*!
 * @brief FTM_PAIR3DEADTIME - Read and modify and write Dead-time Insertion Control for the pair 3 (RW)
 */
#define FTM_RMW_PAIR3DEADTIME(base, mask, value) (((base)->PAIR3DEADTIME) = ((((base)->PAIR3DEADTIME) & ~(mask)) | (value)))

/*!< @brief Channel number for CHAN0.*/
#define CHAN0_IDX (0U)
/*!< @brief Channel number for CHAN1.*/
#define CHAN1_IDX (1U)
/*!< @brief Channel number for CHAN2.*/
#define CHAN2_IDX (2U)
/*!< @brief Channel number for CHAN3.*/
#define CHAN3_IDX (3U)
/*!< @brief Channel number for CHAN4.*/
#define CHAN4_IDX (4U)
/*!< @brief Channel number for CHAN5.*/
#define CHAN5_IDX (5U)
/*!< @brief Channel number for CHAN6.*/
#define CHAN6_IDX (6U)
/*!< @brief Channel number for CHAN7.*/
#define CHAN7_IDX (7U)

/*******************************************************************************
 * Enumerations
 ******************************************************************************/

/*!
 * @brief FlexTimer clock source selection
 *
 * Implements : ftm_clock_source_t_Class
 */
typedef enum
{
    FTM_CLOCK_SOURCE_NONE           = 0x00U,    /*!< None use clock for FTM  */
    FTM_CLOCK_SOURCE_SYSTEMCLK      = 0x01U,    /*!< System clock            */
    FTM_CLOCK_SOURCE_FIXEDCLK       = 0x02U,    /*!< Fixed clock             */
    FTM_CLOCK_SOURCE_EXTERNALCLK    = 0x03U     /*!< External clock          */
} ftm_clock_source_t;

/*!
 * @brief FlexTimer pre-scaler factor selection for the clock source.
 * In quadrature decoder mode set FTM_CLOCK_DIVID_BY_1
 *
 * Implements : ftm_clock_ps_t_Class
 */
typedef enum
{
    FTM_CLOCK_DIVID_BY_1    = 0x00U,    /*!< Divide by 1   */
    FTM_CLOCK_DIVID_BY_2    = 0x01U,    /*!< Divide by 2   */
    FTM_CLOCK_DIVID_BY_4    = 0x02U,    /*!< Divide by 4   */
    FTM_CLOCK_DIVID_BY_8    = 0x03U,    /*!< Divide by 8   */
    FTM_CLOCK_DIVID_BY_16   = 0x04U,    /*!< Divide by 16  */
    FTM_CLOCK_DIVID_BY_32   = 0x05U,    /*!< Divide by 32  */
    FTM_CLOCK_DIVID_BY_64   = 0x06U,    /*!< Divide by 64  */
    FTM_CLOCK_DIVID_BY_128  = 0x07U     /*!< Divide by 128 */
} ftm_clock_ps_t;

/*!
 * @brief FlexTimer pre-scaler factor for the dead-time insertion
 *
 * Implements : ftm_deadtime_ps_t_Class
 */
typedef enum
{
    FTM_DEADTIME_DIVID_BY_1  = 0x01U, /*!< Divide by 1   */
    FTM_DEADTIME_DIVID_BY_4  = 0x02U, /*!< Divide by 4   */
    FTM_DEADTIME_DIVID_BY_16 = 0x03U  /*!< Divide by 16  */
} ftm_deadtime_ps_t;

/*!
 * @brief FlexTimer PWM output pulse mode, high-true or low-true on match up
 *
 * Implements : ftm_polarity_t_Class
 */
typedef enum
{
    FTM_POLARITY_LOW  = 0x00U,  /*!< When counter > CnV output signal is LOW */
    FTM_POLARITY_HIGH = 0x01U   /*!< When counter > CnV output signal is HIGH */
} ftm_polarity_t;

/*!
 * @brief FlexTimer PWM channel (n+1) polarity for combine mode
 *
 * Implements : ftm_second_channel_polarity_t_Class
 */
typedef enum
{
    FTM_MAIN_INVERTED   = 0x01U,  /*!< The channel (n+1) output is the inverse of the
                                   *   channel (n) output  */
    FTM_MAIN_DUPLICATED = 0x00U   /*!< The channel (n+1) output is the same as the
                                   *   channel (n) output */
} ftm_second_channel_polarity_t;

/*!
 * @brief FlexTimer quadrature decode modes, phase encode or count and direction mode
 *
 * Implements : ftm_quad_decode_mode_t_Class
 */
typedef enum
{
    FTM_QUAD_PHASE_ENCODE   = 0x00U,    /*!< Phase encoding mode                 */
    FTM_QUAD_COUNT_AND_DIR  = 0x01U     /*!< Counter and direction encoding mode */
} ftm_quad_decode_mode_t;

/*!
 * @brief FlexTimer quadrature phase polarities, normal or inverted polarity
 *
 * Implements : ftm_quad_phase_polarity_t_Class
 */
typedef enum
{
    FTM_QUAD_PHASE_NORMAL = 0x00U,  /*!< Phase input signal is not inverted before identifying
                                     *   the rising and falling edges of this signal */
    FTM_QUAD_PHASE_INVERT = 0x01U   /*!< Phase input signal is inverted before identifying
                                     *   the rising and falling edges of this signal */
} ftm_quad_phase_polarity_t;

/*!
 * @brief Options for the FlexTimer behavior in BDM Mode
 *
 * Implements : ftm_bdm_mode_t_Class
 */
typedef enum
{
    FTM_BDM_MODE_00 = 0x00U,    /*!< FTM counter stopped, CH(n)F bit can be set, FTM channels
                                 *   in functional mode, writes to MOD,CNTIN and C(n)V registers bypass
                                 *   the register buffers */
    FTM_BDM_MODE_01 = 0x01U,    /*!< FTM counter stopped, CH(n)F bit is not set, FTM channels
                                 *   outputs are forced to their safe value , writes to MOD,CNTIN and
                                 *   C(n)V registers bypass the register buffers */
    FTM_BDM_MODE_10 = 0x02U,    /*!< FTM counter stopped, CH(n)F bit is not set, FTM channels
                                *    outputs are frozen when chip enters in BDM mode, writes to MOD,
                                *    CNTIN and C(n)V registers bypass the register buffers */
    FTM_BDM_MODE_11 = 0x03U     /*!< FTM counter in functional mode, CH(n)F bit can be set,
                                 *   FTM channels in functional mode, writes to MOD,CNTIN and C(n)V
                                 *   registers is in fully functional mode */
} ftm_bdm_mode_t;

/*!
 * @brief FlexTimer fault control
 *
 * Implements : ftm_fault_mode_t_Class
 */
typedef enum
{
    FTM_FAULT_CONTROL_DISABLED  = 0x00U,    /*!< Fault control is disabled for all channels */
    FTM_FAULT_CONTROL_MAN_EVEN  = 0x01U,    /*!< Fault control is enabled for even channels
                                             *   only (channels 0, 2, 4, and 6), and the selected
                                             *   mode is the manual fault clearing */
    FTM_FAULT_CONTROL_MAN_ALL   = 0x02U,    /*!< Fault control is enabled for all channels,
                                             *   and the selected mode is the manual fault clearing */
    FTM_FAULT_CONTROL_AUTO_ALL  = 0x03U     /*!< Fault control is enabled for all channels, and
                                             *   the selected mode is the automatic fault clearing */
} ftm_fault_mode_t;

/*!
 * @brief FTM sync source
 *
 * Implements : ftm_reg_update_t_Class
 */
typedef enum
{
    FTM_SYSTEM_CLOCK    = 0U,       /*!< Register is updated with its buffer value at all rising
                                     *   edges of system clock */
    FTM_PWM_SYNC        = 1U        /*!< Register is updated with its buffer value at the
                                     *   FTM synchronization */
} ftm_reg_update_t;

/*!
 * @brief FTM update register
 *
 * Implements : ftm_pwm_sync_mode_t_Class
 */
typedef enum
{
    FTM_WAIT_LOADING_POINTS = 0U,   /*!< FTM register is updated at first loading point */
    FTM_UPDATE_NOW          = 1U    /*!< FTM register is updated immediately */
} ftm_pwm_sync_mode_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Sets the value for the half cycle reload register.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value The 16 bit counter value
 *
 * Implements : FTM_HAL_SetHalfCycleValue_Activity
 */
static inline void FTM_HAL_SetHalfCycleValue(FTM_Type * const ftmBase,
                                             uint16_t value)
{
    ((ftmBase)->HCR) = value;
}

/*!
 * @brief Sets the filter Pre-scaler divider.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] filterPrescale The FTM peripheral clock pre-scale divider
 *
 * Implements : FTM_HAL_SetClockFilterPs_Activity
 */
static inline void FTM_HAL_SetClockFilterPs(FTM_Type * const ftmBase,
                                            uint8_t filterPrescale)
{
    FTM_RMW_SC(ftmBase, FTM_SC_FLTPS_MASK, FTM_SC_FLTPS(filterPrescale));
}

/*!
 * @brief Reads the FTM filter clock divider.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The FTM filter clock pre-scale divider
 *
 * Implements : FTM_HAL_GetClockFilterPs_Activity
 */
static inline uint8_t FTM_HAL_GetClockFilterPs(const FTM_Type * ftmBase)
{
    return (uint8_t)((((ftmBase)->SC) & FTM_SC_FLTPS_MASK) >> FTM_SC_FLTPS_SHIFT);
}

/*!
 * @brief Sets the FTM clock source.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] clock The FTM peripheral clock selection
 *            - 00: No clock
 *            - 01: system clock
 *            - 10: fixed clock
 *            - 11: External clock
 *
 * Implements : FTM_HAL_SetClockSource_Activity
 */
static inline void FTM_HAL_SetClockSource(FTM_Type * const ftmBase,
                                          ftm_clock_source_t clock)
{
    FTM_RMW_SC(ftmBase, FTM_SC_CLKS_MASK, FTM_SC_CLKS(clock));
}

/*!
 * @brief Reads the FTM clock source.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The FTM clock source selection
 *          - 00: No clock
 *          - 01: system clock
 *          - 10: fixed clock
 *          - 11: External clock
 *
 * Implements : FTM_HAL_GetClockSource_Activity
 */
static inline uint8_t FTM_HAL_GetClockSource(const FTM_Type * ftmBase)
{
    return (uint8_t)((((ftmBase)->SC) & FTM_SC_CLKS_MASK) >> FTM_SC_CLKS_SHIFT);
}

/*!
 * @brief Sets the FTM clock divider.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] ps The FTM peripheral clock pre-scale divider
 *
 * Implements : FTM_HAL_SetClockPs_Activity
 */
static inline void FTM_HAL_SetClockPs(FTM_Type * const ftmBase,
                                      ftm_clock_ps_t ps)
{
    FTM_RMW_SC(ftmBase, FTM_SC_PS_MASK, FTM_SC_PS(ps));
}

/*!
 * @brief Reads the FTM clock divider.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The FTM clock pre-scale divider
 *
 * Implements : FTM_HAL_GetClockPs_Activity
 */
static inline uint8_t FTM_HAL_GetClockPs(const FTM_Type * ftmBase)
{
    return (uint8_t)((((ftmBase)->SC) & FTM_SC_PS_MASK) >> FTM_SC_PS_SHIFT);
}

/*!
 * @brief  Enables the FTM peripheral timer overflow interrupt.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] state - true : Overflow interrupt enabled
 *                  - false: Overflow interrupt disabled
 *
 * Implements : FTM_HAL_SetTimerOverflowInt_Activity
 */
static inline void FTM_HAL_SetTimerOverflowInt(FTM_Type * const ftmBase,
                                               bool state)
{
    FTM_RMW_SC(ftmBase, FTM_SC_TOIE_MASK, FTM_SC_TOIE(state));
}

/*!
 * @brief Reads the bit that controls enabling the FTM timer overflow interrupt.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return State of Timer Overflow Interrupt
 *         - true : Overflow interrupt is enabled
 *         - false: Overflow interrupt is disabled
 *
 * Implements : FTM_HAL_IsOverflowIntEnabled_Activity
 */
static inline bool FTM_HAL_IsOverflowIntEnabled(const FTM_Type * ftmBase)
{
    return ((ftmBase->SC & FTM_SC_TOIE_MASK) >> FTM_SC_TOIE_SHIFT) != 0U;
}

/*!
 * @brief Enable PWM channel Outputs.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM channel
 *
 * Implements : FTM_HAL_EnablePwmChannelOutputs_Activity
 */
static inline void FTM_HAL_EnablePwmChannelOutputs(FTM_Type * const ftmBase,
                                                   uint8_t channel)
{
    FTM_RMW_SC(ftmBase, (1UL << (channel + FTM_FEATURE_OUTPUT_CHANNEL_OFFSET)), (1UL << (channel + FTM_FEATURE_OUTPUT_CHANNEL_OFFSET)));
}

/*!
 * @brief Disable PWM channel Outputs.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM channel
 *
 * Implements : FTM_HAL_DisablePwmChannelOutputs_Activity
 */
static inline void FTM_HAL_DisablePwmChannelOutputs(FTM_Type * const ftmBase,
                                                    uint8_t channel)
{
    uint32_t regValue = ((ftmBase)->SC);
    regValue = regValue & (~(1UL << (channel + FTM_FEATURE_OUTPUT_CHANNEL_OFFSET)));
    ((ftmBase)->SC) = (regValue);
}

/*!
 * @brief Clears the timer overflow interrupt flag.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * Implements : FTM_HAL_ClearTimerOverflow_Activity
 */
static inline void FTM_HAL_ClearTimerOverflow(FTM_Type * const ftmBase)
{
    FTM_RMW_SC(ftmBase, FTM_SC_TOF_MASK, FTM_SC_TOF(0U));
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->SC;
#endif
}

/*!
 * @brief Returns the FTM peripheral timer overflow interrupt flag.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return State of Timer Overflow Flag
 *         - true : FTM counter has overflowed
 *         - false: FTM counter has not overflowed
 *
 * Implements : FTM_HAL_HasTimerOverflowed_Activity
 */
static inline bool FTM_HAL_HasTimerOverflowed(const FTM_Type * ftmBase)
{
    return ((ftmBase->SC & FTM_SC_TOF_MASK) >> FTM_SC_TOF_SHIFT) != 0U;
}

/*!
 * @brief Sets the FTM count direction bit.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode The Center-Aligned PWM selection
 *                 - 1U: Up counting mode
 *                 - 0U: Up down counting mode
 *
 * Implements : FTM_HAL_SetCpwms_Activity
 */
static inline void FTM_HAL_SetCpwms(FTM_Type * const ftmBase,
                                    bool mode)
{
    FTM_RMW_SC(ftmBase, FTM_SC_CPWMS_MASK, FTM_SC_CPWMS(mode));
}

/*!
 * @brief Gets the FTM count direction bit.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The Center-Aligned PWM selection
 *         - 1U: Up counting mode
 *         - 0U: Up down counting mode
 *
 * Implements : FTM_HAL_GetCpwms_Activity
 */
static inline bool FTM_HAL_GetCpwms(const FTM_Type * ftmBase)
{
    return ((ftmBase->SC & FTM_SC_CPWMS_MASK) >> FTM_SC_CPWMS_SHIFT) != 0U;
}

/*!
 * @brief Set the FTM reload interrupt enable.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable - true : Reload interrupt is  enabled
 *                   - false: Reload interrupt is disabled
 *
 * Implements : FTM_HAL_SetReIntEnabledCmd_Activity
 */
static inline void FTM_HAL_SetReIntEnabledCmd(FTM_Type * const ftmBase,
                                              bool enable)
{
    FTM_RMW_SC(ftmBase, FTM_SC_RIE_MASK, FTM_SC_RIE(enable));
}

/*!
 * @brief Get the state whether the FTM counter reached a reload point.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return State of reload point
 *         - true : FTM counter reached a reload point
 *         - false: FTM counter did not reach a reload point
 *
 * Implements : FTM_HAL_GetReloadFlag_Activity
 */
static inline bool FTM_HAL_GetReloadFlag(const FTM_Type * ftmBase)
{
    return ((ftmBase->SC & FTM_SC_RF_MASK) >> FTM_SC_RF_SHIFT) != 0U;
}

/*!
 * @brief Clears the reload flag bit.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * Implements : FTM_HAL_ClearReloadFlag_Activity
 */
static inline void FTM_HAL_ClearReloadFlag(FTM_Type * const ftmBase)
{
    FTM_RMW_SC(ftmBase, FTM_SC_RF_MASK, FTM_SC_RF(0U));
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->SC;
#endif
}

/*!
 * @brief Sets the FTM peripheral current counter value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] val The FTM timer counter value to be set
 *
 * Implements : FTM_HAL_SetCounter_Activity
 */
static inline void FTM_HAL_SetCounter(FTM_Type * const ftmBase,
                                      uint16_t value)
{
    FTM_RMW_CNT(ftmBase, FTM_CNT_COUNT_MASK, FTM_CNT_COUNT(value));
}

/*!
 * @brief Returns the FTM peripheral current counter value.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The current FTM timer counter value
 *
 * Implements : FTM_HAL_GetCounter_Activity
 */
static inline uint16_t FTM_HAL_GetCounter(const FTM_Type * ftmBase)
{
    return (uint16_t)((((ftmBase)->CNT) & FTM_CNT_COUNT_MASK) >> FTM_CNT_COUNT_SHIFT);
}

/*!
 * @brief Sets the FTM peripheral timer modulo value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value The value to be set to the timer modulo
 *
 * Implements : FTM_HAL_SetMod_Activity
 */
static inline void FTM_HAL_SetMod(FTM_Type * const ftmBase,
                                  uint16_t value)
{
    FTM_RMW_MOD(ftmBase, FTM_MOD_MOD_MASK, FTM_MOD_MOD(value));
}

/*!
 * @brief Returns the FTM peripheral counter modulo value.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return FTM timer modulo value
 *
 * Implements : FTM_HAL_GetMod_Activity
 */
static inline uint16_t FTM_HAL_GetMod(const FTM_Type * ftmBase)
{
    return (uint16_t)((((ftmBase)->MOD) & FTM_MOD_MOD_MASK) >> FTM_MOD_MOD_SHIFT);
}

/*!
 * @brief Sets the FTM peripheral timer counter initial value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value initial value to be set
 *
 * Implements : FTM_HAL_SetCounterInitVal_Activity
 */
static inline void FTM_HAL_SetCounterInitVal(FTM_Type * const ftmBase,
                                             uint16_t value)
{
    FTM_RMW_CNTIN(ftmBase, FTM_CNTIN_INIT_MASK, FTM_CNTIN_INIT(value));
}

/*!
 * @brief Returns the FTM peripheral counter initial value.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return FTM timer counter initial value
 *
 * Implements : FTM_HAL_GetCounterInitVal_Activity
 */
static inline uint16_t FTM_HAL_GetCounterInitVal(const FTM_Type * ftmBase)
{
    return (uint16_t)((((ftmBase)->CNTIN) & FTM_CNTIN_INIT_MASK) >> FTM_CNTIN_INIT_SHIFT);
}

/*!
 * @brief Sets the FTM peripheral timer channel mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel  The FTM peripheral channel number
 * @param[in] selection The mode to be set valid value MSnB:MSnA :00, 01, 10, 11
 *
 * Implements : FTM_HAL_SetChnMSnBAMode_Activity
 */
static inline void FTM_HAL_SetChnMSnBAMode(FTM_Type * const ftmBase,
                                           uint8_t channel,
                                           uint8_t selection)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    /* write MSA bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_MSA_MASK, FTM_CnSC_MSA((uint32_t)selection & 0x01U));

    /* write MSB bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_MSB_MASK, FTM_CnSC_MSB(((uint32_t)selection & 0x02U) >> 1U));
}

/*!
 * @brief Sets the FTM peripheral timer channel edge level.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] level ELSnB:ELSnA :00, 01, 10, 11
 *
 * Implements : FTM_HAL_SetChnEdgeLevel_Activity
 */
static inline void FTM_HAL_SetChnEdgeLevel(FTM_Type * const ftmBase,
                                           uint8_t channel,
                                           uint8_t level)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    /* write MSA bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_ELSA_MASK, FTM_CnSC_ELSA((uint32_t)level & 0x01U));

    /* write MSB bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_ELSB_MASK, FTM_CnSC_ELSB(((uint32_t)level & 0x02U) >> 1U));
}

/*!
 * @brief Clears the content of Channel (n) Status And Control.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel  The FTM peripheral channel number
 *
 * Implements : FTM_HAL_ClearChSC_Activity
 */
static inline void FTM_HAL_ClearChSC(FTM_Type * const ftmBase,
                                     uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    ((ftmBase)->CONTROLS[channel].CnSC) = 0U;
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->CONTROLS[channel].CnSC;
#endif
}

/*!
 * @brief Gets the FTM peripheral timer channel mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * @return The MSnB:MSnA mode value, will be 00, 01, 10, 11
 *
 * Implements : FTM_HAL_GetChnMode_Activity
 */
static inline uint8_t FTM_HAL_GetChnMode(const FTM_Type * ftmBase,
                                         uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);
    uint8_t retValue;

    retValue = (uint8_t)((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_MSA_MASK) >> FTM_CnSC_MSA_SHIFT);

    retValue |= (uint8_t)(((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_MSB_MASK) >> FTM_CnSC_MSB_SHIFT) << 1U);

    return retValue;
}

/*!
 * @brief Gets the FTM peripheral timer channel edge level.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * @return The ELSnB:ELSnA mode value, will be 00, 01, 10, 11
 *
 * Implements : FTM_HAL_GetChnEdgeLevel_Activity
 */
static inline uint8_t FTM_HAL_GetChnEdgeLevel(const FTM_Type * ftmBase,
                                              uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);
    uint8_t retValue;

    retValue = (uint8_t)((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_ELSA_MASK) >> FTM_CnSC_ELSA_SHIFT);

    retValue |= (uint8_t)(((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_ELSB_MASK) >> FTM_CnSC_ELSB_SHIFT) << 1U);

    return retValue;
}

/*!
 * @brief Configure the feature of FTM counter reset by the selected input capture event.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] enable Enable the FTM counter reset
 *                   - true : FTM counter is reset
 *                   - false: FTM counter is not reset
 *
 * Implements : FTM_HAL_SetChnIcrstCmd_Activity
 */
static inline void FTM_HAL_SetChnIcrstCmd(FTM_Type * const ftmBase,
                                          uint8_t channel,
                                          bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    /* Write ICRST bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_ICRST_MASK, FTM_CnSC_ICRST(enable));
}

/*!
 * @brief Returns whether the FTM FTM counter is reset.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @return State of the FTM peripheral timer channel ICRST
 *         - true : Enabled the FTM counter reset
 *         - false: Disabled the FTM counter reset
 *
 * Implements : FTM_HAL_IsChnIcrst_Activity
 */
static inline bool FTM_HAL_IsChnIcrst(const FTM_Type * ftmBase,
                                      uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_ICRST_MASK) != 0U;
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel DMA.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] enable Enable DMA transfers for the channel
 *                   - true : Enabled DMA transfers
 *                   - false: Disabled DMA transfers
 *
 * Implements : FTM_HAL_SetChnDmaCmd_Activity
 */
static inline void FTM_HAL_SetChnDmaCmd(FTM_Type * const ftmBase,
                                        uint8_t channel,
                                        bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    /* Write DMA bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_DMA_MASK, FTM_CnSC_DMA(enable));
}

/*!
 * @brief Returns whether the FTM peripheral timer channel DMA is enabled.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @return State of the FTM peripheral timer channel DMA
 *         - true : Enabled DMA transfers
 *         - false: Disabled DMA transfers
 *
 * Implements : FTM_HAL_IsChnDma_Activity
 */
static inline bool FTM_HAL_IsChnDma(const FTM_Type * ftmBase,
                                    uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_DMA_MASK) != 0U;
}

/*!
 * @brief Get FTM channel(n) interrupt enabled or not.
 * @param[in] ftmBase FTM module base address
 * @param[in] channel The FTM peripheral channel number
 *
 * Implements : FTM_HAL_IsChnIntEnabled_Activity
 */
static inline bool FTM_HAL_IsChnIntEnabled(const FTM_Type * ftmBase,
                                           uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_CHIE_MASK) != 0U;
}

/*!
 * @brief Enables the FTM peripheral timer channel(n) interrupt.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * Implements : FTM_HAL_EnableChnInt_Activity
 */
static inline void FTM_HAL_EnableChnInt(FTM_Type * const ftmBase,
                                        uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_CHIE_MASK, FTM_CnSC_CHIE(1U));
}

/*!
 * @brief Disables the FTM peripheral timer channel(n) interrupt.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * Implements : FTM_HAL_DisableChnInt_Activity
 */
static inline void FTM_HAL_DisableChnInt(FTM_Type * const ftmBase,
                                         uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_CHIE_MASK, FTM_CnSC_CHIE(0U));
}

/*!
 * @brief Returns whether any event for the FTM peripheral timer channel has occurred.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * @return State of channel flag
 *         - true : Event occurred
 *         - false: No event occurred.
 *
 * Implements : FTM_HAL_HasChnEventOccurred_Activity
 */
static inline bool FTM_HAL_HasChnEventOccurred(const FTM_Type * ftmBase,
                                               uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_CHF_MASK) != 0U;
}

/*!
 * @brief Clear the channel flag by writing a 0 to the CHF bit.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * Implements : FTM_HAL_ClearChnEventFlag_Activity
 */
static inline void FTM_HAL_ClearChnEventFlag(FTM_Type * const ftmBase,
                                             uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_CHF_MASK, FTM_CnSC_CHF(0U));
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->CONTROLS[channel].CnSC;
#endif
}

/*!
 * @brief Enables or disables the trigger generation on FTM channel outputs.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] enable Trigger mode control
 *                   - false : Enable PWM output without generating a pulse
 *                   - true  : Disable a trigger generation on channel output
 *
 * Implements : FTM_HAL_SetTrigModeControlCmd_Activity
 */
static inline void FTM_HAL_SetTrigModeControlCmd(FTM_Type * const ftmBase,
                                                 uint8_t channel,
                                                 bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    /* Write TRIGMODE bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_TRIGMODE_MASK, FTM_CnSC_TRIGMODE((enable)));
}

/*!
 * @brief Returns whether the trigger mode is enabled.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @return State of the channel outputs
 *         - true : Enabled a trigger generation on channel output
 *         - false: PWM outputs without generating a pulse
 *
 * Implements : FTM_HAL_GetTriggerControled_Activity
 */
static inline bool FTM_HAL_GetTriggerControled(const FTM_Type * ftmBase,
                                               uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_TRIGMODE_MASK) != 0U;
}

/*!
 * @brief Get the state of channel input.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @return State of the channel inputs
 *         - true : The channel input is one
 *         - false: The channel input is zero
 *
 * Implements : FTM_HAL_GetChInputState_Activity
 */
static inline bool FTM_HAL_GetChInputState(const FTM_Type * ftmBase,
                                           uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_CHIS_MASK) != 0U;
}

/*!
 * @brief Get the value of channel output.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @return Value of the channel outputs
 *         - true : The channel output is one
 *         - false: The channel output is zero
 *
 * Implements : FTM_HAL_GetChOutputValue_Activity
 */
static inline bool FTM_HAL_GetChOutputValue(const FTM_Type * ftmBase,
                                            uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_CHOV_MASK) != 0U;
}

/*FTM channel control*/
/*!
 * @brief Sets the FTM peripheral timer channel counter value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] value Counter value to be set
 *
 * Implements : FTM_HAL_SetChnCountVal_Activity
 */
static inline void FTM_HAL_SetChnCountVal(FTM_Type * const ftmBase,
                                          uint8_t channel,
                                          uint16_t value)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    ((ftmBase)->CONTROLS[channel].CnV) = value;
}

/*!
 * @brief Gets the FTM peripheral timer channel counter value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * @return Channel counter value
 *
 * Implements : FTM_HAL_GetChnCountVal_Activity
 */
static inline uint16_t FTM_HAL_GetChnCountVal(const FTM_Type * ftmBase,
                                              uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (uint16_t)((ftmBase)->CONTROLS[channel].CnV);
}

/*!
 * @brief Gets the FTM peripheral timer  channel event status.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * @return Channel event status
 *         - true  : A channel event has occurred
 *         - false : No channel event has occurred
 *
 * Implements : FTM_HAL_GetChnEventStatus_Activity
 */
static inline bool FTM_HAL_GetChnEventStatus(const FTM_Type * ftmBase,
                                             uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->STATUS) & (1UL << channel)) != 0U;
}

/*!
 * @brief Gets the FTM peripheral timer status info for all channels.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return Channel event status value
 *
 * Implements : FTM_HAL_GetEventStatus_Activity
 */
static inline uint32_t FTM_HAL_GetEventStatus(const FTM_Type * ftmBase)
{
    return ((ftmBase)->STATUS) & (0xFFU);
}

/*!
 * @brief Clears the FTM peripheral timer all channel event status.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * Implements : FTM_HAL_ClearChnEventStatus_Activity
 */
static inline void FTM_HAL_ClearChnEventStatus(FTM_Type * const ftmBase,
                                               uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    ((ftmBase)->STATUS) &= (~(1UL << channel));
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->STATUS;
#endif
}

/*!
 * @brief Writes the provided value to the OUTMASK register.
 *
 * This function will mask/unmask multiple channels.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] regVal Value to be written to the register
 *
 * Implements : FTM_HAL_SetOutmaskReg_Activity
 */
static inline void FTM_HAL_SetOutmaskReg(FTM_Type * const ftmBase,
                                         uint32_t regVal)
{
    ((ftmBase)->OUTMASK) = regVal;
}

/*!
 * @brief Sets the FTM peripheral timer channel output mask.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] mask Value to set Output Mask
 *                 - true : Channel output is masked
 *                 - false: Channel output is not masked
 *
 * Implements : FTM_HAL_SetChnOutputMask_Activity
 */
static inline void FTM_HAL_SetChnOutputMask(FTM_Type * const ftmBase,
                                            uint8_t channel,
                                            bool mask)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    if (mask)
    {
        ((ftmBase)->OUTMASK) |= (1UL << channel);
    }
    else
    {
        ((ftmBase)->OUTMASK) &= ~(1UL << channel);
    }
}

/*!
 * @brief Sets the FTM peripheral timer channel output initial state 0 or 1.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] state Initial state for channels output
 *                  - true : The initialization value is 1
 *                  - false: The initialization value is 0
 *
 * Implements : FTM_HAL_SetChnOutputInitStateCmd_Activity
 */
static inline void FTM_HAL_SetChnOutputInitStateCmd(FTM_Type * const ftmBase,
                                                    uint8_t channel,
                                                    bool state)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    if (state)
    {
        ((ftmBase)->OUTINIT) |= (1UL << channel);
    }
    else
    {
        ((ftmBase)->OUTINIT) &= ~(1UL << channel);
    }
}

/*!
 * @brief Sets the FTM peripheral timer channel output polarity.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] polarity The polarity to be set
 *            - FTM_POLARITY_HIGH : The channel polarity is active low
 *            - FTM_POLARITY_LOW  : The channel polarity is active high
 *
 * Implements : FTM_HAL_SetChnOutputPolarityCmd_Activity
 */
static inline void FTM_HAL_SetChnOutputPolarityCmd(FTM_Type * const ftmBase,
                                                   uint8_t channel,
                                                   ftm_polarity_t polarity)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    if (FTM_POLARITY_HIGH == polarity)
    {
        ((ftmBase)->POL) &= ~(1UL << channel);
    }
    else
    {
        ((ftmBase)->POL) |= (1UL << channel);
    }
}

/*!
 * @brief Sets the FTM peripheral timer channel fault input polarity.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel  The FTM peripheral channel number
 * @param[in] polarity The polarity to be set
 *                - FTM_POLARITY_LOW : The fault input polarity is active high
 *                - FTM_POLARITY_HIGH: The fault input polarity is active low
 *
 * Implements : FTM_HAL_SetChnFaultInputPolarityCmd_Activity
 */
static inline void FTM_HAL_SetChnFaultInputPolarityCmd(FTM_Type * const ftmBase,
                                                       uint8_t fltChannel,
                                                       ftm_polarity_t polarity)
{
    DEV_ASSERT(fltChannel < FTM_FEATURE_FAULT_CHANNELS);

    if (FTM_POLARITY_HIGH == polarity)
    {
        ((ftmBase)->FLTPOL) &= ~(1UL << fltChannel);
    }
    else
    {
        ((ftmBase)->FLTPOL) |= (1UL << fltChannel);
    }
}

/*!
 * @brief Enables/disables the FTM peripheral timer fault interrupt.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] state Timer fault interrupt state
 *            - true : Fault control interrupt is enable
 *            - false: Fault control interrupt is disabled
 *
 * Implements : FTM_HAL_SetFaultInt_Activity
 */
static inline void FTM_HAL_SetFaultInt(FTM_Type * const ftmBase,
                                       bool state)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_FAULTIE_MASK, FTM_MODE_FAULTIE(state));
}

/*!
 * @brief Disables the FTM peripheral timer fault interrupt.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * Implements : FTM_HAL_DisableFaultInt_Activity
 */
static inline void FTM_HAL_DisableFaultInt(FTM_Type * const ftmBase)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_FAULTIE_MASK, FTM_MODE_FAULTIE(0U));
}

/*!
 * @brief Return true/false whether the Fault interrupt was enabled or not
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * Implements : FTM_HAL_IsFaultIntEnabled_Activity
 */
static inline bool FTM_HAL_IsFaultIntEnabled(const FTM_Type * ftmBase)
{
    return ((ftmBase->MODE & FTM_MODE_FAULTIE_MASK) >> FTM_MODE_FAULTIE_SHIFT) != 0U;
}

/*!
 * @brief Clears all fault interrupt flags that are active.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * Implements : FTM_HAL_ClearFaultsIsr_Activity
 */
static inline void FTM_HAL_ClearFaultsIsr(FTM_Type * const ftmBase)
{
    FTM_RMW_FMS(ftmBase, FTM_FMS_FAULTF0_MASK | FTM_FMS_FAULTF_MASK, FTM_FMS_FAULTF0(0U) | FTM_FMS_FAULTF(0U));
    FTM_RMW_FMS(ftmBase, FTM_FMS_FAULTF1_MASK | FTM_FMS_FAULTF_MASK, FTM_FMS_FAULTF1(0U) | FTM_FMS_FAULTF(0U));
    FTM_RMW_FMS(ftmBase, FTM_FMS_FAULTF2_MASK | FTM_FMS_FAULTF_MASK, FTM_FMS_FAULTF2(0U) | FTM_FMS_FAULTF(0U));
    FTM_RMW_FMS(ftmBase, FTM_FMS_FAULTF3_MASK | FTM_FMS_FAULTF_MASK, FTM_FMS_FAULTF3(0U) | FTM_FMS_FAULTF(0U));
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->FMS;
#endif
}

/*!
 * @brief Defines the FTM fault control mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode Fault control mode value
 * - FTM_FAULT_CONTROL_DISABLED: Fault control disabled
 * - FTM_FAULT_CONTROL_MAN_EVEN: Fault control enabled for even channel (0, 2, 4, 6) and manual fault clearing.
 * - FTM_FAULT_CONTROL_MAN_ALL : Fault control enabled for all channels and manual fault clearing is enabled.
 * - FTM_FAULT_CONTROL_AUTO_ALL: Fault control enabled for all channels and automatic fault clearing is enabled.
 *
 * Implements : FTM_HAL_SetFaultControlMode_Activity
 */
static inline void FTM_HAL_SetFaultControlMode(FTM_Type * const ftmBase,
                                               ftm_fault_mode_t mode)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_FAULTM_MASK, FTM_MODE_FAULTM((uint32_t)mode));
}

/*!
 * @brief Enables or disables the FTM peripheral timer capture test mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable Capture Test Mode Enable
 *            - true : Capture test mode is enabled
 *            - false: Capture test mode is disabled
 *
 * Implements : FTM_HAL_SetCaptureTestCmd_Activity
 */
static inline void FTM_HAL_SetCaptureTestCmd(FTM_Type * const ftmBase,
                                             bool enable)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_CAPTEST_MASK, FTM_MODE_CAPTEST(enable));
}

/*!
 * @brief Enables or disables the FTM write protection.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable The FTM write protection selection
 *                   - true : Write-protection is enabled
 *                   - false: Write-protection is disabled
 *
 * Implements : FTM_HAL_SetWriteProtectionCmd_Activity
 */
static inline void FTM_HAL_SetWriteProtectionCmd(FTM_Type * const ftmBase,
                                                 bool enable)
{
    if (enable)
    {
        ftmBase->FMS = (ftmBase->FMS & ~FTM_FMS_WPEN_MASK) | FTM_FMS_WPEN(1U);
    }
    else
    {
        ftmBase->MODE = (ftmBase->MODE & ~FTM_MODE_WPDIS_MASK) | FTM_MODE_WPDIS(1U);
    }
}

/*!
 * @brief Enables the FTM peripheral timer group.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable FTM mode selection
 *                   - true : All registers including FTM-specific registers are available
 *                   - false: Only the TPM-compatible registers are available
 *
 * Implements : FTM_HAL_Enable_Activity
 */
static inline void FTM_HAL_Enable(FTM_Type * const ftmBase,
                                  bool enable)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_FTMEN_MASK, FTM_MODE_FTMEN(enable));
}

/*!
 * @brief Get status of the FTMEN bit in the FTM_MODE register.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @return the FTM Enable status
 *         - true : TPM compatibility. Free running counter and synchronization compatible with TPM
 *         - false: Free running counter and synchronization are different from TPM behaviour
 *
 * Implements : FTM_HAL_IsFtmEnable_Activity
 */
static inline bool FTM_HAL_IsFtmEnable(const FTM_Type * ftmBase)
{
    return ((ftmBase->MODE & FTM_MODE_FTMEN_MASK) >> FTM_MODE_FTMEN_SHIFT) != 0U;
}

/*!
 * @brief Initializes the channels output.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable Initialize the channels output
 *                   - true : The channels output is initialized according to the state of OUTINIT reg
 *                   - false: No effect
 *
 * Implements : FTM_HAL_SetInitChnOutputCmd_Activity
 */
static inline void FTM_HAL_SetInitChnOutputCmd(FTM_Type * const ftmBase,
                                               bool enable)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_INIT_MASK, FTM_MODE_INIT(enable));
}

/*!
 * @brief Sets the FTM peripheral timer sync mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable PWM synchronization mode
 *                   - false: No restriction both software and hardware triggers can be used
 *                   - true : Software trigger can only be used for MOD and CnV synchronization,
 *                            hardware trigger only for OUTMASK and FTM counter synchronization.
 *
 * Implements : FTM_HAL_SetPwmSyncMode_Activity
 */
static inline void FTM_HAL_SetPwmSyncMode(FTM_Type * const ftmBase,
                                          bool enable)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_PWMSYNC_MASK, FTM_MODE_PWMSYNC(enable));
}

/*!
 * @brief Enables or disables the FTM peripheral timer software trigger.
 *
 * @param[in] ftmBase The FTM base address pointer.
 * @param[in] enable Software trigger selection
 *                   - true : Software trigger is selected
 *                   - false: Software trigger is not selected
 *
 * Implements : FTM_HAL_SetSoftwareTriggerCmd_Activity
 */
static inline void FTM_HAL_SetSoftwareTriggerCmd(FTM_Type * const ftmBase,
                                                 bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_SWSYNC_MASK, FTM_SYNC_SWSYNC(enable));
}

/*!
 * @brief Sets the FTM hardware synchronization trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] trigger_num Number of trigger
 *                        - 0U: trigger 0
 *                        - 1U: trigger 1
 *                        - 2U: trigger 2
 * @param[in] enable State of trigger
 *                   - true : Enable hardware trigger from field trigger_num for PWM synchronization
 *                   - false: Disable hardware trigger from field trigger_num for PWM synchronization
 *
 * Implements : FTM_HAL_SetHardwareSyncTriggerSrc_Activity
 */
static inline void FTM_HAL_SetHardwareSyncTriggerSrc(FTM_Type * const ftmBase,
                                                     uint8_t trigger_num,
                                                     bool enable)
{
    DEV_ASSERT(trigger_num < 3U);

    if (enable)
    {
        ((ftmBase)->SYNC) |= ((uint32_t)(FTM_SYNC_TRIG0_MASK) << trigger_num);
    }
    else
    {
        ((ftmBase)->SYNC) &= ~((uint32_t)(FTM_SYNC_TRIG0_MASK) << trigger_num);
    }
}

/*!
 * @brief Determines when the OUTMASK register is updated with the value of its buffer.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable Output Mask synchronization selection
 *                   - true : OUTMASK register is updated only by PWM synchronization
 *                   - false: OUTMASK register is updated in all rising edges of the system clock
 *
 * Implements : FTM_HAL_SetOutmaskPwmSyncModeCmd_Activity
 */
static inline void FTM_HAL_SetOutmaskPwmSyncModeCmd(FTM_Type * const ftmBase,
                                                    bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_SYNCHOM_MASK, FTM_SYNC_SYNCHOM(enable));
}

/*!
 * @brief Determines if the FTM counter is re-initialized when the selected trigger for
 * synchronization is detected.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable FTM counter re-initialization selection
 *                   - true : To update FTM counter when triggered
 *                   - false: To count normally
 *
 * Implements : FTM_HAL_SetCountReinitSyncCmd_Activity
 */
static inline void FTM_HAL_SetCountReinitSyncCmd(FTM_Type * const ftmBase,
                                                 bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_REINIT_MASK, FTM_SYNC_REINIT(enable));
}

/*!
 * @brief Enables or disables the FTM peripheral timer maximum loading points.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable Maximum loading point selection
 *                   - true : To enable maximum loading point
 *                   - false: To disable
 *
 * Implements : FTM_HAL_SetMaxLoadingCmd_Activity
 */
static inline void FTM_HAL_SetMaxLoadingCmd(FTM_Type * const ftmBase,
                                            bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_CNTMAX_MASK, FTM_SYNC_CNTMAX(enable));
}

/*!
 * @brief Enables or disables the FTM peripheral timer minimum loading points.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable Minimum loading point selection
 *                   - true : To enable minimum loading point
 *                   - false: To disable
 *
 * Implements : FTM_HAL_SetMinLoadingCmd_Activity
 */
static inline void FTM_HAL_SetMinLoadingCmd(FTM_Type * const ftmBase,
                                            bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_CNTMIN_MASK, FTM_SYNC_CNTMIN(enable));
}

/*!
 * @brief Enables the FTM peripheral timer channel modified combine mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel pair outputs modified combine
 *                   - true : To enable modified combine
 *                   - false: To disable modified combine
 *
 * Implements : FTM_HAL_SetDualChnMofCombineCmd_Activity
 */
static inline void FTM_HAL_SetDualChnMofCombineCmd(FTM_Type * const ftmBase,
                                                   uint8_t chnlPairNum,
                                                   bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->COMBINE) |= ((uint32_t)FTM_COMBINE_MCOMBINE0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~((uint32_t)FTM_COMBINE_MCOMBINE0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables the FTM peripheral timer channel pair fault control.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel pair fault control
 *                   - true : To enable fault control
 *                   - false: To disable
 *
 * Implements : FTM_HAL_SetDualChnFaultCmd_Activity
 */
static inline void FTM_HAL_SetDualChnFaultCmd(FTM_Type * const ftmBase,
                                              uint8_t chnlPairNum,
                                              bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->COMBINE) |= ((uint32_t)FTM_COMBINE_FAULTEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~((uint32_t)FTM_COMBINE_FAULTEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel pair counter PWM sync.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel pair counter PWM sync
 *                   - true : To enable PWM synchronization
 *                   - false: To disable
 *
 * Implements : FTM_HAL_SetDualChnPwmSyncCmd_Activity
 */
static inline void FTM_HAL_SetDualChnPwmSyncCmd(FTM_Type * const ftmBase,
                                                uint8_t chnlPairNum,
                                                bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->COMBINE) |= ((uint32_t)FTM_COMBINE_SYNCEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~((uint32_t)FTM_COMBINE_SYNCEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables or disabled the FTM peripheral timer channel pair deadtime insertion.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel pair deadtime insertion
 *                   - true : To enable deadtime insertion
 *                   - false: To disable
 *
 * Implements : FTM_HAL_SetDualChnDeadtimeCmd_Activity
 */
static inline void FTM_HAL_SetDualChnDeadtimeCmd(FTM_Type * const ftmBase,
                                                 uint8_t chnlPairNum,
                                                 bool enable)
{
    DEV_ASSERT(chnlPairNum < FEATURE_FTM_CHANNEL_COUNT);

    if (enable)
    {
        ((ftmBase)->COMBINE) |= ((uint32_t)FTM_COMBINE_DTEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~((uint32_t)FTM_COMBINE_DTEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel dual edge capture.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel dual edge capture
 *                   - true : To enable dual edge capture mode
 *                   - false: To disable
 *
 * Implements : FTM_HAL_SetDualChnDecapCmd_Activity
 */
static inline void FTM_HAL_SetDualChnDecapCmd(FTM_Type * const ftmBase,
                                              uint8_t chnlPairNum,
                                              bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->COMBINE) |= ((uint32_t)FTM_COMBINE_DECAP0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~((uint32_t)FTM_COMBINE_DECAP0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables the FTM peripheral timer dual edge capture mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of dual edge capture mode
 *                   - true : To enable dual edge capture
 *                   - false: To disable
 *
 * Implements : FTM_HAL_SetDualEdgeCaptureCmd_Activity
 */
static inline void FTM_HAL_SetDualEdgeCaptureCmd(FTM_Type * const ftmBase,
                                                 uint8_t chnlPairNum,
                                                 bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->COMBINE) |= ((uint32_t)FTM_COMBINE_DECAPEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~((uint32_t)FTM_COMBINE_DECAPEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables the FTM peripheral timer dual edge capture mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 *
 * @return Dual edge capture mode status
 *         - true : To enable dual edge capture
 *         - false: To disable
 *
 * Implements : FTM_HAL_GetDualEdgeCaptureBit_Activity
 */
static inline bool FTM_HAL_GetDualEdgeCaptureBit(const FTM_Type * ftmBase,
                                                 uint8_t chnlPairNum)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    return (((ftmBase)->COMBINE) & ((uint32_t)FTM_COMBINE_DECAPEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH))) != 0U;
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel pair output complement mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] polarity State of channel pair output complement mode
 *            - FTM_MAIN_INVERTED : The channel (n+1) output is the complement of the channel (n) output
 *            - FTM_MAIN_DUPLICATED: The channel (n+1) output is the same as the channel (n) output
 *
 * Implements : FTM_HAL_SetDualChnCompCmd_Activity
 */
static inline void FTM_HAL_SetDualChnCompCmd(FTM_Type * const ftmBase,
                                             uint8_t chnlPairNum,
                                             ftm_second_channel_polarity_t polarity)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (polarity == FTM_MAIN_INVERTED)
    {
        ((ftmBase)->COMBINE) |= (FTM_COMBINE_COMP0_MASK << ((uint32_t)(chnlPairNum) * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~(FTM_COMBINE_COMP0_MASK << ((uint32_t)(chnlPairNum) * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel pair output combine mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel pair output combine mode
 *                   - true : Channels pair are combined
 *                   - false: Channels pair are independent
 *
 * Implements : FTM_HAL_SetDualChnCombineCmd_Activity
 */
static inline void FTM_HAL_SetDualChnCombineCmd(FTM_Type * const ftmBase,
                                                uint8_t chnlPairNum,
                                                bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->COMBINE) |= (FTM_COMBINE_COMBINE0_MASK << ((uint32_t)(chnlPairNum) * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~(FTM_COMBINE_COMBINE0_MASK << ((uint32_t)(chnlPairNum) * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Verify if an channels pair is used in combine mode or not.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 *
 * @return Channel pair output combine mode status
 *         - true : Channels pair are combined
 *         - false: Channels pair are independent
 *
 * Implements : FTM_HAL_GetDualChnCombineCmd_Activity
 */
static inline bool FTM_HAL_GetDualChnCombineCmd(const FTM_Type * ftmBase,
                                                uint8_t chnlPairNum)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    return (((ftmBase)->COMBINE) & (FTM_COMBINE_COMBINE0_MASK << ((uint32_t)(chnlPairNum) * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH))) != 0U;
}

/*!
 * @brief Verify if an channels pair is used in the modified combine mode or not.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 *
 * @return Channel pair output combine mode status
 *         - true : Channels pair are combined
 *         - false: Channels pair are independent
 *
 * Implements : FTM_HAL_GetDualChnMofCombineCmd_Activity
 */
static inline bool FTM_HAL_GetDualChnMofCombineCmd(const FTM_Type * ftmBase,
                                                   uint8_t chnlPairNum)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    return (((ftmBase)->COMBINE) & (FTM_COMBINE_MCOMBINE0_MASK << ((uint32_t)(chnlPairNum) * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH))) != 0U;
}
/*!
 * @brief Sets the FTM extended dead-time value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value The FTM peripheral extend pre-scale divider
 *
 * Implements : FTM_HAL_SetExtDeadtimeValue_Activity
 */
static inline void FTM_HAL_SetExtDeadtimeValue(FTM_Type * const ftmBase,
                                               uint8_t value)
{
    DEV_ASSERT(value < 16U);

    FTM_RMW_DEADTIME(ftmBase, FTM_DEADTIME_DTVALEX_MASK, FTM_DEADTIME_DTVALEX(value));
}

/*!
 * @brief Sets the FTM dead time divider.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] divider The FTM peripheral pre-scaler divider
 *                    - FTM_DEADTIME_DIVID_BY_1 : Divide by 1
 *                    - FTM_DEADTIME_DIVID_BY_4 : Divide by 4
 *                    - FTM_DEADTIME_DIVID_BY_16: Divide by 16
 *
 * Implements : FTM_HAL_SetDeadtimePrescale_Activity
 */
static inline void FTM_HAL_SetDeadtimePrescale(FTM_Type * const ftmBase,
                                               ftm_deadtime_ps_t divider)
{
    FTM_RMW_DEADTIME(ftmBase, FTM_DEADTIME_DTPS_MASK, FTM_DEADTIME_DTPS((uint8_t)divider));
}

/*!
 * @brief Sets the FTM deadtime value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] count The FTM peripheral pre-scaler divider
 *                  - 0U : no counts inserted
 *                  - 1U : 1 count is inserted
 *                  - 2U : 2 count is inserted
 *                  - ... up to a possible 63 counts
 *
 * Implements : FTM_HAL_SetDeadtimeCount_Activity
 */
static inline void FTM_HAL_SetDeadtimeCount(FTM_Type * const ftmBase,
                                            uint8_t count)
{
    DEV_ASSERT(count < 64U);

    FTM_RMW_DEADTIME(ftmBase, FTM_DEADTIME_DTVAL_MASK, FTM_DEADTIME_DTVAL(count));
}

/*FTM external trigger */
/*!
 * @brief Enables or disables the generation of the trigger when the FTM counter is equal
 * to the CNTIN register.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of initialization trigger
 *                   - true : To enable
 *                   - false: To disable
 *
 * Implements : FTM_HAL_SetInitTriggerCmd_Activity
 */
static inline void FTM_HAL_SetInitTriggerCmd(FTM_Type * const ftmBase,
                                             bool enable)
{
    ftmBase->EXTTRIG = (ftmBase->EXTTRIG & ~FTM_EXTTRIG_INITTRIGEN_MASK) | FTM_EXTTRIG_INITTRIGEN(enable);
}

/*!
 * @brief Checks whether any channel trigger event has occurred.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @return Channel trigger status
 *         - true : If there is a channel trigger event
 *         - false: If not.
 *
 * Implements : FTM_HAL_IsChnTriggerGenerated_Activity
 */
static inline bool FTM_HAL_IsChnTriggerGenerated(const FTM_Type * ftmBase)
{
    return (ftmBase->EXTTRIG & FTM_EXTTRIG_TRIGF_MASK) != 0U;
}

/*!
 * @brief Clear the channel trigger flag.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * Implements : FTM_HAL_ClearChnTriggerFlag_Activity
 */
static inline void FTM_HAL_ClearChnTriggerFlag(FTM_Type * const ftmBase)
{
    FTM_RMW_EXTTRIG_REG(ftmBase, FTM_EXTTRIG_TRIGF_MASK, FTM_EXTTRIG_TRIGF(0UL));
}

/*Fault mode status*/
/*!
 * @brief Gets the FTM detected fault input.
 *
 * This function reads the status for all fault inputs
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The fault byte
 *         - 0 : No fault condition was detected.
 *         - 1 : A fault condition was detected.
 *
 * Implements : FTM_HAL_GetDetectedFaultInput_Activity
 */
static inline bool FTM_HAL_GetDetectedFaultInput(const FTM_Type * ftmBase)
{
    return (ftmBase->FMS & FTM_FMS_FAULTF_MASK) != 0U;
}

/*!
 * @brief Checks whether the write protection is enabled.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return Write-protection status
 *         - true : If enabled
 *         - false: If not
 *
 * Implements : FTM_HAL_IsWriteProtectionEnabled_Activity
 */
static inline bool FTM_HAL_IsWriteProtectionEnabled(const FTM_Type * ftmBase)
{
    return (ftmBase->FMS & FTM_FMS_WPEN_MASK) != 0U;
}

/*!
 * @brief Checks whether the logic OR of the fault inputs is enabled.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return the enabled fault inputs status
 *         - true : The logic OR of the enabled fault inputs is 1
 *         - false: The logic OR of the enabled fault inputs is 0
 *
 * Implements : FTM_HAL_IsFaultInputEnabled_Activity
 */
static inline bool FTM_HAL_IsFaultInputEnabled(const FTM_Type * ftmBase)
{
    return (ftmBase->FMS & FTM_FMS_FAULTIN_MASK) != 0U;
}

/*!
 * @brief Checks whether a fault condition is detected at the fault input.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel
 *
 * @return the fault condition status
 *         - true : A fault condition was detected at the fault input
 *         - false: No fault condition was detected at the fault input
 *
 * Implements : FTM_HAL_IsFaultFlagDetected_Activity
 */
static inline bool FTM_HAL_IsFaultFlagDetected(const FTM_Type * ftmBase,
                                               uint8_t channel)
{
    DEV_ASSERT(channel < CHAN4_IDX);

    return (ftmBase->FMS & (FTM_FMS_FAULTF0_MASK << channel)) != 0U;
}

/*!
 * @brief Clear a fault condition is detected at the fault input.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel
 *
 * Implements : FTM_HAL_ClearFaultFlagDetected_Activity
 */
static inline void FTM_HAL_ClearFaultFlagDetected(FTM_Type * const ftmBase,
                                                  uint8_t channel)
{
    DEV_ASSERT(channel < CHAN4_IDX);

    ((ftmBase)->FMS) &= (~(1UL << channel));
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->FMS;
#endif
}

/* Quadrature decoder control */
/*!
 * @brief Enables the channel quadrature decoder.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of channel quadrature decoder
 *                  - true : To enable
 *                  - false: To disable
 *
 * Implements : FTM_HAL_SetQuadDecoderCmd_Activity
 */
static inline void FTM_HAL_SetQuadDecoderCmd(FTM_Type * const ftmBase,
                                             bool enable)
{
    if (enable)
    {
        ((ftmBase)->QDCTRL) |= (1UL << FTM_QDCTRL_QUADEN_SHIFT);
    }
    else
    {
        ((ftmBase)->QDCTRL) &= ~(1UL << FTM_QDCTRL_QUADEN_SHIFT);
    }
}

/*!
 * @brief Enables or disables the phase A input filter.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of phase A input filter
 *                   - true : Enables the phase input filter
 *                   - false: Disables the filter
 *
 * Implements : FTM_HAL_SetQuadPhaseAFilterCmd_Activity
 */
static inline void FTM_HAL_SetQuadPhaseAFilterCmd(FTM_Type * const ftmBase,
                                                  bool enable)
{
    if (enable)
    {
        ((ftmBase)->QDCTRL) |= (1UL << FTM_QDCTRL_PHAFLTREN_SHIFT);
    }
    else
    {
        ((ftmBase)->QDCTRL) &= ~(1UL << FTM_QDCTRL_PHAFLTREN_SHIFT);
    }
}

/*!
 * @brief Enables or disables the phase B input filter.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of phase B input filter
 *                   - true : Enables the phase input filter
 *                   - false: Disables the filter
 *
 * Implements : FTM_HAL_SetQuadPhaseBFilterCmd_Activity
 */
static inline void FTM_HAL_SetQuadPhaseBFilterCmd(FTM_Type * const ftmBase,
                                                  bool enable)
{
    if (enable)
    {
        ((ftmBase)->QDCTRL) |= (1UL << FTM_QDCTRL_PHBFLTREN_SHIFT);
    }
    else
    {
        ((ftmBase)->QDCTRL) &= ~(1UL << FTM_QDCTRL_PHBFLTREN_SHIFT);
    }
}

/*!
 * @brief Selects polarity for the quadrature decode phase A input.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode Phase A input polarity selection
 *                - FTM_QUAD_PHASE_NORMAL: Normal polarity
 *                - FTM_QUAD_PHASE_INVERT: Inverted polarity
 *
 * Implements : FTM_HAL_SetQuadPhaseAPolarity_Activity
 */
static inline void FTM_HAL_SetQuadPhaseAPolarity(FTM_Type * const ftmBase,
                                                 ftm_quad_phase_polarity_t mode)
{
    FTM_RMW_QDCTRL(ftmBase, FTM_QDCTRL_PHAPOL_MASK, FTM_QDCTRL_PHAPOL(mode));
}

/*!
 * @brief Selects polarity for the quadrature decode phase B input.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode Phase B input polarity selection
 *                - FTM_QUAD_PHASE_NORMAL: Normal polarity
 *                - FTM_QUAD_PHASE_INVERT: Inverted polarity
 *
 * Implements : FTM_HAL_SetQuadPhaseBPolarity_Activity
 */
static inline void FTM_HAL_SetQuadPhaseBPolarity(FTM_Type * const ftmBase,
                                                 ftm_quad_phase_polarity_t mode)
{
    FTM_RMW_QDCTRL(ftmBase, FTM_QDCTRL_PHBPOL_MASK, FTM_QDCTRL_PHBPOL(mode));
}

/*!
 * @brief Sets the encoding mode used in quadrature decoding mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] quadMode Quadrature decoder mode selection
 *                     - FTM_QUAD_PHASE_ENCODE: Phase A and Phase B encoding mode
 *                     - FTM_QUAD_COUNT_AND_DIR: Count and direction encoding mode
 *
 * Implements : FTM_HAL_SetQuadMode_Activity
 */
static inline void FTM_HAL_SetQuadMode(FTM_Type * const ftmBase,
                                       ftm_quad_decode_mode_t quadMode)
{
    FTM_RMW_QDCTRL(ftmBase, FTM_QDCTRL_QUADMODE_MASK, FTM_QDCTRL_QUADMODE(quadMode));
}

/*!
 * @brief Gets the FTM counter direction in quadrature mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The counting direction
 *         - 1U: if counting direction is increasing
 *         - 0U: if counting direction is decreasing
 *
 * Implements : FTM_HAL_GetQuadDir_Activity
 */
static inline bool FTM_HAL_GetQuadDir(const FTM_Type * ftmBase)
{
    return (ftmBase->QDCTRL & FTM_QDCTRL_QUADIR_MASK) != 0U;
}

/*!
 * @brief Gets the Timer overflow direction in quadrature mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The timer overflow direction
 *         - 1U: if TOF bit was set on the top of counting
 *         - 0U: if TOF bit was set on the bottom of counting
 *
 * Implements : FTM_HAL_GetQuadTimerOverflowDir_Activity
 */
static inline bool FTM_HAL_GetQuadTimerOverflowDir(const FTM_Type * ftmBase)
{
    return (ftmBase->QDCTRL & FTM_QDCTRL_TOFDIR_MASK) != 0U;
}

/*!
 * @brief Sets the fault input filter value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value Fault input filter value
 *
 * Implements : FTM_HAL_SetFaultInputFilterVal_Activity
 */
static inline void FTM_HAL_SetFaultInputFilterVal(FTM_Type * const ftmBase,
                                                  uint32_t value)
{
    FTM_RMW_FLTCTRL(ftmBase, FTM_FLTCTRL_FFVAL_MASK, FTM_FLTCTRL_FFVAL(value));
}

/*!
 * @brief Enables or disables the fault input filter.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] inputNum Fault input to be configured, valid value 0, 1, 2, 3
 * @param[in] enable State of fault input filter
 *                   - true : To enable fault input filter
 *                   - false: To disable fault input filter
 *
 * Implements : FTM_HAL_SetFaultInputFilterCmd_Activity
 */
static inline void FTM_HAL_SetFaultInputFilterCmd(FTM_Type * const ftmBase,
                                                  uint8_t inputNum,
                                                  bool enable)
{
    DEV_ASSERT(inputNum < CHAN4_IDX);

    if (enable)
    {
        ((ftmBase)->FLTCTRL) |=  (1UL << (inputNum + FTM_FLTCTRL_FFLTR0EN_SHIFT));
    }
    else
    {
        ((ftmBase)->FLTCTRL) &=  ~(1UL << (inputNum + FTM_FLTCTRL_FFLTR0EN_SHIFT));
    }
}

/*!
 * @brief Clears the entire content value of the Fault control register.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * Implements : FTM_HAL_ClearFaultControl_Activity
 */
static inline void FTM_HAL_ClearFaultControl(FTM_Type * const ftmBase)
{
    ((ftmBase)->FLTCTRL) =  0U;
}

/*!
 * @brief Enables or disables the fault input.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] inputNum fault input to be configured, valid value 0, 1, 2, 3
 * @param[in] enable State of fault input
 *                   - true : To enable fault input
 *                   - false: To disable fault input
 *
 * Implements : FTM_HAL_SetFaultInputCmd_Activity
 */
static inline void FTM_HAL_SetFaultInputCmd(FTM_Type * const ftmBase,
                                            uint8_t inputNum,
                                            bool enable)
{
    DEV_ASSERT(inputNum < CHAN4_IDX);

    if (enable)
    {
        ((ftmBase)->FLTCTRL) |=  (1UL << inputNum);
    }
    else
    {
        ((ftmBase)->FLTCTRL) &=  ~(1UL << inputNum);
    }
}

/*!
 * @brief Configures the behavior of the PWM outputs when a fault is detected
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of fault output
 *                   - true : Output pins are set tri-state,
 *                   - false: Pins are set to a safe state determined by POL bits
 *
 * Implements : FTM_HAL_SetPwmFaultBehavior_Activity
 */
static inline void FTM_HAL_SetPwmFaultBehavior(FTM_Type * const ftmBase,
                                               bool enable)
{
    if (enable)
    {
        ((ftmBase)->FLTCTRL) |=  (1UL << FTM_FLTCTRL_FSTATE_SHIFT);
    }
    else
    {
        ((ftmBase)->FLTCTRL) &=  ~(1UL << FTM_FLTCTRL_FSTATE_SHIFT);
    }
}

/*!
 * @brief Enables or disables the channel invert for a channel pair.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel invert for a channel pair
 *                   - true : To enable channel inverting
 *                   - false: To disable channel inversion
 *
 * Implements : FTM_HAL_SetDualChnInvertCmd_Activity
 */
static inline void FTM_HAL_SetDualChnInvertCmd(FTM_Type * const ftmBase,
                                               uint8_t chnlPairNum,
                                               bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->INVCTRL) |=  (1UL << chnlPairNum);
    }
    else
    {
        ((ftmBase)->INVCTRL) &=  ~(1UL << chnlPairNum);
    }
}

/*!
 * @brief Writes the provided value to the Inverting control register.
 *
 * This function is enable/disable inverting control on multiple channel pairs.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] regVal Value to be written to the register
 *
 * Implements : FTM_HAL_SetInvctrlReg_Activity
 */
static inline void FTM_HAL_SetInvctrlReg(FTM_Type * const ftmBase,
                                         uint32_t regVal)
{
    ((ftmBase)->INVCTRL) = regVal;
}

/*FTM software output control*/
/*!
 * @brief Enables or disables the channel software output control.
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel Channel to be enabled or disabled
 * @param[in] enable State of channel software output control
 *                   - true : To enable, channel output will be affected by software output control
 *                   - false: To disable, channel output is unaffected
 *
 * Implements : FTM_HAL_SetChnSoftwareCtrlCmd_Activity
 */
static inline void FTM_HAL_SetChnSoftwareCtrlCmd(FTM_Type * const ftmBase,
                                                 uint8_t channel,
                                                 bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    if (enable)
    {
        ((ftmBase)->SWOCTRL) |=  (1UL << channel);
    }
    else
    {
        ((ftmBase)->SWOCTRL) &=  ~(1UL << channel);
    }
}

/*FTM software output control*/
/*!
 * @brief Enables or disables the channel software output control.The
 * main difference between this function and FTM_HAL_SetChnSoftwareCtrlCmd
 * is that this can configure all the channels in the same time.
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channelsMask Channels to be enabled or disabled
 *
 * Implements : FTM_HAL_SetAllChnSoftwareCtrlCmd_Activity
 */
static inline void FTM_HAL_SetAllChnSoftwareCtrlCmd(FTM_Type * const ftmBase,
                                                    uint8_t channelsMask)
{
    uint32_t mask = FTM_SWOCTRL_CH0OC_MASK | FTM_SWOCTRL_CH1OC_MASK | FTM_SWOCTRL_CH2OC_MASK |
                    FTM_SWOCTRL_CH3OC_MASK | FTM_SWOCTRL_CH4OC_MASK | FTM_SWOCTRL_CH5OC_MASK |
                    FTM_SWOCTRL_CH6OC_MASK | FTM_SWOCTRL_CH7OC_MASK;
    ((ftmBase)->SWOCTRL) = (((ftmBase)->SWOCTRL) & (~(mask))) | channelsMask;
}

/*!
 * @brief Sets the channel software output control value.
 *
 * @param[in] ftmBase The FTM base address pointer.
 * @param[in] channel Channel to be configured
 * @param[in] enable State of software output control value
 *                   - true : to force 1 to the channel output
 *                   - false: to force 0 to the channel output
 *
 * Implements : FTM_HAL_SetChnSoftwareCtrlVal_Activity
 */
static inline void FTM_HAL_SetChnSoftwareCtrlVal(FTM_Type * const ftmBase,
                                                 uint8_t channel,
                                                 bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    if (enable)
    {
        ((ftmBase)->SWOCTRL) |=  (1UL << (channel + FTM_SWOCTRL_CH0OCV_SHIFT));
    }
    else
    {
        ((ftmBase)->SWOCTRL) &=  ~(1UL << (channel + FTM_SWOCTRL_CH0OCV_SHIFT));
    }
}

/*!
 * @brief Sets the channel software output control value.
 *
 * @param[in] ftmBase The FTM base address pointer.
 * @param[in] channelsValues The values which will overwrite the output channels
 *
 * Implements : FTM_HAL_SetAllChnSoftwareCtrlVal_Activity
 */
static inline void FTM_HAL_SetAllChnSoftwareCtrlVal(FTM_Type * const ftmBase,
                                                    uint8_t channelsValues)
{
    uint32_t mask = FTM_SWOCTRL_CH0OCV_MASK | FTM_SWOCTRL_CH1OCV_MASK | FTM_SWOCTRL_CH2OCV_MASK |
                        FTM_SWOCTRL_CH3OCV_MASK | FTM_SWOCTRL_CH4OCV_MASK | FTM_SWOCTRL_CH5OCV_MASK |
                        FTM_SWOCTRL_CH6OCV_MASK | FTM_SWOCTRL_CH7OCV_MASK;
   ((ftmBase)->SWOCTRL) = (((ftmBase)->SWOCTRL) & (~(mask))) | ((uint32_t)channelsValues << FTM_SWOCTRL_CH0OCV_SHIFT);
}

/*FTM PWM load control*/
/*!
 * @brief Set the global load mechanism.
 *
 * @param[in] ftmBase The FTM base address pointer
 *                   - true : LDOK bit is set
 *                   - false: No action
 *
 * Implements : FTM_HAL_SetGlobalLoadCmd_Activity
 */
static inline void FTM_HAL_SetGlobalLoadCmd(FTM_Type * const ftmBase)
{
    ((ftmBase)->PWMLOAD) |=  (1UL << FTM_PWMLOAD_GLDOK_SHIFT);
}

/*!
 * @brief Enable the global load.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of the global load mechanism
 *                   - true : Global Load OK enabled
 *                   - false: Global Load OK disabled
 *
 * Implements : FTM_HAL_SetLoadCmd_Activity
 */
static inline void FTM_HAL_SetLoadCmd(FTM_Type * const ftmBase,
                                      bool enable)
{
    if (enable)
    {
        ((ftmBase)->PWMLOAD) |=  (1UL << FTM_PWMLOAD_GLEN_SHIFT);
    }
    else
    {
        ((ftmBase)->PWMLOAD) &=  ~(1UL << FTM_PWMLOAD_GLEN_SHIFT);
    }
}

/*!
 * @brief Enable the half cycle reload.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of the half cycle match as a reload opportunity
 *                   - true : Half cycle reload is enabled
 *                   - false: Half cycle reload is disabled
 *
 * Implements : FTM_HAL_SetHalfCycleCmd_Activity
 */
static inline void FTM_HAL_SetHalfCycleCmd(FTM_Type * const ftmBase,
                                           bool enable)
{
    if (enable)
    {
        ((ftmBase)->PWMLOAD) |=  (1UL << FTM_PWMLOAD_HCSEL_SHIFT);
    }
    else
    {
        ((ftmBase)->PWMLOAD) &=  ~(1UL << FTM_PWMLOAD_HCSEL_SHIFT);
    }
}

/*!
 * @brief Enables or disables the loading of MOD, CNTIN and CV with values of their write buffer.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of loading updated values
 *                   - true : To enable
 *                   - false: To disable
 *
 * Implements : FTM_HAL_SetPwmLoadCmd_Activity
 */
static inline void FTM_HAL_SetPwmLoadCmd(FTM_Type * const ftmBase,
                                         bool enable)
{
    if (enable)
    {
        ((ftmBase)->PWMLOAD) |=  (1UL << FTM_PWMLOAD_LDOK_SHIFT);
    }
    else
    {
        ((ftmBase)->PWMLOAD) &=  ~(1UL << FTM_PWMLOAD_LDOK_SHIFT);
    }
}

/*!
 * @brief Includes or excludes the channel in the matching process.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel Channel to be configured
 * @param[in] enable State of channel
 *                - true : means include the channel in the matching process
 *                - false: means do not include channel in the matching process
 *
 * Implements : FTM_HAL_SetPwmLoadChnSelCmd_Activity
 */
static inline void FTM_HAL_SetPwmLoadChnSelCmd(FTM_Type * const ftmBase,
                                               uint8_t channel,
                                               bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    if (enable)
    {
        ((ftmBase)->PWMLOAD) |=  (1UL << channel);
    }
    else
    {
        ((ftmBase)->PWMLOAD) &=  ~(1UL << channel);
    }
}

/*FTM configuration*/
/*!
 * @brief Enables or disables the FTM initialization trigger on Reload Point.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable bit controls whether an initialization trigger is generated
 *                   - true : Trigger is generated when a reload point is reached
 *                   - false: Trigger is generated on counter wrap events
 *
 * Implements : FTM_HAL_SetInitTrigOnReloadCmd_Activity
 */
static inline void FTM_HAL_SetInitTrigOnReloadCmd(FTM_Type * const ftmBase,
                                                  bool enable)
{
    ftmBase->CONF = (ftmBase->CONF & ~FTM_CONF_ITRIGR_MASK) | FTM_CONF_ITRIGR(enable);
}

/*!
 * @brief Enables or disables the FTM global time base signal generation to other FTM's.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of global time base signal
 *                   - true : To enable
 *                   - false: To disable
 *
 * Implements : FTM_HAL_SetGlobalTimeBaseOutputCmd_Activity
 */
static inline void FTM_HAL_SetGlobalTimeBaseOutputCmd(FTM_Type * const ftmBase,
                                                      bool enable)
{
    ftmBase->CONF = (ftmBase->CONF & ~FTM_CONF_GTBEOUT_MASK) | FTM_CONF_GTBEOUT(enable);
}

/*!
 * @brief Enables or disables the FTM timer global time base.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of global time base
 *                   - true : To enable
 *                   - false: To disable
 *
 * Implements : FTM_HAL_SetGlobalTimeBaseCmd_Activity
 */
static inline void FTM_HAL_SetGlobalTimeBaseCmd(FTM_Type * const ftmBase,
                                                bool enable)
{
    ftmBase->CONF = (ftmBase->CONF & ~FTM_CONF_GTBEEN_MASK) | FTM_CONF_GTBEEN(enable);
}

/*!
 * @brief Sets the BDM mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] val The FTM behavior in BDM mode
 *                - FTM_BDM_MODE_00: FTM counter stopped, CH(n)F bit can be set, FTM channels
 *                                   in functional mode, writes to MOD,CNTIN and C(n)V registers bypass
 *                                   the register buffers
 *                - FTM_BDM_MODE_01: FTM counter stopped, CH(n)F bit is not set, FTM channels
 *                                   outputs are forced to their safe value , writes to MOD,CNTIN and
 *                                   C(n)V registers bypass the register buffers
 *                - FTM_BDM_MODE_10: FTM counter stopped, CH(n)F bit is not set, FTM channels
 *                                   outputs are frozen when chip enters in BDM mode, writes to MOD,
 *                                   CNTIN and C(n)V registers bypass the register buffers
 *                - FTM_BDM_MODE_11: FTM counter in functional mode, CH(n)F bit can be set,
 *                                   FTM channels in functional mode, writes to MOD,CNTIN and C(n)V
 *                                   registers is in fully functional mode
 *
 * Implements : FTM_HAL_SetBdmMode_Activity
 */
static inline void FTM_HAL_SetBdmMode(FTM_Type * const ftmBase,
                                      ftm_bdm_mode_t val)
{
    FTM_RMW_CONF(ftmBase, FTM_CONF_BDMMODE_MASK, FTM_CONF_BDMMODE(val));
}

/*!
 * @brief Sets the FTM timer TOF Frequency
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] val Value of the TOF bit set frequency
 *
 * Implements : FTM_HAL_SetLoadFreq_Activity
 */
static inline void FTM_HAL_SetLoadFreq(FTM_Type * const ftmBase,
                                       uint8_t val)
{
    FTM_RMW_CONF(ftmBase, FTM_CONF_LDFQ_MASK, FTM_CONF_LDFQ(val));
}

/*FTM Synchronization configuration*/
/*!
 * @brief Sets the sync mode for the FTM SWOCTRL register when using a hardware trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of software output control synchronization
 *                   - true : The hardware trigger activates SWOCTRL register sync
 *                   - false: The hardware trigger does not activate SWOCTRL register sync
 *
 * Implements : FTM_HAL_SetSwoctrlHardwareSyncModeCmd_Activity
 */
static inline void FTM_HAL_SetSwoctrlHardwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_HWSOC_MASK) | FTM_SYNCONF_HWSOC(enable);
}

/*!
 * @brief Sets sync mode for FTM INVCTRL register when using a hardware trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of inverting control synchronization
 *                   - true : The hardware trigger activates INVCTRL register sync
 *                   - false: The hardware trigger does not activate INVCTRL register sync
 *
 * Implements : FTM_HAL_SetInvctrlHardwareSyncModeCmd_Activity
 */
static inline void FTM_HAL_SetInvctrlHardwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_HWINVC_MASK) | FTM_SYNCONF_HWINVC(enable);
}

/*!
 * @brief Sets sync mode for FTM OUTMASK register when using a hardware trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of output mask synchronization
 *                   - true : The hardware trigger activates OUTMASK register sync
 *                   - false: The hardware trigger does not activate OUTMASK register sync
 *
 * Implements : FTM_HAL_SetOutmaskHardwareSyncModeCmd_Activity
 */
static inline void FTM_HAL_SetOutmaskHardwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_HWOM_MASK) | FTM_SYNCONF_HWOM(enable);
}

/*!
 * @brief Sets sync mode for FTM MOD, CNTIN and CV registers when using a hardware trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of registers synchronization
 *                   - true : The hardware trigger activates  MOD, HCR, CNTIN, and CV registers sync
 *                   - false: The hardware trigger does not activate MOD, HCR, CNTIN, and CV registers sync
 *
 * Implements : FTM_HAL_SetModCntinCvHardwareSyncModeCmd_Activity
 */
static inline void FTM_HAL_SetModCntinCvHardwareSyncModeCmd(FTM_Type * const ftmBase,
                                                            bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_HWWRBUF_MASK) | FTM_SYNCONF_HWWRBUF(enable);
}

/*!
 * @brief Sets sync mode for FTM counter register when using a hardware trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of FTM counter synchronization
 *                   - true : The hardware trigger activates FTM counter sync
 *                   - false: The hardware trigger does not activate FTM counter sync
 *
 * Implements : FTM_HAL_SetCounterHardwareSyncModeCmd_Activity
 */
static inline void FTM_HAL_SetCounterHardwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_HWRSTCNT_MASK) | FTM_SYNCONF_HWRSTCNT(enable);
}

/*!
 * @brief Sets sync mode for FTM SWOCTRL register when using a software trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of software output control synchronization
 *                   - true : The software trigger activates SWOCTRL register sync
 *                   - false: The software trigger does not activate SWOCTRL register sync
 *
 * Implements : FTM_HAL_SetSwoctrlSoftwareSyncModeCmd_Activity
 */
static inline void FTM_HAL_SetSwoctrlSoftwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SWSOC_MASK) | FTM_SYNCONF_SWSOC(enable);
}

/*!
 * @brief Sets sync mode for FTM INVCTRL register when using a software trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of State of inverting control synchronization
 *                   - true : The software trigger activates INVCTRL register sync
 *                   - false: The software trigger does not activate INVCTRL register sync
 *
 * Implements : FTM_HAL_SetInvctrlSoftwareSyncModeCmd_Activity
 */
static inline void FTM_HAL_SetInvctrlSoftwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SWINVC_MASK) | FTM_SYNCONF_SWINVC(enable);
}

/*!
 * @brief Sets sync mode for FTM OUTMASK register when using a software trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of output mask synchronization
 *                   - true : The software trigger activates OUTMASK register sync
 *                   - false: The software trigger does not activate OUTMASK register sync
 *
 * Implements : FTM_HAL_SetOutmaskSoftwareSyncModeCmd_Activity
 */
static inline void FTM_HAL_SetOutmaskSoftwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SWOM_MASK) | FTM_SYNCONF_SWOM(enable);
}

/*!
 * @brief Sets sync mode for FTM MOD, CNTIN and CV registers when using a software trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of registers synchronization
 *                   - true : The software trigger activates FTM MOD, CNTIN and CV registers sync
 *                   - false: The software trigger does not activate FTM MOD, CNTIN and CV registers sync
 *
 * Implements : FTM_HAL_SetModCntinCvSoftwareSyncModeCmd_Activity
 */
static inline void FTM_HAL_SetModCntinCvSoftwareSyncModeCmd(FTM_Type * const ftmBase,
                                                            bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SWWRBUF_MASK) | FTM_SYNCONF_SWWRBUF(enable);
}

/*!
 * @brief Sets hardware trigger mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of hardware trigger mode
 *                   - true : FTM does not clear the TRIGx bit when the hardware trigger j is detected
 *                   - false: FTM clears the TRIGx bit when the hardware trigger j is detected
 *
 * Implements : FTM_HAL_SetHwTriggerSyncModeCmd_Activity
 */
static inline void FTM_HAL_SetHwTriggerSyncModeCmd(FTM_Type * const ftmBase,
                                                   bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_HWTRIGMODE_MASK) | FTM_SYNCONF_HWTRIGMODE(enable);
}

/*!
 * @brief Sets sync mode for FTM counter register when using a software trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] update_mode State of FTM counter synchronization
 *                   - true : The software trigger activates FTM counter sync
 *                   - false: The software trigger does not activate FTM counter sync
 *
 * Implements : FTM_HAL_SetCounterSoftwareSyncModeCmd_Activity
 */
static inline void FTM_HAL_SetCounterSoftwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         ftm_pwm_sync_mode_t update_mode)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SWRSTCNT_MASK) | FTM_SYNCONF_SWRSTCNT(update_mode);
}

/*!
 * @brief Sets the PWM synchronization mode to enhanced or legacy.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode State of synchronization mode
 *                   - true : Enhanced PWM synchronization is selected
 *                   - false: Legacy PWM synchronization is selected
 *
 * Implements : FTM_HAL_SetPwmSyncModeCmd_Activity
 */
static inline void FTM_HAL_SetPwmSyncModeCmd(FTM_Type * const ftmBase,
                                             bool mode)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SYNCMODE_MASK) | FTM_SYNCONF_SYNCMODE(mode);
}

/*!
 * @brief Sets the SWOCTRL register PWM synchronization mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode State of register synchronization
 *                   - true : SWOCTRL register is updated by PWM sync
 *                   - false: SWOCTRL register is updated at all rising edges of system clock
 *
 * Implements : FTM_HAL_SetSwoctrlPwmSyncModeCmd_Activity
 */
static inline void FTM_HAL_SetSwoctrlPwmSyncModeCmd(FTM_Type * const ftmBase,
                                                    ftm_reg_update_t mode)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SWOC_MASK) | FTM_SYNCONF_SWOC(mode);
}

/*!
 * @brief Sets the INVCTRL register PWM synchronization mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode State of register synchronization
 *                   - true : INVCTRL register is updated by PWM sync
 *                   - false: INVCTRL register is updated at all rising edges of system clock
 *
 * Implements : FTM_HAL_SetInvctrlPwmSyncModeCmd_Activity
 */
static inline void FTM_HAL_SetInvctrlPwmSyncModeCmd(FTM_Type * const ftmBase,
                                                    ftm_reg_update_t mode)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_INVC_MASK) | FTM_SYNCONF_INVC(mode);
}

/*!
 * @brief Sets the CNTIN register PWM synchronization mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode State of register synchronization
 *                   - true : CNTIN register is updated by PWM sync
 *                   - false: CNTIN register is updated at all rising edges of system clock
 *
 * Implements : FTM_HAL_SetCntinPwmSyncModeCmd_Activity
 */
static inline void FTM_HAL_SetCntinPwmSyncModeCmd(FTM_Type * const ftmBase,
                                                  ftm_reg_update_t mode)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_CNTINC_MASK) | FTM_SYNCONF_CNTINC(mode);
}

/*!
 * @brief Sets the FTM extended dead-time value for the channel pair.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channelPair The FTM peripheral channel pair (n)
 * @param[in] value The FTM peripheral extend pre-scale divider using the concatenation with the dead-time value
 *
 * Implements : FTM_HAL_SetExtPairDeadtimeValue_Activity
 */
static inline void FTM_HAL_SetExtPairDeadtimeValue(FTM_Type * const ftmBase,
                                                   uint8_t channelPair,
                                                   uint8_t value)
{
    DEV_ASSERT(value < 16U);
    DEV_ASSERT(channelPair < CHAN4_IDX);

    switch (channelPair)
    {
        case CHAN0_IDX:
            FTM_RMW_PAIR0DEADTIME(ftmBase, FTM_PAIR0DEADTIME_DTVALEX_MASK, FTM_PAIR0DEADTIME_DTVALEX(value));
            break;
        case CHAN1_IDX:
            FTM_RMW_PAIR1DEADTIME(ftmBase, FTM_PAIR1DEADTIME_DTVALEX_MASK, FTM_PAIR1DEADTIME_DTVALEX(value));
            break;
        case CHAN2_IDX:
            FTM_RMW_PAIR2DEADTIME(ftmBase, FTM_PAIR2DEADTIME_DTVALEX_MASK, FTM_PAIR2DEADTIME_DTVALEX(value));
            break;
        case CHAN3_IDX:
            FTM_RMW_PAIR3DEADTIME(ftmBase, FTM_PAIR3DEADTIME_DTVALEX_MASK, FTM_PAIR3DEADTIME_DTVALEX(value));
            break;
        default:
            /* Nothing to do */
            break;
    }
}

/*!
 * @brief Sets the FTM dead time divider for the channel pair.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channelPair The FTM peripheral channel pair (n)
 * @param[in] divider The FTM peripheral pre-scaler divider
 *                    - FTM_DEADTIME_DIVID_BY_1 : Divide by 1
 *                    - FTM_DEADTIME_DIVID_BY_4 : Divide by 4
 *                    - FTM_DEADTIME_DIVID_BY_16: Divide by 16
 *
 * Implements : FTM_HAL_SetPairDeadtimePrescale_Activity
 */
static inline void FTM_HAL_SetPairDeadtimePrescale(FTM_Type * const ftmBase,
                                                   uint8_t channelPair,
                                                   ftm_deadtime_ps_t divider)
{
    DEV_ASSERT(channelPair < CHAN4_IDX);

    switch (channelPair)
    {
        case CHAN0_IDX:
            FTM_RMW_PAIR0DEADTIME(ftmBase, FTM_PAIR0DEADTIME_DTPS_MASK, FTM_PAIR0DEADTIME_DTPS((uint8_t)divider));
            break;
        case CHAN1_IDX:
            FTM_RMW_PAIR1DEADTIME(ftmBase, FTM_PAIR1DEADTIME_DTPS_MASK, FTM_PAIR1DEADTIME_DTPS((uint8_t)divider));
            break;
        case CHAN2_IDX:
            FTM_RMW_PAIR2DEADTIME(ftmBase, FTM_PAIR2DEADTIME_DTPS_MASK, FTM_PAIR2DEADTIME_DTPS((uint8_t)divider));
            break;
        case CHAN3_IDX:
            FTM_RMW_PAIR3DEADTIME(ftmBase, FTM_PAIR3DEADTIME_DTPS_MASK, FTM_PAIR3DEADTIME_DTPS((uint8_t)divider));
            break;
        default:
            /* Nothing to do */
            break;
    }
}

/*!
 * @brief Sets the FTM dead-time value for the channel pair.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channelPair The FTM peripheral channel pair (n)
 * @param[in] count The FTM peripheral selects the dead-time value
 *                  - 0U : no counts inserted
 *                  - 1U : 1 count is inserted
 *                  - 2U : 2 count is inserted
 *                  - ... up to a possible 63 counts
 *
 * Implements : FTM_HAL_SetPairDeadtimeCount_Activity
 */
static inline void FTM_HAL_SetPairDeadtimeCount(FTM_Type * const ftmBase,
                                                uint8_t channelPair,
                                                uint8_t count)
{
    DEV_ASSERT(channelPair < CHAN4_IDX);
    DEV_ASSERT(count < 64U);

    switch (channelPair)
    {
        case CHAN0_IDX:
            FTM_RMW_PAIR0DEADTIME(ftmBase, FTM_PAIR0DEADTIME_DTVAL_MASK, FTM_PAIR0DEADTIME_DTVAL(count));
            break;
        case CHAN1_IDX:
            FTM_RMW_PAIR1DEADTIME(ftmBase, FTM_PAIR1DEADTIME_DTVAL_MASK, FTM_PAIR1DEADTIME_DTVAL(count));
            break;
        case CHAN2_IDX:
            FTM_RMW_PAIR2DEADTIME(ftmBase, FTM_PAIR2DEADTIME_DTVAL_MASK, FTM_PAIR2DEADTIME_DTVAL(count));
            break;
        case CHAN3_IDX:
            FTM_RMW_PAIR3DEADTIME(ftmBase, FTM_PAIR3DEADTIME_DTVAL_MASK, FTM_PAIR3DEADTIME_DTVAL(count));
            break;
        default:
            /* Nothing to do */
            break;
    }
}

/*HAL functionality*/
/*!
 * @brief Resets the FTM registers. All the register use in the driver should be
 * reset to default value of each register.
 *
 * @param[in] ftmBase The FTM base address pointer
 */
void FTM_HAL_Reset(FTM_Type * const ftmBase);

/*!
 * @brief Initializes the FTM. This function will enable the flexTimer module
 * and selects one pre-scale factor for the FTM clock source.
 *
 * @param[in] ftmBase The FTM base address pointer
 */
void FTM_HAL_Init(FTM_Type * const ftmBase,
                  ftm_clock_ps_t ftmClockPrescaler);

/*!
 * @brief Enables or disables the generation of the FTM peripheral timer channel trigger when the
 * FTM counter is equal to its initial value
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] enable Enables the generation of the channel trigger
 *                   - true : The generation of the channel trigger is enabled
 *                   - false: The generation of the channel trigger is disabled
 */
void FTM_HAL_SetChnTriggerCmd(FTM_Type * const ftmBase,
                              uint8_t channel,
                              bool enable);

/*!
 * @brief Sets the FTM peripheral timer channel input capture filter value.
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number, only 0,1,2,3, channel 4, 5,6, 7 don't have
 * @param[in] value Filter value to be set
 */
void FTM_HAL_SetChnInputCaptureFilter(FTM_Type * const ftmBase,
                                      uint8_t channel,
                                      uint8_t value);

#if defined(__cplusplus)
}
#endif

/*! @}*/


#endif /* FTM_HAL_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
