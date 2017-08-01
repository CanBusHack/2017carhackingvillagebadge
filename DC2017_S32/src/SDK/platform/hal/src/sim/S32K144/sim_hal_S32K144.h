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

#if !defined(SIM_HAL_S32K144_H)
#define SIM_HAL_S32K144_H

#include "device_registers.h"
#include <stdbool.h>
#include <stddef.h>

/*!
 * @file sim_hal_S32K144.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.3, Global typedef not referenced.
 * sim_clock_config_t is referenced from clock manager.
 */

/*!
 * @ingroup sim_hal
 * @defgroup sim_hal_s32k144
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* @brief TClk clock frequency. */
#define NUMBER_OF_TCLK_INPUTS 3U
extern uint32_t g_TClkFreq[NUMBER_OF_TCLK_INPUTS];      /* TCLKx clocks */


/*!
 * @brief Internal supplies monitored by ADC_SUPPLY
 * Implements sim_adc_supply_src_t_Class
 */
typedef enum
{
    ADC_SUPPLY_VDD             = 0U,         /*!< 5V input VDD supply              */
    ADC_SUPPLY_VDDA            = 1U,         /*!< 5V input analog supply           */
    ADC_SUPPLY_VREFH           = 2U,         /*!< ADC Reference Supply             */
    ADC_SUPPLY_VDD_3V          = 3U,         /*!< 3.3V Oscillator Regulator Output */
    ADC_SUPPLY_VDD_FLASH_3V    = 4U,         /*!< 3.3V flash regulator output      */
    ADC_SUPPLY_LV              = 5U         /*!< 1.2V core regulator output        */
} sim_adc_supply_src_t;

/*!
 * @brief Debug trace clock source select
 * Implements clock_trace_src_t_Class
 */
typedef enum
{
    CLOCK_TRACE_SRC_CORE_CLK,         /*!< core clock     */
    CLOCK_TRACE_SRC_PLATFORM_CLK      /*!< platform clock  */
} clock_trace_src_t;


/*!
 * @brief PDB back-to-back select
 * Implements sim_pdb_bb_src_t_Class
 */
typedef enum
{
    PDB_BACK_TO_BACK_OPTION_0,     /*!< PDBx ch0 back-to-back operation with ADCx COCO[7:0]     */
    PDB_BACK_TO_BACK_OPTION_1      /*!< Ch0 of PDBx back-to-back operation with COCO[7:0] of ADCx  */
} sim_pdb_bb_src_t;

/*!
 * @brief SIM CLKOUT select
 * Implements sim_clkout_src_t_Class
 */
typedef enum
{
    SIM_CLKOUT_SEL_SYSTEM_SCG_CLKOUT     = 0U,  /*!< SCG CLKOUT                                   */
    SIM_CLKOUT_SEL_SYSTEM_SOSC_DIV2_CLK  = 2U,  /*!< SOSC DIV2 CLK                                */
    SIM_CLKOUT_SEL_SYSTEM_SIRC_DIV2_CLK  = 4U,  /*!< SIRC DIV2 CLK                                */
    SIM_CLKOUT_SEL_SYSTEM_FIRC_DIV2_CLK  = 6U,  /*!< FIRC DIV2 CLK                                */
    SIM_CLKOUT_SEL_SYSTEM_SPLL_DIV2_CLK  = 8U,  /*!< SPLL DIV2 CLK                                */
    SIM_CLKOUT_SEL_SYSTEM_LPO_128K_CLK   = 10U, /*!< LPO CLK 128 Khz                              */
    SIM_CLKOUT_SEL_SYSTEM_LPO_CLK        = 12U, /*!< LPO CLK as selected by SIM LPO CLK Select    */
    SIM_CLKOUT_SEL_SYSTEM_RTC_CLK        = 14U  /*!< RTC CLK as selected by SIM CLK 32 KHz Select */
} sim_clkout_src_t;

/*!
 * @brief SIM CLKOUT divider
 * Implements sim_clkout_div_t_Class
 */
typedef enum
{
    SIM_CLKOUT_DIV_BY_1     = 0U,               /*!< Divided by 1 */
    SIM_CLKOUT_DIV_BY_2     = 1U,               /*!< Divided by 2 */
    SIM_CLKOUT_DIV_BY_3     = 2U,               /*!< Divided by 3 */
    SIM_CLKOUT_DIV_BY_4     = 3U,               /*!< Divided by 4 */
    SIM_CLKOUT_DIV_BY_5     = 4U,               /*!< Divided by 5 */
    SIM_CLKOUT_DIV_BY_6     = 5U,               /*!< Divided by 6 */
    SIM_CLKOUT_DIV_BY_7     = 6U,               /*!< Divided by 7 */
    SIM_CLKOUT_DIV_BY_8     = 7U,               /*!< Divided by 8 */
} sim_clkout_div_t;

/*!
 * @brief SIM FlexTimer external clock select
 * Implements sim_ftm_clk_sel_t_Class
 */
typedef enum
{
    SIM_FTM_CLK_SEL_00 = 0U,                 /*!< FTM external clock driven by TCLK0 pin. */
    SIM_FTM_CLK_SEL_01 = 1U,                 /*!< FTM external clock driven by TCLK1 pin. */
    SIM_FTM_CLK_SEL_10 = 2U,                 /*!< FTM external clock driven by TCLK2 pin. */
    SIM_FTM_CLK_SEL_11 = 3U                  /*!< No clock input */
} sim_ftm_clk_sel_t;

/*!
 * @brief SIM CLK32KSEL clock source select
 * Implements sim_rtc_clk_sel_src_t_Class
 */
typedef enum
{
    SIM_RTCCLK_SEL_SOSCDIV1_CLK,      /* SOSCDIV1 clock          */
    SIM_RTCCLK_SEL_LPO_32K,           /* 32 kHz LPO clock        */
    SIM_RTCCLK_SEL_RTC_CLKIN,         /* RTC_CLKIN clock         */
    SIM_RTCCLK_SEL_FIRCDIV1_CLK       /* FIRCDIV1 clock          */
} sim_rtc_clk_sel_src_t;

/*!
 * @brief SIM LPOCLKSEL clock source select
 * Implements sim_lpoclk_sel_src_t_Class
 */
typedef enum
{
    SIM_LPO_CLK_SEL_LPO_128K,           /* 128 kHz LPO clock */
    SIM_LPO_CLK_SEL_NO_CLOCK,           /* No clock */
    SIM_LPO_CLK_SEL_LPO_32K,            /* 32 kHz LPO clock which is divided by the 128 kHz LPO clock */
    SIM_LPO_CLK_SEL_LPO_1K              /* 1 kHz LPO clock which is divided by the 128 kHz LPO clock */
} sim_lpoclk_sel_src_t;

/*!
 * @brief SIM ADCx pre-trigger select
 * Implements sim_adc_pretrg_sel_t_Class
 */
typedef enum
{
    SIM_ADC_PRETRG_SEL_PDB,                 /*!< PDB pre-trigger */
    SIM_ADC_PRETRG_SEL_TRGMUX,              /*!< TRGMUX pre-trigger */
    SIM_ADC_PRETRG_SEL_SOFTWARE,            /*!< Software pre-trigger */
    SIM_ADC_PRETRG_SEL_RESERVED             /*!< Reserved */
} sim_adc_pretrg_sel_t;

/*!
 * @brief SIM ADCx software pre-trigger select
 * Implements sim_adc_sw_pretrg_sel_t_Class
 */
typedef enum
{
    SIM_ADC_SW_PRETRG_SEL_DISABLED  = 0U,   /*!< Software pre-trigger disabled */
    SIM_ADC_SW_PRETRG_SEL_RESERVED0 = 1U,   /*!< Reserved */
    SIM_ADC_SW_PRETRG_SEL_RESERVED1 = 2U,   /*!< Reserved */
    SIM_ADC_SW_PRETRG_SEL_RESERVED2 = 3U,   /*!< Reserved */
    SIM_ADC_SW_PRETRG_SEL_0         = 4U,   /*!< Software pre-trigger 0 */
    SIM_ADC_SW_PRETRG_SEL_1         = 5U,   /*!< Software pre-trigger 1 */
    SIM_ADC_SW_PRETRG_SEL_2         = 6U,   /*!< Software pre-trigger 2 */
    SIM_ADC_SW_PRETRG_SEL_3         = 7U    /*!< Software pre-trigger 3 */
} sim_adc_sw_pretrg_sel_t;

/*!
 * @brief SIM ADCx trigger select
 * Implements sim_adc_trg_sel_t_Class
 */
typedef enum
{
    SIM_ADC_TRG_SEL_PDB            = 0U,    /*!< PDB output     */
    SIM_ADC_TRG_SEL_TRGMUX         = 1U     /*!< TRGMUX output  */
} sim_adc_trg_sel_t;

/*!
 * @brief SIM FlexTimer x channel y output source select
 * Implements sim_ftm_ch_out_src_t_Class
 */
typedef enum
{
    SIM_FTM_CH_OUT_SRC_0, /*!< FlexTimer x channel y output source 0. */
    SIM_FTM_CH_OUT_SRC_1, /*!< FlexTimer x channel y output source 1. */
} sim_ftm_ch_out_src_t;

/*!
 * @brief SIM FlexTimer x channel y input source select
 * Implements sim_ftm_ch_src_t_Class
 */
typedef enum
{
    SIM_FTM_CH_SRC_0 = 0U,    /*!< FlexTimer x channel y input source 0. */
    SIM_FTM_CH_SRC_1 = 1U,    /*!< FlexTimer x channel y input source 1. */
    SIM_FTM_CH_SRC_2 = 2U,    /*!< FlexTimer x channel y input source 2. */
    SIM_FTM_CH_SRC_3 = 3U,    /*!< FlexTimer x channel y input source 3. */
} sim_ftm_ch_src_t;

/*!
 * @brief SIM ClockOut configuration.
 * Implements sim_clock_out_config_t_Class
 */
typedef struct
{
    bool              initialize;     /*!< Initialize or not the ClockOut clock.  */
    bool              enable;         /*!< SIM ClockOut enable.                   */
    sim_clkout_src_t  source;         /*!< SIM ClockOut source select.            */
    sim_clkout_div_t  divider;        /*!< SIM ClockOut divide ratio.             */
} sim_clock_out_config_t;

/*!
 * @brief SIM LPO Clocks configuration.
 * Implements sim_lpo_clock_config_t_Class
 */
typedef struct
{
    bool                  initialize;       /*!< Initialize or not the LPO clock.     */
    sim_rtc_clk_sel_src_t sourceRtcClk;     /*!< RTC_CLK source select.               */
    sim_lpoclk_sel_src_t  sourceLpoClk;     /*!< LPO clock source select.             */
    bool                  enableLpo32k;     /*!< MSCM Clock Gating Control enable.    */
    bool                  enableLpo1k;      /*!< MSCM Clock Gating Control enable.    */
} sim_lpo_clock_config_t;

/*!
 * @brief SIM  Platform Gate Clock configuration.
 * Implements sim_plat_gate_config_t_Class
 */
typedef struct
{
    bool    initialize;     /*!< Initialize or not the Trace clock.  */
    bool    enableMscm;     /*!< MSCM Clock Gating Control enable.         */
    bool    enableMpu;      /*!< MPU Clock Gating Control enable.          */
    bool    enableDma;      /*!< DMA Clock Gating Control enable.          */
    bool    enableErm;      /*!< ERM Clock Gating Control enable.          */
    bool    enableEim;      /*!< EIM Clock Gating Control enable.          */
} sim_plat_gate_config_t;

/*!
 * @brief SIM  Platform Gate Clock configuration.
 * Implements sim_tclk_config_t_Class
 */
typedef struct
{
    bool      initialize;                         /*!< Initialize or not the Trace clock.  */
    uint32_t  tclkFreq[NUMBER_OF_TCLK_INPUTS];    /*!< TCLKx frequency.                    */
} sim_tclk_config_t;

/*!
 * @brief SIM RAM size
 * Implements sim_ram_size_t_Class
 */
typedef enum
{
    SIM_RAM_SIZE_48KB = 0xDU,   /*!< SIM Ram size - 48KB */
    SIM_RAM_SIZE_64KB = 0xFU,   /*!< SIM Ram size - 64KB */
} sim_ram_size_t;

/*!
 * @brief SIM Package
 * Implements sim_package_t_Class
 */
typedef enum
{
    SIM_PACKAGE_64_LQFP     = 0x3U,   /*!< SIM Package - 64 LQFP     */
    SIM_PACKAGE_100_LQFP    = 0x4U,   /*!< SIM Package - 100 LQFP    */
    SIM_PACKAGE_100_MAP_BGA = 0x8U,   /*!< SIM Package - 100 MAP BGA */
} sim_package_t;

/*!
 * @brief SIM Features
 * Implements sim_features_t_Class
 */
typedef enum
{
    SIM_FEATURE_FLEXIO      = (1U << 5U),    /*!< FlexIO      */
    SIM_FEATURE_ISO_CAN_FD  = (1U << 6U),    /*!< ISO CAN-FD  */
    SIM_FEATURE_SECURITY    = (1U << 7U),    /*!< Security    */
} sim_features_t;

/*!
 * @brief SIM EEE SRAM Size
 * Implements sim_eee_sram_size_t_Class
 */
typedef enum
{
    SIM_EEE_SRAM_SIZE_4KB  = 0x2U,   /*!< SIM EEE SRAM size - 4 KB      */
    SIM_EEE_SRAM_SIZE_2KB  = 0x3U,   /*!< SIM EEE SRAM size - 2 KB      */
    SIM_EEE_SRAM_SIZE_1KB  = 0x4U,   /*!< SIM EEE SRAM size - 1 KB      */
    SIM_EEE_SRAM_SIZE_512B = 0x5U,   /*!< SIM EEE SRAM size - 512 Bytes */
    SIM_EEE_SRAM_SIZE_256B = 0x6U,   /*!< SIM EEE SRAM size - 256 Bytes */
    SIM_EEE_SRAM_SIZE_128B = 0x7U,   /*!< SIM EEE SRAM size - 128 Bytes */
    SIM_EEE_SRAM_SIZE_64B  = 0x8U,   /*!< SIM EEE SRAM size - 64 Bytes  */
    SIM_EEE_SRAM_SIZE_32B  = 0x9U,   /*!< SIM EEE SRAM size - 32 Bytes  */
} sim_eee_sram_size_t;

/*!
 * @brief SIM FlexNVM partition
 * Implements sim_flexnvm_partition_t_Class
 */
typedef enum
{
    SIM_DEPART_0000 = 0x0U,   /*!< Data flash 64 KByte, EEPROM backup  0 KByte */
    SIM_DEPART_0011 = 0x3U,   /*!< Data flash 32 KByte, EEPROM backup 32 KByte */
    SIM_DEPART_0100 = 0x4U,   /*!< Data flash  0 KByte, EEPROM backup 64 KByte */
    SIM_DEPART_1000 = 0x8U,   /*!< Data flash  0 KByte, EEPROM backup 64 KByte */
    SIM_DEPART_1010 = 0xAU,   /*!< Data flash 16 KByte, EEPROM backup 48 KByte */
    SIM_DEPART_1011 = 0xBU,   /*!< Data flash 32 KByte, EEPROM backup 32 KByte */
    SIM_DEPART_1100 = 0xCU,   /*!< Data flash 64 KByte, EEPROM backup  0 KByte */
    SIM_DEPART_1111 = 0xFU,   /*!< Data flash 64 KByte, EEPROM backup  0 KByte */
} sim_flexnvm_partition_t;

/*!
 * @brief SIM Debug Trace clock configuration.
 * Implements sim_trace_clock_config_t_Class
 */
typedef struct
{
    bool               initialize;    /*!< Initialize or not the Trace clock.  */
    bool               divEnable;     /*!< Trace clock divider enable.         */
    clock_trace_src_t  source;        /*!< Trace clock select.                 */
    uint8_t            divider;       /*!< Trace clock divider divisor.        */
    bool               divFraction;   /*!< Trace clock divider fraction.       */
} sim_trace_clock_config_t;

/*!
 * @brief SIM configure structure.
 * Implements sim_clock_config_t_Class
 */
typedef struct
{
    sim_clock_out_config_t    clockOutConfig;                 /*!< Clock Out configuration.           */
    sim_lpo_clock_config_t    lpoClockConfig;                 /*!< Low Power Clock configuration.     */
    sim_tclk_config_t         tclkConfig;                     /*!< Platform Gate Clock configuration. */
    sim_plat_gate_config_t    platGateConfig;                 /*!< Platform Gate Clock configuration. */
    sim_trace_clock_config_t  traceClockConfig;               /*!< Trace clock configuration.         */
} sim_clock_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

 /*!
 * @brief Set SRAM L Retention setting.
 *
 * This function sets SRAM L Retention setting.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] setting The value to set.
 * Implements SIM_HAL_SetSramLRetentionCmd_Activity
 */
static inline void SIM_HAL_SetSramLRetentionCmd(SIM_Type * base, bool setting)
{
    uint32_t regValue = (uint32_t)base->CHIPCTL;
    regValue &= (uint32_t)(~(SIM_CHIPCTL_SRAML_RETEN_MASK));
    regValue |= SIM_CHIPCTL_SRAML_RETEN(setting ? 1UL : 0UL);
    base->CHIPCTL = (uint32_t)regValue;
}

/*!
 * @brief Get SRAM L Retention setting.
 *
 * This function gets SRAM L Retention setting.
 *
 * @param[in] base  Base address for current SIM instance.
 * @return Current selection.
 * Implements SIM_HAL_GetSramLRetentionCmd_Activity
 */
static inline bool SIM_HAL_GetSramLRetentionCmd(const SIM_Type * base)
{
    uint32_t regValue = (uint32_t)base->CHIPCTL;
    regValue = (regValue & SIM_CHIPCTL_SRAML_RETEN_MASK) >> SIM_CHIPCTL_SRAML_RETEN_SHIFT;
    return (regValue != 0U) ? true : false;
}


 /*!
 * @brief Set SRAM U Retention setting.
 *
 * This function sets SRAM U Retention setting.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] setting  The value to set.
 * Implements SIM_HAL_SetSramURetentionCmd_Activity
 */
static inline void SIM_HAL_SetSramURetentionCmd(SIM_Type * base, bool setting)
{
    uint32_t regValue = (uint32_t)base->CHIPCTL;
    regValue &= (uint32_t)(~(SIM_CHIPCTL_SRAMU_RETEN_MASK));
    regValue |= SIM_CHIPCTL_SRAMU_RETEN(setting ? 1UL : 0UL);
    base->CHIPCTL = (uint32_t)regValue;
}

/*!
 * @brief Get SRAM U Retention setting.
 *
 * This function gets SRAM U Retention setting.
 *
 * @param[in] base Base address for current SIM instance.
 * @return Current selection.
 * Implements SIM_HAL_GetSramURetentionCmd_Activity
 */
static inline bool SIM_HAL_GetSramURetentionCmd(const SIM_Type * base)
{
    uint32_t regValue = (uint32_t)base->CHIPCTL;
    regValue = (regValue & SIM_CHIPCTL_SRAMU_RETEN_MASK) >> SIM_CHIPCTL_SRAMU_RETEN_SHIFT;
    return (regValue != 0U) ? true : false;
}

 /*!
 * @brief Set ADC Supply Enable setting.
 *
 * This function sets internal supply monitoring on ADC0 channel AD21
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] setting The value to set.
 * Implements SIM_HAL_SetAdcSupplyEnCmd_Activity
 */
static inline void SIM_HAL_SetAdcSupplyEnCmd(SIM_Type * base, bool setting)
{
    uint32_t regValue = (uint32_t)base->CHIPCTL;
    regValue &= (uint32_t)(~(SIM_CHIPCTL_ADC_SUPPLYEN_MASK));
    regValue |= SIM_CHIPCTL_ADC_SUPPLYEN(setting ? 1UL : 0UL);
    base->CHIPCTL = (uint32_t)regValue;
}

/*!
 * @brief Get ADC Supply Enable setting.
 *
 * This function gets internal supply monitoring on ADC0 channel AD21 setting.
 *
 * @param[in] base Base address for current SIM instance.
 * @return Current selection.
 * Implements SIM_HAL_GetAdcSupplyEnCmd_Activity
 */
static inline bool SIM_HAL_GetAdcSupplyEnCmd(const SIM_Type * base)
{
    uint32_t regValue = (uint32_t)base->CHIPCTL;
    regValue = (regValue & SIM_CHIPCTL_ADC_SUPPLYEN_MASK) >> SIM_CHIPCTL_ADC_SUPPLYEN_SHIFT;
    return (regValue != 0U) ? true : false;
}

 /*!
 * @brief Set ADC Supply Enable setting.
 *
 * This function sets internal supply monitoring on ADC0 channel AD21 setting.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] setting The value to set.
 * Implements SIM_HAL_SetAdcSupplySrc_Activity
 */
static inline void SIM_HAL_SetAdcSupplySrc(SIM_Type * base, sim_adc_supply_src_t setting)
{
    uint32_t regValue;
    DEV_ASSERT(((uint32_t)setting) < (1UL<<SIM_CHIPCTL_ADC_SUPPLY_WIDTH));
    regValue = base->CHIPCTL;
    regValue &= ~(SIM_CHIPCTL_ADC_SUPPLY_MASK);
    regValue |= SIM_CHIPCTL_ADC_SUPPLY(setting);
    base->CHIPCTL = regValue;
}

/*!
 * @brief Get ADC supply source.
 *
 * This function gets ADC supply source.
 *
 * @param[in] base Base address for current SIM instance.
 * @return Current selection.
 * Implements SIM_HAL_GetAdcSupplySrc_Activity
 */
static inline sim_adc_supply_src_t SIM_HAL_GetAdcSupplySrc(const SIM_Type * base)
{
    sim_adc_supply_src_t retValue;
    switch((base->CHIPCTL & SIM_CHIPCTL_ADC_SUPPLY_MASK) >> SIM_CHIPCTL_ADC_SUPPLY_SHIFT)
    {
        case 0x5U:
            retValue = ADC_SUPPLY_LV;
            break;
        case 0x4U:
            retValue = ADC_SUPPLY_VDD_FLASH_3V;
            break;
        case 0x3U:
            retValue = ADC_SUPPLY_VDD_3V;
            break;
        case 0x2U:
            retValue = ADC_SUPPLY_VREFH;
            break;
        case 0x1U:
            retValue = ADC_SUPPLY_VDDA;
            break;
        case 0x0U:
        /* pass-through */
        default:
            retValue = ADC_SUPPLY_VDD;
            break;
    }
    return retValue;
}


 /*!
 * @brief Set PDB back-to-back selection.
 *
 * This function sets PDB back-to-back selection.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] setting The value to set.
 * Implements SIM_HAL_SetPdbBackToBackSrc_Activity
 */
static inline void SIM_HAL_SetPdbBackToBackSrc(SIM_Type * base, sim_pdb_bb_src_t setting)
{
    uint32_t settingValue, regValue;
    switch(setting)
    {
        case  PDB_BACK_TO_BACK_OPTION_1:
            settingValue = 1U;
            break;
        case PDB_BACK_TO_BACK_OPTION_0:
        /* Pass-through */
        default:
            settingValue = 0U;
            break;
    }

    regValue = (uint32_t)base->CHIPCTL;
    regValue &= (uint32_t)(~(SIM_CHIPCTL_PDB_BB_SEL_MASK));
    regValue |= SIM_CHIPCTL_PDB_BB_SEL(settingValue);
    base->CHIPCTL = (uint32_t)regValue;
}

/*!
 * @brief Get PDB back-to-back selection.
 *
 * This function gets PDB back-to-back selection.
 *
 * @param[in] base Base address for current SIM instance.
 * @return Current selection.
 * Implements SIM_HAL_GetPdbBackToBackSrc_Activity
 */
static inline sim_pdb_bb_src_t SIM_HAL_GetPdbBackToBackSrc(const SIM_Type * base)
{
    sim_pdb_bb_src_t retValue;

    switch ((base->CHIPCTL & SIM_CHIPCTL_PDB_BB_SEL_MASK) >> SIM_CHIPCTL_PDB_BB_SEL_SHIFT)
    {
        case 1U:
            retValue = PDB_BACK_TO_BACK_OPTION_1;
            break;
        case 0U:
        /* pass-through */
        default:
            retValue = PDB_BACK_TO_BACK_OPTION_0;
            break;
    }
    return retValue;
}

/*!
 * @brief Get the default SIM CLKOUT clock configuration.
 *
 * This function gets the default CLKOUT clock configuration.
 *
 * @param[out] config Pointer to the configuration structure.
 */
void SIM_HAL_GetClkoutDefaultConfig(sim_clock_out_config_t *config);

/*!
 * @brief Get the SIM CLKOUT clock configuration.
 *
 * This function gets the CLKOUT clock configuration.
 * @param[in] base Register base address for the SIM instance.
 * @param[in] config Pointer to the configuration structure.
 */
void SIM_HAL_GetClkoutConfig(const SIM_Type * base, sim_clock_out_config_t *config);

/*!
 * @brief Initialize SIM CLKOUT.
 *
 * This function enables the SIM CLKOUT clock according to the
 * configuration.
 *
 * @param[in] base Register base address for the SIM instance.
 * @param[in] config Pointer to the configuration structure.
 *
 */
void SIM_HAL_InitClkout(SIM_Type * base, const sim_clock_out_config_t *config);

/*!
 * @brief De-initialize SIM CLKOUT.
 *
 * This function disables the SIM CLKOUT.
 *
 * @param[in] base Register base address for the SIM instance.
 * Implements SIM_HAL_DeinitClkout_Activity
 */
static inline void SIM_HAL_DeinitClkout(SIM_Type * base)
{
    /* Disable divider. */
    base->CHIPCTL &= (uint32_t)(~(SIM_CHIPCTL_CLKOUTEN_MASK));
}

/*!
 * @brief Set ADC interleave channel select.
 *
 * This function sets value of ADC interleave channel select.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] setting The value to set.
 * Implements SIM_HAL_SetAdcInterleaveSel_Activity
 */
static inline void SIM_HAL_SetAdcInterleaveSel(SIM_Type * base, uint8_t setting)
{
    uint32_t regValue;
    DEV_ASSERT(setting < (1U<<SIM_CHIPCTL_ADC_INTERLEAVE_EN_WIDTH));

    regValue = base->CHIPCTL;
    regValue &= ~(SIM_CHIPCTL_ADC_INTERLEAVE_EN_MASK);
    regValue |= SIM_CHIPCTL_ADC_INTERLEAVE_EN(setting);
    base->CHIPCTL = regValue;
}

/*!
 * @brief Get ADC interleave channel select.
 *
 * This function gets value of ADC interleave channel select.
 *
 * @param[in] base Base address for current SIM instance.
 * @return Current value.
 * Implements SIM_HAL_GetAdcInterleaveSel_Activity
 */
static inline uint8_t SIM_HAL_GetAdcInterleaveSel(const SIM_Type * base)
{
    uint32_t regValue = base->CHIPCTL;
    regValue = (regValue & SIM_CHIPCTL_ADC_INTERLEAVE_EN_MASK) >> SIM_CHIPCTL_ADC_INTERLEAVE_EN_SHIFT;
    return (uint8_t)regValue;
}

/*!
 * @brief Sets the FlexTimer x external clock pin select setting.
 *
 * This function  selects the source of FTMx external clock pin select.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] instance Device instance.
 * @param[in] select FTMx external clock pin select
 */
void SIM_HAL_SetFtmExternalClkPinMode(SIM_Type * base,
                                      uint32_t instance,
                                      sim_ftm_clk_sel_t select);

/*!
 * @brief Gets the FlexTimer x external clock pin select setting.
 *
 * This function gets the FlexTimer x external clock pin select setting.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] instance Device instance.
 * @return select   FlexTimer x external clock pin select setting
 */
sim_ftm_clk_sel_t SIM_HAL_GetFtmExternalClkPinMode(const SIM_Type * base,
                                                   uint32_t instance);

/*!
 * @brief Sets the FlexTimer x faults select settings.
 *
 * This function  sets the FlexTimer x faults select settings.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] instance Device instance.
 * @param[in] select FlexTimer x faults select settings.
 */
void SIM_HAL_SetFtmFaultSelMode(SIM_Type * base,
                                uint32_t instance,
                                uint8_t select);

/*!
 * @brief Gets the FlexTimer x faults select settings.
 *
 * This function  gets the FlexTimer x faults select settings.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] instance Device instance.
 * @return select   FlexTimer x faults select settings.
 */
uint8_t SIM_HAL_GetFtmFaultSelMode(const SIM_Type * base,
                                   uint32_t instance);

/*!
 * @brief Get the clock selection of RTCCLKSEL.
 *
 * This function gets the clock selection of RTCCLKSEL.
 *
 * @param[in] base Base address for current SIM instance.
 * @return Current selection.
 * Implements SIM_HAL_GetRtcClkSrc_Activity
 */
static inline sim_rtc_clk_sel_src_t SIM_HAL_GetRtcClkSrc(const SIM_Type * base)
{
    sim_rtc_clk_sel_src_t retValue;
    switch((base->LPOCLKS & SIM_LPOCLKS_RTCCLKSEL_MASK) >> SIM_LPOCLKS_RTCCLKSEL_SHIFT)
    {
        case 0x3U:
            retValue = SIM_RTCCLK_SEL_FIRCDIV1_CLK;
            break;
        case 0x2U:
            retValue = SIM_RTCCLK_SEL_RTC_CLKIN;
            break;
        case 0x1U:
            retValue = SIM_RTCCLK_SEL_LPO_32K;
            break;
        case 0x0U:
        /* pass-through */
        default:
            retValue = SIM_RTCCLK_SEL_SOSCDIV1_CLK;
            break;
    }
    return retValue;
}


/*!
 * @brief Set the clock selection of LPOCLKSEL.
 *
 * This function sets the clock selection of LPOCLKSEL.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] setting The value to set.
 * @note This function ignores initialize member
 * Implements SIM_HAL_SetLpoClocks_Activity
 */
static inline void SIM_HAL_SetLpoClocks(SIM_Type* base, sim_lpo_clock_config_t setting)
{
    uint32_t regValue = base->LPOCLKS;

    regValue &= ~( SIM_LPOCLKS_LPO1KCLKEN_MASK  |
                   SIM_LPOCLKS_LPO32KCLKEN_MASK |
                   SIM_LPOCLKS_LPOCLKSEL_MASK   |
                   SIM_LPOCLKS_RTCCLKSEL_MASK   );

    regValue |= SIM_LPOCLKS_LPO1KCLKEN(setting.enableLpo1k ? 1UL : 0UL);
    regValue |= SIM_LPOCLKS_LPO32KCLKEN(setting.enableLpo32k ? 1UL : 0UL);
    regValue |= SIM_LPOCLKS_LPOCLKSEL(setting.sourceLpoClk);
    regValue |= SIM_LPOCLKS_RTCCLKSEL(setting.sourceRtcClk);

    /* Write value to register. */
    base->LPOCLKS = regValue;
}

/*!
 * @brief Get the clock selection of LPOCLKSEL.
 *
 * This function gets the clock selection of LPOCLKSEL.
 *
 * @param[in] base Base address for current SIM instance.
 * @return Current selection.
 * Implements SIM_HAL_GetLpoClkSrc_Activity
 */
static inline sim_lpoclk_sel_src_t SIM_HAL_GetLpoClkSrc(const SIM_Type * base)
{
    sim_lpoclk_sel_src_t retValue;

    switch((base->LPOCLKS & SIM_LPOCLKS_LPOCLKSEL_MASK) >> SIM_LPOCLKS_LPOCLKSEL_SHIFT)
    {
        case 0x3U:
            retValue = SIM_LPO_CLK_SEL_LPO_1K;
            break;
        case 0x2U:
            retValue = SIM_LPO_CLK_SEL_LPO_32K;
            break;
        case 0x1U:
            retValue = SIM_LPO_CLK_SEL_NO_CLOCK;
            break;
        case 0x0U:
        /* pass-through */
        default:
            retValue = SIM_LPO_CLK_SEL_LPO_128K;
            break;
    }
    return retValue;
}

/*!
 * @brief Gets the 32 kHz LPO clock Control.
 *
 * This function  gets the 32 kHz LPO clock enable setting.
 *
 * @param[in] base Base address for current SIM instance.
 * @return Current selection.
 * Implements SIM_HAL_GetLpo32kClkEnCmd_Activity
 */
static inline bool SIM_HAL_GetLpo32kClkEnCmd(const SIM_Type * base)
{
    uint32_t regValue = (uint32_t)base->LPOCLKS;
    regValue = (regValue & SIM_LPOCLKS_LPO32KCLKEN_MASK) >> SIM_LPOCLKS_LPO32KCLKEN_SHIFT;
    return (regValue != 0U) ? true : false;
}

/*!
 * @brief Gets the 1 kHz LPO clock Control.
 *
 * This function  gets the 1 kHz LPO clock enable setting.
 *
 * @param[in] base Base address for current SIM instance.
 * @return Current selection.
 * Implements SIM_HAL_GetLpo1kClkEnCmd_Activity
 */
static inline bool SIM_HAL_GetLpo1kClkEnCmd(const SIM_Type * base)
{
    uint32_t regValue = (uint32_t)base->LPOCLKS;
    regValue = (regValue & SIM_LPOCLKS_LPO1KCLKEN_MASK) >> SIM_LPOCLKS_LPO1KCLKEN_SHIFT;
    return (regValue != 0U) ? true : false;
}

/*!
 * @brief Get SIM LPO clock frequency (LPO_CLOCK).
 *
 * @param[in] base Register base address for the SIM instance.
 * @return Clock frequency, if clock is invalid, return 0.
 */
uint32_t SIM_HAL_GetLpoFreq(const SIM_Type * base);

/*!
 * @brief Get SIM LPO 128KHz clock frequency (LPO_128K_CLOCK).
 *
 * @param[in] base Register base address for the SIM instance.
 * @return  Clock frequency, if clock is invalid, return 0.
 */
uint32_t SIM_HAL_GetLpo128KFreq(const SIM_Type * base);

/*!
 * @brief Get SIM LPO 32KHz clock frequency (LPO_32K_CLOCK).
 *
 * @param[in] base Register base address for the SIM instance.
 * @return  Clock frequency, if clock is invalid, return 0.
 */
uint32_t SIM_HAL_GetLpo32KFreq(const SIM_Type * base);

/*!
 * @brief Get SIM LPO 1KHz clock frequency (LPO_1K_CLOCK).
 *
 * @param[in] base Register base address for the SIM instance.
 * @return  Clock frequency, if clock is invalid, return 0.
 */
uint32_t SIM_HAL_GetLpo1KFreq(const SIM_Type * base);

/*!
 * @brief Sets the ADCx pre-trigger select setting.
 *
 * This function selects the ADCx pre-trigger source.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] instance Device instance.
 * @param[in] select Pre-trigger select setting for ADCx
 */
void SIM_HAL_SetAdcPreTriggerMode(SIM_Type * base,
                                  uint32_t instance,
                                  sim_adc_pretrg_sel_t select);

/*!
 * @brief Gets the ADCx pre-trigger select setting.
 *
 * This function  gets the ADCx pre-trigger select setting.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] instance Device instance.
 * @return select  ADCax pre-trigger select setting
 */
sim_adc_pretrg_sel_t SIM_HAL_GetAdcPreTriggerMode(const SIM_Type * base,
                                                  uint32_t instance);

/*!
 * @brief Sets the ADCx software pre-trigger select setting.
 *
 * This function selects the ADCx software pre-trigger source.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] instance Device instance.
 * @param[in] select pre-trigger select setting for ADCx
 */
void SIM_HAL_SetAdcSwPreTriggerMode(SIM_Type * base,
                                    uint32_t instance,
                                    sim_adc_sw_pretrg_sel_t select);

/*!
 * @brief Gets the ADCx software pre-trigger select setting.
 *
 * This function  gets the ADCx software pre-trigger select setting.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] instance Device instance.
 * @return select ADCx pre-trigger select setting
 */
sim_adc_sw_pretrg_sel_t SIM_HAL_GetAdcSwPreTriggerMode(const SIM_Type * base,
                                                       uint32_t instance);

/*!
 * @brief Sets the ADCx trigger select setting.
 *
 * This function  selects the ADCx trigger source
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] instance Device instance.
 * @param[in] select Trigger select setting for ADCx
*/
void SIM_HAL_SetAdcTriggerMode(SIM_Type * base,
                               uint32_t instance,
                               sim_adc_trg_sel_t select);

/*!
 * @brief Gets the ADCx trigger select setting.
 *
 * This function  gets the ADCx trigger select setting.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] instance Device instance.
 * @return ADCx trigger select setting
 */
sim_adc_trg_sel_t SIM_HAL_GetAdcTriggerMode(const SIM_Type * base,
                                            uint32_t instance);


/*!
 * @brief Configure the FTM Global Load from the FTM Option Register 1.
 *
 * This function configures the FTM Global Load in the FTM Option Register 1.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] enable FTM global load setting
 * Implements SIM_HAL_SetFtmGlobalLoad_Activity
 */
static inline void SIM_HAL_SetFtmGlobalLoad(SIM_Type* base, bool enable)
{
    uint32_t regValue = (uint32_t)base->FTMOPT1;
    regValue &= (uint32_t)(~(SIM_FTMOPT1_FTMGLDOK_MASK));
    regValue |= SIM_FTMOPT1_FTMGLDOK(enable ? 1UL : 0UL);
    base->FTMOPT1 = (uint32_t)regValue;
}

/*!
 * @brief Gets the FTM Global Load from the FTM Option Register 1.
 *
 * This function gets the FTM Global Load from the FTM Option Register 1.
 *
 * @param[in] base Base address for current SIM instance.
 * @return enable FTM global load setting
 * Implements SIM_HAL_GetFtmGlobalLoad_Activity
 */
static inline bool SIM_HAL_GetFtmGlobalLoad(const SIM_Type * base)
{
    uint32_t regValue = (uint32_t)base->FTMOPT1;
    regValue = (regValue & SIM_FTMOPT1_FTMGLDOK_MASK) >> SIM_FTMOPT1_FTMGLDOK_SHIFT;
    return (regValue != 0U) ? true : false;
}


 /*!
 * @brief Sets the FlexTimer x channel y output source select setting.
 *
 * This function  selects the FlexTimer x channel y output source.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] instance device instance.
 * @param[in] channel FlexTimer channel y
 * @param[in] select FlexTimer x channel y output source
 */
void SIM_HAL_SetFtmChOutSrcMode(SIM_Type * base,
                                uint32_t instance,
                                uint8_t channel,
                                sim_ftm_ch_out_src_t select);

/*!
 * @brief Gets the FlexTimer x channel y output source select setting.
 *
 * This function gets the FlexTimer x channel y output
 * source select setting.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] instance Device instance.
 * @param[in] channel FlexTimer channel y
 * @return select   FlexTimer x channel y output source select setting
 */
sim_ftm_ch_out_src_t SIM_HAL_GetFtmChOutSrcMode(const SIM_Type * base,
                                                uint32_t instance,
                                                uint8_t channel);

/*!
 * @brief Sets the FlexTimer x channel y input source select setting.
 *
 * This function  selects the FlexTimer x channel y input source.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] instance Device instance.
 * @param[in] channel FlexTimer channel y
 * @param[in] select FlexTimer x channel y input source
 */
void SIM_HAL_SetFtmChSrcMode(SIM_Type * base,
                             uint32_t instance,
                             uint8_t channel,
                             sim_ftm_ch_src_t select);

/*!
 * @brief Gets the FlexTimer x channel y input source select setting.
 *
 * This function gets the FlexTimer x channel y input source select setting.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] instance Device instance.
 * @param[in] channel FlexTimer channel y
 * @return select FlexTimer x channel y input source select setting
 */
sim_ftm_ch_src_t SIM_HAL_GetFtmChSrcMode(const SIM_Type * base,
                                         uint32_t instance,
                                         uint8_t channel);

/*!
 * @brief Set FlexTimer x hardware trigger 0 software synchronization.
 *
 * This function sets FlexTimer x hardware trigger 0 software synchronization.
 * FTMxSYNCBIT.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] instance device instance.
 * @param[in] sync Synchronize or not.
 */
void SIM_HAL_SetFtmSyncCmd(SIM_Type * base, uint32_t instance, bool sync);

/*!
 * @brief Get FlexTimer x hardware trigger software synchronization setting.
 *
 * This function gets FlexTimer x hardware trigger software synchronization.
 * FTMxSYNCBIT.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] instance device instance.
 * @return enable    hardware trigger software synchronization setting
 * Implements SIM_HAL_GetFtmSyncCmd_Activity
 */
static inline bool SIM_HAL_GetFtmSyncCmd(const SIM_Type * base, uint32_t instance)
{
    uint32_t regValue = base->FTMOPT1;
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);

    switch (instance)
    {
    case 0U:
        regValue = (regValue & SIM_FTMOPT1_FTM0SYNCBIT_MASK) >> SIM_FTMOPT1_FTM0SYNCBIT_SHIFT;
        break;
    case 1U:
        regValue = (regValue & SIM_FTMOPT1_FTM1SYNCBIT_MASK) >> SIM_FTMOPT1_FTM1SYNCBIT_SHIFT;
        break;
    case 2U:
        regValue = (regValue & SIM_FTMOPT1_FTM2SYNCBIT_MASK) >> SIM_FTMOPT1_FTM2SYNCBIT_SHIFT;
        break;
    case 3U:
        regValue = (regValue & SIM_FTMOPT1_FTM3SYNCBIT_MASK) >> SIM_FTMOPT1_FTM3SYNCBIT_SHIFT;
        break;
#if FTM_INSTANCE_COUNT > 4U
    case 4U:
        regValue = (regValue & SIM_FTMOPT1_FTM4SYNCBIT_MASK) >> SIM_FTMOPT1_FTM4SYNCBIT_SHIFT;
        break;
#endif
#if FTM_INSTANCE_COUNT > 5U
    case 5U:
        regValue = (regValue & SIM_FTMOPT1_FTM5SYNCBIT_MASK) >> SIM_FTMOPT1_FTM5SYNCBIT_SHIFT;
        break;
#endif
#if FTM_INSTANCE_COUNT > 6U
    case 6U:
        regValue = (regValue & SIM_FTMOPT1_FTM6SYNCBIT_MASK) >> SIM_FTMOPT1_FTM6SYNCBIT_SHIFT;
        break;
#endif
#if FTM_INSTANCE_COUNT > 7U
    case 7U:
        regValue = (regValue & SIM_FTMOPT1_FTM7SYNCBIT_MASK) >> SIM_FTMOPT1_FTM7SYNCBIT_SHIFT;
        break;
#endif
    default:
        regValue = 0U;
        break;
    }

    return (regValue == 0U) ? false : true;
}

/*!
* @brief Sets FTM channel state.
*
* This function sets FTM channel state.
*
* @param[in] base Base address for current SIM instance.
* @param[in] instance Device instance.
* @param[in] setting The value to set.
* Implements SIM_HAL_SetObeCtrlCmd_Activity
*/
static inline void SIM_HAL_SetObeCtrlCmd(SIM_Type * base, uint32_t instance, bool setting)
{
   uint32_t settingValue = (setting == false) ? 0UL : 1UL;
   uint32_t regValue = base->MISCTRL0;
   DEV_ASSERT(instance < FTM_INSTANCE_COUNT);

   switch (instance)
   {
   case 0U:
       regValue &= ~(SIM_MISCTRL0_FTM0_OBE_CTRL_MASK);
       regValue |= SIM_MISCTRL0_FTM0_OBE_CTRL(settingValue);
       break;
   case 1U:
       regValue &= ~(SIM_MISCTRL0_FTM1_OBE_CTRL_MASK);
       regValue |= SIM_MISCTRL0_FTM1_OBE_CTRL(settingValue);
       break;
   case 2U:
       regValue &= ~(SIM_MISCTRL0_FTM2_OBE_CTRL_MASK);
       regValue |= SIM_MISCTRL0_FTM2_OBE_CTRL(settingValue);
       break;
   case 3U:
       regValue &= ~(SIM_MISCTRL0_FTM3_OBE_CTRL_MASK);
       regValue |= SIM_MISCTRL0_FTM3_OBE_CTRL(settingValue);
       break;
#if FTM_INSTANCE_COUNT > 4U
   case 4U:
       regValue &= ~(SIM_MISCTRL0_FTM4_OBE_CTRL_MASK);
       regValue |= SIM_MISCTRL0_FTM4_OBE_CTRL(settingValue);
       break;
#endif
#if FTM_INSTANCE_COUNT > 5U
   case 5U:
       regValue &= ~(SIM_MISCTRL0_FTM5_OBE_CTRL_MASK);
       regValue |= SIM_MISCTRL0_FTM5_OBE_CTRL(settingValue);
       break;
#endif
#if FTM_INSTANCE_COUNT > 6U
   case 6U:
       regValue &= ~(SIM_MISCTRL0_FTM6_OBE_CTRL_MASK);
       regValue |= SIM_MISCTRL0_FTM6_OBE_CTRL(settingValue);
       break;
#endif
#if FTM_INSTANCE_COUNT > 7U
   case 7U:
       regValue &= ~(SIM_MISCTRL0_FTM7_OBE_CTRL_MASK);
       regValue |= SIM_MISCTRL0_FTM7_OBE_CTRL(settingValue);
       break;
#endif
   default:
        /* Nothing to do, error is caught by DEV_ASSERT(instance < FTM_INSTANCE_COUNT) */
       break;
   }
   base->MISCTRL0 = regValue;
}

/*!
* @brief Gets FTM channel state.
*
* This function gets FTM current selection.
*
* @param[in] base Base address for current SIM instance.
* @param[in] instance Device instance.
* @return Current selection.
* Implements SIM_HAL_GetObeCtrlCmd_Activity
*/
static inline bool SIM_HAL_GetObeCtrlCmd(const SIM_Type * base, uint32_t instance)
{
   uint32_t regValue = base->MISCTRL0;
   DEV_ASSERT(instance < FTM_INSTANCE_COUNT);

   switch (instance)
   {
   case 0U:
        regValue = (regValue & SIM_MISCTRL0_FTM0_OBE_CTRL_MASK) >> SIM_MISCTRL0_FTM0_OBE_CTRL_SHIFT;
        break;
   case 1U:
        regValue = (regValue & SIM_MISCTRL0_FTM1_OBE_CTRL_MASK) >> SIM_MISCTRL0_FTM1_OBE_CTRL_SHIFT;
        break;
   case 2U:
        regValue = (regValue & SIM_MISCTRL0_FTM2_OBE_CTRL_MASK) >> SIM_MISCTRL0_FTM2_OBE_CTRL_SHIFT;
        break;
   case 3U:
        regValue = (regValue & SIM_MISCTRL0_FTM3_OBE_CTRL_MASK) >> SIM_MISCTRL0_FTM3_OBE_CTRL_SHIFT;
        break;
#if FTM_INSTANCE_COUNT > 4U
   case 4U:
        regValue = (regValue & SIM_MISCTRL0_FTM4_OBE_CTRL_MASK) >> SIM_MISCTRL0_FTM4_OBE_CTRL_SHIFT;
        break;
#endif
#if FTM_INSTANCE_COUNT > 5U
   case 5U:
        regValue = (regValue & SIM_MISCTRL0_FTM5_OBE_CTRL_MASK) >> SIM_MISCTRL0_FTM5_OBE_CTRL_SHIFT;
        break;
#endif
#if FTM_INSTANCE_COUNT > 6U
   case 6U:
        regValue = (regValue & SIM_MISCTRL0_FTM6_OBE_CTRL_MASK) >> SIM_MISCTRL0_FTM6_OBE_CTRL_SHIFT;
        break;
#endif
#if FTM_INSTANCE_COUNT > 7U
   case 7U:
        regValue = (regValue & SIM_MISCTRL0_FTM7_OBE_CTRL_MASK) >> SIM_MISCTRL0_FTM7_OBE_CTRL_SHIFT;
        break;
#endif
   default:
        regValue = 0U;
        break;
   }
   return (regValue == 0U) ? false : true;
}


/*!
 * @brief Gets the product series Generation from System Device ID register (SIM_SDID).
 *
 * This function gets the product series Generation from System Device ID register.
 *
 * @param[in] base Base address for current SIM instance.
 * @return Id  generation
 * Implements SIM_HAL_GetGeneration_Activity
 */
static inline uint32_t SIM_HAL_GetGeneration(const SIM_Type * base)
{
    uint32_t regValue = base->SDID;
    regValue = (regValue & SIM_SDID_GENERATION_MASK) >> SIM_SDID_GENERATION_SHIFT;
    return regValue;
}

/*!
 * @brief Gets the sub-series in the System Device ID register (SIM_SDID).
 *
 * This function  gets the sub-series in System Device ID register.
 *
 * @param[in] base Base address for current SIM instance.
 * @return Id sub-series
 * Implements SIM_HAL_GetSubSeries_Activity
 */
static inline uint32_t SIM_HAL_GetSubSeries(const SIM_Type * base)
{
    uint32_t regValue = base->SDID;
    regValue = (regValue & SIM_SDID_SUBSERIES_MASK) >> SIM_SDID_SUBSERIES_SHIFT;
    return regValue;
}

/*!
 * @brief Gets the Derivate from the System Device ID register (SIM_SDID).
 *
 * This function  gets the Derivate from System Device ID register.
 *
 * @param[in] base Base address for current SIM instance.
 * @return Id Derivate
 * Implements SIM_HAL_GetDerivate_Activity
 */
static inline uint32_t SIM_HAL_GetDerivate(const SIM_Type * base)
{
    uint32_t regValue = base->SDID;
    regValue = (regValue & SIM_SDID_DERIVATE_MASK) >> SIM_SDID_DERIVATE_SHIFT;
    return regValue;
}

/*!
 * @brief Gets RAM size.
 *
 * This function gets the RAM size. The field specifies the amount of system RAM
 * available on the device.
 *
 * @param[in] base Base address for current SIM instance.
 * @return RAM size on the device
 * Implements SIM_HAL_GetRamSize_Activity
 */
static inline sim_ram_size_t SIM_HAL_GetRamSize(const SIM_Type * base)
{
    sim_ram_size_t retValue;

    switch((base->SDID & SIM_SDID_RAMSIZE_MASK) >> SIM_SDID_RAMSIZE_SHIFT)
    {
        case 0xDU:
            retValue = SIM_RAM_SIZE_48KB;
            break;
        case 0xFU:
        /* pass-through */
        default:
            retValue = SIM_RAM_SIZE_64KB;
            break;
    }
    return retValue;
}

/*!
 * @brief Gets the Revision ID in the System Device ID register (SIM_SDID).
 *
 * This function  gets the Revision ID in System Device ID register.
 *
 * @param[in] base Base address for current SIM instance.
 * @return id  Revision ID
 * Implements SIM_HAL_GetRevId_Activity
 */
static inline uint32_t SIM_HAL_GetRevId(const SIM_Type * base)
{
    uint32_t regValue = base->SDID;
    regValue = (regValue & SIM_SDID_REVID_MASK) >> SIM_SDID_REVID_SHIFT;
    return regValue;
}

/*!
 * @brief Gets the Package in System Device ID register (SIM_SDID).
 *
 * This function  gets the Package in System Device ID register.
 *
 * @param[in] base Base address for current SIM instance.
 * @return Id package
 * Implements SIM_HAL_GetPackage_Activity
 */
static inline sim_package_t SIM_HAL_GetPackage(const SIM_Type * base)
{
    sim_package_t retValue;

    switch((base->SDID & SIM_SDID_PACKAGE_MASK) >> SIM_SDID_PACKAGE_SHIFT)
    {
        case 0x3U:
            retValue = SIM_PACKAGE_64_LQFP;
            break;
        case 0x4U:
            retValue = SIM_PACKAGE_100_LQFP;
            break;
        case 0x8U:
        /* pass-through */
        default:
            retValue = SIM_PACKAGE_100_MAP_BGA;
            break;
    }
    return retValue;
}

/*!
 * @brief Gets the Features from System Device ID register (SIM_SDID).
 *
 * This function gets the Features from System Device ID register.
 * See sim_features_t enumeration.
 *
 * @param[in] base Base address for current SIM instance.
 * @return Id features.
 * Implements SIM_HAL_GetFeatures_Activity
 */
static inline uint32_t SIM_HAL_GetFeatures(const SIM_Type * base)
{
    uint32_t regValue = base->SDID;
    regValue = (regValue & SIM_SDID_FEATURES_MASK) >> SIM_SDID_FEATURES_SHIFT;
    return regValue;
}

/*!
 * @brief Set the EIM Clock Gate from the Platform Clock Gating Control Register.
 *
 * This function configures the EIM Clock Gate in the Platform Clock Gating Control Register.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] EIM clock gate enable setting
 * Implements SIM_HAL_SetEimClockGate_Activity
 */
static inline void SIM_HAL_SetEimClockGate(SIM_Type* base, bool enable)
{
    uint32_t regValue = (uint32_t)base->PLATCGC;
    regValue &= (uint32_t)(~(SIM_PLATCGC_CGCEIM_MASK));
    regValue |= SIM_PLATCGC_CGCEIM(enable ? 1UL : 0UL);
    base->PLATCGC = (uint32_t)regValue;
}

/*!
 * @brief Gets the EIM Clock Gate from the Platform Clock Gating Control Register.
 *
 * This function gets the EIM Clock Gate in the Platform Clock Gating Control Register.
 *
 * @param[in] base Base address for current SIM instance.
 * @return EIM Clock Gating
 * Implements SIM_HAL_GetEimClockGate_Activity
 */
static inline bool SIM_HAL_GetEimClockGate(const SIM_Type * base)
{
    uint32_t regValue = (uint32_t)base->PLATCGC;
    regValue = (regValue & SIM_PLATCGC_CGCEIM_MASK) >> SIM_PLATCGC_CGCEIM_SHIFT;
    return (regValue != 0U) ? true : false;
}

/*!
 * @brief Set the ERM Clock Gate from the Platform Clock Gating Control Register.
 *
 * This function configures the ERM Clock Gate in the Platform Clock Gating Control Register.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] enable ERM clock gate enable setting
 * Implements SIM_HAL_SetErmClockGate_Activity
 */
static inline void SIM_HAL_SetErmClockGate(SIM_Type* base, bool enable)
{
    uint32_t regValue = (uint32_t)base->PLATCGC;
    regValue &= (uint32_t)(~(SIM_PLATCGC_CGCERM_MASK));
    regValue |= SIM_PLATCGC_CGCERM(enable ? 1UL : 0UL);
    base->PLATCGC = (uint32_t)regValue;
}

/*!
 * @brief Gets the ERM Clock Gate from the Platform Clock Gating Control Register.
 *
 * This function gets the ERM Clock Gate in the Platform Clock Gating Control Register.
 *
 * @param[in] base Base address for current SIM instance.
 * @return ERM Clock Gating
 * Implements SIM_HAL_GetErmClockGate_Activity
 */
static inline bool SIM_HAL_GetErmClockGate(const SIM_Type * base)
{
    uint32_t regValue = (uint32_t)base->PLATCGC;
    regValue = (regValue & SIM_PLATCGC_CGCERM_MASK) >> SIM_PLATCGC_CGCERM_SHIFT;
    return (regValue != 0U) ? true : false;
}

/*!
 * @brief Set the DMA Clock Gate from the Platform Clock Gating Control Register.
 *
 * This function configures the DMA Clock Gate in the Platform Clock Gating Control Register.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] enable DMA clock gate enable setting
 * Implements SIM_HAL_SetDmaClockGate_Activity
 */
static inline void SIM_HAL_SetDmaClockGate(SIM_Type* base, bool enable)
{
    uint32_t regValue = (uint32_t)base->PLATCGC;
    regValue &= (uint32_t)(~(SIM_PLATCGC_CGCDMA_MASK));
    regValue |= SIM_PLATCGC_CGCDMA(enable ? 1UL : 0UL);
    base->PLATCGC = (uint32_t)regValue;
}

/*!
 * @brief Gets the DMA Clock Gate from the Platform Clock Gating Control Register.
 *
 * This function gets the DMA Clock Gate in the Platform Clock Gating Control Register.
 *
 * @param[in] base Base address for current SIM instance.
 * @return DMA Clock Gating
 * Implements SIM_HAL_GetDmaClockGate_Activity
 */
static inline bool SIM_HAL_GetDmaClockGate(const SIM_Type * base)
{
    uint32_t regValue = (uint32_t)base->PLATCGC;
    regValue = (regValue & SIM_PLATCGC_CGCDMA_MASK) >> SIM_PLATCGC_CGCDMA_SHIFT;
    return (regValue != 0U) ? true : false;
}

/*!
 * @brief Configure the MPU Clock Gating from the Platform Clock Gating Control Register.
 *
 * This function configures the MPU Clock Gating in the Platform Clock Gating Control Register.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] enable MPU clock gate enable setting
 * Implements SIM_HAL_SetMpuClockGate_Activity
 */
static inline void SIM_HAL_SetMpuClockGate(SIM_Type* base, bool enable)
{
    uint32_t regValue = (uint32_t)base->PLATCGC;
    regValue &= (uint32_t)(~(SIM_PLATCGC_CGCMPU_MASK));
    regValue |= SIM_PLATCGC_CGCMPU(enable ? 1UL : 0UL);
    base->PLATCGC = (uint32_t)regValue;
}

/*!
 * @brief Gets the MPU Clock Gating from the Platform Clock Gating Control Register.
 *
 * This function gets the MPU Clock Gating in the Platform Clock Gating Control Register.
 *
 * @param[in] base Base address for current SIM instance.
 * @return MPU Clock Gating
 * Implements SIM_HAL_GetMpuClockGate_Activity
 */
static inline bool SIM_HAL_GetMpuClockGate(const SIM_Type * base)
{
    uint32_t regValue = (uint32_t)base->PLATCGC;
    regValue = (regValue & SIM_PLATCGC_CGCMPU_MASK) >> SIM_PLATCGC_CGCMPU_SHIFT;
    return (regValue != 0U) ? true : false;
}

/*!
 * @brief Configure the MSCM Clock Gating from the Platform Clock Gating Control Register.
 *
 * This function configures the MSCM Clock Gating in the Platform Clock Gating Control Register.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] enable MPU clock gate enable setting
 * Implements SIM_HAL_SetMscmClockGate_Activity
 */
static inline void SIM_HAL_SetMscmClockGate(SIM_Type* base, bool enable)
{
    uint32_t regValue = (uint32_t)base->PLATCGC;
    regValue &= (uint32_t)(~(SIM_PLATCGC_CGCMSCM_MASK));
    regValue |= SIM_PLATCGC_CGCMSCM(enable ? 1UL : 0UL);
    base->PLATCGC = (uint32_t)regValue;
}

/*!
 * @brief Gets the MSCM Clock Gating from the Platform Clock Gating Control Register.
 *
 * This function gets the MSCM Clock Gating in the Platform Clock Gating Control Register.
 *
 * @param[in] base Base address for current SIM instance.
 * @return MSCM Clock Gating
 * Implements SIM_HAL_GetMscmClockGate_Activity
 */
static inline bool SIM_HAL_GetMscmClockGate(const SIM_Type * base)
{
    uint32_t regValue = (uint32_t)base->PLATCGC;
    regValue = (regValue & SIM_PLATCGC_CGCMSCM_MASK) >> SIM_PLATCGC_CGCMSCM_SHIFT;
    return (regValue != 0U) ? true : false;
}

/*!
 * @brief Gets the EEE SRAM size in the Flash Configuration Register 1.
 *
 * This function  gets the EEE SRAM size in the Flash Configuration Register 1.
 *
 * @param[in] base Base address for current SIM instance.
 * @return EEE SRAM size
 * Implements SIM_HAL_GetEeeSramSize_Activity
 */
static inline sim_eee_sram_size_t SIM_HAL_GetEeeSramSize(const SIM_Type * base)
{
    sim_eee_sram_size_t retValue;

    switch((base->FCFG1 & SIM_FCFG1_EEERAMSIZE_MASK) >> SIM_FCFG1_EEERAMSIZE_SHIFT)
    {
        case 0x9U:
            retValue = SIM_EEE_SRAM_SIZE_32B;
            break;
        case 0x8U:
            retValue = SIM_EEE_SRAM_SIZE_64B;
            break;
        case 0x7U:
            retValue = SIM_EEE_SRAM_SIZE_128B;
            break;
        case 0x6U:
            retValue = SIM_EEE_SRAM_SIZE_256B;
            break;
        case 0x5U:
            retValue = SIM_EEE_SRAM_SIZE_512B;
            break;
        case 0x4U:
            retValue = SIM_EEE_SRAM_SIZE_1KB;
            break;
        case 0x3U:
            retValue = SIM_EEE_SRAM_SIZE_2KB;
            break;
        case 0x2U:
        /* pass-through */
        default:
            retValue = SIM_EEE_SRAM_SIZE_4KB;
            break;
    }
    return retValue;
}

/*!
 * @brief Gets the FlexNVM partition in the Flash Configuration Register 1.
 *
 * This function  gets the FlexNVM partition in the Flash Configuration Register 1
 *
 * @param[in] base  Base address for current SIM instance.
 * @return FlexNVM partition setting
 * Implements SIM_HAL_GetFlexNvmPartition_Activity
 */
static inline sim_flexnvm_partition_t SIM_HAL_GetFlexNvmPartition(const SIM_Type * base)
{
    sim_flexnvm_partition_t retValue;

    switch((base->FCFG1 & SIM_FCFG1_DEPART_MASK) >> SIM_FCFG1_DEPART_SHIFT)
    {
        case 0xFU:
            retValue = SIM_DEPART_1111;
            break;
        case 0xCU:
            retValue = SIM_DEPART_1100;
            break;
        case 0xBU:
            retValue = SIM_DEPART_1011;
            break;
        case 0xAU:
            retValue = SIM_DEPART_1010;
            break;
        case 0x8U:
            retValue = SIM_DEPART_1000;
            break;
        case 0x4U:
            retValue = SIM_DEPART_0100;
            break;
        case 0x3U:
            retValue = SIM_DEPART_0011;
            break;
        case 0x0U:
        /* pass-through */
        default:
            retValue = SIM_DEPART_0000;
            break;
    }
    return retValue;
}

/*!
 * @brief Gets the UID127_96 from Unique Identification Register High.
 *
 * This function gets the UID127_96 from Unique Identification Register High.
 *
 * @param[in] base  Base address for current SIM instance.
 * @return UID127_96 setting
 * Implements SIM_HAL_GetUniqueIdHigh_Activity
 */
static inline uint32_t SIM_HAL_GetUniqueIdHigh(const SIM_Type * base)
{
    return base->UIDH;
}

/*!
 * @brief Gets the UID95_64 from Unique Identification Register Mid High.
 *
 * This function gets the UID95_64 from Unique Identification Register Mid High.
 *
 * @param[in] base Base address for current SIM instance.
 * @return UID95_64 setting
 * Implements SIM_HAL_GetUniqueIdMidHigh_Activity
 */
static inline uint32_t SIM_HAL_GetUniqueIdMidHigh(const SIM_Type * base)
{
    return base->UIDMH;
}

/*!
 * @brief Gets the UID63_32 from Unique Identification Register Mid Low.
 *
 * This function gets the UID63_32 from Unique Identification Register Mid Low.
 *
 * @param[in] base Base address for current SIM instance.
 * @return UID63_32 setting
 * Implements SIM_HAL_GetUniqueIdMidLow_Activity
 */
static inline uint32_t SIM_HAL_GetUniqueIdMidLow(const SIM_Type * base)
{
    return base->UIDML;
}

/*!
 * @brief Gets the UID31_0 from Unique Identification Register Low.
 *
 * This function gets the UID31_0 from Unique Identification Register Low.
 *
 * @param[in] base Base address for current SIM instance.
 * @return UID31_0 setting
 * Implements SIM_HAL_GetUniqueIdLow_Activity
 */
static inline uint32_t SIM_HAL_GetUniqueIdLow(const SIM_Type * base)
{
    return base->UIDL;
}

/*!
 * @brief Get the default Debug Trace clock configuration.
 *
 * This function gets the default Debug Trace clock configuration.
 *
 * @param[in] config Pointer to the configuration structure.
 */
void SIM_HAL_GetTraceClockDefaultConfig(sim_trace_clock_config_t *config);

/*!
 * @brief Initialize SIM Debug Trace.
 *
 * This function enables the SIM Debug Trace clock according to the
 * configuration.
 *
 * @param[in] base Register base address for the SIM instance.
 * @param[in] config Pointer to the configuration structure.
 *
 */
void SIM_HAL_InitTraceClock(SIM_Type * base,
                            const sim_trace_clock_config_t *config);

/*!
 * @brief De-initialize SIM Debug Trace.
 *
 * This function disables the SIM Debug Trace clock.
 *
 * @param[in] base Register base address for the SIM instance.
 * Implements SIM_HAL_DeinitTraceClock_Activity
 */
static inline void SIM_HAL_DeinitTraceClock(SIM_Type * base)
{
    /* Disable divider. */
    base->CLKDIV4 &= (uint32_t)(~(SIM_CLKDIV4_TRACEDIVEN_MASK));
}

/*!
 * @brief Sets the Software Trigger bit to TRGMUX setting.
 *
 * This function sets the Software Trigger bit to TRGMUX in Miscellaneous Control register.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] disable Software Trigger bit
 * Implements SIM_HAL_SetSwTriggerTrgmux_Activity
 */
static inline void SIM_HAL_SetSwTriggerTrgmux(SIM_Type * base, bool disable)
{
    uint32_t regValue = (uint32_t)base->MISCTRL1;
    regValue &= (uint32_t)(~(SIM_MISCTRL1_SW_TRG_MASK));
    regValue |= SIM_MISCTRL1_SW_TRG(disable ? 1UL : 0UL);
    base->MISCTRL1 = (uint32_t)regValue;
}

/*!
 * @brief Gets the Software Trigger bit to TRGMUX.
 *
 * This function gets the Software Trigger bit to TRGMUX in Miscellaneous Control register.
 *
 * @param[in] base Base address for current SIM instance.
 * @return Software Trigger bit setting
 * Implements SIM_HAL_GetSwTriggerTrgmux_Activity
 */
static inline uint32_t SIM_HAL_GetSwTriggerTrgmux(const SIM_Type * base)
{
    uint32_t regValue = (uint32_t)base->MISCTRL1;
    regValue = (regValue & SIM_MISCTRL1_SW_TRG_MASK) >> SIM_MISCTRL1_SW_TRG_SHIFT;
    return (uint32_t)regValue;
}

/*!
 * @brief Sets the TClk Frequency
 *
 * This function sets the TClk Frequency.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] index Index of the TClk.
 * @param[in] frequency The frequency of the specified TClk
 * Implements SIM_HAL_SetTClkFreq_Activity
 */
static inline void SIM_HAL_SetTClkFreq(SIM_Type * base, uint8_t index, uint32_t frequency)
{
    (void) base;
    if(index < NUMBER_OF_TCLK_INPUTS) {
        g_TClkFreq[index] = frequency;
    }
}

/*!
 * @brief Gets the TClk Frequency
 *
 * This function gets the TClk Frequency.
 *
 * @param[in] base Base address for current SIM instance.
 * @param[in] index Index of the TClk.
 * @return frequency The configured frequency of the specified TClk
 * Implements SIM_HAL_GetTClkFreq_Activity
 */
static inline uint32_t SIM_HAL_GetTClkFreq(SIM_Type * base, uint8_t index)
{
    uint32_t retValue = 0U;
    (void) base;
    if(index < NUMBER_OF_TCLK_INPUTS) {
        retValue = g_TClkFreq[index];
    }

    return retValue;
}

#if defined(__cplusplus)
}
#endif /* __cplusplus*/


/*! @}*/

#endif /* SIM_HAL_S32K144_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/

