/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_vref.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief Gets the instance from the base address
 *
 * @param base VREF peripheral base address
 *
 * @return The VREF instance
 */
static uint32_t VREF_GetInstance(VREF_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Pointers to VREF bases for each instance. */
static VREF_Type *const s_vrefBases[] = VREF_BASE_PTRS;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Pointers to VREF clocks for each instance. */
static const clock_ip_name_t s_vrefClocks[] = VREF_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/*******************************************************************************
 * Code
 ******************************************************************************/

static uint32_t VREF_GetInstance(VREF_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_vrefBases); instance++)
    {
        if (s_vrefBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ARRAY_SIZE(s_vrefBases));

    return instance;
}

void VREF_Init(VREF_Type *base, const vref_config_t *config)
{
    assert(config != NULL);

    uint8_t reg = 0U;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Ungate clock for VREF */
    CLOCK_EnableClock(s_vrefClocks[VREF_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/* Configure VREF to a known state */
#if defined(FSL_FEATURE_VREF_HAS_CHOP_OSC) && FSL_FEATURE_VREF_HAS_CHOP_OSC
    /* Set chop oscillator bit */
    base->TRM |= VREF_TRM_CHOPEN_MASK;
#endif /* FSL_FEATURE_VREF_HAS_CHOP_OSC */
    /* Get current SC register */
#if defined(FSL_FEATURE_VREF_HAS_LOW_REFERENCE) && FSL_FEATURE_VREF_HAS_LOW_REFERENCE
    reg = base->VREFH_SC;
#else
    reg = base->SC;
#endif/* FSL_FEATURE_VREF_HAS_LOW_REFERENCE */
    /* Clear old buffer mode selection bits */
    reg &= ~VREF_SC_MODE_LV_MASK;
    /* Set buffer Mode selection and Regulator enable bit */
    reg |= VREF_SC_MODE_LV(config->bufferMode) | VREF_SC_REGEN(1U);
#if defined(FSL_FEATURE_VREF_HAS_COMPENSATION) && FSL_FEATURE_VREF_HAS_COMPENSATION
    /* Set second order curvature compensation enable bit */
    reg |= VREF_SC_ICOMPEN(1U);
#endif /* FSL_FEATURE_VREF_HAS_COMPENSATION */
    /* Enable VREF module */
    reg |= VREF_SC_VREFEN(1U);
    /* Update bit-field from value to Status and Control register */
#if defined(FSL_FEATURE_VREF_HAS_LOW_REFERENCE) && FSL_FEATURE_VREF_HAS_LOW_REFERENCE
    base->VREFH_SC = reg;
#else
    base->SC = reg;
#endif/* FSL_FEATURE_VREF_HAS_LOW_REFERENCE */
#if defined(FSL_FEATURE_VREF_HAS_LOW_REFERENCE) && FSL_FEATURE_VREF_HAS_LOW_REFERENCE
    reg = base->VREFL_TRM;
    /* Clear old select external voltage reference and VREFL (0.4 V) reference buffer enable bits */
    reg &= ~(VREF_VREFL_TRM_VREFL_EN_MASK | VREF_VREFL_TRM_VREFL_SEL_MASK);
    /* Select external voltage reference and set VREFL (0.4 V) reference buffer enable */
    reg |= VREF_VREFL_TRM_VREFL_SEL(config->enableExternalVoltRef) | VREF_VREFL_TRM_VREFL_EN(config->enableLowRef);
    base->VREFL_TRM = reg;
#endif /* FSL_FEATURE_VREF_HAS_LOW_REFERENCE */

#if defined(FSL_FEATURE_VREF_HAS_TRM4) && FSL_FEATURE_VREF_HAS_TRM4
    reg = base->TRM4;
    /* Clear old select internal voltage reference bit (2.1V) */
    reg &= ~VREF_TRM4_VREF2V1_EN_MASK;
    /* Select internal voltage reference (2.1V) */
    reg |= VREF_TRM4_VREF2V1_EN(config->enable2V1VoltRef);
    base->TRM4 = reg;
#endif /* FSL_FEATURE_VREF_HAS_TRM4 */

    /* Wait until internal voltage stable */
#if defined(FSL_FEATURE_VREF_HAS_LOW_REFERENCE) && FSL_FEATURE_VREF_HAS_LOW_REFERENCE
     while ((base->VREFH_SC & VREF_SC_VREFST_MASK) == 0)
#else
    while ((base->SC & VREF_SC_VREFST_MASK) == 0)
#endif/* FSL_FEATURE_VREF_HAS_LOW_REFERENCE */
    {
    }
}

void VREF_Deinit(VREF_Type *base)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Gate clock for VREF */
    CLOCK_DisableClock(s_vrefClocks[VREF_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

void VREF_GetDefaultConfig(vref_config_t *config)
{
    assert(config);

/* Set High power buffer mode in */
#if defined(FSL_FEATURE_VREF_MODE_LV_TYPE) && FSL_FEATURE_VREF_MODE_LV_TYPE
    config->bufferMode = kVREF_ModeHighPowerBuffer;
#else
    config->bufferMode = kVREF_ModeTightRegulationBuffer;
#endif /* FSL_FEATURE_VREF_MODE_LV_TYPE */

#if defined(FSL_FEATURE_VREF_HAS_LOW_REFERENCE) && FSL_FEATURE_VREF_HAS_LOW_REFERENCE
    /* Select internal voltage reference */
    config->enableExternalVoltRef = false;
    /* Set VREFL (0.4 V) reference buffer disable */
    config->enableLowRef = false;
#endif /* FSL_FEATURE_VREF_HAS_LOW_REFERENCE */

#if defined(FSL_FEATURE_VREF_HAS_TRM4) && FSL_FEATURE_VREF_HAS_TRM4
    /* Disable internal voltage reference (2.1V) */
    config->enable2V1VoltRef = false;
#endif /* FSL_FEATURE_VREF_HAS_TRM4 */
}

void VREF_SetTrimVal(VREF_Type *base, uint8_t trimValue)
{
    uint8_t reg = 0U;

    /* Set TRIM bits value in voltage reference */
    reg = base->TRM;
    reg = ((reg & ~VREF_TRM_TRIM_MASK) | VREF_TRM_TRIM(trimValue));
    base->TRM = reg;
    /* Wait until internal voltage stable */
#if defined(FSL_FEATURE_VREF_HAS_LOW_REFERENCE) && FSL_FEATURE_VREF_HAS_LOW_REFERENCE
     while ((base->VREFH_SC & VREF_SC_VREFST_MASK) == 0)
#else
    while ((base->SC & VREF_SC_VREFST_MASK) == 0)
#endif/* FSL_FEATURE_VREF_HAS_LOW_REFERENCE */
    {
    }
}

#if defined(FSL_FEATURE_VREF_HAS_TRM4) && FSL_FEATURE_VREF_HAS_TRM4
void VREF_SetTrim2V1Val(VREF_Type *base, uint8_t trimValue)
{
    uint8_t reg = 0U;

    /* Set TRIM bits value in voltage reference (2V1) */
    reg = base->TRM4;
    reg = ((reg & ~VREF_TRM4_TRIM2V1_MASK) | VREF_TRM4_TRIM2V1(trimValue));
    base->TRM4 = reg;
    /* Wait until internal voltage stable */
    while ((base->SC & VREF_SC_VREFST_MASK) == 0)
    {
    }
}
#endif /* FSL_FEATURE_VREF_HAS_TRM4 */

#if defined(FSL_FEATURE_VREF_HAS_LOW_REFERENCE) && FSL_FEATURE_VREF_HAS_LOW_REFERENCE
void VREF_SetLowReferenceTrimVal(VREF_Type *base, uint8_t trimValue)
{
    /* The values 111b and 110b are NOT valid/allowed */
    assert((trimValue != 0x7U) && (trimValue != 0x6U));

    uint8_t reg = 0U;

    /* Set TRIM bits value in low voltage reference */
    reg = base->VREFL_TRM;
    reg = ((reg & ~VREF_VREFL_TRM_VREFL_TRIM_MASK) | VREF_VREFL_TRM_VREFL_TRIM(trimValue));
    base->VREFL_TRM = reg;
    /* Wait until internal voltage stable */

     while ((base->VREFH_SC & VREF_SC_VREFST_MASK) == 0)
    {
    }
}
#endif /* FSL_FEATURE_VREF_HAS_LOW_REFERENCE */
