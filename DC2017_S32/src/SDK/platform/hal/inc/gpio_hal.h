/*
 * Copyright (c) 2014 - 2015, Freescale Semiconductor, Inc.
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

#ifndef GPIO_HAL_H
#define GPIO_HAL_H

#include <stdint.h>
#include "device_registers.h"

/*! @file */

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Local macro not referenced.
 * The defined macro is used as include guard.
 *
 */

/*!
 * @addtogroup gpio_hal
 * @{
 */

/*******************************************************************************
 * API
 ******************************************************************************/

 #if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name GPIO
 * General GPIO functions.
 */
/*! @{*/

/*!
 * @brief Write a pin of a port with a given value
 *
 * This function writes the given pin from a port, with the given value
 * ('0' represents LOW, '1' represents HIGH).
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @param pinNumber pin number to be written
 * @param pinValue pin value to be written
 *        - 0: corresponding pin is set to LOW
 *        - 1: corresponding pin is set to HIGH
 * Implements : GPIO_HAL_WritePin_Activity
 */
static inline void GPIO_HAL_WritePin(GPIO_Type* const baseAddr,
		const uint32_t pinNumber, const uint32_t pinValue)
{
    uint32_t pinsValues = (uint32_t)(baseAddr->PDOR);
    pinsValues &= ~(1UL << pinNumber);
    pinsValues |= pinValue << pinNumber;
	baseAddr->PDOR = GPIO_PDOR_PDO(pinsValues);
}

/*!
 * @brief Write all pins of a port
 *
 * This function writes all pins configured as output with the values given in
 * the parameter pins. '0' represents LOW, '1' represents HIGH.
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @param pins pin mask to be written
 *        - 0: corresponding pin is set to LOW
 *        - 1: corresponding pin is set to HIGH
 * Implements : GPIO_HAL_WritePins_Activity
 */
static inline void GPIO_HAL_WritePins(GPIO_Type* const baseAddr, const uint32_t pins)
{
    baseAddr->PDOR = GPIO_PDOR_PDO(pins);
}

/*!
 * @brief Get the current output from a port
 *
 * This function returns the current output that is written to a port. Only pins
 * that are configured as output will have meaningful values.
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @return GPIO outputs. Each bit represents one pin (LSB is pin 0, MSB is pin
 * 31). For each bit:
 *        - 0: corresponding pin is set to LOW
 *        - 1: corresponding pin is set to HIGH
 * Implements : GPIO_HAL_GetPinsOutput_Activity
 */
static inline uint32_t GPIO_HAL_GetPinsOutput(const GPIO_Type* const baseAddr)
{
    return (uint32_t)(baseAddr->PDOR);
}

/*!
 * @brief Write pins with 'Set' value
 *
 * This function configures output pins listed in parameter pins (bits that are
 * '1') to have a value of 'set' (HIGH). Pins corresponding to '0' will be
 * unaffected.
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @param pins pin mask of bits to be set.  Each bit represents one pin (LSB is
 * pin 0, MSB is pin 31). For each bit:
 *        - 0: corresponding pin is unaffected
 *        - 1: corresponding pin is set to HIGH
 * Implements : GPIO_HAL_SetPins_Activity
 */
static inline void GPIO_HAL_SetPins(GPIO_Type* const baseAddr, const uint32_t pins)
{
	baseAddr->PSOR = GPIO_PSOR_PTSO(pins);
}

/*!
 * @brief Write pins to 'Clear' value
 *
 * This function configures output pins listed in parameter pins (bits that are
 * '1') to have a 'cleared' value (LOW). Pins corresponding to '0' will be
 * unaffected.
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @param pins pin mask of bits to be cleared.  Each bit represents one pin (LSB
 * is pin 0, MSB is pin 31). For each bit:
 *        - 0: corresponding pin is unaffected
 *        - 1: corresponding pin is cleared (set to LOW)
 * Implements : GPIO_HAL_ClearPins_Activity
 */
static inline void GPIO_HAL_ClearPins(GPIO_Type* const baseAddr, const uint32_t pins)
{
	baseAddr->PCOR = GPIO_PCOR_PTCO(pins);
}

/*!
 * @brief Toggle pins value
 *
 * This function toggles output pins listed in parameter pins (bits that are
 * '1'). Pins corresponding to '0' will be unaffected.
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @param pins pin mask of bits to be toggled.  Each bit represents one pin (LSB
 * is pin 0, MSB is pin 31). For each bit:
 *        - 0: corresponding pin is unaffected
 *        - 1: corresponding pin is toggled
 * Implements : GPIO_HAL_TogglePins_Activity
 */
static inline void GPIO_HAL_TogglePins(GPIO_Type* const baseAddr, const uint32_t pins)
{
	baseAddr->PTOR = GPIO_PTOR_PTTO(pins);
}

/*!
 * @brief Read input pins
 *
 * This function returns the current input values from a port. Only pins
 * configured as input will have meaningful values.
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @return GPIO inputs. Each bit represents one pin (LSB is pin 0, MSB is pin
 * 31). For each bit:
 *        - 0: corresponding pin is read as LOW
 *        - 1: corresponding pin is read as HIGH
 * Implements : GPIO_HAL_ReadPins_Activity
 */
static inline uint32_t GPIO_HAL_ReadPins(const GPIO_Type* const baseAddr)
{
    return (uint32_t)(baseAddr->PDIR);
}

/*!
 * @brief Get the pins directions configuration for a port
 *
 * This function returns the current pins directions for a port. Pins
 * corresponding to bits with value of '1' are configured as output and
 * pins corresponding to bits with value of '0' are configured as input.
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @return GPIO directions. Each bit represents one pin (LSB is pin 0, MSB is
 * pin 31). For each bit:
 *        - 0: corresponding pin is set to input
 *        - 1: corresponding pin is set to output
 * Implements : GPIO_HAL_GetPinsDirection_Activity
 */
static inline uint32_t GPIO_HAL_GetPinsDirection(const GPIO_Type* const baseAddr)
{
    return (uint32_t)(baseAddr->PDDR);
}

/*!
 * @brief Configure the direction for a certain pin from a port
 *
 * This function configures the direction for the given pin, with the
 * given value('1' for pin to be configured as output and '0' for pin to
 * be configured as input)
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @param pinNumber the pin number for which to configure the direction
 * @param pinDirection the pin direction:
 *        - 0: corresponding pin is set to input
 *        - 1: corresponding pin is set to output
 * Implements : GPIO_HAL_SetPinDirection_Activity
 */
static inline void GPIO_HAL_SetPinDirection(GPIO_Type* const baseAddr,
		const uint32_t pinNumber, const uint32_t pinDirection)
{
	uint32_t pinsDirections = (uint32_t) (baseAddr->PDDR);
	pinsDirections &= ~(1UL << pinNumber);
	pinsDirections |= (pinDirection << pinNumber);
	baseAddr->PDDR = GPIO_PDDR_PDD(pinsDirections);
}

/*!
 * @brief Set the pins directions configuration for a port
 *
 * This function sets the direction configuration for all pins
 * in a port. Pins corresponding to bits with value of '1' will be configured as
 * output and pins corresponding to bits with value of '0' will be configured as
 * input.
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @param pins pin mask where each bit represents one pin (LSB
 * is pin 0, MSB is pin 31). For each bit:
 *        - 0: corresponding pin is set to input
 *        - 1: corresponding pin is set to output
 * Implements : GPIO_HAL_SetPinsDirection_Activity
 */
static inline void GPIO_HAL_SetPinsDirection(GPIO_Type* const baseAddr, const uint32_t pins)
{
	baseAddr->PDDR = GPIO_PDDR_PDD(pins);
}

/*!
 * @brief Set the pins input disable state for a port
 *
 * This function sets the pins input state for a port.
 * Pins corresponding to bits with value of '1' will not be configured
 * as input and pins corresponding to bits with value of '0' will be configured
 * as input.
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @param pins pin mask where each bit represents one pin (LSB is pin 0, MSB is
 * pin 31). For each bit:
 *        - 0: corresponding pin is set to input
 *        - 1: corresponding pin is not set to input
 * Implements : GPIO_HAL_SetPortInputDisable_Activity
 */
static inline void GPIO_HAL_SetPortInputDisable(GPIO_Type* const baseAddr, const uint32_t pins)
{
	baseAddr->PIDR = GPIO_PDDR_PDD(pins);
}

/*!
 * @brief Get the pins input disable state for a port
 *
 * This function returns the current pins input state for a port. Pins
 * corresponding to bits with value of '1' are not configured as input and
 * pins corresponding to bits with value of '0' are configured as input.
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @return GPIO input state. Each bit represents one pin (LSB is pin 0, MSB is
 * pin 31). For each bit:
 *        - 0: corresponding pin is set to input
 *        - 1: corresponding pin is not set to input
 * Implements : GPIO_HAL_GetPortInputDisable_Activity
 */
static inline uint32_t GPIO_HAL_GetPortInputDisable(const GPIO_Type* const baseAddr)
{
	return (uint32_t)baseAddr->PIDR;
}

/*! @}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* GPIO_HAL_H*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
