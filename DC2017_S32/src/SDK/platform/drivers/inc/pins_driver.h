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

#ifndef Pins_DRIVER_H
#define Pins_DRIVER_H

#include <stdint.h>
#include "device_registers.h"
#include "status.h"
#include "port_hal.h"
#include "gpio_hal.h"

 /*! @file */

/*!
  @defgroup pins_driver Pins
  @brief The S32 SDK Pins Driver provides a set of API/services to configure the Port Control and Interrupt IP: PORT.
  @{

  ## Hardware background ##
  <p>
  The Port Control and Interrupt (PORT) module provides support for port control, digital
  filtering, and external interrupt functions. Most functions can be configured independently
  for each pin in the 32-bit port and affect the pin regardless of its pin muxing state.
  There is one instance of the PORT module for each port. Not all pins within each port are
  implemented on a specific device.
  The PORT module has the following features:
    - Pin interrupt
    - Digital input filter
    - Port control
  </p>
  ## Driver consideration ##
  <p>
  The Pins driver is developed on top of the PORT HAL.
  Pins Driver is designed for:
  - Configuration of pin routing/muxing
  - Assignment of signal names to pins (customize pin names)
  - Configuration of pin functional/electrical properties
  </p>
  <p>
  The graphical user interface for routing functionality consists of two main views: Collapsed View and Pins View.
  </p>
  <p>
  The Collapsed view is designed for pin routing configuration.
  The Pin/Signal Selection column displays the list of pins that can be configured for selected
  Function or Signal. If not configured, "No pin routed" is displayed, which means no user
  requirement for configuration. You can see the popup menu for setting on/off automatic
  value. The Direction column contains the list of directions that can be configured for selected
  function or signal, if direction configuration is supported. For fixed direction signals, the
  direction text is displayed and grayed-out.
  </p>
  <p>
  The Pins view allows the configuration of electrical properties of pins and displays all pins. It
  displays the pad configuration that is available in Processor view where each pin is associated with
  signal name and function. If you want to view specific pin, use Pin Filter. It filters the User Pin/Signal Name
  column as shown below. You can also sort the first two columns, Pin and User Pin/Signal Name.
  </p>
  <p>
  Functional Properties view allows configuration of functional properties on pins. The user graphical interface is mapped
  over the port control features.
  </p>
  <p>
  The Driver uses structures for configuration. The user application can use the
  default for most settings, changing only what is necessary. All port control
  features are part of data structure configuration.Please see pin_settings_config_t.
  </p>
  <p>
  This is an example to write a pin configuration:
  @code

  User number of configured pins
  #define NUM_OF_CONFIGURED_PINS 1

  Array of pin configuration structures
  pin_settings_config_t g_pin_mux_InitConfigArr[NUM_OF_CONFIGURED_PINS] = {
    {
           .base          = PORTE,
           .pinPortIdx    = 16u,
           .pullConfig    = PORT_INTERNAL_PULL_UP_ENABLED,
           .passiveFilter = false,
           .driveSelect   = PORT_LOW_DRIVE_STRENGTH,
           .mux           = PORT_MUX_AS_GPIO,
           .pinLock       = false,
           .intConfig     = PORT_DMA_INT_DISABLED,
           .clearIntFlag  = false,
    }
  };

  write pins configuration
  PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

  @endcode
  </p>
 */



/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*!
 * @brief Configures the port data direction
 * Implements : port_data_direction_t_Class
 */
typedef enum {
    GPIO_INPUT_DIRECTION       = 0x0U,  /*!< General purpose input direction. */
    GPIO_OUTPUT_DIRECTION      = 0x1U,  /*!< General purpose output direction. */
    GPIO_UNSPECIFIED_DIRECTION = 0x2U,  /*!< General purpose unspecified direction. */
} port_data_direction_t;

/*!
 * @brief Defines the converter configuration
 *
 * This structure is used to configure the pins
 * Implements : pin_settings_config_t_Class
 */
typedef struct
{
    PORT_Type *             base;          /*!< Port base pointer.                        */
    uint32_t                pinPortIdx;    /*!< Port pin number.                          */
#if FEATURE_PORT_HAS_PULL_SELECTION
    port_pull_config_t      pullConfig;    /*!< Internal resistor pull feature selection. */
#endif
#if FEATURE_PORT_HAS_SLEW_RATE
    port_slew_rate_t        rateSelect;    /*!< Slew rate selection.                      */
#endif
#if FEATURE_PORT_HAS_PASSIVE_FILTER
    bool                    passiveFilter; /*!< Passive filter configuration.             */
#endif
#if FEATURE_PORT_HAS_OPEN_DRAIN
    bool                    openDrain;     /*!< Enable open drain or not.                 */
#endif
#if FEATURE_PORT_HAS_DRIVE_STRENGTH
    port_drive_strength_t   driveSelect;   /*! @brief Configures the drive strength.      */
#endif
    port_mux_t              mux;           /*! @brief Pin mux selection.                  */
#if FEATURE_PORT_HAS_PIN_CONTROL_LOCK
    bool                    pinLock;       /*! Lock pin control register or not.          */
#endif
    port_interrupt_config_t intConfig;     /*! Interrupt generation condition.            */
    bool                    clearIntFlag;  /*! Clears the interrupt status flag.          */
    GPIO_Type *             gpioBase;      /*!< GPIO base pointer.                        */
    port_data_direction_t   direction;     /*! Configures the port data direction.        */

} pin_settings_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
 /*!
 * @name Pins DRV.
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the pins with the given configuration structure
 *
 * This function configures the pins with the options provided in the
 * provided structure.
 *
 * @param[in] pin_count the number of configured pins in structure
 * @param[in] config the configuration structure
 * @return the status of the operation
 */
status_t PINS_DRV_Init(const uint32_t pin_count, const pin_settings_config_t config[]);


/*! @}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* Pins_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
