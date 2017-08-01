/******************************************************************************
 * @file
 *
 * Copyright 2012-2017 Specialized Solutions LLC
 *
 * Title to the Materials (contents of this file) remain with Specialized
 * Solutions LLC.  The Materials are copyrighted and are protected by United
 * States copyright laws.  Copyright notices cannot be removed from the
 * Materials.
 *
 * See the file titled "Specialized Solutions LLC License Agreement.txt"
 * that has been distributed with this file for further licensing details.
 *
 * @brief LED module declarations
 *
 *****************************************************************************/

#ifndef LED_H
#define LED_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "datatypes.h"

/******************************************************************************
 * Macros and Constants
 *****************************************************************************/

/******************************************************************************
 * Typedefs
 *****************************************************************************/
typedef enum
{
    LED_TRUCK_0 = 0,
    LED_TRUCK_1,
    LED_TRUCK_2,
    LED_TRUCK_3,

    /* must be last */
    LED_TRUCK_COUNT
} led_truck_t;

typedef enum
{
    LED_RING_0 = 0,
    LED_RING_1,
    LED_RING_2,
    LED_RING_3,
    LED_RING_4,
    LED_RING_5,
    LED_RING_6,
    LED_RING_7,
    LED_RING_8,
    LED_RING_9,
    LED_RING_10,
    LED_RING_11,
    LED_RING_12,
    LED_RING_13,
    LED_RING_14,
    LED_RING_15,
    LED_RING_16,
    LED_RING_17,
    LED_RING_18,
    LED_RING_19,
    LED_RING_20,
    LED_RING_21,
    LED_RING_22,
    LED_RING_23,
    LED_RING_24,
    LED_RING_25,
    LED_RING_26,
    LED_RING_27,
    LED_RING_28,
    LED_RING_29,
    LED_RING_30,
    LED_RING_31,

    /* must be last */
    LED_RING_COUNT
} led_ring_t;

/******************************************************************************
 * Global Variables
 *****************************************************************************/

/******************************************************************************
 * Functions
 *****************************************************************************/
void led_init(void);
void led_set_truck(led_truck_t const led, BOOL enabled);
void led_set_ring(led_ring_t const led, BOOL enabled);
void led_update_ring(void);
void led_ring_shift(uint8_t const steps);
void led_ring_state(BOOL const enabled);
void led_ring_clear(void);
void led_ring_set_array(BOOL const * const ring_state);
#endif

