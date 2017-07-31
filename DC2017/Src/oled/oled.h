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
 * @brief OLED module declarations
 *
 *****************************************************************************/

#ifndef OLED_H
#define OLED_H

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

/******************************************************************************
 * Global Variables
 *****************************************************************************/

/******************************************************************************
 * Functions
 *****************************************************************************/
void oled_init(void);
void oled_write_framebuffer(void);
void oled_put_pixel(uint8_t const x, uint8_t const y);
void oled_clear(void);

#endif

