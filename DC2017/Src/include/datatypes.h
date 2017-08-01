/******************************************************************************
 * @file
 *
 * Copyright 2014 Specialized Solutions LLC
 *
 * Title to the Materials (contents of this file) remain with Specialized
 * Solutions LLC.  The Materials are copyrighted and are protected by United
 * States copyright laws.  Copyright notices cannot be removed from the
 * Materials.
 *
 * See the file titled "Specialized Solutions LLC License Agreement.txt"
 * that has been distributed with this file for further licensing details.
 *
 * @brief Project data type declarations and common operations.
 *        Most types come from stdint.h.
 *
 *****************************************************************************/

#ifndef DATATYPES_H
#define DATATYPES_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>

/******************************************************************************
 * Typedefs
 *****************************************************************************/
typedef enum
{
    FALSE = 0,
    TRUE = 1
} BOOL;

/******************************************************************************
 * Macros and Constants
 *****************************************************************************/

/* MUST BE CHANGED FOR EVERY PUBLIC RELEASE */
#define SOFTWARE_VERSION (0x00000001)

#define BIT0 (1<<0)
#define BIT1 (1<<1)
#define BIT2 (1<<2)
#define BIT3 (1<<3)
#define BIT4 (1<<4)
#define BIT5 (1<<5)
#define BIT6 (1<<6)
#define BIT7 (1<<7)

#define BIT_TEST(val, bitnum)       (((val) & (1 << (bitnum))) > 0)
#define BIT_SET(val, bitnum)        ((val) | (1 << (bitnum)))
#define BIT_CLEAR(val, bitnum)      ((val) & (~(1 << (bitnum))))

#define MAKE_UINT16(msb, lsb)       (((uint16_t)(msb) << 8) + ((uint16_t)(lsb)))
#define MAKE_UINT32(msb, a, b, lsb) (((uint32_t)(msb) << 24) + ((uint32_t)(a) << 16) + ((uint32_t)(b) << 8) + ((uint32_t)(lsb)))

#define NULL ((void *)0)

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif

