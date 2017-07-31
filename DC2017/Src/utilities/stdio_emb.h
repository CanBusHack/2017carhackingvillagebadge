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
 * @brief CPU management declarations
 *
 *****************************************************************************/

#ifndef STDIO_EMB_H
#define STDIO_EMB_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "datatypes.h"
#include <stdarg.h>

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
float atof_emb(char *s);

int sprintf_emb(char *out, const char *format, ...);
int vsprintf_emb(char *buf, const char *fmt, va_list args);

unsigned char util_hex2num(char const hex);

#endif

