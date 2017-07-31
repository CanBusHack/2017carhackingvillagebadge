/******************************************************************************
 * @file
 *
 * Copyright 2014-2017 Specialized Solutions LLC
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

#ifndef CPU_H
#define CPU_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "datatypes.h"
#include "fsl_common.h"

/******************************************************************************
 * Macros and Constants
 *****************************************************************************/
#define CPU_PANIC()	cpu_panic(__FILE__, __LINE__)

/******************************************************************************
 * Typedefs
 *****************************************************************************/

typedef uint32_t cpu_int_state_t;

/******************************************************************************
 * Global Variables
 *****************************************************************************/

/******************************************************************************
 * Functions
 *****************************************************************************/

void cpu_init(void);
void cpu_stop(void);

void cpu_panic(char *file_name, uint32_t file_line);
cpu_int_state_t cpu_interrupt_disable(void);
void cpu_interrupt_restore(cpu_int_state_t const prev_state);

#endif

