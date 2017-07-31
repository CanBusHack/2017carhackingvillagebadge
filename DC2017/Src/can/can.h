/******************************************************************************
 * @file
 *
 * Copyright 2012-2014 Specialized Solutions LLC
 *
 * Title to the Materials (contents of this file) remain with Specialized
 * Solutions LLC.  The Materials are copyrighted and are protected by United
 * States copyright laws.  Copyright notices cannot be removed from the
 * Materials.
 *
 * See the file titled "Specialized Solutions LLC License Agreement.txt"
 * that has been distributed with this file for further licensing details.
 *
 * @brief CAN module declarations
 *
 *****************************************************************************/

#ifndef CAN_H
#define CAN_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "datatypes.h"
#include "cpu.h"

/******************************************************************************
 * Macros and Constants
 *****************************************************************************/

/******************************************************************************
 * Typedefs
 *****************************************************************************/
typedef struct
{
    uint32_t id;
    BOOL is_extended;
    uint8_t dlc;
    uint8_t data[8];
    uint8_t filter_id;
} can_msg_t;

/******************************************************************************
 * Global Variables
 *****************************************************************************/


/******************************************************************************
 * Functions
 *****************************************************************************/

void can_init(uint32_t baud_rate);

void can_configure_id_filter(uint8_t const filter_id, uint32_t const can_id, BOOL const ide);
BOOL can_receive_message(can_msg_t * const rx_message);
BOOL can_send_message(can_msg_t const * const tx_message, uint32_t const timeout_ms);

#endif

