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
 * System CAN bus settings:
 *
 * @brief CANBUS Management Routines
 *
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "fsl_flexcan.h"
#include "datatypes.h"
#include "cpu.h"
#include "can.h"

/******************************************************************************
 * Macros and Constants
 *****************************************************************************/

#define TX_MESSAGE_BUFFER_NUM (15)

#define NUM_FIFO_IDS 32

/******************************************************************************
 * Typedefs
 *****************************************************************************/

/******************************************************************************
 * Local Function Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/
static uint32_t _fifo_id_table[NUM_FIFO_IDS];

/******************************************************************************
 * Global Variables
 *****************************************************************************/

/******************************************************************************
 * Functions
 *****************************************************************************/

 void can_init(uint32_t const baud_rate)

{
    flexcan_rx_fifo_config_t fifo_config;

	flexcan_config_t flexcanConfig;
	flexcan_rx_mb_config_t mbConfig;

	FLEXCAN_GetDefaultConfig(&flexcanConfig);

    flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
	flexcanConfig.baudRate = baud_rate;

	FLEXCAN_Init(CAN0, &flexcanConfig, CLOCK_GetFreq(kCLOCK_BusClk));

    fifo_config.idFilterNum = NUM_FIFO_IDS;
    fifo_config.idFilterTable = _fifo_id_table;
    fifo_config.idFilterType = kFLEXCAN_RxFifoFilterTypeA;
    fifo_config.priority = kFLEXCAN_RxFifoPrioLow;

    memset(_fifo_id_table, 0, sizeof(_fifo_id_table));

    FLEXCAN_SetRxFifoConfig(CAN0, &fifo_config, false);

    FLEXCAN_SetTxMbConfig(CAN0, TX_MESSAGE_BUFFER_NUM, true);
}

void can_configure_id_filter(uint8_t const filter_id, uint32_t const can_id, BOOL const ide)
{
    flexcan_rx_fifo_config_t fifo_config;

    if (filter_id < NUM_FIFO_IDS)
    {
        fifo_config.idFilterNum = NUM_FIFO_IDS;
        fifo_config.idFilterTable = _fifo_id_table;
        fifo_config.idFilterType = kFLEXCAN_RxFifoFilterTypeA;
        fifo_config.priority = kFLEXCAN_RxFifoPrioLow;

        /* populate the element in the filter table */
        if (ide)
        {
            _fifo_id_table[filter_id] = FLEXCAN_RX_FIFO_EXT_MASK_TYPE_A(can_id, 0, 0);

        }
        else
        {
            _fifo_id_table[filter_id] = FLEXCAN_RX_FIFO_STD_MASK_TYPE_A(can_id, 0, 1);
        }

        FLEXCAN_SetRxFifoConfig(CAN0, &fifo_config, true);
    }

}

BOOL can_send_message(can_msg_t const * const tx_message, uint32_t const timeout_ms)
{
    TickType_t timeout = xTaskGetTickCount() + OS_MS_TO_TICKS(timeout_ms);
	flexcan_frame_t tx_frame;

    if (tx_message->is_extended)
    {
        tx_frame.format = kFLEXCAN_FrameFormatExtend;
        tx_frame.id = FLEXCAN_ID_EXT(tx_message->id);
    }
    else
    {
        tx_frame.format = kFLEXCAN_FrameFormatStandard;
        tx_frame.id = FLEXCAN_ID_STD(tx_message->id);
    }

	tx_frame.type = kFLEXCAN_FrameTypeData;
	tx_frame.length = tx_message->dlc;
	tx_frame.dataWord0 = MAKE_UINT32(tx_message->data[0], tx_message->data[1], tx_message->data[2], tx_message->data[3]);
	tx_frame.dataWord1 = MAKE_UINT32(tx_message->data[4], tx_message->data[5], tx_message->data[6], tx_message->data[7]);

    while ((kStatus_Fail == FLEXCAN_WriteTxMb(CAN0, TX_MESSAGE_BUFFER_NUM, &tx_frame)) &&
	           (xTaskGetTickCount() <= timeout))
    {
        vTaskDelay(OS_MS_TO_TICKS(1));
    }

    if (xTaskGetTickCount() > timeout)
    {
        return false;
    }

    return true;
}

BOOL can_receive_message(can_msg_t * const rx_message)
{
	BOOL ret_val = FALSE;
	flexcan_frame_t rxFrame;

    if (FLEXCAN_GetMbStatusFlags(CAN0, kFLEXCAN_RxFifoFrameAvlFlag))
    {
		if (kStatus_Success == FLEXCAN_ReadRxFifo(CAN0, &rxFrame))
		{

            if (rxFrame.format == kFLEXCAN_FrameFormatStandard)
            {
                rx_message->is_extended = false;
                rx_message->id = (rxFrame.id & CAN_ID_STD_MASK) >> CAN_ID_STD_SHIFT;
            }
            else
            {
                rx_message->is_extended = true;
                rx_message->id = (rxFrame.id & CAN_ID_EXT_MASK) >> CAN_ID_EXT_SHIFT;
            }

            rx_message->dlc = rxFrame.length;
            rx_message->data[0] = rxFrame.dataByte0;
            rx_message->data[1] = rxFrame.dataByte1;
            rx_message->data[2] = rxFrame.dataByte2;
            rx_message->data[3] = rxFrame.dataByte3;
            rx_message->data[4] = rxFrame.dataByte4;
            rx_message->data[5] = rxFrame.dataByte5;
            rx_message->data[6] = rxFrame.dataByte6;
            rx_message->data[7] = rxFrame.dataByte7;
            rx_message->filter_id = rxFrame.idhit;

            FLEXCAN_ClearMbStatusFlags(CAN0, kFLEXCAN_RxFifoFrameAvlFlag);

            ret_val = TRUE;
		}
    }

	return ret_val;

}
