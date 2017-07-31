/* ###################################################################
**     Filename    : main.c
**     Processor   : S32K144
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.00
** @brief
**         Main module.
**         This module contains user's application code.
*/
/*!
**  @addtogroup main_module main module documentation
**  @{
*/
/* MODULE main */


/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "pin_mux.h"
#include "clockMan1.h"
#include "canCom1.h"
#include "dmaController1.h"
#include "osif1.h"
#include "canCom2.h"
#include "canCom3.h"
#include "lpuart1.h"
#include "lpspiCom1.h"
#include "pwrMan1.h"
#include "flexTimer1.h"
#include "crc1.h"
#include "csec1.h"
#include "gpio_hal.h"
#include "pcc_hal.h"
#include <string.h>

static void send_can_frame(uint8_t channel)
{

    uint8_t data[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 , 11, 12, 13, 14, 15};

    flexcan_data_info_t dataInfo =
    {
            .data_length = 8,
            .msg_id_type = FLEXCAN_MSG_ID_STD,
            .enable_brs  = false,
            .fd_enable   = false,
            .fd_padding  = 0U
    };

    if (channel == 0)
    {
        dataInfo.fd_enable = true;
        dataInfo.data_length = 16;
        dataInfo.enable_brs  = true;
    }

    /* Configure TX message buffer with index TX_MSG_ID and TX_MAILBOX*/
    switch (channel)
    {
        case 0:
            FLEXCAN_DRV_ConfigTxMb(INST_CANCOM1, 1, &dataInfo, 0x100);

            /* Execute send non-blocking */
            FLEXCAN_DRV_Send(INST_CANCOM1, 1, &dataInfo, 0x100, data);
            break;
        case 1:
            FLEXCAN_DRV_ConfigTxMb(INST_CANCOM2, 1, &dataInfo, 0x100);

            /* Execute send non-blocking */
            FLEXCAN_DRV_Send(INST_CANCOM2, 1, &dataInfo, 0x100, data);
            break;
        case 2:
            FLEXCAN_DRV_ConfigTxMb(INST_CANCOM3, 1, &dataInfo, 0x100);

            /* Execute send non-blocking */
            FLEXCAN_DRV_Send(INST_CANCOM3, 1, &dataInfo, 0x100, data);
            break;
    }
}

static void setup_cpu(void)
{
    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,  g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0, CLOCK_MANAGER_POLICY_FORCIBLE);

    /* HSRUN mode */
    POWER_SYS_Init(powerConfigsArr, POWER_MANAGER_CONFIG_CNT, powerStaticCallbacksConfigsArr, POWER_MANAGER_CALLBACK_CNT);
    POWER_SYS_SetMode(1, POWER_MANAGER_POLICY_FORCIBLE);

    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
}

/*!
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
 * - startup asm routine
 * - main()
*/
int main(void)
{
    uint32_t bytesRemaining;
    uint8_t buffer[2];
    volatile int i;

    setup_cpu();

    FLEXCAN_DRV_Init(INST_CANCOM1, &canCom1_State, &canCom1_InitConfig0);
    FLEXCAN_DRV_Init(INST_CANCOM2, &canCom2_State, &canCom2_InitConfig0);
    FLEXCAN_DRV_Init(INST_CANCOM3, &canCom3_State, &canCom3_InitConfig0);

    //FLEXCAN_DRV_ConfigRxMb(INST_CANCOM3, 0, &dataInfo, 0x100);

    while (1)
    {
        send_can_frame(1);

        for (i = 0; i < 50000; i++);

        __asm("NOP");
    }

    return 0;
}

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the Freescale S32K series of microcontrollers.
**
** ###################################################################
*/
