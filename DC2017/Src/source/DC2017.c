
#include <string.h>

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "led.h"
#include "graphics.h"
#include "oled.h"
#include "resources.h"
#include "accel.h"
#include "stdio_emb.h"
#include "fsl_uart.h"
#include "fsl_sysmpu.h"

#include "pin_mux.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

#include "power.h"

static void hello_task(void *pvParameters)
{
    SYSMPU_Enable(SYSMPU, 0);
    usb_init();
    ethernet_init();

    while (1)
    {
        vTaskDelay(10000);

    }

    return;


#if 1
    uart_config_t config;

    accel_data_t accel_data;
    char buffer[30];
    int led = 0;
    int counter = 0;
    led_init();
    accel_init(NULL);
    led_ring_clear();

    led_ring_state(TRUE);

    usb_init();
#if 0
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = 6000000;
    config.enableTx = true;
    config.enableRx = true;

    UART_Init(UART0, &config, CLOCK_GetFreq(UART0_CLK_SRC));
#endif


    ethernet_init();

    led_set_truck(LED_TRUCK_0, TRUE);
    led_set_truck(LED_TRUCK_1, TRUE);
    led_set_truck(LED_TRUCK_2, TRUE);
    led_set_truck(LED_TRUCK_3, TRUE);

    PRINTF("Hello world\r\n");

    oled_init();
    graphics_put_text("Truck Overlay Test", 0, 0, &font_medium);
    oled_write_framebuffer();

    while(1)
    {
        led_ring_clear();
        led_set_ring(led, TRUE);
        led_set_ring(led + 4, TRUE);
        led_set_ring(led + 8, TRUE);
        led_set_ring(led + 12, TRUE);
        led_set_ring(led + 16, TRUE);
        led_set_ring(led + 20, TRUE);
        led_set_ring(led + 24, TRUE);
        led_set_ring(led + 28, TRUE);

        led_update_ring();

        led++;

        accel_read(&accel_data);
        oled_clear();

        graphics_put_text("Truck Overlay Test", 0, 0, &font_medium);

        sprintf_emb(buffer, "X:%6d M:%d", accel_data.accel_x, accel_data.magn_x);
        graphics_put_text(buffer, 0, 15, &font_medium);
        sprintf_emb(buffer, "Y:%6d M:%d", accel_data.accel_y, accel_data.magn_y);
        graphics_put_text(buffer, 0, 30, &font_medium);
        sprintf_emb(buffer, "Z:%6d M:%d", accel_data.accel_z, accel_data.magn_z);
        graphics_put_text(buffer, 0, 45, &font_medium);

        oled_write_framebuffer();

#if 0
        buffer[0] = 0xA5;
        UART_WriteBlocking(UART0, buffer, 1);
        UART_ReadBlocking(UART0, &(buffer[1]), 1);
#endif

        vTaskDelay(OS_MS_TO_TICKS(100));

        if (led == 8)
        {
            led = 0;
        }






    }

#else
    SYSMPU_Enable(SYSMPU, 0);
    sdcard_test();

    while(1)
    {
        __asm("NOP");
    }
#endif

}

/*!
 * @brief Application entry point.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    usb_init();

    /* Create RTOS task */
    dc2017_init();
    power_init();

    //xTaskCreate(hello_task, "hello", 1024, NULL, 3, NULL);
    vTaskStartScheduler();

    for(;;)   /* Infinite loop to avoid leaving the main function */
    {
        __asm("NOP"); /* something to use as a breakpoint stop while looping */
    }
}


void vApplicationIdleHook( void )
{
    SMC_SetPowerModeWait(SMC);
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
    while(1)
    {

    }
}
