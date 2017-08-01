
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
#include "accel.h"
#include "can.h"

typedef enum
{
    TEST_STATE_ACCEL,
    TEST_STATE_ACCEL_WAIT,
    TEST_STATE_CAN,
    TEST_STATE_CAN_WAIT,
    TEST_STATE_SD_CARD,
    TEST_STATE_ENET,
    TEST_STATE_FAILED,
    TEST_STATE_DONE
} test_state_t;

static bool accel_callback_received = false;

extern const tImage bat_chrg;
extern const tImage bat_full;
extern const tImage bat_75;
extern const tImage bat_50;
extern const tImage bat_25;
extern const tImage bat_empty;

/*
 * Task that maintains the LED's
 */
static void led_task(void *pvParameters)
{
    int led = 0;
    uint32_t array_state = 0;
    uint32_t new_state;
    int truck_counter = 0;

    bool temp;
    int i;

    for (i = 0; i < 32; i++)
    {
        if ((i % 4) == 0)
        {
            array_state |= (1 << i);
        }
    }

    led_init();
    led_ring_clear();

    led_ring_state(TRUE);

    led_set_truck(LED_TRUCK_0, FALSE);
    led_set_truck(LED_TRUCK_1, FALSE);
    led_set_truck(LED_TRUCK_2, FALSE);
    led_set_truck(LED_TRUCK_3, FALSE);


    while (1)
    {
        led_ring_clear();
        for (i = 0; i < 32; i++)
        {
            led_set_ring(i, ((array_state & (1 << i)) > 0));
        }
        led_update_ring();

        led_set_truck(truck_counter, false);
        truck_counter = (truck_counter + 1) % LED_TRUCK_COUNT;
        led_set_truck(truck_counter, true);

        asm("mov %[result], %[value], ror #31" : [result] "=r" (new_state) : [value] "r" (array_state));

        array_state = new_state;

        vTaskDelay(OS_MS_TO_TICKS(75));

    }
}

static void accel_callback(accel_data_t * const accel_data, bool tap_event)
{
    if (tap_event)
    {
        accel_callback_received = true;
    }
}


/*
 * DC2017 belt buckle default program
 */
static void dc2017_task(void *pvParameters)
{
    uint8_t battery_level;
    accel_data_t accel_data;
    char buffer[40];
    can_msg_t tx_msg, rx_msg;

    test_state_t state = TEST_STATE_ACCEL;

    oled_init();

    accel_init(accel_callback);

    xTaskCreate(led_task, LED_TASK_NAME, LED_TASK_STACK, NULL, LED_TASK_PRIORITY, NULL);

    while(1)
    {
        oled_clear();

        switch (state)
        {
            case TEST_STATE_ACCEL:
                graphics_put_text("Accel Test", 0, 0, &font_medium);
                state = TEST_STATE_ACCEL_WAIT;
                break;

            case TEST_STATE_ACCEL_WAIT:
                accel_read(&accel_data);
                graphics_put_text("Accel Test", 0, 0, &font_medium);
                sprintf_emb(buffer, "X:%6d M:%d", accel_data.accel_x, accel_data.magn_x);
                graphics_put_text(buffer, 0, 15, &font_medium);
                sprintf_emb(buffer, "Y:%6d M:%d", accel_data.accel_y, accel_data.magn_y);
                graphics_put_text(buffer, 0, 30, &font_medium);
                sprintf_emb(buffer, "Z:%6d M:%d", accel_data.accel_z, accel_data.magn_z);
                graphics_put_text(buffer, 0, 45, &font_medium);

                if (accel_callback_received)
                {
                    accel_callback_received = false;
                    state = TEST_STATE_CAN;
                }
                break;

            case TEST_STATE_CAN:
                graphics_put_text("CAN Test", 0, 0, &font_medium);
                oled_write_framebuffer();
                can_init(125000);
                can_configure_id_filter(0, 0x234, true);
                memset(&tx_msg, 0, sizeof(tx_msg));

                tx_msg.id = 0x123;
                tx_msg.is_extended = true;
                tx_msg.dlc = 8;
                tx_msg.data[0] = 0x11;
                tx_msg.data[1] = 0x22;
                tx_msg.data[2] = 0x33;
                tx_msg.data[3] = 0x44;
                tx_msg.data[4] = 0x55;
                tx_msg.data[5] = 0x66;
                tx_msg.data[6] = 0x77;
                tx_msg.data[7] = 0x88;

                can_send_message(&tx_msg, 100);

                state = TEST_STATE_CAN_WAIT;

                break;

            case TEST_STATE_CAN_WAIT:
                can_send_message(&tx_msg, 100);

                if (can_receive_message(&rx_msg))
                {
                    state = TEST_STATE_SD_CARD;
                }
                else
                {
                    graphics_put_text("CAN Wait RX", 0, 0, &font_medium);
                }
                break;

            case TEST_STATE_SD_CARD:
                graphics_put_text("SD Test", 0, 0, &font_medium);
                oled_write_framebuffer();
                SYSMPU_Enable(SYSMPU, 0);

                if (sdcard_test())
                {
                    state = TEST_STATE_ENET;
                }
                else
                {
                    graphics_put_text("SD Test Fail", 0, 0, &font_medium);
                    oled_write_framebuffer();
                    vTaskDelay(OS_MS_TO_TICKS(1000));
                    state = TEST_STATE_ENET;
                }
                break;

            case TEST_STATE_ENET:
                graphics_put_text("Ethernet Test", 0, 0, &font_medium);
                oled_write_framebuffer();
                ethernet_init();

                while (1)
                {
                    vTaskDelay(1000);
                }

                state = TEST_STATE_DONE;
                break;

            case TEST_STATE_DONE:
            case TEST_STATE_FAILED:
            default:
                break;
        }

        if (power_is_charging())
        {
            //graphics_put_text("Charging", 0, 0, &font_medium);
            graphics_draw_bitmap_mono(112, 4, &bat_chrg);
        }
        else
        {
            battery_level = power_get_battery_level();

            if (battery_level > 75)
            {
                graphics_draw_bitmap_mono(112,4, &bat_full);
            }
            else if (battery_level > 50)
            {
                graphics_draw_bitmap_mono(112,4, &bat_75);
            }
            else if (battery_level > 25)
            {
                graphics_draw_bitmap_mono(112,4, &bat_50);
            }
            else if (battery_level > 10)
            {
                graphics_draw_bitmap_mono(112,4, &bat_25);
            }
            else
            {
                graphics_draw_bitmap_mono(112,4, &bat_empty);
            }
        }

        oled_write_framebuffer();
        vTaskDelay(OS_MS_TO_TICKS(100));


    }

}

/*!
 * @brief Application entry point.
 */
int dc2017_init(void) {

  xTaskCreate(dc2017_task, DC2017_TASK_NAME, DC2017_TASK_STACK, NULL, DC2017_TASK_PRIORITY, NULL);
}

