
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
#include <stdlib.h>

extern const tImage NXP;
extern const tImage rapid7;
extern const tImage bat_chrg;
extern const tImage bat_full;
extern const tImage bat_75;
extern const tImage bat_50;
extern const tImage bat_25;
extern const tImage bat_empty;

typedef enum
{
    STATE_NXP_LOGO,
    STATE_RAPID7_LOGO
} buckle_state_t;

typedef enum
{
    LED_STATE_DEFAULT,
    LED_STATE_RANDOM,
    LED_STATE_CHASER,
    LED_STATE_ACCEL,
    LED_STATE_COUNT
} led_state_t;

static bool _tap_event_received = false;

/*
 * Task that maintains the LED's
 */
static void led_task(void *pvParameters)
{
    int led = 0;
    uint32_t array_state = 0;
    uint32_t new_state;
    int truck_counter = 0;
    led_state_t led_state = LED_STATE_DEFAULT;
    uint8_t chase_led = 0;
    accel_data_t accel_data;
    uint16_t magnitude;

    srand(0);

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
        if (_tap_event_received)
        {
            _tap_event_received = false;
            led_state++;
            if (led_state >= LED_STATE_COUNT)
            {
                led_state = LED_STATE_DEFAULT;
            }
        }

        switch (led_state)
        {
            case LED_STATE_DEFAULT:
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
                break;

            case LED_STATE_RANDOM:
                for (i = 0; i < 32; i++)
                {
                    led_set_ring(i, rand() % 2);
                }
                led_update_ring();

                for(i = 0; i < LED_TRUCK_COUNT; i++)
                {
                    led_set_truck(i, rand() % 2);
                }

                break;

            case LED_STATE_CHASER:
                led_ring_clear();
                led_set_ring(chase_led++, true);
                led_update_ring();
                if (chase_led >= LED_RING_COUNT)
                {
                    chase_led = 0;
                }
                break;

            case LED_STATE_ACCEL:
                accel_read(&accel_data);
                if (accel_data.accel_y < 0)
                {
                    accel_data.accel_y = -accel_data.accel_y;
                }

                magnitude = accel_data.accel_y / 200;

                led_ring_clear();

                for (i = 0; i < magnitude; i++)
                {
                    led_set_ring(i, true);
                }
                led_update_ring();
                break;

            default:
                led_state = LED_STATE_DEFAULT;
                break;
        }

        vTaskDelay(OS_MS_TO_TICKS(75));

    }
}

static void accel_callback(accel_data_t * const accel_data, bool const tap_event)
{
    if (tap_event)
    {
        _tap_event_received = true;
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


    buckle_state_t state = STATE_NXP_LOGO;

    oled_init();

    accel_init(accel_callback);

    xTaskCreate(led_task, LED_TASK_NAME, LED_TASK_STACK, NULL, LED_TASK_PRIORITY, NULL);

    while(1)
    {
        oled_clear();

        switch (state)
        {
            case STATE_NXP_LOGO:
                graphics_draw_bitmap_mono(0, 16, &NXP);
                state = STATE_RAPID7_LOGO;
                break;

            case STATE_RAPID7_LOGO:
                graphics_draw_bitmap_mono(0, 29, &rapid7);
                state = STATE_NXP_LOGO;
                break;
            default:
                state = STATE_NXP_LOGO;
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
        vTaskDelay(OS_MS_TO_TICKS(2000));


    }

}

/*!
 * @brief Application entry point.
 */
int dc2017_init(void) {

  xTaskCreate(dc2017_task, DC2017_TASK_NAME, DC2017_TASK_STACK, NULL, DC2017_TASK_PRIORITY, NULL);
}

