/******************************************************************************
 * @file
 *
 * Copyright 2014-2016 Specialized Solutions LLC
 *
 * Title to the Materials (contents of this file) remain with Specialized
 * Solutions LLC.  The Materials are copyrighted and are protected by United
 * States copyright laws.  Copyright notices cannot be removed from the
 * Materials.
 *
 * @brief LED management routines
 *
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "datatypes.h"
#include "cpu.h"
#include "led.h"
#include "pin_mux.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_ftm.h"

#include "FreeRTOS.h"
#include "task.h"

/******************************************************************************
 * Macros and Constants
 *****************************************************************************/
#define PULSE_DURATION_CYCLES (100)
#define PWM_FREQUENCY_HZ (100)
#define PWM_DUTY_PCT (20)

/******************************************************************************
 * Typedefs
 *****************************************************************************/

/******************************************************************************
 * Local Function Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/
static BOOL _ring_states[LED_RING_COUNT] = { 0 };

/******************************************************************************
 * Global Variables
 *****************************************************************************/

/******************************************************************************
 * Functions
 *****************************************************************************/
static void delay(void)
{
    volatile int i = 0;

    for (i = 0; i < PULSE_DURATION_CYCLES; i++);
}

static void apply_ring_changes(void)
{
    int i;

    /* pulse clear line */
    GPIO_WritePinOutput(BOARD_INITPINS_LED_CLR_GPIO, BOARD_INITPINS_LED_CLR_GPIO_PIN, 0);
    delay();
    GPIO_WritePinOutput(BOARD_INITPINS_LED_CLR_GPIO, BOARD_INITPINS_LED_CLR_GPIO_PIN, 1);

    for (i = LED_RING_COUNT; i > 0; i--)
    {
        /* Assert data line */
        GPIO_WritePinOutput(BOARD_INITPINS_LED_DATA_GPIO, BOARD_INITPINS_LED_DATA_GPIO_PIN, _ring_states[(i - 1)]);
        delay();

        /* pulse shift clock line */
        GPIO_WritePinOutput(BOARD_INITPINS_LED_SH_CLK_GPIO, BOARD_INITPINS_LED_SH_CLK_GPIO_PIN, 1);
        delay();
        GPIO_WritePinOutput(BOARD_INITPINS_LED_SH_CLK_GPIO, BOARD_INITPINS_LED_SH_CLK_GPIO_PIN, 0);
    }

    /* pulse store clock */
    GPIO_WritePinOutput(BOARD_INITPINS_LED_SH_CLK_GPIO, BOARD_INITPINS_LED_ST_CLK_GPIO_PIN, 1);
    delay();
    GPIO_WritePinOutput(BOARD_INITPINS_LED_SH_CLK_GPIO, BOARD_INITPINS_LED_ST_CLK_GPIO_PIN, 0);
}

static void control_ftm(FTM_Type *ftm, ftm_chnl_t const channel, uint8_t const level, bool enabled)
{
    if (enabled)
    {
        /* start PWM on LED_OE */
        FTM_UpdateChnlEdgeLevelSelect(ftm, channel, 0U);

        /* Update PWM duty cycle */
        FTM_UpdatePwmDutycycle(ftm, channel, kFTM_CenterAlignedPwm, PWM_DUTY_PCT);

        /* Software trigger to update registers */
        FTM_SetSoftwareTrigger(ftm, true);

        /* Start channel output with updated dutycycle */
        FTM_UpdateChnlEdgeLevelSelect(ftm, channel, level);
    }
    else
    {
        /* turn off LED_OE */
        FTM_UpdateChnlEdgeLevelSelect(ftm, channel, 0U);

        /* Update PWM duty cycle */
        FTM_UpdatePwmDutycycle(ftm, channel, kFTM_CenterAlignedPwm, 0);

        /* Software trigger to update registers */
        FTM_SetSoftwareTrigger(ftm, true);

        /* Start channel output with updated dutycycle */
        FTM_UpdateChnlEdgeLevelSelect(ftm, channel, level);
    }
}

void led_init(void)
{
    gpio_pin_config_t pin_config = {
        kGPIO_DigitalOutput, 0,
    };

    /* setup PWM output on FTM3_CH0 */
    ftm_config_t ftmInfo;
    ftm_chnl_pwm_signal_param_t ftmParam;

    /* setup default levels on control lines */
    pin_config.outputLogic = 0;
    GPIO_PinInit(BOARD_INITPINS_LED_CLR_GPIO, BOARD_INITPINS_LED_CLR_GPIO_PIN, &pin_config);
    pin_config.outputLogic = 0;
    GPIO_PinInit(BOARD_INITPINS_LED_DATA_GPIO, BOARD_INITPINS_LED_DATA_GPIO_PIN, &pin_config);
    GPIO_PinInit(BOARD_INITPINS_LED_SH_CLK_GPIO, BOARD_INITPINS_LED_SH_CLK_GPIO_PIN, &pin_config);
    GPIO_PinInit(BOARD_INITPINS_LED_ST_CLK_GPIO, BOARD_INITPINS_LED_ST_CLK_GPIO_PIN, &pin_config);

    ftmParam.chnlNumber = kFTM_Chnl_0;
    ftmParam.level = kFTM_LowTrue;
    ftmParam.dutyCyclePercent = PWM_DUTY_PCT;
    ftmParam.firstEdgeDelayPercent = 0U;

    FTM_GetDefaultConfig(&ftmInfo);
    ftmInfo.prescale = kFTM_Prescale_Divide_64;

    /* Initialize FTM module for ring */
    FTM_Init(FTM3, &ftmInfo);

    FTM_SetupPwm(FTM3, &ftmParam, 1U, kFTM_CenterAlignedPwm, PWM_FREQUENCY_HZ, CLOCK_GetFreq(kCLOCK_BusClk));

    FTM_StartTimer(FTM3, kFTM_SystemClock);

    /* Init FTM for truck LED's */
    FTM_Init(FTM0, &ftmInfo);
    ftmParam.chnlNumber = kFTM_Chnl_0;
    ftmParam.level = kFTM_HighTrue;
    FTM_SetupPwm(FTM0, &ftmParam, 1U, kFTM_CenterAlignedPwm, PWM_FREQUENCY_HZ, CLOCK_GetFreq(kCLOCK_BusClk));
    ftmParam.chnlNumber = kFTM_Chnl_1;
    FTM_SetupPwm(FTM0, &ftmParam, 1U, kFTM_CenterAlignedPwm, PWM_FREQUENCY_HZ, CLOCK_GetFreq(kCLOCK_BusClk));
    ftmParam.chnlNumber = kFTM_Chnl_3;
    FTM_SetupPwm(FTM0, &ftmParam, 1U, kFTM_CenterAlignedPwm, PWM_FREQUENCY_HZ, CLOCK_GetFreq(kCLOCK_BusClk));
    ftmParam.chnlNumber = kFTM_Chnl_6;
    FTM_SetupPwm(FTM0, &ftmParam, 1U, kFTM_CenterAlignedPwm, PWM_FREQUENCY_HZ, CLOCK_GetFreq(kCLOCK_BusClk));

    FTM_StartTimer(FTM0, kFTM_SystemClock);

    /* de-assert clear on ring */
    GPIO_WritePinOutput(BOARD_INITPINS_LED_CLR_GPIO, BOARD_INITPINS_LED_CLR_GPIO_PIN, 1);

}

void led_set_truck(led_truck_t const led, BOOL enabled)
{
    if (led < LED_TRUCK_COUNT)
    {
        switch (led)
        {
            case LED_TRUCK_0:
                control_ftm(FTM0, kFTM_Chnl_1, kFTM_HighTrue, enabled);
                break;
            case LED_TRUCK_1:
                control_ftm(FTM0, kFTM_Chnl_6, kFTM_HighTrue, enabled);
                break;
            case LED_TRUCK_2:
                control_ftm(FTM0, kFTM_Chnl_3, kFTM_HighTrue, enabled);
                break;
            case LED_TRUCK_3:
                control_ftm(FTM0, kFTM_Chnl_0, kFTM_HighTrue, enabled);
                break;
        }
    }
}

void led_set_ring(led_ring_t const led, BOOL enabled)
{
    if (led < LED_RING_COUNT)
    {
        /* the first 8 led's are backwards on the board (oops - no time to fix for easy shifting :-( ) */
        if (led < 8)
        {
            _ring_states[7 - led] = enabled;
        }
        else
        {
            _ring_states[led] = enabled;
        }
    }
}

void led_update_ring(void)
{
    apply_ring_changes();
}

void led_ring_shift(uint8_t const steps)
{
    /* pulse shift clock */
    GPIO_WritePinOutput(BOARD_INITPINS_LED_SH_CLK_GPIO, BOARD_INITPINS_LED_SH_CLK_GPIO_PIN, 1);
    delay();
    GPIO_WritePinOutput(BOARD_INITPINS_LED_SH_CLK_GPIO, BOARD_INITPINS_LED_SH_CLK_GPIO_PIN, 0);

    /* pulse store clock */
    GPIO_WritePinOutput(BOARD_INITPINS_LED_SH_CLK_GPIO, BOARD_INITPINS_LED_ST_CLK_GPIO_PIN, 1);
    delay();
    GPIO_WritePinOutput(BOARD_INITPINS_LED_SH_CLK_GPIO, BOARD_INITPINS_LED_ST_CLK_GPIO_PIN, 0);

}

void led_ring_state(BOOL const enabled)
{
    control_ftm(FTM3, kFTM_Chnl_0, kFTM_LowTrue, enabled);
}

void led_ring_clear(void)
{
    int i;

    for (i = 0; i < LED_RING_COUNT; i++)
    {
        _ring_states[i] = FALSE;
    }
}

void led_ring_set_array(BOOL const * const ring_state)
{
    memcpy(_ring_states, ring_state, LED_RING_COUNT);

    apply_ring_changes();
}
