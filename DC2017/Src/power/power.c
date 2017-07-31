
#include <string.h>

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "pin_mux.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_adc16.h"

static bool _is_charging = false;
static uint16_t _vbat_adc = 0;
/*
 * This task ensures charging only happens when the badge is connected to USB
 */
static void power_task(void *pvParameters)
{
    adc16_config_t adc16ConfigStruct;
    adc16_channel_config_t adc16ChannelConfigStruct;
    uint16_t adc_value;

    gpio_pin_config_t pin_config =
    {
        kGPIO_DigitalOutput, 0,
    };

    /* turn off charge enable */
    pin_config.outputLogic = 1;
    GPIO_PinInit(BOARD_INITPINS_CHARGE_ENA_GPIO, BOARD_INITPINS_CHARGE_ENA_GPIO_PIN, &pin_config);

    ADC16_GetDefaultConfig(&adc16ConfigStruct);
    adc16ConfigStruct.resolution = kADC16_Resolution16Bit;
    ADC16_Init(ADC0, &adc16ConfigStruct);
    ADC16_EnableHardwareTrigger(ADC0, false); /* Make sure the software trigger is used. */
    ADC16_DoAutoCalibration(ADC0);

    adc16ChannelConfigStruct.channelNumber = 16U;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
    adc16ChannelConfigStruct.enableDifferentialConversion = false;



    while (1)
    {
        /* examine external power */
        if (0 == GPIO_ReadPinInput(BOARD_INITPINS_OBD2_DET_GPIO, BOARD_INITPINS_OBD2_DET_GPIO_PIN))
        {
            /* enable charging */
            GPIO_WritePinOutput(BOARD_INITPINS_CHARGE_ENA_GPIO, BOARD_INITPINS_CHARGE_ENA_GPIO_PIN, 0);
            _is_charging = true;

        }
        else
        {
            /* disable charging */
            GPIO_WritePinOutput(BOARD_INITPINS_CHARGE_ENA_GPIO, BOARD_INITPINS_CHARGE_ENA_GPIO_PIN, 1);
            _is_charging = false;
        }

        ADC16_SetChannelConfig(ADC0, 0U, &adc16ChannelConfigStruct);

        while (0U == (kADC16_ChannelConversionDoneFlag &
                      ADC16_GetChannelStatusFlags(ADC0, 0U)))
        {
        }

        _vbat_adc = ADC16_GetChannelConversionValue(ADC0, 0U);

        vTaskDelay(OS_MS_TO_TICKS(500));

    }
}

bool power_is_charging(void)
{
    if (0 == GPIO_ReadPinInput(BOARD_INITPINS_BAT_CHG_GPIO, BOARD_INITPINS_BAT_CHG_GPIO_PIN))
    {
        return true;
    }
    else
    {
        return false;
    }
}

uint8_t power_get_battery_level(void)
{
    /* 100 % is @ 4.2v, 0 % is at 3.2v
     * Resistance divider yields 3.3v at 4.2v.
     * Samples range from 0xFFFF at 4.2v to 0xC30B at 3.2v
     * Just do a simple linear calculation for now.  This could be made much better.
     */

    float percent;

    percent = (((float)_vbat_adc - (float)0xC30B) / (float)0x3CF4) * 100;

    if (percent < 0)
    {
        percent = 0;
    }

    return (uint8_t)percent;
}

void power_init(void)
{
    port_pin_config_t pin_cfg = { 0 };

    pin_cfg.pullSelect = kPORT_PullUp;
    pin_cfg.mux = kPORT_MuxAsGpio;

    PORT_SetPinConfig(BOARD_INITPINS_BAT_CHG_PORT, BOARD_INITPINS_BAT_CHG_GPIO_PIN, &pin_cfg);
    PORT_SetPinConfig(BOARD_INITPINS_BAT_PPR_PORT, BOARD_INITPINS_BAT_PPR_GPIO_PIN, &pin_cfg);

    xTaskCreate(power_task, POWER_TASK_NAME, POWER_TASK_STACK, NULL, POWER_TASK_PRIORITY, NULL);
}


