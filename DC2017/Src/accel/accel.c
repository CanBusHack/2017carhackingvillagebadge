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
 * @brief Accelerometer management routines
 *
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "datatypes.h"
#include "cpu.h"
#include "accel.h"
#include "pin_mux.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_ftm.h"
#include "fsl_i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/******************************************************************************
 * Macros and Constants
 *****************************************************************************/
#define FXOS8700CQ_ADDR (0x1E)
#define FXOS8700CQ_STATUS 0x00
#define FXOS8700CQ_XYZ_DATA_CFG 0x0E
#define FXOS8700CQ_CTRL_REG1 0x2A
#define FXOS8700CQ_CTRL_REG4 0x2D
#define FXOS8700CQ_CTRL_REG5 0x2E
#define FXOS8700CQ_M_CTRL_REG1 0x5B
#define FXOS8700CQ_M_CTRL_REG2 0x5C

/******************************************************************************
 * Typedefs
 *****************************************************************************/

/******************************************************************************
 * Local Function Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/
static i2c_master_handle_t g_m_handle;
static volatile bool completionFlag = false;
static volatile bool nakFlag = false;
static SemaphoreHandle_t xSemaphore;
static accel_data_t _accel_data = { 0 };
static accel_callback_t _accel_callback = NULL;
static volatile bool _tap_event_received = false;
static volatile bool _data_ready_received = false;
/******************************************************************************
 * Global Variables
 *****************************************************************************/

/******************************************************************************
 * Functions
 *****************************************************************************/
static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        completionFlag = true;
    }
    /* Signal transfer success when received success status. */
    if ((status == kStatus_I2C_Nak) || (status == kStatus_I2C_Addr_Nak))
    {
        nakFlag = true;
    }
}

static bool I2C_WriteAccelReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));
    portTickType timeout;

    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &value;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MasterTransferNonBlocking(I2C1, &g_m_handle, &masterXfer);

    /*  wait for transfer completed. */
    timeout = xTaskGetTickCount() + OS_MS_TO_TICKS(100);

    while ((!nakFlag) && (!completionFlag) && (timeout > xTaskGetTickCount()))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

static bool I2C_ReadAccelRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = rxBuff;
    masterXfer.dataSize = rxSize;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MasterTransferNonBlocking(I2C1, &g_m_handle, &masterXfer);

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

static void accel_task(void *pvParameters)
{
    uint8_t data_buffer[13];

    while (1)
    {
        if( xSemaphoreTake( xSemaphore, 1000) == pdTRUE )
        {
            /* get the accel data */
            if (_data_ready_received)
            {
                _data_ready_received = false;

                if (I2C_ReadAccelRegs(I2C1, FXOS8700CQ_ADDR, FXOS8700CQ_STATUS, data_buffer, 13))
                {
                    _accel_data.accel_x = (int16_t)(((data_buffer[1] << 8) | data_buffer[2]))>> 2;
                    _accel_data.accel_y = (int16_t)(((data_buffer[3] << 8) | data_buffer[4]))>> 2;
                    _accel_data.accel_z = (int16_t)(((data_buffer[5] << 8) | data_buffer[6]))>> 2;

                    // copy the magnetometer byte data into 16 bit words
                    _accel_data.magn_x = (data_buffer[7] << 8) | data_buffer[8];
                    _accel_data.magn_y = (data_buffer[9] << 8) | data_buffer[10];
                    _accel_data.magn_z = (data_buffer[11] << 8) | data_buffer[12];

                    /* invoke callback, if registered */
                    if (_accel_callback != NULL)
                    {
                        _accel_callback(&_accel_data, false);
                    }
                }
            }

            if (_tap_event_received)
            {
                _tap_event_received = false;
                _accel_callback(&_accel_data, true);
            }
        }
    }
}


void accel_init(accel_callback_t update_callback)
{
    const port_pin_config_t portc_pin10_config = {
        kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
        kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
        kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
        kPORT_OpenDrainEnable,                                   /* Open drain is enabled */
        kPORT_LowDriveStrength,                                  /* Low drive strength is configured */
        kPORT_MuxAlt2,                                           /* Pin is configured as I2C1_SCL */
        kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
      };

    PORT_SetPinConfig(PORTC, 10, &portc_pin10_config); /* PORTE24 (pin 31) is configured as I2C0_SCL */

    const port_pin_config_t portc_pin11_config = {
        kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
        kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
        kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
        kPORT_OpenDrainEnable,                                   /* Open drain is enabled */
        kPORT_LowDriveStrength,                                  /* Low drive strength is configured */
        kPORT_MuxAlt2,                                           /* Pin is configured as I2C1_SDA */
        kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
      };
    PORT_SetPinConfig(PORTC, 11, &portc_pin11_config);

    i2c_master_config_t masterConfig;

    I2C_MasterGetDefaultConfig(&masterConfig);

    masterConfig.baudRate_Bps = 250000;

    I2C_MasterInit(I2C1, &masterConfig, CLOCK_GetFreq(I2C0_CLK_SRC));

    I2C_MasterTransferCreateHandle(I2C1, &g_m_handle, i2c_master_callback, NULL);

    gpio_pin_config_t pin_config = {
        kGPIO_DigitalOutput, 1,
    };

    GPIO_PinInit(BOARD_INITPINS_ACCEL_3V3_GPIO, BOARD_INITPINS_ACCEL_3V3_GPIO_PIN, &pin_config);
    pin_config.outputLogic = 0;
    GPIO_PinInit(BOARD_INITPINS_ACCEL_RST_GPIO, BOARD_INITPINS_ACCEL_RST_GPIO_PIN, &pin_config);

    /* wait 1 ms */
    vTaskDelay(OS_MS_TO_TICKS(2));

    /* standby mode */
    I2C_WriteAccelReg(I2C1, FXOS8700CQ_ADDR, FXOS8700CQ_CTRL_REG1, 0x00);

    // write 0001 1111 = 0x1F to magnetometer control register 1
    // [7]: m_acal=0: auto calibration disabled
    // [6]: m_rst=0: no one-shot magnetic reset
    // [5]: m_ost=0: no one-shot magnetic measurement
    // [4-2]: m_os=111=7: 8x oversampling (for 200Hz) to reduce magnetometer noise
    // [1-0]: m_hms=11=3: select hybrid mode with accel and magnetometer active
    I2C_WriteAccelReg(I2C1, FXOS8700CQ_ADDR, FXOS8700CQ_M_CTRL_REG1, 0x1F);

    // write 0010 0000 = 0x20 to magnetometer control register 2
    // [7]: reserved
    // [6]: reserved
    // [5]: hyb_autoinc_mode=1 to map the magnetometer registers to follow the
    // accelerometer registers
    // [4]: m_maxmin_dis=0 to retain default min/max latching even though not used
    // [3]: m_maxmin_dis_ths=0
    // [2]: m_maxmin_rst=0
    // [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle
    I2C_WriteAccelReg(I2C1, FXOS8700CQ_ADDR, FXOS8700CQ_M_CTRL_REG2, 0x20);

    // write to XYZ_DATA_CFG register
    // [7]: reserved
    // [6]: reserved
    // [5]: reserved
    // [4]: hpf_out=0
    // [3]: reserved
    // [2]: reserved
    // [1-0]: fs=00 for accelerometer range of +/-2g range
    I2C_WriteAccelReg(I2C1, FXOS8700CQ_ADDR, FXOS8700CQ_XYZ_DATA_CFG, 0x00);

    /* enable double-tap event on Z-axis */
    I2C_WriteAccelReg(I2C1, FXOS8700CQ_ADDR, 0x21, 0b00100000);
    I2C_WriteAccelReg(I2C1, FXOS8700CQ_ADDR, 0x23, 0x10);
    I2C_WriteAccelReg(I2C1, FXOS8700CQ_ADDR, 0x24, 0x10);
    I2C_WriteAccelReg(I2C1, FXOS8700CQ_ADDR, 0x25, 0x10);
    I2C_WriteAccelReg(I2C1, FXOS8700CQ_ADDR, 0x26, 0x06);
    I2C_WriteAccelReg(I2C1, FXOS8700CQ_ADDR, 0x27, 0x28);
    I2C_WriteAccelReg(I2C1, FXOS8700CQ_ADDR, 0x28, 0x0F);

    /* setup DR on INT2 and pulse on INT1 */
    I2C_WriteAccelReg(I2C1, FXOS8700CQ_ADDR, FXOS8700CQ_CTRL_REG5, 0x08);

    // write 0000 0001= 0x01 to accelerometer control register 4
    // to enable DR int on INT2
    // enable double tap event in INT1
    I2C_WriteAccelReg(I2C1, FXOS8700CQ_ADDR, FXOS8700CQ_CTRL_REG4, 0x09);

    _accel_callback = update_callback;

    xSemaphore = xSemaphoreCreateBinary();
    xSemaphoreTake(xSemaphore, 100);

    xTaskCreate(accel_task, ACCEL_TASK_NAME, ACCEL_TASK_STACK, NULL, ACCEL_TASK_PRIORITY, NULL);

    PORT_SetPinInterruptConfig(BOARD_INITPINS_ACCEL_INT2_PORT, BOARD_INITPINS_ACCEL_INT2_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_ACCEL_INT1_PORT, BOARD_INITPINS_ACCEL_INT1_GPIO_PIN, kPORT_InterruptFallingEdge);
    NVIC_SetPriority(PORTC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    EnableIRQ(PORTC_IRQn);

    // write to accelerometer control register 1
    // [7-6]: aslp_rate=00
    // [5-3]: dr=010 for 200Hz data rate
    // [2]: lnoise=1 for no low noise mode
    // [1]: f_read=0 for normal 16 bit reads
    // [0]: active=1 to take the part out of standby and enable sampling
    I2C_WriteAccelReg(I2C1, FXOS8700CQ_ADDR, FXOS8700CQ_CTRL_REG1, 0b00010101);


}

void PORTC_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken;
    uint32_t flags;

    flags = GPIO_GetPinsInterruptFlags(GPIOC);

    if (flags & (1 << BOARD_INITPINS_ACCEL_INT1_GPIO_PIN))
    {
        GPIO_ClearPinsInterruptFlags(GPIOC, 1U << BOARD_INITPINS_ACCEL_INT1_GPIO_PIN);
        _tap_event_received = true;
    }

    if (flags & (1 << BOARD_INITPINS_ACCEL_INT2_GPIO_PIN))
    {
        GPIO_ClearPinsInterruptFlags(GPIOC, 1U << BOARD_INITPINS_ACCEL_INT2_GPIO_PIN);
        _data_ready_received = true;
    }

    xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


void accel_read(accel_data_t * const accel_data)
{
    taskENTER_CRITICAL();
    memcpy(accel_data, &_accel_data, sizeof(accel_data_t));
    taskEXIT_CRITICAL();
}
