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
 * @brief OLED management routines
 *
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "datatypes.h"
#include "cpu.h"
#include "oled.h"
#include "pin_mux.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_dspi_freertos.h"

#include "FreeRTOS.h"
#include "task.h"

#include "oled.h"

/******************************************************************************
 * Macros and Constants
 *****************************************************************************/
#define TRANSFER_BAUDRATE (10000000)

/******************************************************************************
 * Typedefs
 *****************************************************************************/

/******************************************************************************
 * Local Function Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/
static uint8_t frame_buffer[128 * (64 / 8)];
static BOOL buffer_modified;

/******************************************************************************
 * Global Variables
 *****************************************************************************/

/******************************************************************************
 * Functions
 *****************************************************************************/
static void spi_transaction(uint8_t const * const buffer, uint32_t const buffer_size)
{
    dspi_transfer_t masterXfer;

    masterXfer.txData = buffer;
    masterXfer.rxData = NULL;
    masterXfer.dataSize = buffer_size;
    masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

    DSPI_MasterTransferBlocking(DSPI1, &masterXfer);
}

void oled_init(void)
{
    uint8_t cmd_buffer[3];
    dspi_master_config_t masterConfig;


    gpio_pin_config_t pin_config = {
        kGPIO_DigitalOutput, 0,
    };

    /* init gpio pins */
    GPIO_PinInit(BOARD_INITPINS_LCD_VBAT_ENA_GPIO, BOARD_INITPINS_LCD_VBAT_ENA_GPIO_PIN, &pin_config);
    GPIO_PinInit(BOARD_INITPINS_LCD_3V3_GPIO, BOARD_INITPINS_LCD_3V3_GPIO_PIN, &pin_config);
    GPIO_PinInit(BOARD_INITPINS_LCD_DC_GPIO, BOARD_INITPINS_LCD_DC_GPIO_PIN, &pin_config);
    GPIO_PinInit(BOARD_INITPINS_LCD_RST_GPIO, BOARD_INITPINS_LCD_RST_GPIO_PIN, &pin_config);

    /* init SPI */
    masterConfig.whichCtar = kDSPI_Ctar0;
    masterConfig.ctarConfig.baudRate = TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.bitsPerFrame = 8U;
    masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
    masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
    masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
    masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;

    masterConfig.whichPcs = kDSPI_Pcs0;
    masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

    masterConfig.enableContinuousSCK = false;
    masterConfig.enableRxFifoOverWrite = false;
    masterConfig.enableModifiedTimingFormat = false;
    masterConfig.samplePoint = kDSPI_SckToSin0Clock;

    DSPI_MasterInit(DSPI1, &masterConfig, CLOCK_GetFreq(DSPI1_CLK_SRC));

    /* Assert VDD */
    GPIO_WritePinOutput(BOARD_INITPINS_LCD_3V3_GPIO, BOARD_INITPINS_LCD_3V3_GPIO_PIN, 1);

    vTaskDelay(OS_MS_TO_TICKS(50));

    /* de-assert reset */
    GPIO_WritePinOutput(BOARD_INITPINS_LCD_RST_GPIO, BOARD_INITPINS_LCD_RST_GPIO_PIN, 1);

    vTaskDelay(OS_MS_TO_TICKS(50));

    /* panel off */
    cmd_buffer[0] = 0xAE;
    spi_transaction(cmd_buffer, 1);

    /* set addressing mode */
    cmd_buffer[0] = 0x00;
    spi_transaction(cmd_buffer, 1);

    cmd_buffer[0] = 0x10;
    spi_transaction(cmd_buffer, 1);

    cmd_buffer[0] = 0x40;
    spi_transaction(cmd_buffer, 1);

    cmd_buffer[0] = 0x20;
    cmd_buffer[1] = 0x00;
    spi_transaction(cmd_buffer, 2);

    /* set position */
    cmd_buffer[0] = 0x21;
    cmd_buffer[1] = 0;
    cmd_buffer[2] = 127;
    spi_transaction(cmd_buffer, 3);

    /* contrast control */
    cmd_buffer[0] = 0x81;
    cmd_buffer[1] = 0xCF;
    spi_transaction(cmd_buffer, 2);

    /* segment remap */
    cmd_buffer[0] = 0xA1;
    spi_transaction(cmd_buffer, 1);

    /* normal display */
    cmd_buffer[0] = 0xA6;
    spi_transaction(cmd_buffer, 1);

    /* multiplex ratio */
    cmd_buffer[0] = 0xA8;
    cmd_buffer[1] = 0x3F;
    spi_transaction(cmd_buffer, 2);

    /* display clock */
    cmd_buffer[0] = 0xD3;
    cmd_buffer[1] = 0x00;
    spi_transaction(cmd_buffer, 2);

    cmd_buffer[0] = 0xD5;
    cmd_buffer[1] = 0x80;
    spi_transaction(cmd_buffer, 2);


    /* pre-charge */
    cmd_buffer[0] = 0xD9;
    cmd_buffer[1] = 0xF1;
    spi_transaction(cmd_buffer, 2);

    /* com pins */
    cmd_buffer[0] = 0xDA;
    cmd_buffer[1] = 0x12;
    spi_transaction(cmd_buffer, 2);

    /* vcomh */
    cmd_buffer[0] = 0xDB;
    cmd_buffer[1] = 0x40;
    spi_transaction(cmd_buffer, 2);

    /* charge pump enable */
    cmd_buffer[0] = 0x8D;
    cmd_buffer[1] = 0x14;
    spi_transaction(cmd_buffer, 2);

    oled_clear();
    oled_write_framebuffer();

    GPIO_WritePinOutput(BOARD_INITPINS_LCD_VBAT_ENA_GPIO, BOARD_INITPINS_LCD_VBAT_ENA_GPIO_PIN, 1);

    vTaskDelay(OS_MS_TO_TICKS(50));

    /* panel on */
    cmd_buffer[0] = 0xAF;
    spi_transaction(cmd_buffer, 1);
}

void oled_write_framebuffer(void)
{
    if (!buffer_modified)
    {
        return;
    }

    /* assert D/C# */
    GPIO_WritePinOutput(BOARD_INITPINS_LCD_DC_GPIO, BOARD_INITPINS_LCD_DC_GPIO_PIN, 1);

    spi_transaction(frame_buffer, sizeof(frame_buffer));

    GPIO_WritePinOutput(BOARD_INITPINS_LCD_DC_GPIO, BOARD_INITPINS_LCD_DC_GPIO_PIN, 0);

    buffer_modified = FALSE;
}

void oled_put_pixel(uint8_t const x, uint8_t const y)
{
    int temp_y;
    int bit_y;
    int new_y;

    /* remap y to be at top left */
    new_y = 63 - y;

    /* figure out location in frame buffer, given 1bpp */
    temp_y = new_y / 8;

    bit_y = new_y - (temp_y * 8);

    frame_buffer[x + (128 * temp_y)] |=  (1 << (bit_y));

    buffer_modified = TRUE;
}

void oled_clear(void)
{
    unsigned long i;

    for(i = 0; i < (128 * (64 / 8)); i++)
    {
        frame_buffer[i] = 0x00;
    }

    buffer_modified = TRUE;
}

