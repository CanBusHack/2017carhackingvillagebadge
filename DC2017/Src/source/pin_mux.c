/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v3.0
processor: MK63FN1M0xxx12
package_id: MK63FN1M0VLQ12
mcu_data: ksdk2_0
processor_version: 2.0.0
pin_labels:
- {pin_num: '10', pin_signal: PTE7/UART3_RTS_b/I2S0_RXD0/FTM3_CH2, label: QH, identifier: QH}
- {pin_num: '54', pin_signal: PTA4/LLWU_P3/FTM0_CH1/NMI_b/EZP_CS_b, label: ETH_RST, identifier: ETH_RST}
- {pin_num: '58', pin_signal: PTA6/FTM0_CH3/TRACE_CLKOUT, label: PHY_ENA, identifier: PHY_ENA}
- {pin_num: '59', pin_signal: ADC0_SE10/PTA7/FTM0_CH4/TRACE_D3, label: PHY_INT, identifier: PHY_INT}
- {pin_num: '75', pin_signal: PTA24/MII0_TXD2/FB_A29, label: STATUS_0, identifier: STATUS_0}
- {pin_num: '76', pin_signal: PTA25/MII0_TXCLK/FB_A28, label: STATUS_1, identifier: STATUS_1}
- {pin_num: '77', pin_signal: PTA26/MII0_TXD3/FB_A27, label: STATUS_2, identifier: STATUS_2}
- {pin_num: '78', pin_signal: PTA27/MII0_CRS/FB_A26, label: STATUS_3, identifier: STATUS_3}
- {pin_num: '86', pin_signal: ADC1_SE11/PTB5/ENET0_1588_TMR3/FTM2_FLT0, label: LCD_VBAT_ENA, identifier: LCD_VBAT_ENA}
- {pin_num: '87', pin_signal: ADC1_SE12/PTB6/FB_AD23, label: LCD_3V3, identifier: LCD_3V3}
- {pin_num: '89', pin_signal: PTB8/UART3_RTS_b/FB_AD21, label: LCD_DC, identifier: LCD_DC}
- {pin_num: '90', pin_signal: PTB9/SPI1_PCS1/UART3_CTS_b/FB_AD20, label: LCD_RST, identifier: LCD_RST}
- {pin_num: '96', pin_signal: PTB17/SPI1_SIN/UART0_TX/FTM_CLKIN1/FB_AD16/EWM_OUT_b, label: S32_RST, identifier: S32_RST}
- {pin_num: '103', pin_signal: ADC0_SE14/PTC0/SPI0_PCS4/PDB0_EXTRG/USB_SOF_OUT/FB_AD14/I2S0_TXD1, label: LED_TRUCK_3, identifier: LED_TRUCK_3}
- {pin_num: '104', pin_signal: ADC0_SE15/PTC1/LLWU_P6/SPI0_PCS3/UART1_RTS_b/FTM0_CH0/FB_AD13/I2S0_TXD0, label: LED_TRUCK_0, identifier: LED_TRUCK_0}
- {pin_num: '109', pin_signal: PTC4/LLWU_P8/SPI0_PCS0/UART1_TX/FTM0_CH3/FB_AD11/CMP1_OUT, label: LED_TRUCK_2, identifier: LED_TRUCK_2}
- {pin_num: '110', pin_signal: PTC5/LLWU_P9/SPI0_SCK/LPTMR0_ALT2/I2S0_RXD0/FB_AD10/CMP0_OUT/FTM0_CH2, label: ACCEL_INT1, identifier: ACCEL_INT1}
- {pin_num: '111', pin_signal: CMP0_IN0/PTC6/LLWU_P10/SPI0_SOUT/PDB0_EXTRG/I2S0_RX_BCLK/FB_AD9/I2S0_MCLK, label: ACCEL_INT2, identifier: ACCEL_INT2}
- {pin_num: '113', pin_signal: ADC1_SE4b/CMP0_IN2/PTC8/FTM3_CH4/I2S0_MCLK/FB_AD7, label: ACCEL_3V3, identifier: ACCEL_3V3}
- {pin_num: '114', pin_signal: ADC1_SE5b/CMP0_IN3/PTC9/FTM3_CH5/I2S0_RX_BCLK/FB_AD6/FTM2_FLT0, label: ACCEL_RST, identifier: ACCEL_RST}
- {pin_num: '118', pin_signal: PTC13/UART4_CTS_b/FB_AD26, label: LIN_ENA, identifier: LIN_ENA}
- {pin_num: '127', pin_signal: PTD0/LLWU_P12/SPI0_PCS0/UART2_RTS_b/FTM3_CH0/FB_ALE/FB_CS1_b/FB_TS_b, label: LED_OE, identifier: LED_OE}
- {pin_num: '128', pin_signal: ADC0_SE5b/PTD1/SPI0_SCK/UART2_CTS_b/FTM3_CH1/FB_CS0_b, label: LED_CLR, identifier: LED_CLR}
- {pin_num: '129', pin_signal: PTD2/LLWU_P13/SPI0_SOUT/UART2_RX/FTM3_CH2/FB_AD4/I2C0_SCL, label: LED_DATA, identifier: LED_DATA}
- {pin_num: '130', pin_signal: PTD3/SPI0_SIN/UART2_TX/FTM3_CH3/FB_AD3/I2C0_SDA, label: LED_SH_CLK, identifier: LED_SH_CLK}
- {pin_num: '131', pin_signal: PTD4/LLWU_P14/SPI0_PCS1/UART0_RTS_b/FTM0_CH4/FB_AD2/EWM_IN/SPI1_PCS0, label: LED_ST_CLK, identifier: LED_ST_CLK}
- {pin_num: '133', pin_signal: ADC0_SE7b/PTD6/LLWU_P15/SPI0_PCS3/UART0_RX/FTM0_CH6/FB_AD0/FTM0_FLT0/SPI1_SOUT, label: LED_TRUCK_1, identifier: LED_TRUCK_1}
- {pin_num: '141', pin_signal: PTD12/SPI2_SCK/FTM3_FLT0/SDHC0_D4/FB_A20, label: OBD2_DET, identifier: OBD2_DET}
- {pin_num: '142', pin_signal: PTD13/SPI2_SOUT/SDHC0_D5/FB_A21, label: BAT_PPR, identifier: BAT_PPR}
- {pin_num: '143', pin_signal: PTD14/SPI2_SIN/SDHC0_D6/FB_A22, label: BAT_CHG, identifier: BAT_CHG}
- {pin_num: '144', pin_signal: PTD15/SPI2_PCS1/SDHC0_D7/FB_A23, label: SD_DET, identifier: SD_DET}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 *END**************************************************************************/
void BOARD_InitBootPins(void) {
    BOARD_InitPins();
}

#define PIN0_IDX                         0u   /*!< Pin number for pin 0 in a port */
#define PIN1_IDX                         1u   /*!< Pin number for pin 1 in a port */
#define PIN2_IDX                         2u   /*!< Pin number for pin 2 in a port */
#define PIN3_IDX                         3u   /*!< Pin number for pin 3 in a port */
#define PIN4_IDX                         4u   /*!< Pin number for pin 4 in a port */
#define PIN5_IDX                         5u   /*!< Pin number for pin 5 in a port */
#define PIN6_IDX                         6u   /*!< Pin number for pin 6 in a port */
#define PIN7_IDX                         7u   /*!< Pin number for pin 7 in a port */
#define PIN8_IDX                         8u   /*!< Pin number for pin 8 in a port */
#define PIN9_IDX                         9u   /*!< Pin number for pin 9 in a port */
#define PIN10_IDX                       10u   /*!< Pin number for pin 10 in a port */
#define PIN11_IDX                       11u   /*!< Pin number for pin 11 in a port */
#define PIN12_IDX                       12u   /*!< Pin number for pin 12 in a port */
#define PIN13_IDX                       13u   /*!< Pin number for pin 13 in a port */
#define PIN14_IDX                       14u   /*!< Pin number for pin 14 in a port */
#define PIN15_IDX                       15u   /*!< Pin number for pin 15 in a port */
#define PIN16_IDX                       16u   /*!< Pin number for pin 16 in a port */
#define PIN17_IDX                       17u   /*!< Pin number for pin 17 in a port */
#define PIN18_IDX                       18u   /*!< Pin number for pin 18 in a port */
#define PIN19_IDX                       19u   /*!< Pin number for pin 19 in a port */
#define PIN20_IDX                       20u   /*!< Pin number for pin 20 in a port */
#define PIN21_IDX                       21u   /*!< Pin number for pin 21 in a port */
#define PIN22_IDX                       22u   /*!< Pin number for pin 22 in a port */
#define PIN23_IDX                       23u   /*!< Pin number for pin 23 in a port */
#define PIN24_IDX                       24u   /*!< Pin number for pin 24 in a port */
#define PIN25_IDX                       25u   /*!< Pin number for pin 25 in a port */
#define PIN26_IDX                       26u   /*!< Pin number for pin 26 in a port */
#define PIN27_IDX                       27u   /*!< Pin number for pin 27 in a port */
#define SOPT2_RMIISRC_EXTAL           0x00u   /*!< RMII clock source select: EXTAL clock */
#define SOPT5_UART0TXSRC_UART_TX      0x00u   /*!< UART 0 transmit data source select: UART0_TX pin */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '98', peripheral: CAN0, signal: RX, pin_signal: PTB19/CAN0_RX/FTM2_CH1/I2S0_TX_FS/FB_OE_b/FTM2_QD_PHB}
  - {pin_num: '97', peripheral: CAN0, signal: TX, pin_signal: PTB18/CAN0_TX/FTM2_CH0/I2S0_TX_BCLK/FB_AD15/FTM2_QD_PHA}
  - {pin_num: '47', peripheral: ENET, signal: CLKIN_1588, pin_signal: PTE26/ENET_1588_CLKIN/UART4_CTS_b/RTC_CLKOUT/USB_CLKIN}
  - {pin_num: '72', peripheral: ENET, signal: RMII_CLKIN, pin_signal: EXTAL0/PTA18/FTM0_FLT2/FTM_CLKIN0}
  - {pin_num: '66', peripheral: ENET, signal: RMII_CRS_DV, pin_signal: PTA14/SPI0_PCS0/UART0_TX/RMII0_CRS_DV/MII0_RXDV/I2C2_SCL/I2S0_RX_BCLK/I2S0_TXD1}
  - {pin_num: '82', peripheral: ENET, signal: RMII_MDC, pin_signal: ADC0_SE9/ADC1_SE9/PTB1/I2C0_SDA/FTM1_CH1/RMII0_MDC/MII0_MDC/FTM1_QD_PHB}
  - {pin_num: '81', peripheral: ENET, signal: RMII_MDIO, pin_signal: ADC0_SE8/ADC1_SE8/PTB0/LLWU_P5/I2C0_SCL/FTM1_CH0/RMII0_MDIO/MII0_MDIO/FTM1_QD_PHA}
  - {pin_num: '65', peripheral: ENET, signal: RMII_RXD0, pin_signal: CMP2_IN1/PTA13/LLWU_P4/CAN0_RX/FTM1_CH1/RMII0_RXD0/MII0_RXD0/I2C2_SDA/I2S0_TX_FS/FTM1_QD_PHB}
  - {pin_num: '64', peripheral: ENET, signal: RMII_RXD1, pin_signal: CMP2_IN0/PTA12/CAN0_TX/FTM1_CH0/RMII0_RXD1/MII0_RXD1/I2C2_SCL/I2S0_TXD0/FTM1_QD_PHA}
  - {pin_num: '55', peripheral: ENET, signal: RMII_RXER, pin_signal: PTA5/USB_CLKIN/FTM0_CH2/RMII0_RXER/MII0_RXER/CMP2_OUT/I2S0_TX_BCLK/JTAG_TRST_b}
  - {pin_num: '68', peripheral: ENET, signal: RMII_TXD0, pin_signal: PTA16/SPI0_SOUT/UART0_CTS_b/UART0_COL_b/RMII0_TXD0/MII0_TXD0/I2S0_RX_FS/I2S0_RXD1}
  - {pin_num: '69', peripheral: ENET, signal: RMII_TXD1, pin_signal: ADC1_SE17/PTA17/SPI0_SIN/UART0_RTS_b/RMII0_TXD1/MII0_TXD1/I2S0_MCLK}
  - {pin_num: '67', peripheral: ENET, signal: RMII_TXEN, pin_signal: PTA15/SPI0_SCK/UART0_RX/RMII0_TXEN/MII0_TXEN/I2S0_RXD0}
  - {pin_num: '4', peripheral: SDHC, signal: CMD, pin_signal: ADC0_DM2/ADC1_SE7a/PTE3/SPI1_SIN/UART1_RTS_b/SDHC0_CMD/TRACE_D1/SPI1_SOUT}
  - {pin_num: '2', peripheral: SDHC, signal: 'DATA, 0', pin_signal: ADC1_SE5a/PTE1/LLWU_P0/SPI1_SOUT/UART1_RX/SDHC0_D0/TRACE_D3/I2C1_SCL/SPI1_SIN}
  - {pin_num: '1', peripheral: SDHC, signal: 'DATA, 1', pin_signal: ADC1_SE4a/PTE0/SPI1_PCS1/UART1_TX/SDHC0_D1/TRACE_CLKOUT/I2C1_SDA/RTC_CLKOUT}
  - {pin_num: '8', peripheral: SDHC, signal: 'DATA, 2', pin_signal: PTE5/SPI1_PCS2/UART3_RX/SDHC0_D2/FTM3_CH0}
  - {pin_num: '7', peripheral: SDHC, signal: 'DATA, 3', pin_signal: PTE4/LLWU_P2/SPI1_PCS0/UART3_TX/SDHC0_D3/TRACE_D0}
  - {pin_num: '3', peripheral: SDHC, signal: DCLK, pin_signal: ADC0_DP2/ADC1_SE6a/PTE2/LLWU_P1/SPI1_SCK/UART1_CTS_b/SDHC0_DCLK/TRACE_D2}
  - {pin_num: '12', peripheral: UART5, signal: RX, pin_signal: PTE9/I2S0_TXD1/UART5_RX/I2S0_RX_BCLK/FTM3_CH4}
  - {pin_num: '11', peripheral: UART5, signal: TX, pin_signal: PTE8/I2S0_RXD1/UART5_TX/I2S0_RX_FS/FTM3_CH3}
  - {pin_num: '20', peripheral: USB0, signal: DM, pin_signal: USB0_DM}
  - {pin_num: '19', peripheral: USB0, signal: DP, pin_signal: USB0_DP}
  - {pin_num: '21', peripheral: USB0, signal: VOUT33, pin_signal: VOUT33}
  - {pin_num: '22', peripheral: USB0, signal: VREGIN, pin_signal: VREGIN}
  - {pin_num: '36', peripheral: ADC0, signal: 'SE, 16', pin_signal: ADC0_SE16/CMP1_IN2/ADC0_SE21}
  - {pin_num: '32', peripheral: ADC0, signal: VREFH, pin_signal: VREFH}
  - {pin_num: '33', peripheral: ADC0, signal: VREFL, pin_signal: VREFL}
  - {pin_num: '5', peripheral: SUPPLY, signal: 'VDD, 0', pin_signal: VDD5}
  - {pin_num: '16', peripheral: SUPPLY, signal: 'VDD, 1', pin_signal: VDD16}
  - {pin_num: '56', peripheral: SUPPLY, signal: 'VDD, 3', pin_signal: VDD63}
  - {pin_num: '70', peripheral: SUPPLY, signal: 'VDD, 4', pin_signal: VDD80}
  - {pin_num: '94', peripheral: SUPPLY, signal: 'VDD, 5', pin_signal: VDD110}
  - {pin_num: '108', peripheral: SUPPLY, signal: 'VDD, 6', pin_signal: VDD124}
  - {pin_num: '122', peripheral: SUPPLY, signal: 'VDD, 7', pin_signal: VDD140}
  - {pin_num: '135', peripheral: SUPPLY, signal: 'VDD, 8', pin_signal: VDD153}
  - {pin_num: '31', peripheral: SUPPLY, signal: 'VDDA, 0', pin_signal: VDDA}
  - {pin_num: '6', peripheral: SUPPLY, signal: 'VSS, 0', pin_signal: VSS6}
  - {pin_num: '17', peripheral: SUPPLY, signal: 'VSS, 1', pin_signal: VSS17}
  - {pin_num: '18', peripheral: SUPPLY, signal: 'VSS, 2', pin_signal: VSS18}
  - {pin_num: '44', peripheral: SUPPLY, signal: 'VSS, 3', pin_signal: VSS51}
  - {pin_num: '57', peripheral: SUPPLY, signal: 'VSS, 4', pin_signal: VSS64}
  - {pin_num: '71', peripheral: SUPPLY, signal: 'VSS, 5', pin_signal: VSS81}
  - {pin_num: '93', peripheral: SUPPLY, signal: 'VSS, 6', pin_signal: VSS109}
  - {pin_num: '107', peripheral: SUPPLY, signal: 'VSS, 7', pin_signal: VSS123}
  - {pin_num: '121', peripheral: SUPPLY, signal: 'VSS, 8', pin_signal: VSS139}
  - {pin_num: '134', peripheral: SUPPLY, signal: 'VSS, 9', pin_signal: VSS152}
  - {pin_num: '34', peripheral: SUPPLY, signal: 'VSSA, 0', pin_signal: VSSA}
  - {pin_num: '51', peripheral: UART0, signal: RX, pin_signal: PTA1/UART0_RX/FTM0_CH6/JTAG_TDI/EZP_DI}
  - {pin_num: '52', peripheral: UART0, signal: TX, pin_signal: PTA2/UART0_TX/FTM0_CH7/JTAG_TDO/TRACE_SWO/EZP_DO}
  - {pin_num: '73', peripheral: OSC, signal: XTAL0, pin_signal: XTAL0/PTA19/FTM1_FLT0/FTM_CLKIN1/LPTMR0_ALT1}
  - {pin_num: '53', peripheral: JTAG, signal: JTAG_TMS_SWD_DIO, pin_signal: PTA3/UART0_RTS_b/FTM0_CH0/JTAG_TMS/SWD_DIO}
  - {pin_num: '50', peripheral: JTAG, signal: JTAG_TCLK_SWD_CLK, pin_signal: PTA0/UART0_CTS_b/UART0_COL_b/FTM0_CH5/JTAG_TCLK/SWD_CLK/EZP_CLK}
  - {pin_num: '74', peripheral: RCM, signal: RESET, pin_signal: RESET_b}
  - {pin_num: '91', peripheral: SPI1, signal: PCS0_SS, pin_signal: ADC1_SE14/PTB10/SPI1_PCS0/UART3_RX/FB_AD19/FTM0_FLT1}
  - {pin_num: '92', peripheral: SPI1, signal: SCK, pin_signal: ADC1_SE15/PTB11/SPI1_SCK/UART3_TX/FB_AD18/FTM0_FLT2}
  - {pin_num: '95', peripheral: SPI1, signal: SOUT, pin_signal: PTB16/SPI1_SOUT/UART0_RX/FTM_CLKIN0/FB_AD17/EWM_IN}
  - {pin_num: '99', peripheral: SPI2, signal: PCS0_SS, pin_signal: PTB20/SPI2_PCS0/FB_AD31/CMP0_OUT}
  - {pin_num: '100', peripheral: SPI2, signal: SCK, pin_signal: PTB21/SPI2_SCK/FB_AD30/CMP1_OUT}
  - {pin_num: '102', peripheral: SPI2, signal: SIN, pin_signal: PTB23/SPI2_SIN/SPI0_PCS5/FB_AD28}
  - {pin_num: '101', peripheral: SPI2, signal: SOUT, pin_signal: PTB22/SPI2_SOUT/FB_AD29/CMP2_OUT}
  - {pin_num: '106', peripheral: SIM, signal: CLKOUT, pin_signal: CMP1_IN1/PTC3/LLWU_P7/SPI0_PCS1/UART1_RX/FTM0_CH2/CLKOUT/I2S0_TX_BCLK}
  - {pin_num: '115', peripheral: I2C1, signal: SCL, pin_signal: ADC1_SE6b/PTC10/I2C1_SCL/FTM3_CH6/I2S0_RX_FS/FB_AD5}
  - {pin_num: '116', peripheral: I2C1, signal: SDA, pin_signal: ADC1_SE7b/PTC11/LLWU_P11/I2C1_SDA/FTM3_CH7/I2S0_RXD1/FB_RW_b}
  - {pin_num: '119', peripheral: UART4, signal: RX, pin_signal: PTC14/UART4_RX/FB_AD25}
  - {pin_num: '120', peripheral: UART4, signal: TX, pin_signal: PTC15/UART4_TX/FB_AD24}
  - {pin_num: '123', peripheral: UART3, signal: RX, pin_signal: PTC16/UART3_RX/ENET0_1588_TMR0/FB_CS5_b/FB_TSIZ1/FB_BE23_16_BLS15_8_b}
  - {pin_num: '124', peripheral: UART3, signal: TX, pin_signal: PTC17/UART3_TX/ENET0_1588_TMR1/FB_CS4_b/FB_TSIZ0/FB_BE31_24_BLS7_0_b}
  - {pin_num: '10', peripheral: GPIOE, signal: 'GPIO, 7', pin_signal: PTE7/UART3_RTS_b/I2S0_RXD0/FTM3_CH2, direction: INPUT}
  - {pin_num: '54', peripheral: GPIOA, signal: 'GPIO, 4', pin_signal: PTA4/LLWU_P3/FTM0_CH1/NMI_b/EZP_CS_b, direction: OUTPUT}
  - {pin_num: '58', peripheral: GPIOA, signal: 'GPIO, 6', pin_signal: PTA6/FTM0_CH3/TRACE_CLKOUT, direction: OUTPUT}
  - {pin_num: '59', peripheral: GPIOA, signal: 'GPIO, 7', pin_signal: ADC0_SE10/PTA7/FTM0_CH4/TRACE_D3, direction: INPUT}
  - {pin_num: '75', peripheral: GPIOA, signal: 'GPIO, 24', pin_signal: PTA24/MII0_TXD2/FB_A29, direction: INPUT}
  - {pin_num: '76', peripheral: GPIOA, signal: 'GPIO, 25', pin_signal: PTA25/MII0_TXCLK/FB_A28, direction: INPUT}
  - {pin_num: '77', peripheral: GPIOA, signal: 'GPIO, 26', pin_signal: PTA26/MII0_TXD3/FB_A27, direction: OUTPUT}
  - {pin_num: '78', peripheral: GPIOA, signal: 'GPIO, 27', pin_signal: PTA27/MII0_CRS/FB_A26, direction: OUTPUT}
  - {pin_num: '86', peripheral: GPIOB, signal: 'GPIO, 5', pin_signal: ADC1_SE11/PTB5/ENET0_1588_TMR3/FTM2_FLT0, direction: OUTPUT}
  - {pin_num: '87', peripheral: GPIOB, signal: 'GPIO, 6', pin_signal: ADC1_SE12/PTB6/FB_AD23, direction: OUTPUT}
  - {pin_num: '89', peripheral: GPIOB, signal: 'GPIO, 8', pin_signal: PTB8/UART3_RTS_b/FB_AD21, direction: OUTPUT}
  - {pin_num: '90', peripheral: GPIOB, signal: 'GPIO, 9', pin_signal: PTB9/SPI1_PCS1/UART3_CTS_b/FB_AD20, direction: OUTPUT}
  - {pin_num: '96', peripheral: GPIOB, signal: 'GPIO, 17', pin_signal: PTB17/SPI1_SIN/UART0_TX/FTM_CLKIN1/FB_AD16/EWM_OUT_b, direction: OUTPUT}
  - {pin_num: '103', peripheral: GPIOC, signal: 'GPIO, 0', pin_signal: ADC0_SE14/PTC0/SPI0_PCS4/PDB0_EXTRG/USB_SOF_OUT/FB_AD14/I2S0_TXD1, direction: OUTPUT}
  - {pin_num: '110', peripheral: GPIOC, signal: 'GPIO, 5', pin_signal: PTC5/LLWU_P9/SPI0_SCK/LPTMR0_ALT2/I2S0_RXD0/FB_AD10/CMP0_OUT/FTM0_CH2, direction: INPUT}
  - {pin_num: '111', peripheral: GPIOC, signal: 'GPIO, 6', pin_signal: CMP0_IN0/PTC6/LLWU_P10/SPI0_SOUT/PDB0_EXTRG/I2S0_RX_BCLK/FB_AD9/I2S0_MCLK, direction: INPUT}
  - {pin_num: '113', peripheral: GPIOC, signal: 'GPIO, 8', pin_signal: ADC1_SE4b/CMP0_IN2/PTC8/FTM3_CH4/I2S0_MCLK/FB_AD7, direction: OUTPUT}
  - {pin_num: '114', peripheral: GPIOC, signal: 'GPIO, 9', pin_signal: ADC1_SE5b/CMP0_IN3/PTC9/FTM3_CH5/I2S0_RX_BCLK/FB_AD6/FTM2_FLT0, direction: OUTPUT}
  - {pin_num: '118', peripheral: GPIOC, signal: 'GPIO, 13', pin_signal: PTC13/UART4_CTS_b/FB_AD26, direction: OUTPUT}
  - {pin_num: '128', peripheral: GPIOD, signal: 'GPIO, 1', pin_signal: ADC0_SE5b/PTD1/SPI0_SCK/UART2_CTS_b/FTM3_CH1/FB_CS0_b, direction: OUTPUT}
  - {pin_num: '129', peripheral: GPIOD, signal: 'GPIO, 2', pin_signal: PTD2/LLWU_P13/SPI0_SOUT/UART2_RX/FTM3_CH2/FB_AD4/I2C0_SCL, direction: OUTPUT}
  - {pin_num: '130', peripheral: GPIOD, signal: 'GPIO, 3', pin_signal: PTD3/SPI0_SIN/UART2_TX/FTM3_CH3/FB_AD3/I2C0_SDA, direction: OUTPUT}
  - {pin_num: '131', peripheral: GPIOD, signal: 'GPIO, 4', pin_signal: PTD4/LLWU_P14/SPI0_PCS1/UART0_RTS_b/FTM0_CH4/FB_AD2/EWM_IN/SPI1_PCS0, direction: OUTPUT}
  - {pin_num: '141', peripheral: GPIOD, signal: 'GPIO, 12', pin_signal: PTD12/SPI2_SCK/FTM3_FLT0/SDHC0_D4/FB_A20, direction: INPUT}
  - {pin_num: '142', peripheral: GPIOD, signal: 'GPIO, 13', pin_signal: PTD13/SPI2_SOUT/SDHC0_D5/FB_A21, direction: INPUT}
  - {pin_num: '143', peripheral: GPIOD, signal: 'GPIO, 14', pin_signal: PTD14/SPI2_SIN/SDHC0_D6/FB_A22, direction: INPUT}
  - {pin_num: '144', peripheral: GPIOD, signal: 'GPIO, 15', pin_signal: PTD15/SPI2_PCS1/SDHC0_D7/FB_A23, direction: INPUT}
  - {pin_num: '127', peripheral: FTM3, signal: 'CH, 0', pin_signal: PTD0/LLWU_P12/SPI0_PCS0/UART2_RTS_b/FTM3_CH0/FB_ALE/FB_CS1_b/FB_TS_b, direction: OUTPUT}
  - {pin_num: '133', peripheral: GPIOD, signal: 'GPIO, 6', pin_signal: ADC0_SE7b/PTD6/LLWU_P15/SPI0_PCS3/UART0_RX/FTM0_CH6/FB_AD0/FTM0_FLT0/SPI1_SOUT, direction: OUTPUT}
  - {pin_num: '109', peripheral: GPIOC, signal: 'GPIO, 4', pin_signal: PTC4/LLWU_P8/SPI0_PCS0/UART1_TX/FTM0_CH3/FB_AD11/CMP1_OUT, direction: OUTPUT}
  - {pin_num: '104', peripheral: GPIOC, signal: 'GPIO, 1', pin_signal: ADC0_SE15/PTC1/LLWU_P6/SPI0_PCS3/UART1_RTS_b/FTM0_CH0/FB_AD13/I2S0_TXD0, direction: OUTPUT}
  - {pin_num: '43', peripheral: SUPPLY, signal: 'VDD, 2', pin_signal: VDD50}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void BOARD_InitPins(void) {
  CLOCK_EnableClock(kCLOCK_PortA);                           /* Port A Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortB);                           /* Port B Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortC);                           /* Port C Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortD);                           /* Port D Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortE);                           /* Port E Clock Gate Control: Clock enabled */

  PORT_SetPinMux(PORTA, PIN0_IDX, kPORT_MuxAlt7);            /* PORTA0 (pin 50) is configured as JTAG_TCLK */
  PORT_SetPinMux(PORTA, PIN1_IDX, kPORT_MuxAlt2);            /* PORTA1 (pin 51) is configured as UART0_RX */
  PORT_SetPinMux(PORTA, PIN12_IDX, kPORT_MuxAlt4);           /* PORTA12 (pin 64) is configured as RMII0_RXD1 */
  PORT_SetPinMux(PORTA, PIN13_IDX, kPORT_MuxAlt4);           /* PORTA13 (pin 65) is configured as RMII0_RXD0 */
  PORT_SetPinMux(PORTA, PIN14_IDX, kPORT_MuxAlt4);           /* PORTA14 (pin 66) is configured as RMII0_CRS_DV */
  PORT_SetPinMux(PORTA, PIN15_IDX, kPORT_MuxAlt4);           /* PORTA15 (pin 67) is configured as RMII0_TXEN */
  PORT_SetPinMux(PORTA, PIN16_IDX, kPORT_MuxAlt4);           /* PORTA16 (pin 68) is configured as RMII0_TXD0 */
  PORT_SetPinMux(PORTA, PIN17_IDX, kPORT_MuxAlt4);           /* PORTA17 (pin 69) is configured as RMII0_TXD1 */
  PORT_SetPinMux(PORTA, PIN18_IDX, kPORT_PinDisabledOrAnalog); /* PORTA18 (pin 72) is configured as EXTAL0 */
  PORT_SetPinMux(PORTA, PIN19_IDX, kPORT_PinDisabledOrAnalog); /* PORTA19 (pin 73) is configured as XTAL0 */
  PORT_SetPinMux(PORTA, PIN2_IDX, kPORT_MuxAlt2);            /* PORTA2 (pin 52) is configured as UART0_TX */
  PORT_SetPinMux(PORTA, PIN24_IDX, kPORT_MuxAsGpio);         /* PORTA24 (pin 75) is configured as PTA24 */
  PORT_SetPinMux(PORTA, PIN25_IDX, kPORT_MuxAsGpio);         /* PORTA25 (pin 76) is configured as PTA25 */
  PORT_SetPinMux(PORTA, PIN26_IDX, kPORT_MuxAsGpio);         /* PORTA26 (pin 77) is configured as PTA26 */
  PORT_SetPinMux(PORTA, PIN27_IDX, kPORT_MuxAsGpio);         /* PORTA27 (pin 78) is configured as PTA27 */
  PORT_SetPinMux(PORTA, PIN3_IDX, kPORT_MuxAlt7);            /* PORTA3 (pin 53) is configured as JTAG_TMS */
  PORT_SetPinMux(PORTA, PIN4_IDX, kPORT_MuxAsGpio);          /* PORTA4 (pin 54) is configured as PTA4 */
  PORT_SetPinMux(PORTA, PIN5_IDX, kPORT_MuxAlt4);            /* PORTA5 (pin 55) is configured as RMII0_RXER */
  PORT_SetPinMux(PORTA, PIN6_IDX, kPORT_MuxAsGpio);          /* PORTA6 (pin 58) is configured as PTA6 */
  PORT_SetPinMux(PORTA, PIN7_IDX, kPORT_MuxAsGpio);          /* PORTA7 (pin 59) is configured as PTA7 */
  PORT_SetPinMux(PORTB, PIN0_IDX, kPORT_MuxAlt4);            /* PORTB0 (pin 81) is configured as RMII0_MDIO */
  PORT_SetPinMux(PORTB, PIN1_IDX, kPORT_MuxAlt4);            /* PORTB1 (pin 82) is configured as RMII0_MDC */
  PORT_SetPinMux(PORTB, PIN10_IDX, kPORT_MuxAlt2);           /* PORTB10 (pin 91) is configured as SPI1_PCS0 */
  PORT_SetPinMux(PORTB, PIN11_IDX, kPORT_MuxAlt2);           /* PORTB11 (pin 92) is configured as SPI1_SCK */
  PORT_SetPinMux(PORTB, PIN16_IDX, kPORT_MuxAlt2);           /* PORTB16 (pin 95) is configured as SPI1_SOUT */
  PORT_SetPinMux(PORTB, PIN17_IDX, kPORT_MuxAsGpio);         /* PORTB17 (pin 96) is configured as PTB17 */
  PORT_SetPinMux(PORTB, PIN18_IDX, kPORT_MuxAlt2);           /* PORTB18 (pin 97) is configured as CAN0_TX */
  PORT_SetPinMux(PORTB, PIN19_IDX, kPORT_MuxAlt2);           /* PORTB19 (pin 98) is configured as CAN0_RX */
  PORT_SetPinMux(PORTB, PIN20_IDX, kPORT_MuxAlt2);           /* PORTB20 (pin 99) is configured as SPI2_PCS0 */
  PORT_SetPinMux(PORTB, PIN21_IDX, kPORT_MuxAlt2);           /* PORTB21 (pin 100) is configured as SPI2_SCK */
  PORT_SetPinMux(PORTB, PIN22_IDX, kPORT_MuxAlt2);           /* PORTB22 (pin 101) is configured as SPI2_SOUT */
  PORT_SetPinMux(PORTB, PIN23_IDX, kPORT_MuxAlt2);           /* PORTB23 (pin 102) is configured as SPI2_SIN */
  PORT_SetPinMux(PORTB, PIN5_IDX, kPORT_MuxAsGpio);          /* PORTB5 (pin 86) is configured as PTB5 */
  PORT_SetPinMux(PORTB, PIN6_IDX, kPORT_MuxAsGpio);          /* PORTB6 (pin 87) is configured as PTB6 */
  PORT_SetPinMux(PORTB, PIN8_IDX, kPORT_MuxAsGpio);          /* PORTB8 (pin 89) is configured as PTB8 */
  PORT_SetPinMux(PORTB, PIN9_IDX, kPORT_MuxAsGpio);          /* PORTB9 (pin 90) is configured as PTB9 */
  PORT_SetPinMux(PORTC, PIN0_IDX, kPORT_MuxAsGpio);          /* PORTC0 (pin 103) is configured as PTC0 */
  PORT_SetPinMux(PORTC, PIN1_IDX, kPORT_MuxAlt4);          /* PORTC1 (pin 104) is configured as PTC1 */
  PORT_SetPinMux(PORTC, PIN2_IDX, kPORT_MuxAlt4);          /* PORTC1 (pin 105) is configured as PTC2 */
  PORT_SetPinMux(PORTC, PIN10_IDX, kPORT_MuxAlt2);           /* PORTC10 (pin 115) is configured as I2C1_SCL */
  PORT_SetPinMux(PORTC, PIN11_IDX, kPORT_MuxAlt2);           /* PORTC11 (pin 116) is configured as I2C1_SDA */
  PORT_SetPinMux(PORTC, PIN13_IDX, kPORT_MuxAsGpio);         /* PORTC13 (pin 118) is configured as PTC13 */
  PORT_SetPinMux(PORTC, PIN14_IDX, kPORT_MuxAlt3);           /* PORTC14 (pin 119) is configured as UART4_RX */
  PORT_SetPinMux(PORTC, PIN15_IDX, kPORT_MuxAlt3);           /* PORTC15 (pin 120) is configured as UART4_TX */
  PORT_SetPinMux(PORTC, PIN16_IDX, kPORT_MuxAlt3);           /* PORTC16 (pin 123) is configured as UART3_RX */
  PORT_SetPinMux(PORTC, PIN17_IDX, kPORT_MuxAlt3);           /* PORTC17 (pin 124) is configured as UART3_TX */
  PORT_SetPinMux(PORTC, PIN3_IDX, kPORT_MuxAlt5);            /* PORTC3 (pin 106) is configured as CLKOUT */
  PORT_SetPinMux(PORTC, PIN4_IDX, kPORT_MuxAlt4);          /* PORTC4 (pin 109) is configured as PTC4 */
  PORT_SetPinMux(PORTC, PIN5_IDX, kPORT_MuxAsGpio);          /* PORTC5 (pin 110) is configured as PTC5 */
  PORT_SetPinMux(PORTC, PIN6_IDX, kPORT_MuxAsGpio);          /* PORTC6 (pin 111) is configured as PTC6 */
  PORT_SetPinMux(PORTC, PIN8_IDX, kPORT_MuxAsGpio);          /* PORTC8 (pin 113) is configured as PTC8 */
  PORT_SetPinMux(PORTC, PIN9_IDX, kPORT_MuxAsGpio);          /* PORTC9 (pin 114) is configured as PTC9 */
  PORT_SetPinMux(PORTD, PIN0_IDX, kPORT_MuxAlt4);            /* PORTD0 (pin 127) is configured as FTM3_CH0 */
  PORT_SetPinMux(PORTD, PIN1_IDX, kPORT_MuxAsGpio);          /* PORTD1 (pin 128) is configured as PTD1 */
  PORT_SetPinMux(PORTD, PIN12_IDX, kPORT_MuxAsGpio);         /* PORTD12 (pin 141) is configured as PTD12 */
  PORT_SetPinMux(PORTD, PIN13_IDX, kPORT_MuxAsGpio);         /* PORTD13 (pin 142) is configured as PTD13 */
  PORT_SetPinMux(PORTD, PIN14_IDX, kPORT_MuxAsGpio);         /* PORTD14 (pin 143) is configured as PTD14 */
  PORT_SetPinMux(PORTD, PIN15_IDX, kPORT_MuxAsGpio);         /* PORTD15 (pin 144) is configured as PTD15 */
  PORT_SetPinMux(PORTD, PIN2_IDX, kPORT_MuxAsGpio);          /* PORTD2 (pin 129) is configured as PTD2 */
  PORT_SetPinMux(PORTD, PIN3_IDX, kPORT_MuxAsGpio);          /* PORTD3 (pin 130) is configured as PTD3 */
  PORT_SetPinMux(PORTD, PIN4_IDX, kPORT_MuxAsGpio);          /* PORTD4 (pin 131) is configured as PTD4 */
  PORT_SetPinMux(PORTD, PIN6_IDX, kPORT_MuxAlt4);          /* PORTD6 (pin 133) is configured as PTD6 */
  PORT_SetPinMux(PORTE, PIN0_IDX, kPORT_MuxAlt4);            /* PORTE0 (pin 1) is configured as SDHC0_D1 */
  PORT_SetPinMux(PORTE, PIN1_IDX, kPORT_MuxAlt4);            /* PORTE1 (pin 2) is configured as SDHC0_D0 */
  PORT_SetPinMux(PORTE, PIN2_IDX, kPORT_MuxAlt4);            /* PORTE2 (pin 3) is configured as SDHC0_DCLK */
  PORT_SetPinMux(PORTE, PIN26_IDX, kPORT_MuxAlt2);           /* PORTE26 (pin 47) is configured as ENET_1588_CLKIN */
  PORT_SetPinMux(PORTE, PIN3_IDX, kPORT_MuxAlt4);            /* PORTE3 (pin 4) is configured as SDHC0_CMD */
  PORT_SetPinMux(PORTE, PIN4_IDX, kPORT_MuxAlt4);            /* PORTE4 (pin 7) is configured as SDHC0_D3 */
  PORT_SetPinMux(PORTE, PIN5_IDX, kPORT_MuxAlt4);            /* PORTE5 (pin 8) is configured as SDHC0_D2 */
  PORT_SetPinMux(PORTE, PIN7_IDX, kPORT_MuxAsGpio);          /* PORTE7 (pin 10) is configured as PTE7 */
  PORT_SetPinMux(PORTE, PIN8_IDX, kPORT_MuxAlt3);            /* PORTE8 (pin 11) is configured as UART5_TX */
  PORT_SetPinMux(PORTE, PIN9_IDX, kPORT_MuxAlt3);            /* PORTE9 (pin 12) is configured as UART5_RX */
  SIM->SOPT2 = ((SIM->SOPT2 &
    (~(SIM_SOPT2_RMIISRC_MASK)))                             /* Mask bits to zero which are setting */
      | SIM_SOPT2_RMIISRC(SOPT2_RMIISRC_EXTAL)               /* RMII clock source select: EXTAL clock */
    );
  SIM->SOPT5 = ((SIM->SOPT5 &
    (~(SIM_SOPT5_UART0TXSRC_MASK)))                          /* Mask bits to zero which are setting */
      | SIM_SOPT5_UART0TXSRC(SOPT5_UART0TXSRC_UART_TX)       /* UART 0 transmit data source select: UART0_TX pin */
    );
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
