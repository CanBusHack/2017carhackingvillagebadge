#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Direction type  */
typedef enum _pin_mux_direction
{
  kPIN_MUX_DirectionInput = 0U,         /* Input direction */
  kPIN_MUX_DirectionOutput = 1U,        /* Output direction */
  kPIN_MUX_DirectionInputOrOutput = 2U  /* Input or output direction */
} pin_mux_direction_t;

/*!
 * @addtogroup pin_mux
 * @{
 */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/* PORTE7 (number 10), QH */
#define BOARD_INITPINS_QH_GPIO                                             GPIOE   /*!< GPIO device name: GPIOE */
#define BOARD_INITPINS_QH_PORT                                             PORTE   /*!< PORT device name: PORTE */
#define BOARD_INITPINS_QH_GPIO_PIN                                            7U   /*!< PORTE pin index: 7 */
#define BOARD_INITPINS_QH_PIN_NAME                                          PTE7   /*!< Pin name */
#define BOARD_INITPINS_QH_LABEL                                             "QH"   /*!< Label */
#define BOARD_INITPINS_QH_NAME                                              "QH"   /*!< Identifier name */
#define BOARD_INITPINS_QH_DIRECTION                      kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTA4 (number 54), ETH_RST */
#define BOARD_INITPINS_ETH_RST_GPIO                                        GPIOA   /*!< GPIO device name: GPIOA */
#define BOARD_INITPINS_ETH_RST_PORT                                        PORTA   /*!< PORT device name: PORTA */
#define BOARD_INITPINS_ETH_RST_GPIO_PIN                                       4U   /*!< PORTA pin index: 4 */
#define BOARD_INITPINS_ETH_RST_PIN_NAME                                     PTA4   /*!< Pin name */
#define BOARD_INITPINS_ETH_RST_LABEL                                   "ETH_RST"   /*!< Label */
#define BOARD_INITPINS_ETH_RST_NAME                                    "ETH_RST"   /*!< Identifier name */
#define BOARD_INITPINS_ETH_RST_DIRECTION                kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTA6 (number 58), PHY_ENA */
#define BOARD_INITPINS_PHY_ENA_GPIO                                        GPIOA   /*!< GPIO device name: GPIOA */
#define BOARD_INITPINS_PHY_ENA_PORT                                        PORTA   /*!< PORT device name: PORTA */
#define BOARD_INITPINS_PHY_ENA_GPIO_PIN                                       6U   /*!< PORTA pin index: 6 */
#define BOARD_INITPINS_PHY_ENA_PIN_NAME                                     PTA6   /*!< Pin name */
#define BOARD_INITPINS_PHY_ENA_LABEL                                   "PHY_ENA"   /*!< Label */
#define BOARD_INITPINS_PHY_ENA_NAME                                    "PHY_ENA"   /*!< Identifier name */
#define BOARD_INITPINS_PHY_ENA_DIRECTION                kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTA7 (number 59), PHY_INT */
#define BOARD_INITPINS_PHY_INT_GPIO                                        GPIOA   /*!< GPIO device name: GPIOA */
#define BOARD_INITPINS_PHY_INT_PORT                                        PORTA   /*!< PORT device name: PORTA */
#define BOARD_INITPINS_PHY_INT_GPIO_PIN                                       7U   /*!< PORTA pin index: 7 */
#define BOARD_INITPINS_PHY_INT_PIN_NAME                                     PTA7   /*!< Pin name */
#define BOARD_INITPINS_PHY_INT_LABEL                                   "PHY_INT"   /*!< Label */
#define BOARD_INITPINS_PHY_INT_NAME                                    "PHY_INT"   /*!< Identifier name */
#define BOARD_INITPINS_PHY_INT_DIRECTION                 kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTA24 (number 75), STATUS_0 */
#define BOARD_INITPINS_STATUS_0_GPIO                                       GPIOA   /*!< GPIO device name: GPIOA */
#define BOARD_INITPINS_STATUS_0_PORT                                       PORTA   /*!< PORT device name: PORTA */
#define BOARD_INITPINS_STATUS_0_GPIO_PIN                                     24U   /*!< PORTA pin index: 24 */
#define BOARD_INITPINS_STATUS_0_PIN_NAME                                   PTA24   /*!< Pin name */
#define BOARD_INITPINS_STATUS_0_LABEL                                 "STATUS_0"   /*!< Label */
#define BOARD_INITPINS_STATUS_0_NAME                                  "STATUS_0"   /*!< Identifier name */
#define BOARD_INITPINS_STATUS_0_DIRECTION                kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTA25 (number 76), STATUS_1 */
#define BOARD_INITPINS_STATUS_1_GPIO                                       GPIOA   /*!< GPIO device name: GPIOA */
#define BOARD_INITPINS_STATUS_1_PORT                                       PORTA   /*!< PORT device name: PORTA */
#define BOARD_INITPINS_STATUS_1_GPIO_PIN                                     25U   /*!< PORTA pin index: 25 */
#define BOARD_INITPINS_STATUS_1_PIN_NAME                                   PTA25   /*!< Pin name */
#define BOARD_INITPINS_STATUS_1_LABEL                                 "STATUS_1"   /*!< Label */
#define BOARD_INITPINS_STATUS_1_NAME                                  "STATUS_1"   /*!< Identifier name */
#define BOARD_INITPINS_STATUS_1_DIRECTION                kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTA26 (number 77), STATUS_2 */
#define BOARD_INITPINS_STATUS_2_GPIO                                       GPIOA   /*!< GPIO device name: GPIOA */
#define BOARD_INITPINS_STATUS_2_PORT                                       PORTA   /*!< PORT device name: PORTA */
#define BOARD_INITPINS_STATUS_2_GPIO_PIN                                     26U   /*!< PORTA pin index: 26 */
#define BOARD_INITPINS_STATUS_2_PIN_NAME                                   PTA26   /*!< Pin name */
#define BOARD_INITPINS_STATUS_2_LABEL                                 "STATUS_2"   /*!< Label */
#define BOARD_INITPINS_STATUS_2_NAME                                  "STATUS_2"   /*!< Identifier name */
#define BOARD_INITPINS_STATUS_2_DIRECTION               kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTA27 (number 78), STATUS_3 */
#define BOARD_INITPINS_STATUS_3_GPIO                                       GPIOA   /*!< GPIO device name: GPIOA */
#define BOARD_INITPINS_STATUS_3_PORT                                       PORTA   /*!< PORT device name: PORTA */
#define BOARD_INITPINS_STATUS_3_GPIO_PIN                                     27U   /*!< PORTA pin index: 27 */
#define BOARD_INITPINS_STATUS_3_PIN_NAME                                   PTA27   /*!< Pin name */
#define BOARD_INITPINS_STATUS_3_LABEL                                 "STATUS_3"   /*!< Label */
#define BOARD_INITPINS_STATUS_3_NAME                                  "STATUS_3"   /*!< Identifier name */
#define BOARD_INITPINS_STATUS_3_DIRECTION               kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTB5 (number 86), LCD_VBAT_ENA */
#define BOARD_INITPINS_LCD_VBAT_ENA_GPIO                                   GPIOB   /*!< GPIO device name: GPIOB */
#define BOARD_INITPINS_LCD_VBAT_ENA_PORT                                   PORTB   /*!< PORT device name: PORTB */
#define BOARD_INITPINS_LCD_VBAT_ENA_GPIO_PIN                                  5U   /*!< PORTB pin index: 5 */
#define BOARD_INITPINS_LCD_VBAT_ENA_PIN_NAME                                PTB5   /*!< Pin name */
#define BOARD_INITPINS_LCD_VBAT_ENA_LABEL                         "LCD_VBAT_ENA"   /*!< Label */
#define BOARD_INITPINS_LCD_VBAT_ENA_NAME                          "LCD_VBAT_ENA"   /*!< Identifier name */
#define BOARD_INITPINS_LCD_VBAT_ENA_DIRECTION           kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTB6 (number 87), LCD_3V3 */
#define BOARD_INITPINS_LCD_3V3_GPIO                                        GPIOB   /*!< GPIO device name: GPIOB */
#define BOARD_INITPINS_LCD_3V3_PORT                                        PORTB   /*!< PORT device name: PORTB */
#define BOARD_INITPINS_LCD_3V3_GPIO_PIN                                       6U   /*!< PORTB pin index: 6 */
#define BOARD_INITPINS_LCD_3V3_PIN_NAME                                     PTB6   /*!< Pin name */
#define BOARD_INITPINS_LCD_3V3_LABEL                                   "LCD_3V3"   /*!< Label */
#define BOARD_INITPINS_LCD_3V3_NAME                                    "LCD_3V3"   /*!< Identifier name */
#define BOARD_INITPINS_LCD_3V3_DIRECTION                kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTB8 (number 89), LCD_DC */
#define BOARD_INITPINS_LCD_DC_GPIO                                         GPIOB   /*!< GPIO device name: GPIOB */
#define BOARD_INITPINS_LCD_DC_PORT                                         PORTB   /*!< PORT device name: PORTB */
#define BOARD_INITPINS_LCD_DC_GPIO_PIN                                        8U   /*!< PORTB pin index: 8 */
#define BOARD_INITPINS_LCD_DC_PIN_NAME                                      PTB8   /*!< Pin name */
#define BOARD_INITPINS_LCD_DC_LABEL                                     "LCD_DC"   /*!< Label */
#define BOARD_INITPINS_LCD_DC_NAME                                      "LCD_DC"   /*!< Identifier name */
#define BOARD_INITPINS_LCD_DC_DIRECTION                 kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTB9 (number 90), LCD_RST */
#define BOARD_INITPINS_LCD_RST_GPIO                                        GPIOB   /*!< GPIO device name: GPIOB */
#define BOARD_INITPINS_LCD_RST_PORT                                        PORTB   /*!< PORT device name: PORTB */
#define BOARD_INITPINS_LCD_RST_GPIO_PIN                                       9U   /*!< PORTB pin index: 9 */
#define BOARD_INITPINS_LCD_RST_PIN_NAME                                     PTB9   /*!< Pin name */
#define BOARD_INITPINS_LCD_RST_LABEL                                   "LCD_RST"   /*!< Label */
#define BOARD_INITPINS_LCD_RST_NAME                                    "LCD_RST"   /*!< Identifier name */
#define BOARD_INITPINS_LCD_RST_DIRECTION                kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTB17 (number 96), S32_RST */
#define BOARD_INITPINS_S32_RST_GPIO                                        GPIOB   /*!< GPIO device name: GPIOB */
#define BOARD_INITPINS_S32_RST_PORT                                        PORTB   /*!< PORT device name: PORTB */
#define BOARD_INITPINS_S32_RST_GPIO_PIN                                      17U   /*!< PORTB pin index: 17 */
#define BOARD_INITPINS_S32_RST_PIN_NAME                                    PTB17   /*!< Pin name */
#define BOARD_INITPINS_S32_RST_LABEL                                   "S32_RST"   /*!< Label */
#define BOARD_INITPINS_S32_RST_NAME                                    "S32_RST"   /*!< Identifier name */
#define BOARD_INITPINS_S32_RST_DIRECTION                kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTC0 (number 103), LED_TRUCK_3 */
#define BOARD_INITPINS_LED_TRUCK_3_GPIO                                    GPIOC   /*!< GPIO device name: GPIOC */
#define BOARD_INITPINS_LED_TRUCK_3_PORT                                    PORTC   /*!< PORT device name: PORTC */
#define BOARD_INITPINS_LED_TRUCK_3_GPIO_PIN                                   1U   /*!< PORTC pin index: 0 */
#define BOARD_INITPINS_LED_TRUCK_3_PIN_NAME                                 PTC1   /*!< Pin name */
#define BOARD_INITPINS_LED_TRUCK_3_LABEL                           "LED_TRUCK_3"   /*!< Label */
#define BOARD_INITPINS_LED_TRUCK_3_NAME                            "LED_TRUCK_3"   /*!< Identifier name */
#define BOARD_INITPINS_LED_TRUCK_3_DIRECTION            kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTC5 (number 110), ACCEL_INT1 */
#define BOARD_INITPINS_ACCEL_INT1_GPIO                                     GPIOC   /*!< GPIO device name: GPIOC */
#define BOARD_INITPINS_ACCEL_INT1_PORT                                     PORTC   /*!< PORT device name: PORTC */
#define BOARD_INITPINS_ACCEL_INT1_GPIO_PIN                                    5U   /*!< PORTC pin index: 5 */
#define BOARD_INITPINS_ACCEL_INT1_PIN_NAME                                  PTC5   /*!< Pin name */
#define BOARD_INITPINS_ACCEL_INT1_LABEL                             "ACCEL_INT1"   /*!< Label */
#define BOARD_INITPINS_ACCEL_INT1_NAME                              "ACCEL_INT1"   /*!< Identifier name */
#define BOARD_INITPINS_ACCEL_INT1_DIRECTION              kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTC6 (number 111), ACCEL_INT2 */
#define BOARD_INITPINS_ACCEL_INT2_GPIO                                     GPIOC   /*!< GPIO device name: GPIOC */
#define BOARD_INITPINS_ACCEL_INT2_PORT                                     PORTC   /*!< PORT device name: PORTC */
#define BOARD_INITPINS_ACCEL_INT2_GPIO_PIN                                    6U   /*!< PORTC pin index: 6 */
#define BOARD_INITPINS_ACCEL_INT2_PIN_NAME                                  PTC6   /*!< Pin name */
#define BOARD_INITPINS_ACCEL_INT2_LABEL                             "ACCEL_INT2"   /*!< Label */
#define BOARD_INITPINS_ACCEL_INT2_NAME                              "ACCEL_INT2"   /*!< Identifier name */
#define BOARD_INITPINS_ACCEL_INT2_DIRECTION              kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTC8 (number 113), ACCEL_3V3 */
#define BOARD_INITPINS_ACCEL_3V3_GPIO                                      GPIOC   /*!< GPIO device name: GPIOC */
#define BOARD_INITPINS_ACCEL_3V3_PORT                                      PORTC   /*!< PORT device name: PORTC */
#define BOARD_INITPINS_ACCEL_3V3_GPIO_PIN                                     8U   /*!< PORTC pin index: 8 */
#define BOARD_INITPINS_ACCEL_3V3_PIN_NAME                                   PTC8   /*!< Pin name */
#define BOARD_INITPINS_ACCEL_3V3_LABEL                               "ACCEL_3V3"   /*!< Label */
#define BOARD_INITPINS_ACCEL_3V3_NAME                                "ACCEL_3V3"   /*!< Identifier name */
#define BOARD_INITPINS_ACCEL_3V3_DIRECTION              kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTC9 (number 114), ACCEL_RST */
#define BOARD_INITPINS_ACCEL_RST_GPIO                                      GPIOC   /*!< GPIO device name: GPIOC */
#define BOARD_INITPINS_ACCEL_RST_PORT                                      PORTC   /*!< PORT device name: PORTC */
#define BOARD_INITPINS_ACCEL_RST_GPIO_PIN                                     9U   /*!< PORTC pin index: 9 */
#define BOARD_INITPINS_ACCEL_RST_PIN_NAME                                   PTC9   /*!< Pin name */
#define BOARD_INITPINS_ACCEL_RST_LABEL                               "ACCEL_RST"   /*!< Label */
#define BOARD_INITPINS_ACCEL_RST_NAME                                "ACCEL_RST"   /*!< Identifier name */
#define BOARD_INITPINS_ACCEL_RST_DIRECTION              kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTC13 (number 118), LIN_ENA */
#define BOARD_INITPINS_LIN_ENA_GPIO                                        GPIOC   /*!< GPIO device name: GPIOC */
#define BOARD_INITPINS_LIN_ENA_PORT                                        PORTC   /*!< PORT device name: PORTC */
#define BOARD_INITPINS_LIN_ENA_GPIO_PIN                                      13U   /*!< PORTC pin index: 13 */
#define BOARD_INITPINS_LIN_ENA_PIN_NAME                                    PTC13   /*!< Pin name */
#define BOARD_INITPINS_LIN_ENA_LABEL                                   "LIN_ENA"   /*!< Label */
#define BOARD_INITPINS_LIN_ENA_NAME                                    "LIN_ENA"   /*!< Identifier name */
#define BOARD_INITPINS_LIN_ENA_DIRECTION                kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTD1 (number 128), LED_CLR */
#define BOARD_INITPINS_LED_CLR_GPIO                                        GPIOD   /*!< GPIO device name: GPIOD */
#define BOARD_INITPINS_LED_CLR_PORT                                        PORTD   /*!< PORT device name: PORTD */
#define BOARD_INITPINS_LED_CLR_GPIO_PIN                                       1U   /*!< PORTD pin index: 1 */
#define BOARD_INITPINS_LED_CLR_PIN_NAME                                     PTD1   /*!< Pin name */
#define BOARD_INITPINS_LED_CLR_LABEL                                   "LED_CLR"   /*!< Label */
#define BOARD_INITPINS_LED_CLR_NAME                                    "LED_CLR"   /*!< Identifier name */
#define BOARD_INITPINS_LED_CLR_DIRECTION                kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTD2 (number 129), LED_DATA */
#define BOARD_INITPINS_LED_DATA_GPIO                                       GPIOD   /*!< GPIO device name: GPIOD */
#define BOARD_INITPINS_LED_DATA_PORT                                       PORTD   /*!< PORT device name: PORTD */
#define BOARD_INITPINS_LED_DATA_GPIO_PIN                                      2U   /*!< PORTD pin index: 2 */
#define BOARD_INITPINS_LED_DATA_PIN_NAME                                    PTD2   /*!< Pin name */
#define BOARD_INITPINS_LED_DATA_LABEL                                 "LED_DATA"   /*!< Label */
#define BOARD_INITPINS_LED_DATA_NAME                                  "LED_DATA"   /*!< Identifier name */
#define BOARD_INITPINS_LED_DATA_DIRECTION               kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTD3 (number 130), LED_SH_CLK */
#define BOARD_INITPINS_LED_SH_CLK_GPIO                                     GPIOD   /*!< GPIO device name: GPIOD */
#define BOARD_INITPINS_LED_SH_CLK_PORT                                     PORTD   /*!< PORT device name: PORTD */
#define BOARD_INITPINS_LED_SH_CLK_GPIO_PIN                                    3U   /*!< PORTD pin index: 3 */
#define BOARD_INITPINS_LED_SH_CLK_PIN_NAME                                  PTD3   /*!< Pin name */
#define BOARD_INITPINS_LED_SH_CLK_LABEL                             "LED_SH_CLK"   /*!< Label */
#define BOARD_INITPINS_LED_SH_CLK_NAME                              "LED_SH_CLK"   /*!< Identifier name */
#define BOARD_INITPINS_LED_SH_CLK_DIRECTION             kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTD4 (number 131), LED_ST_CLK */
#define BOARD_INITPINS_LED_ST_CLK_GPIO                                     GPIOD   /*!< GPIO device name: GPIOD */
#define BOARD_INITPINS_LED_ST_CLK_PORT                                     PORTD   /*!< PORT device name: PORTD */
#define BOARD_INITPINS_LED_ST_CLK_GPIO_PIN                                    4U   /*!< PORTD pin index: 4 */
#define BOARD_INITPINS_LED_ST_CLK_PIN_NAME                                  PTD4   /*!< Pin name */
#define BOARD_INITPINS_LED_ST_CLK_LABEL                             "LED_ST_CLK"   /*!< Label */
#define BOARD_INITPINS_LED_ST_CLK_NAME                              "LED_ST_CLK"   /*!< Identifier name */
#define BOARD_INITPINS_LED_ST_CLK_DIRECTION             kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTD12 (number 141), OBD2_DET */
#define BOARD_INITPINS_OBD2_DET_GPIO                                       GPIOD   /*!< GPIO device name: GPIOD */
#define BOARD_INITPINS_OBD2_DET_PORT                                       PORTD   /*!< PORT device name: PORTD */
#define BOARD_INITPINS_OBD2_DET_GPIO_PIN                                     12U   /*!< PORTD pin index: 12 */
#define BOARD_INITPINS_OBD2_DET_PIN_NAME                                   PTD12   /*!< Pin name */
#define BOARD_INITPINS_OBD2_DET_LABEL                                 "OBD2_DET"   /*!< Label */
#define BOARD_INITPINS_OBD2_DET_NAME                                  "OBD2_DET"   /*!< Identifier name */
#define BOARD_INITPINS_OBD2_DET_DIRECTION                kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTD13 (number 142), BAT_PPR */
#define BOARD_INITPINS_BAT_PPR_GPIO                                        GPIOD   /*!< GPIO device name: GPIOD */
#define BOARD_INITPINS_BAT_PPR_PORT                                        PORTD   /*!< PORT device name: PORTD */
#define BOARD_INITPINS_BAT_PPR_GPIO_PIN                                      13U   /*!< PORTD pin index: 13 */
#define BOARD_INITPINS_BAT_PPR_PIN_NAME                                    PTD13   /*!< Pin name */
#define BOARD_INITPINS_BAT_PPR_LABEL                                   "BAT_PPR"   /*!< Label */
#define BOARD_INITPINS_BAT_PPR_NAME                                    "BAT_PPR"   /*!< Identifier name */
#define BOARD_INITPINS_BAT_PPR_DIRECTION                 kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTD14 (number 143), BAT_CHG */
#define BOARD_INITPINS_BAT_CHG_GPIO                                        GPIOD   /*!< GPIO device name: GPIOD */
#define BOARD_INITPINS_BAT_CHG_PORT                                        PORTD   /*!< PORT device name: PORTD */
#define BOARD_INITPINS_BAT_CHG_GPIO_PIN                                      14U   /*!< PORTD pin index: 14 */
#define BOARD_INITPINS_BAT_CHG_PIN_NAME                                    PTD14   /*!< Pin name */
#define BOARD_INITPINS_BAT_CHG_LABEL                                   "BAT_CHG"   /*!< Label */
#define BOARD_INITPINS_BAT_CHG_NAME                                    "BAT_CHG"   /*!< Identifier name */
#define BOARD_INITPINS_BAT_CHG_DIRECTION                 kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTD15 (number 144), SD_DET */
#define BOARD_INITPINS_SD_DET_GPIO                                         GPIOD   /*!< GPIO device name: GPIOD */
#define BOARD_INITPINS_SD_DET_PORT                                         PORTD   /*!< PORT device name: PORTD */
#define BOARD_INITPINS_SD_DET_GPIO_PIN                                       15U   /*!< PORTD pin index: 15 */
#define BOARD_INITPINS_SD_DET_PIN_NAME                                     PTD15   /*!< Pin name */
#define BOARD_INITPINS_SD_DET_LABEL                                     "SD_DET"   /*!< Label */
#define BOARD_INITPINS_SD_DET_NAME                                      "SD_DET"   /*!< Identifier name */
#define BOARD_INITPINS_SD_DET_DIRECTION                  kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTD0 (number 127), LED_OE */
#define BOARD_INITPINS_LED_OE_PERIPHERAL                                    FTM3   /*!< Device name: FTM3 */
#define BOARD_INITPINS_LED_OE_SIGNAL                                          CH   /*!< FTM3 signal: CH */
#define BOARD_INITPINS_LED_OE_CHANNEL                                          0   /*!< FTM3 channel: 0 */
#define BOARD_INITPINS_LED_OE_PIN_NAME                                  FTM3_CH0   /*!< Pin name */
#define BOARD_INITPINS_LED_OE_LABEL                                     "LED_OE"   /*!< Label */
#define BOARD_INITPINS_LED_OE_NAME                                      "LED_OE"   /*!< Identifier name */
#define BOARD_INITPINS_LED_OE_DIRECTION                 kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTD6 (number 133), LED_TRUCK_1 */
#define BOARD_INITPINS_LED_TRUCK_1_GPIO                                    GPIOD   /*!< GPIO device name: GPIOD */
#define BOARD_INITPINS_LED_TRUCK_1_PORT                                    PORTD   /*!< PORT device name: PORTD */
#define BOARD_INITPINS_LED_TRUCK_1_GPIO_PIN                                   6U   /*!< PORTD pin index: 6 */
#define BOARD_INITPINS_LED_TRUCK_1_PIN_NAME                                 PTD6   /*!< Pin name */
#define BOARD_INITPINS_LED_TRUCK_1_LABEL                           "LED_TRUCK_1"   /*!< Label */
#define BOARD_INITPINS_LED_TRUCK_1_NAME                            "LED_TRUCK_1"   /*!< Identifier name */
#define BOARD_INITPINS_LED_TRUCK_1_DIRECTION            kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTC4 (number 109), LED_TRUCK_2 */
#define BOARD_INITPINS_LED_TRUCK_2_GPIO                                    GPIOC   /*!< GPIO device name: GPIOC */
#define BOARD_INITPINS_LED_TRUCK_2_PORT                                    PORTC   /*!< PORT device name: PORTC */
#define BOARD_INITPINS_LED_TRUCK_2_GPIO_PIN                                   4U   /*!< PORTC pin index: 4 */
#define BOARD_INITPINS_LED_TRUCK_2_PIN_NAME                                 PTC4   /*!< Pin name */
#define BOARD_INITPINS_LED_TRUCK_2_LABEL                           "LED_TRUCK_2"   /*!< Label */
#define BOARD_INITPINS_LED_TRUCK_2_NAME                            "LED_TRUCK_2"   /*!< Identifier name */
#define BOARD_INITPINS_LED_TRUCK_2_DIRECTION            kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTC0 (number 103), CHARGE_ENA */
#define BOARD_INITPINS_CHARGE_ENA_GPIO                                    GPIOC   /*!< GPIO device name: GPIOC */
#define BOARD_INITPINS_CHARGE_ENA_PORT                                    PORTC   /*!< PORT device name: PORTC */
#define BOARD_INITPINS_CHARGE_ENA_GPIO_PIN                                   0U   /*!< PORTC pin index: 0 */
#define BOARD_INITPINS_CHARGE_ENA_PIN_NAME                                 PTC0   /*!< Pin name */
#define BOARD_INITPINS_CHARGE_ENA_LABEL                           "CHARGE_ENA"   /*!< Label */
#define BOARD_INITPINS_CHARGE_ENA_NAME                            "CHARGE_ENA"   /*!< Identifier name */
#define BOARD_INITPINS_CHARGE_ENA_DIRECTION            kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTC1 (number 104), LED_TRUCK_0 */
#define BOARD_INITPINS_LED_TRUCK_0_GPIO                                    GPIOC   /*!< GPIO device name: GPIOC */
#define BOARD_INITPINS_LED_TRUCK_0_PORT                                    PORTC   /*!< PORT device name: PORTC */
#define BOARD_INITPINS_LED_TRUCK_0_GPIO_PIN                                   2U   /*!< PORTC pin index: 1 */
#define BOARD_INITPINS_LED_TRUCK_0_PIN_NAME                                 PTC2   /*!< Pin name */
#define BOARD_INITPINS_LED_TRUCK_0_LABEL                           "LED_TRUCK_0"   /*!< Label */
#define BOARD_INITPINS_LED_TRUCK_0_NAME                            "LED_TRUCK_0"   /*!< Identifier name */
#define BOARD_INITPINS_LED_TRUCK_0_DIRECTION            kPIN_MUX_DirectionOutput   /*!< Direction */


/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
