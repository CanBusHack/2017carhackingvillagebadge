/*
 * Copyright (c) 2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file S32K144_features.h
 * @brief Chip specific module features
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.3, Global typedef not referenced.
 * Type used only in some modules of the SDK.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * The macros defined are used to define features for each driver, so this might be reported
 * when the analysis is made only on one driver.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Directive 4.9, Function-like macro
 * These are very simple macros used for abstracting hw implementation.
 * They help make the code easy to understand.
 */

#if !defined(S32K144_FEATURES_H)
#define S32K144_FEATURES_H

/* ERRATA sections*/

/* @brief ARM Errata 838869: Store immediate overlapping exception return operation might vector to
 * incorrect interrupt. */
#define ERRATA_E9005

/* @brief ARM Errata 709718: VDIV or VSQRT instructions might not complete correctly when very
 * short ISRs are used. */
#define ERRATA_E6940

/* @brief E10655: When using LPSPI in master mode and the SR[MBF] bit is read as a one, then, the
 * flag is set. If it is read as a zero, it must be read second time and this second read will be
 * the correct state of the bit.​ */
#define ERRATA_E10655

/* @brief E10792: LPI2C: Slave Transmit Data Flag may incorrectly read as one when TXCFG is zero.
 * Interrupts for transfer data should be enabled after the address valid event is detected and
 * disabled at the end of the transfer. */
#define ERRATA_E10792

/* PCC module features */

/* @brief Has InUse feature (register bit PCC[INUSE]). */
#define FEATURE_PCC_HAS_IN_USE_FEATURE (0)

/* PORT module features */

/* @brief Has control lock (register bit PCR[LK]). */
#define FEATURE_PORT_HAS_PIN_CONTROL_LOCK (1)
/* @brief Has open drain control (register bit PCR[ODE]). */
#define FEATURE_PORT_HAS_OPEN_DRAIN (0)
/* @brief Has digital filter (registers DFER, DFCR and DFWR). */
#define FEATURE_PORT_HAS_DIGITAL_FILTER (1)
/* @brief Has DMA request (register bit field PCR[IRQC] values). */
#define FEATURE_PORT_HAS_DMA_REQUEST (1)
/* @brief Has pull resistor selection available. */
#define FEATURE_PORT_HAS_PULL_SELECTION (1)
/* @brief Has slew rate control (register bit PCR[SRE]). */
#define FEATURE_PORT_HAS_SLEW_RATE (0)
/* @brief Has passive filter (register bit field PCR[PFE]). */
#define FEATURE_PORT_HAS_PASSIVE_FILTER (1)
/* @brief Has drive strength control (register bit PCR[DSE]). */
#define FEATURE_PORT_HAS_DRIVE_STRENGTH (1)

/* SOC module features */

/* @brief PORT availability on the SoC. */
#define FEATURE_SOC_PORT_COUNT (5)

#define FEATURE_SOC_SCG_COUNT (1)
/* @brief Slow IRC low range clock frequency. */
#define FEATURE_SCG_SIRC_LOW_RANGE_FREQ  (2000000U)
/* @brief Slow IRC high range clock frequency. */
#define FEATURE_SCG_SIRC_HIGH_RANGE_FREQ (8000000U)

/* @brief Fast IRC trimmed clock frequency(48MHz). */
#define FEATURE_SCG_FIRC_FREQ0  (48000000U)
/* @brief Fast IRC trimmed clock frequency(52MHz). */
#define FEATURE_SCG_FIRC_FREQ1  (52000000U)
/* @brief Fast IRC trimmed clock frequency(56MHz). */
#define FEATURE_SCG_FIRC_FREQ2  (56000000U)
/* @brief Fast IRC trimmed clock frequency(60MHz). */
#define FEATURE_SCG_FIRC_FREQ3  (60000000U)

/* FLASH module features */

/* @brief Is of type FTFA. */
#define FEATURE_FLS_IS_FTFA (0u)
/* @brief Is of type FTFC. */
#define FEATURE_FLS_IS_FTFC (1u)
/* @brief Is of type FTFE. */
#define FEATURE_FLS_IS_FTFE (0u)
/* @brief Is of type FTFL. */
#define FEATURE_FLS_IS_FTFL (0u)
/* @brief Has flags indicating the status of the FlexRAM (register bits FCNFG[EEERDY], FCNFG[RAMRDY] and FCNFG[PFLSH]). */
#define FEATURE_FLS_HAS_FLEX_RAM_FLAGS (1u)
/* @brief Has program flash swapping status flag (register bit FCNFG[SWAP]). */
#define FEATURE_FLS_HAS_PF_SWAPPING_STATUS_FLAG (0u)
/* @brief Has EEPROM region protection (register FEPROT). */
#define FEATURE_FLS_HAS_EEROM_REGION_PROTECTION (1u)
/* @brief Has data flash region protection (register FDPROT). */
#define FEATURE_FLS_HAS_DATA_FLS_REGION_PROTECTION (1u)
/* @brief P-Flash block count. */
#define FEATURE_FLS_PF_BLOCK_COUNT (1u)
/* @brief P-Flash block size. */
#define FEATURE_FLS_PF_BLOCK_SIZE (524288u)
/* @brief P-Flash sector size. */
#define FEATURE_FLS_PF_BLOCK_SECTOR_SIZE (4096u)
/* @brief P-Flash write unit size. */
#define FEATURE_FLS_PF_BLOCK_WRITE_UNIT_SIZE (8u)
/* @brief P-Flash block swap feature. */
#define FEATURE_FLS_HAS_PF_BLOCK_SWAP (0u)
/* @brief Has FlexNVM memory. */
#define FEATURE_FLS_HAS_FLEX_NVM (1u)
/* @brief FlexNVM block count. */
#define FEATURE_FLS_DF_BLOCK_COUNT (1u)
/* @brief FlexNVM block size. */
#define FEATURE_FLS_DF_BLOCK_SIZE (65536u)
/* @brief FlexNVM sector size. */
#define FEATURE_FLS_DF_BLOCK_SECTOR_SIZE (2048u)
/* @brief FlexNVM write unit size. */
#define FEATURE_FLS_DF_BLOCK_WRITE_UNIT_SIZE (8u)
/* @brief FlexNVM start address. (Valid only if FlexNVM is available.) */
#define FEATURE_FLS_DF_START_ADDRESS (0x10000000u)
/* @brief Has FlexRAM memory. */
#define FEATURE_FLS_HAS_FLEX_RAM (1u)
/* @brief FlexRAM size. */
#define FEATURE_FLS_FLEX_RAM_SIZE (4096u)
/* @brief FlexRAM start address. (Valid only if FlexRAM is available.) */
#define FEATURE_FLS_FLEX_RAM_START_ADDRESS (0x14000000u)
/* @brief Has 0x00 Read 1s Block command. */
#define FEATURE_FLS_HAS_READ_1S_BLOCK_CMD (1u)
/* @brief Has 0x01 Read 1s Section command. */
#define FEATURE_FLS_HAS_READ_1S_SECTION_CMD (1u)
/* @brief Has 0x02 Program Check command. */
#define FEATURE_FLS_HAS_PROGRAM_CHECK_CMD (1u)
/* @brief Has 0x03 Read Resource command. */
#define FEATURE_FLS_HAS_READ_RESOURCE_CMD (0u)
/* @brief Has 0x06 Program Longword command. */
#define FEATURE_FLS_HAS_PROGRAM_LONGWORD_CMD (0u)
/* @brief Has 0x07 Program Phrase command. */
#define FEATURE_FLS_HAS_PROGRAM_PHRASE_CMD (1u)
/* @brief Has 0x08 Erase Flash Block command. */
#define FEATURE_FLS_HAS_ERASE_BLOCK_CMD (1u)
/* @brief Has 0x09 Erase Flash Sector command. */
#define FEATURE_FLS_HAS_ERASE_SECTOR_CMD (1u)
/* @brief Has 0x0B Program Section command. */
#define FEATURE_FLS_HAS_PROGRAM_SECTION_CMD (1u)
/* @brief Has 0x40 Read 1s All Blocks command. */
#define FEATURE_FLS_HAS_READ_1S_ALL_BLOCKS_CMD (1u)
/* @brief Has 0x41 Read Once command. */
#define FEATURE_FLS_HAS_READ_ONCE_CMD (1u)
/* @brief Has 0x43 Program Once command. */
#define FEATURE_FLS_HAS_PROGRAM_ONCE_CMD (1u)
/* @brief Has 0x44 Erase All Blocks command. */
#define FEATURE_FLS_HAS_ERASE_ALL_CMD (1u)
/* @brief Has 0x45 Verify Backdoor Access Key command. */
#define FEATURE_FLS_HAS_VERIFY_BACKDOOR_ACCESS_KEY_CMD (1u)
/* @brief Has 0x46 Swap Control command. */
#define FEATURE_FLS_HAS_SWAP_CONTROL_CMD (0u)
/* @brief Has 0x49 Erase All Blocks unsecure command. */
#define FEATURE_FLS_HAS_ERASE_ALL_BLOCKS_UNSECURE_CMD (1u)
/* @brief Has 0x80 Program Partition command. */
#define FEATURE_FLS_HAS_PROGRAM_PARTITION_CMD (1u)
/* @brief Has 0x81 Set FlexRAM Function command. */
#define FEATURE_FLS_HAS_SET_FLEXRAM_FUNCTION_CMD (1u)
/* @brief P-Flash Erase/Read 1st all block command address alignment. */
#define FEATURE_FLS_PF_BLOCK_CMD_ADDRESS_ALIGMENT (16u)
/* @brief P-Flash Erase sector command address alignment. */
#define FEATURE_FLS_PF_SECTOR_CMD_ADDRESS_ALIGMENT (16u)
/* @brief P-Flash Program/Verify section command address alignment. */
#define FEATURE_FLS_PF_SECTION_CMD_ADDRESS_ALIGMENT (16u)
/* @brief P-Flash Read resource command address alignment. */
#define FEATURE_FLS_PF_RESOURCE_CMD_ADDRESS_ALIGMENT (8u)
/* @brief P-Flash Program check command address alignment. */
#define FEATURE_FLS_PF_CHECK_CMD_ADDRESS_ALIGMENT (4u)
/* @brief P-Flash Program check command address alignment. */
#define FEATURE_FLS_PF_SWAP_CONTROL_CMD_ADDRESS_ALIGMENT (0u)
/* @brief FlexNVM Erase/Read 1st all block command address alignment. */
#define FEATURE_FLS_DF_BLOCK_CMD_ADDRESS_ALIGMENT (8u)
/* @brief FlexNVM Erase sector command address alignment. */
#define FEATURE_FLS_DF_SECTOR_CMD_ADDRESS_ALIGMENT (8u)
/* @brief FlexNVM Program/Verify section command address alignment. */
#define FEATURE_FLS_DF_SECTION_CMD_ADDRESS_ALIGMENT (8u)
/* @brief FlexNVM Read resource command address alignment. */
#define FEATURE_FLS_DF_RESOURCE_CMD_ADDRESS_ALIGMENT (8u)
/* @brief FlexNVM Program check command address alignment. */
#define FEATURE_FLS_DF_CHECK_CMD_ADDRESS_ALIGMENT (4u)
/* @brief FlexNVM partition code 0000 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FEATURE_FLS_DF_SIZE_0000 (0x00010000u)
/* @brief FlexNVM partition code 0001 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FEATURE_FLS_DF_SIZE_0001 (0xFFFFFFFFu)
/* @brief FlexNVM partition code 0010 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FEATURE_FLS_DF_SIZE_0010 (0xFFFFFFFFu)
/* @brief FlexNVM partition code 0011 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FEATURE_FLS_DF_SIZE_0011 (0x00008000u)
/* @brief FlexNVM partition code 0100 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FEATURE_FLS_DF_SIZE_0100 (0x00000000u)
/* @brief FlexNVM partition code 0101 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FEATURE_FLS_DF_SIZE_0101 (0xFFFFFFFFu)
/* @brief FlexNVM partition code 0110 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FEATURE_FLS_DF_SIZE_0110 (0xFFFFFFFFu)
/* @brief FlexNVM partition code 0111 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FEATURE_FLS_DF_SIZE_0111 (0xFFFFFFFFu)
/* @brief FlexNVM partition code 1000 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FEATURE_FLS_DF_SIZE_1000 (0x00000000u)
/* @brief FlexNVM partition code 1001 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FEATURE_FLS_DF_SIZE_1001 (0xFFFFFFFFu)
/* @brief FlexNVM partition code 1010 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FEATURE_FLS_DF_SIZE_1010 (0x00004000u)
/* @brief FlexNVM partition code 1011 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FEATURE_FLS_DF_SIZE_1011 (0x00008000u)
/* @brief FlexNVM partition code 1100 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FEATURE_FLS_DF_SIZE_1100 (0x00010000u)
/* @brief FlexNVM partition code 1101 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FEATURE_FLS_DF_SIZE_1101 (0xFFFFFFFFu)
/* @brief FlexNVM partition code 1110 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FEATURE_FLS_DF_SIZE_1110 (0xFFFFFFFFu)
/* @brief FlexNVM partition code 1111 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FEATURE_FLS_DF_SIZE_1111 (0x00010000u)
/* @brief Emulated EEPROM size code 0000 mapping to emulated EEPROM size in bytes (0xFFFF = reserved). */
#define FEATURE_FLS_EE_SIZE_0000 (0xFFFFu)
/* @brief Emulated EEPROM size code 0001 mapping to emulated EEPROM size in bytes (0xFFFF = reserved). */
#define FEATURE_FLS_EE_SIZE_0001 (0xFFFFu)
/* @brief Emulated EEPROM size code 0010 mapping to emulated EEPROM size in bytes (0xFFFF = reserved). */
#define FEATURE_FLS_EE_SIZE_0010 (0x1000u)
/* @brief Emulated EEPROM size code 0011 mapping to emulated EEPROM size in bytes (0xFFFF = reserved). */
#define FEATURE_FLS_EE_SIZE_0011 (0x0800u)
/* @brief Emulated EEPROM size code 0100 mapping to emulated EEPROM size in bytes (0xFFFF = reserved). */
#define FEATURE_FLS_EE_SIZE_0100 (0x0400u)
/* @brief Emulated EEPROM size code 0101 mapping to emulated EEPROM size in bytes (0xFFFF = reserved). */
#define FEATURE_FLS_EE_SIZE_0101 (0x0200u)
/* @brief Emulated EEPROM size code 0110 mapping to emulated EEPROM size in bytes (0xFFFF = reserved). */
#define FEATURE_FLS_EE_SIZE_0110 (0x0100u)
/* @brief Emulated EEPROM size code 0111 mapping to emulated EEPROM size in bytes (0xFFFF = reserved). */
#define FEATURE_FLS_EE_SIZE_0111 (0x0080u)
/* @brief Emulated EEPROM size code 1000 mapping to emulated EEPROM size in bytes (0xFFFF = reserved). */
#define FEATURE_FLS_EE_SIZE_1000 (0x0040u)
/* @brief Emulated EEPROM size code 1001 mapping to emulated EEPROM size in bytes (0xFFFF = reserved). */
#define FEATURE_FLS_EE_SIZE_1001 (0x0020u)
/* @brief Emulated EEPROM size code 1010 mapping to emulated EEPROM size in bytes (0xFFFF = reserved). */
#define FEATURE_FLS_EE_SIZE_1010 (0xFFFFu)
/* @brief Emulated EEPROM size code 1011 mapping to emulated EEPROM size in bytes (0xFFFF = reserved). */
#define FEATURE_FLS_EE_SIZE_1011 (0xFFFFu)
/* @brief Emulated EEPROM size code 1100 mapping to emulated EEPROM size in bytes (0xFFFF = reserved). */
#define FEATURE_FLS_EE_SIZE_1100 (0xFFFFu)
/* @brief Emulated EEPROM size code 1101 mapping to emulated EEPROM size in bytes (0xFFFF = reserved). */
#define FEATURE_FLS_EE_SIZE_1101 (0xFFFFu)
/* @brief Emulated EEPROM size code 1110 mapping to emulated EEPROM size in bytes (0xFFFF = reserved). */
#define FEATURE_FLS_EE_SIZE_1110 (0xFFFFu)
/* @brief Emulated EEPROM size code 1111 mapping to emulated EEPROM size in bytes (0xFFFF = reserved). */
#define FEATURE_FLS_EE_SIZE_1111 (0x0000u)

/* CAN module features */

/* @brief Frames available in Rx FIFO flag shift */
#define FEATURE_CAN_RXFIFO_FRAME_AVAILABLE  (5U)
/* @brief Rx FIFO warning flag shift */
#define FEATURE_CAN_RXFIFO_WARNING          (6U)
/* @brief Rx FIFO overflow flag shift */
#define FEATURE_CAN_RXFIFO_OVERFLOW         (7U)
/* @brief Has Flexible Data Rate for CAN0 */
#define FEATURE_CAN0_HAS_FD                 (1)
/* @brief Has Flexible Data Rate for CAN1 */
#define FEATURE_CAN1_HAS_FD                 (0)
/* @brief Has Flexible Data Rate for CAN2 */
#define FEATURE_CAN2_HAS_FD                 (0)
/* @brief Maximum number of Message Buffers supported for payload size 8 for CAN0 */
#define FEATURE_CAN0_MAX_MB_NUM             (32U)
/* @brief Maximum number of Message Buffers supported for payload size 8 for CAN1 */
#define FEATURE_CAN1_MAX_MB_NUM             (16U)
/* @brief Maximum number of Message Buffers supported for payload size 8 for CAN2 */
#define FEATURE_CAN2_MAX_MB_NUM             (16U)
/* @brief Has PE clock source select (bit field CAN_CTRL1[CLKSRC]). */
#define FEATURE_CAN_HAS_PE_CLKSRC_SELECT     (1)
/* @brief Has DMA enable (bit field MCR[DMA]). */
#define FEATURE_CAN_HAS_DMA_ENABLE          (1)
/* @brief Maximum number of Message Buffers supported for payload size 8 for any of the CAN instances */
#define FEATURE_CAN_MAX_MB_NUM              (32U)
/* @brief Has Pretending Networking mode */
#define FEATURE_CAN_HAS_PRETENDED_NETWORKING    (1)
/* @brief Has Stuff Bit Count Enable Bit */
#define FEATURE_CAN_HAS_STFCNTEN_ENABLE     (0)
/* @brief Has ISO CAN FD Enable Bit */
#define FEATURE_CAN_HAS_ISOCANFDEN_ENABLE   (1)
/* @brief Has Message Buffer Data Size Region 1 */
#define FEATURE_CAN_HAS_MBDSR1              (0)
/* @brief Maximum number of Message Buffers IRQs */
#define FEATURE_CAN_MB_IRQS_MAX_COUNT       (2U)
/* @brief Message Buffers IRQs */
#define FEATURE_CAN_MB_IRQS                 { { CAN0_ORed_0_15_MB_IRQn, CAN0_ORed_16_31_MB_IRQn }, \
                                              { CAN1_ORed_0_15_MB_IRQn, NotAvail_IRQn }, \
                                              { CAN2_ORed_0_15_MB_IRQn, NotAvail_IRQn } }

/* LPUART module features */

/* @brief Has receive FIFO overflow detection (bit field CFIFO[RXOFE]). */
#define FEATURE_LPUART_HAS_IRQ_EXTENDED_FUNCTIONS (0)
/* @brief Has low power features (can be enabled in wait mode via register bit C1[DOZEEN] or CTRL[DOZEEN] if the registers are 32-bit wide). */
#define FEATURE_LPUART_HAS_LOW_POWER_UART_SUPPORT (1)
/* @brief Has extended data register ED (or extra flags in the DATA register if the registers are 32-bit wide). */
#define FEATURE_LPUART_HAS_EXTENDED_DATA_REGISTER_FLAGS (1)
/* @brief Capacity (number of entries) of the transmit/receive FIFO (or zero if no FIFO is available). */
#define FEATURE_LPUART_HAS_FIFO (0)
/* @brief Has 32-bit register MODIR */
#define FEATURE_LPUART_HAS_MODIR (1)
/* @brief Hardware flow control (RTS, CTS) is supported. */
#define FEATURE_LPUART_HAS_MODEM_SUPPORT (1)
/* @brief Infrared (modulation) is supported. */
#define FEATURE_LPUART_HAS_IR_SUPPORT (1)
/* @brief 2 bits long stop bit is available. */
#define FEATURE_LPUART_HAS_STOP_BIT_CONFIG_SUPPORT (1)
/* @brief Maximal data width without parity bit. */
#define FEATURE_LPUART_HAS_10BIT_DATA_SUPPORT (1)
/* @brief Baud rate oversampling is available (has bit fields C4[OSR], C5[BOTHEDGE], C5[RESYNCDIS] or BAUD[OSR], BAUD[BOTHEDGE], BAUD[RESYNCDIS] if the registers are 32-bit wide). */
#define FEATURE_LPUART_HAS_BAUD_RATE_OVER_SAMPLING_SUPPORT (1)
/* @brief Baud rate oversampling is available. */
#define FEATURE_LPUART_HAS_RX_RESYNC_SUPPORT (1)
/* @brief Baud rate oversampling is available. */
#define FEATURE_LPUART_HAS_BOTH_EDGE_SAMPLING_SUPPORT (1)
/* @brief Peripheral type. */
#define FEATURE_LPUART_IS_SCI (1)
/* @brief Capacity (number of entries) of the transmit/receive FIFO (or zero if no FIFO is available). */
#define FEATURE_LPUART_FIFO_SIZE (4U)
/* @brief Supports two match addresses to filter incoming frames. */
#define FEATURE_LPUART_HAS_ADDRESS_MATCHING (1)
/* @brief Has transmitter/receiver DMA enable bits C5[TDMAE]/C5[RDMAE] (or BAUD[TDMAE]/BAUD[RDMAE] if the registers are 32-bit wide). */
#define FEATURE_LPUART_HAS_DMA_ENABLE (1)
/* @brief Has transmitter/receiver DMA select bits C4[TDMAS]/C4[RDMAS], resp. C5[TDMAS]/C5[RDMAS] if IS_SCI = 0. */
#define FEATURE_LPUART_HAS_DMA_SELECT (0)
/* @brief Data character bit order selection is supported (bit field S2[MSBF] or STAT[MSBF] if the registers are 32-bit wide). */
#define FEATURE_LPUART_HAS_BIT_ORDER_SELECT (1)
/* @brief Has smart card (ISO7816 protocol) support and no improved smart card support. */
#define FEATURE_LPUART_HAS_SMART_CARD_SUPPORT (0)
/* @brief Has improved smart card (ISO7816 protocol) support. */
#define FEATURE_LPUART_HAS_IMPROVED_SMART_CARD_SUPPORT (0)
/* @brief Has local operation network (CEA709.1-B protocol) support. */
#define FEATURE_LPUART_HAS_LOCAL_OPERATION_NETWORK_SUPPORT (0)
/* @brief Has 32-bit registers (BAUD, STAT, CTRL, DATA, MATCH, MODIR) instead of 8-bit (BDH, BDL, C1, S1, D, etc.). */
#define FEATURE_LPUART_HAS_32BIT_REGISTERS (1)
/* @brief Lin break detect available (has bit BDH[LBKDIE]). */
#define FEATURE_LPUART_HAS_LIN_BREAK_DETECT (0)
/* @brief UART stops in Wait mode available (has bit C1[UARTSWAI]). */
#define FEATURE_LPUART_HAS_WAIT_MODE_OPERATION (0)
/* @brief Flag clearance mask for STAT register. */
#define FEATURE_LPUART_STAT_REG_FLAGS_MASK (0xC01FC000U)
/* @brief Flag clearance mask for FIFO register. */
#define FEATURE_LPUART_FIFO_REG_FLAGS_MASK (0x00030000U)
/* @brief Default oversampling ratio. */
#define FEATURE_LPUART_DEFAULT_OSR (0x0FUL)
/* @brief Default baud rate modulo divisor. */
#define FEATURE_LPUART_DEFAULT_SBR (0x04UL)

/* FLEXIO module features */

/* @brief Define the maximum number of shifters for any FlexIO instance. */
#define FEATURE_FLEXIO_MAX_SHIFTER_COUNT  (4U)

/* LPSPI module features */

/* @brief DMA instance used for LPSPI module */
#define LPSPI_DMA_INSTANCE 0U

/* @brief DMA instance used for LPI2C module */
#define LPI2C_DMA_INSTANCE 0U

/* PDB module features */

/* @brief Define the count of supporting ADC channels per each PDB. */
#define FEATURE_PDB_ADC_CHANNEL_COUNT     (2U)
/* @brief Define the count of supporting ADC pre-trigger for each channel. */
#define FEATURE_PDB_ADC_PRE_CHANNEL_COUNT (8U)
/* @brief Define the count of supporting Pulse-Out outputs per each PDB. */
#define FEATURE_PDB_PODLY_COUNT           (1U)

/* Interrupt module features */

/* @brief Lowest interrupt request number. */
#define FEATURE_INTERRUPT_IRQ_MIN         (NonMaskableInt_IRQn)
/* @brief Highest interrupt request number. */
#define FEATURE_INTERRUPT_IRQ_MAX         (FTM3_Ovf_Reload_IRQn)
/**< Number of priority bits implemented in the NVIC */
#define FEATURE_NVIC_PRIO_BITS            (4U)


/* System Control Block module features */

/* @brief VECTKEY value so that AIRCR register write is not ignored. */
#define FEATURE_SCB_VECTKEY               (0x05FAU)


/* SMC module features */

/* @brief Has stop option (register bit STOPCTRL[STOPO]). */
#define FEATURE_SMC_HAS_STOPO (1)
/* @brief Has partial stop option (register bit STOPCTRL[PSTOPO]). */
#define FEATURE_SMC_HAS_PSTOPO (0)
/* @brief Has WAIT and VLPW options. */
#define FEATURE_SMC_HAS_WAIT_VLPW (0)
/* @brief Has high speed run mode (register bit PMPROT[AHSRUN]). */
#define FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE (1)


/* MPU module features */

/* @brief Has process identifier support. */
#define FEATURE_MPU_HAS_PROCESS_IDENTIFIER              (1U)
/* @brief Specifies total number of bus masters. */
#define FEATURE_MPU_MASTER_COUNT                        (3U)
/* @brief Specifies maximum number of masters which have separated
privilege rights for user and supervisor mode accesses (e.g. master0~3 in S32K144).
*/
#define FEATURE_MPU_MAX_LOW_MASTER_NUMBER               (3U)
/* @brief Specifies maximum number of masters which have only
read and write permissions (e.g. master4~7 in S32K144).
*/
#define FEATURE_MPU_MAX_HIGH_MASTER_NUMBER              (7U)

/* @brief Specifies number of set access control right bits for
   masters which have separated privilege rights for user and
   supervisor mode accesses (e.g. master0~3 in S32K144).
*/
#define FEATURE_MPU_LOW_MASTER_CONTROL_WIDTH            (6U)
/* @brief Specifies number of set access control right bits for
   masters which have only read and write permissions(e.g. master4~7 in S32K144).
*/
#define FEATURE_MPU_HIGH_MASTER_CONTROL_WIDTH           (2U)

/* @brief The MPU Logical Bus Master Number for core bus master. */
#define FEATURE_MPU_MASTER_CORE                         (0U)
/* @brief The MPU Logical Bus Master Number for Debugger master. */
#define FEATURE_MPU_MASTER_DEBUGGER                     (1U)
/* @brief The MPU Logical Bus Master Number for DMA master. */
#define FEATURE_MPU_MASTER_DMA                          (2U)

/* @brief Specifies total number of slave ports. */
#define FEATURE_MPU_SLAVE_COUNT                         (4U)
/* @brief The MPU Slave Port Assignment for Flash Controller and boot ROM. */
#define FEATURE_MPU_SLAVE_FLASH_BOOTROM                 (0U)
/* @brief The MPU Slave Port Assignment for SRAM back door. */
#define FEATURE_MPU_SLAVE_SRAM_BACKDOOR                 (1U)
/* @brief The MPU Slave Port Assignment for SRAM_L front door. */
#define FEATURE_MPU_SLAVE_SRAM_L_FRONTDOOR              (2U)
/* @brief The MPU Slave Port Assignment for SRAM_U front door. */
#define FEATURE_MPU_SLAVE_SRAM_U_FRONTDOOR              (3U)
/* @brief The MPU Slave Port mask. */
#define FEATURE_MPU_SLAVE_MASK                          (0xF0000000U)

/* WDOG module features */

/* @brief The 32-bit value used for unlocking the WDOG. */
#define FEATURE_WDOG_UNLOCK_VALUE       (0xD928C520U)
/* @brief The 32-bit value used for resetting the WDOG counter. */
#define FEATURE_WDOG_TRIGGER_VALUE      (0xB480A602U)
/* @brief The reset value of the timeout register. */
#define FEATURE_WDOG_TO_RESET_VALUE     (0x400U)
/* @brief The reset value of the window register. */
#define FEATURE_WDOG_WIN_RESET_VALUE    (0x0U)
/* @brief The mask of the reserved bit in the CS register. */
#define FEATURE_WDOG_CS_RESERVED_MASK   (0x2000U)
/* @brief The value used to set WDOG source clock from LPO. */
#define FEATURE_WDOG_CLK_FROM_LPO       (0x1UL)
/* @brief The first 16-bit value used for unlocking the WDOG. */
#define FEATURE_WDOG_UNLOCK16_FIRST_VALUE     (0xC520U)
/* @brief The second 16-bit value used for unlocking the WDOG. */
#define FEATURE_WDOG_UNLOCK16_SECOND_VALUE    (0xD928U)
/* @brief The first 16-bit value used for resetting the WDOG counter. */
#define FEATURE_WDOG_TRIGGER16_FIRST_VALUE    (0xA602U)
/* @brief The second 16-bit value used for resetting the WDOG counter. */
#define FEATURE_WDOG_TRIGGER16_SECOND_VALUE   (0xB480U)


/* EDMA module features */

/* @brief Number of EDMA channels. */
#define FEATURE_EDMA_MODULE_CHANNELS (16U)
/* @brief Number of EDMA channel interrupt lines. */
#define FEATURE_CHANNEL_INTERRUPT_LINES (16U)
/* @brief Number of EDMA error interrupt lines. */
#define FEATURE_ERROR_INTERRUPT_LINES (1U)
/* @brief eDMA module has error interrupt. */
#define FEATURE_EDMA_HAS_ERROR_IRQ
/* @brief eDMA module has separate interrupt lines for each channel. */
#define FEATURE_EDMA_SEPARATE_IRQ_LINES_PER_CHN
/* @brief Conversion from channel index to DCHPRI index. */
#define FEATURE_EDMA_CHN_TO_DCHPRI_INDEX(x) ((x) ^ 3U)
/* @brief eDMA channel groups count. */
#define FEATURE_EDMA_CHANNEL_GROUP_COUNT (1U)
/* @brief Number of eDMA channels with asynchronous request capability. */
#define FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT (16U)
/* @brief Clock names for eDMA. */
#define EDMA_CLOCK_NAMES {SIM_DMA_CLOCK}

/* DMAMUX module features */

/* @brief Number of DMA channels. */
#define FEATURE_DMAMUX_MODULE_CHANNELS (16U)
/* @brief Has the periodic trigger capability */
#define FEATURE_DMAMUX_HAS_TRIG (1)
/* @brief Conversion from request source to the actual DMAMUX channel */
#define FEATURE_DMAMUX_REQ_SRC_TO_CHN(x) (x)
/* @brief Mapping between request source and DMAMUX instance */
#define FEATURE_DMAMUX_REQ_SRC_TO_INSTANCE(x) (0U)
/* @brief Conversion from eDMA channel index to DMAMUX channel. */
#define FEATURE_DMAMUX_CHN_FOR_EDMA_CHN(x) (x)
/* @brief Conversion from DMAMUX channel DMAMUX register index. */
#define FEATURE_DMAMUX_CHN_REG_INDEX(x) (x)
/* @brief Clock names for DMAMUX. */
#define DMAMUX_CLOCK_NAMES {PCC_DMAMUX0_CLOCK}
/*!
 * @brief Structure for the DMA hardware request
 *
 * Defines the structure for the DMA hardware request collections. The user can configure the
 * hardware request into DMAMUX to trigger the DMA transfer accordingly. The index
 * of the hardware request varies according  to the to SoC.
 */

typedef enum {
    EDMA_REQ_DISABLED = 0U,
    EDMA_REQ_LPUART0_RX = 2U,
    EDMA_REQ_LPUART0_TX = 3U,
    EDMA_REQ_LPUART1_RX = 4U,
    EDMA_REQ_LPUART1_TX = 5U,
    EDMA_REQ_LPUART2_RX = 6U,
    EDMA_REQ_LPUART2_TX = 7U,
    EDMA_REQ_FLEXIO_SHIFTER0 = 10U,
    EDMA_REQ_FLEXIO_SHIFTER1 = 11U,
    EDMA_REQ_FLEXIO_SHIFTER2 = 12U,
    EDMA_REQ_FLEXIO_SHIFTER3 = 13U,
    EDMA_REQ_LPSPI0_RX = 14U,
    EDMA_REQ_LPSPI0_TX = 15U,
    EDMA_REQ_LPSPI1_RX = 16U,
    EDMA_REQ_LPSPI1_TX = 17U,
    EDMA_REQ_LPSPI2_RX = 18U,
    EDMA_REQ_LPSPI2_TX = 19U,
    EDMA_REQ_FTM1_CHANNEL_0 = 20U,
    EDMA_REQ_FTM1_CHANNEL_1 = 21U,
    EDMA_REQ_FTM1_CHANNEL_2 = 22U,
    EDMA_REQ_FTM1_CHANNEL_3 = 23U,
    EDMA_REQ_FTM1_CHANNEL_4 = 24U,
    EDMA_REQ_FTM1_CHANNEL_5 = 25U,
    EDMA_REQ_FTM1_CHANNEL_6 = 26U,
    EDMA_REQ_FTM1_CHANNEL_7 = 27U,
    EDMA_REQ_FTM2_CHANNEL_0 = 28U,
    EDMA_REQ_FTM2_CHANNEL_1 = 29U,
    EDMA_REQ_FTM2_CHANNEL_2 = 30U,
    EDMA_REQ_FTM2_CHANNEL_3 = 31U,
    EDMA_REQ_FTM2_CHANNEL_4 = 32U,
    EDMA_REQ_FTM2_CHANNEL_5 = 33U,
    EDMA_REQ_FTM2_CHANNEL_6 = 34U,
    EDMA_REQ_FTM2_CHANNEL_7 = 35U,
    EDMA_REQ_FTM0_OR_CH0_CH7 = 36U,
    EDMA_REQ_FTM3_OR_CH0_CH7 = 37U,
    EDMA_REQ_ADC0 = 42U,
    EDMA_REQ_ADC1 = 43U,
    EDMA_REQ_LPI2C0_RX = 44U,
    EDMA_REQ_LPI2C0_TX = 45U,
    EDMA_REQ_PDB0 = 46U,
    EDMA_REQ_PDB1 = 47U,
    EDMA_REQ_CMP0 = 48U,
    EDMA_REQ_PORTA = 49U,
    EDMA_REQ_PORTB = 50U,
    EDMA_REQ_PORTC = 51U,
    EDMA_REQ_PORTD = 52U,
    EDMA_REQ_PORTE = 53U,
    EDMA_REQ_FLEXCAN0 = 54U,
    EDMA_REQ_FLEXCAN1 = 55U,
    EDMA_REQ_FLEXCAN2 = 56U,
    EDMA_REQ_LPTMR0 = 59U,
    EDMA_REQ_DMAMUX_ALWAYS_ENABLED0 = 62U,
    EDMA_REQ_DMAMUX_ALWAYS_ENABLED1 = 63U
} dma_request_source_t;

/* LPI2C module features */

/* @brief Disable high-speed and ultra-fast operating modes for S32K144. */
#define LPI2C_HAS_FAST_PLUS_MODE (0U)
#define LPI2C_HAS_HIGH_SPEED_MODE (0U)
#define LPI2C_HAS_ULTRA_FAST_MODE (0U)

/* FTM module features */
/* @brief Number of PWM channels */
#define FEATURE_FTM_CHANNEL_COUNT           (8U)
/* @brief Number of fault channels */
#define FTM_FEATURE_FAULT_CHANNELS              (4U)
/* @brief Width of control channel */
#define FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH     (8U)
/* @brief Output channel offset */
#define FTM_FEATURE_OUTPUT_CHANNEL_OFFSET       (16U)
/* @brief Max counter value */
#define FTM_FEATURE_CNT_MAX_VALUE_U32           (0x0000FFFFU)
/* @brief Input capture for single shot */
#define FTM_FEATURE_INPUT_CAPTURE_SINGLE_SHOT   (2U)

/* EWM module features */

/* @brief First byte of the EWM Service key        */
#define FEATURE_EWM_KEY_FIRST_BYTE      (0xB4U)
/* @brief Second byte of the EWM Service key       */
#define FEATURE_EWM_KEY_SECOND_BYTE     (0x2CU)
/* @brief EWM Compare High register maximum value  */
#define FEATURE_EWM_CMPH_MAX_VALUE      (0xFEU)
/* @brief EWM Compare Low register minimum value  */
#define FEATURE_EWM_CMPL_MIN_VALUE      (0x00U)

/* CLOCK names */

/*! @brief Clock names. */
typedef enum {

    /* Main clocks */
    CORE_CLOCK                   = 0u,       /*!< Core clock                     */
    BUS_CLOCK                    = 1u,       /*!< Bus clock                      */
    SLOW_CLOCK                   = 2u,       /*!< Slow clock                     */
    CLKOUT_CLOCK                 = 3u,       /*!< CLKOUT clock                   */

    /* Other internal clocks used by peripherals. */
    SIRC_CLOCK                   = 4u,       /*!< SIRC clock                     */
    FIRC_CLOCK                   = 5u,       /*!< FIRC clock                     */
    SOSC_CLOCK                   = 6u,       /*!< SOSC clock                     */
    SPLL_CLOCK                   = 7u,       /*!< SPLL clock                     */
    RTC_CLKIN_CLOCK              = 8u,       /*!< RTC_CLKIN clock                */
    SCG_CLKOUT_CLOCK             = 9u,       /*!< SCG CLK_OUT clock              */
    SCG_END_OF_CLOCKS            = 10u,      /*!< End of SCG clocks              */

    /* SIM clocks */
    SIM_FTM0_CLOCKSEL            = 21u,      /*!< FTM0 External Clock Pin Select */
    SIM_FTM1_CLOCKSEL            = 22u,      /*!< FTM1 External Clock Pin Select */
    SIM_FTM2_CLOCKSEL            = 23u,      /*!< FTM2 External Clock Pin Select */
    SIM_FTM3_CLOCKSEL            = 24u,      /*!< FTM3 External Clock Pin Select */
    SIM_CLKOUTSELL               = 25u,      /*!< CLKOUT Select                  */
    SIM_RTCCLK_CLOCK             = 26u,      /*!< RTCCLK clock                   */
    SIM_LPO_CLOCK                = 27u,      /*!< LPO clock                      */
    SIM_LPO_1K_CLOCK             = 28u,      /*!< LPO 1KHz clock                 */
    SIM_LPO_32K_CLOCK            = 29u,      /*!< LPO 32KHz clock                */
    SIM_LPO_128K_CLOCK           = 30u,      /*!< LPO 128KHz clock               */
    SIM_EIM_CLOCK                = 31u,      /*!< EIM clock source               */
    SIM_ERM_CLOCK                = 32u,      /*!< ERM clock source               */
    SIM_DMA_CLOCK                = 33u,      /*!< DMA clock source               */
    SIM_MPU_CLOCK                = 34u,      /*!< MPU clock source               */
    SIM_MSCM_CLOCK               = 35u,      /*!< MSCM clock source              */
    SIM_END_OF_CLOCKS            = 36u,      /*!< End of SIM clocks              */

    /* PCC clocks */
    PCC_DMAMUX0_CLOCK            = 41u,      /*!< DMAMUX0 clock source           */
    PCC_CRC0_CLOCK               = 42u,      /*!< CRC0 clock source              */
    PCC_RTC0_CLOCK               = 43u,      /*!< RTC0 clock source              */
    PCC_PORTA_CLOCK              = 44u,      /*!< PORTA clock source             */
    PCC_PORTB_CLOCK              = 45u,      /*!< PORTB clock source             */
    PCC_PORTC_CLOCK              = 46u,      /*!< PORTC clock source             */
    PCC_PORTD_CLOCK              = 47u,      /*!< PORTD clock source             */
    PCC_PORTE_CLOCK              = 48u,      /*!< PORTE clock source             */
    PCC_EWM0_CLOCK               = 49u,      /*!< EWM0 clock source              */
    PCC_CMP0_CLOCK               = 50u,      /*!< CMP0 clock source              */
    PCC_END_OF_BUS_CLOCKS        = 51u,      /*!< End of BUS clocks              */
    PCC_FlexCAN0_CLOCK           = 52u,      /*!< FlexCAN0 clock source          */
    PCC_FlexCAN1_CLOCK           = 53u,      /*!< FlexCAN1 clock source          */
    PCC_FlexCAN2_CLOCK           = 54u,      /*!< FlexCAN2 clock source          */
    PCC_PDB1_CLOCK               = 55u,      /*!< PDB1 clock source              */
    PCC_PDB0_CLOCK               = 56u,      /*!< PDB0 clock source              */
    PCC_END_OF_SYS_CLOCKS        = 57u,      /*!< End of SYS clocks              */
    PCC_FTFC0_CLOCK              = 58u,      /*!< FTFC0 clock source             */
    PCC_END_OF_SLOW_CLOCKS       = 59u,      /*!< End of SLOW clocks             */
    PCC_FTM3_CLOCK               = 60u,      /*!< FTM3 clock source              */
    PCC_FTM0_CLOCK               = 61u,      /*!< FTM0 clock source              */
    PCC_FTM1_CLOCK               = 62u,      /*!< FTM1 clock source              */
    PCC_FTM2_CLOCK               = 63u,      /*!< FTM2 clock source              */
    PCC_END_OF_ASYNCH_DIV1_CLOCKS= 64u,      /*!< End of ASYNCH DIV1 clocks      */
    PCC_ADC1_CLOCK               = 65u,      /*!< ADC1 clock source              */
    PCC_LPSPI0_CLOCK             = 66u,      /*!< LPSPI0 clock source            */
    PCC_LPSPI1_CLOCK             = 67u,      /*!< LPSPI1 clock source            */
    PCC_LPSPI2_CLOCK             = 68u,      /*!< LPSPI2 clock source            */
    PCC_LPIT0_CLOCK              = 69u,      /*!< LPIT0 clock source             */
    PCC_ADC0_CLOCK               = 70u,      /*!< ADC0 clock source              */
    PCC_LPTMR0_CLOCK             = 71u,      /*!< LPTMR0 clock source            */
    PCC_FLEXIO0_CLOCK            = 72u,      /*!< FLEXIO0 clock source           */
    PCC_LPI2C0_CLOCK             = 73u,      /*!< LPI2C0 clock source            */
    PCC_LPUART0_CLOCK            = 74u,      /*!< LPUART0 clock source           */
    PCC_LPUART1_CLOCK            = 75u,      /*!< LPUART1 clock source           */
    PCC_LPUART2_CLOCK            = 76u,      /*!< LPUART2 clock source           */
    PCC_END_OF_ASYNCH_DIV2_CLOCKS= 77u,      /*!< End of ASYNCH DIV2 clocks      */
    PCC_END_OF_CLOCKS            = 78u,      /*!< End of PCC clocks              */
    CLOCK_NAME_COUNT             = 79u,      /*!< The total number of entries    */

} clock_names_t;

#define PCC_INVALID_INDEX  0

  /*! @brief PCC clock name mappings
   *  Mappings between clock names and peripheral clock control indexes.
   *  If there is no peripheral clock control index for a clock name,
   *  then the corresponding value is PCC_INVALID_INDEX.
   */
#define PCC_CLOCK_NAME_MAPPINGS \
{                                                                                \
PCC_INVALID_INDEX,                  /*!< Core clock                      0  */   \
PCC_INVALID_INDEX,                  /*!< Bus clock                       1  */   \
PCC_INVALID_INDEX,                  /*!< Slow clock                      2  */   \
PCC_INVALID_INDEX,                  /*!< CLKOUT clock                    3  */   \
PCC_INVALID_INDEX,                  /*!< SIRC clock                      4  */   \
PCC_INVALID_INDEX,                  /*!< FIRC clock                      5  */   \
PCC_INVALID_INDEX,                  /*!< SOSC clock                      6  */   \
PCC_INVALID_INDEX,                  /*!< SPLL clock                      7  */   \
PCC_INVALID_INDEX,                  /*!< RTC_CLKIN clock                 8  */   \
PCC_INVALID_INDEX,                  /*!< SCG CLK_OUT clock               9  */   \
PCC_INVALID_INDEX,                  /*!< End of SCG clocks               10 */   \
PCC_INVALID_INDEX,                  /*!< No clock entry in clock_names_t 11 */   \
PCC_INVALID_INDEX,                  /*!< No clock entry in clock_names_t 12 */   \
PCC_INVALID_INDEX,                  /*!< No clock entry in clock_names_t 13 */   \
PCC_INVALID_INDEX,                  /*!< No clock entry in clock_names_t 14 */   \
PCC_INVALID_INDEX,                  /*!< No clock entry in clock_names_t 15 */   \
PCC_INVALID_INDEX,                  /*!< No clock entry in clock_names_t 16 */   \
PCC_INVALID_INDEX,                  /*!< No clock entry in clock_names_t 17 */   \
PCC_INVALID_INDEX,                  /*!< No clock entry in clock_names_t 18 */   \
PCC_INVALID_INDEX,                  /*!< No clock entry in clock_names_t 19 */   \
PCC_INVALID_INDEX,                  /*!< No clock entry in clock_names_t 20 */   \
PCC_INVALID_INDEX,                  /*!< FTM0 External Clock Pin Select  21 */   \
PCC_INVALID_INDEX,                  /*!< FTM1 External Clock Pin Select  22 */   \
PCC_INVALID_INDEX,                  /*!< FTM2 External Clock Pin Select  23 */   \
PCC_INVALID_INDEX,                  /*!< FTM3 External Clock Pin Select  24 */   \
PCC_INVALID_INDEX,                  /*!< CLKOUT Select                   25 */   \
PCC_INVALID_INDEX,                  /*!< CLK32K clock                    26 */   \
PCC_INVALID_INDEX,                  /*!< LPO clock                       27 */   \
PCC_INVALID_INDEX,                  /*!< LPO 1KHz clock                  28 */   \
PCC_INVALID_INDEX,                  /*!< LPO 32KHz clock                 29 */   \
PCC_INVALID_INDEX,                  /*!< LPO 128KHz clock                30 */   \
PCC_INVALID_INDEX,                  /*!< EIM clock source                31 */   \
PCC_INVALID_INDEX,                  /*!< ERM clock source                32 */   \
PCC_INVALID_INDEX,                  /*!< DMA clock source                33 */   \
PCC_INVALID_INDEX,                  /*!< MPU clock source                34 */   \
PCC_INVALID_INDEX,                  /*!< MSCM clock source               35 */   \
PCC_INVALID_INDEX,                  /*!< No clock entry in clock_names_t 36 */   \
PCC_INVALID_INDEX,                  /*!< No clock entry in clock_names_t 37 */   \
PCC_INVALID_INDEX,                  /*!< No clock entry in clock_names_t 38 */   \
PCC_INVALID_INDEX,                  /*!< No clock entry in clock_names_t 39 */   \
PCC_INVALID_INDEX,                  /*!< No clock entry in clock_names_t 40 */   \
PCC_DMAMUX_INDEX,                   /*!< DMAMUX clock source             41 */   \
PCC_CRC_INDEX,                      /*!< CRC clock source                42 */   \
PCC_RTC_INDEX,                      /*!< RTC clock source                43 */   \
PCC_PORTA_INDEX,                    /*!< PORTA clock source              44 */   \
PCC_PORTB_INDEX,                    /*!< PORTB clock source              45 */   \
PCC_PORTC_INDEX,                    /*!< PORTC clock source              46 */   \
PCC_PORTD_INDEX,                    /*!< PORTD clock source              47 */   \
PCC_PORTE_INDEX,                    /*!< PORTE clock source              48 */   \
PCC_EWM_INDEX,                      /*!< EWM clock source                49 */   \
PCC_CMP0_INDEX,                     /*!< CMP0 clock source               50 */   \
PCC_INVALID_INDEX,                  /*!< End of BUS clocks               51 */   \
PCC_FlexCAN0_INDEX,                 /*!< FlexCAN0 clock source           52 */   \
PCC_FlexCAN1_INDEX,                 /*!< FlexCAN1 clock source           53 */   \
PCC_FlexCAN2_INDEX,                 /*!< FlexCAN2 clock source           54 */   \
PCC_PDB1_INDEX,                     /*!< PDB1 clock source               55 */   \
PCC_PDB0_INDEX,                     /*!< PDB0 clock source               56 */   \
PCC_INVALID_INDEX,                  /*!< End of SYS clocks               57 */   \
PCC_FTFC_INDEX,                     /*!< FTFC clock source               58 */   \
PCC_INVALID_INDEX,                  /*!< End of SLOW clocks              59 */   \
PCC_FTM3_INDEX,                     /*!< FTM3 clock source               60 */   \
PCC_FTM0_INDEX,                     /*!< FTM0 clock source               61 */   \
PCC_FTM1_INDEX,                     /*!< FTM1 clock source               62 */   \
PCC_FTM2_INDEX,                     /*!< FTM2 clock source               63 */   \
PCC_INVALID_INDEX,                  /*!< End of ASYNCH DIV1 clocks       64 */   \
PCC_ADC1_INDEX,                     /*!< ADC1 clock source               65 */   \
PCC_LPSPI0_INDEX,                   /*!< LPSPI0 clock source             66 */   \
PCC_LPSPI1_INDEX,                   /*!< LPSPI1 clock source             67 */   \
PCC_LPSPI2_INDEX,                   /*!< LPSPI2 clock source             68 */   \
PCC_LPIT_INDEX,                     /*!< LPIT clock source               69 */   \
PCC_ADC0_INDEX,                     /*!< ADC0 clock source               70 */   \
PCC_LPTMR0_INDEX,                   /*!< LPTMR0 clock source             71 */   \
PCC_FLEXIO_INDEX,                   /*!< FLEXIO clock source             72 */   \
PCC_LPI2C0_INDEX,                   /*!< LPI2C0 clock source             73 */   \
PCC_LPUART0_INDEX,                  /*!< LPUART0 clock source            74 */   \
PCC_LPUART1_INDEX,                  /*!< LPUART1 clock source            75 */   \
PCC_LPUART2_INDEX,                  /*!< LPUART2 clock source            76 */   \
PCC_INVALID_INDEX,                  /*!< End of ASYNCH DIV2 clocks       77 */   \
PCC_INVALID_INDEX,                  /*!< End of PCC clocks               78 */   \
}

/* Time to wait for SIRC to stabilize (number of
 * cycles when core runs at maximum speed - 112 MHz */
#define SIRC_STABILIZATION_TIMEOUT 26U;

/* Time to wait for FIRC to stabilize (number of
 * cycles when core runs at maximum speed - 112 MHz */
#define FIRC_STABILIZATION_TIMEOUT 10U;

/* Time to wait for SOSC to stabilize (number of
 * cycles when core runs at maximum speed - 112 MHz */
#define SOSC_STABILIZATION_TIMEOUT 3205000U;

/* Time to wait for SPLL to stabilize (number of
 * cycles when core runs at maximum speed - 112 MHz */
#define SPLL_STABILIZATION_TIMEOUT 1000U;

/*! @brief Maximum frequencies of core, bus and flash clocks.
 *  Each entry represents the maximum frequency of
 *  SYS_CLK (CORE_CLK), BUS_CLK, SLOW_CLK (FLASH_CLK)
 *  in VLPR, RUN and HSRUN modes.
 *
 *          SYS_CLK  BUS_CLK  SLOW_CLK
 *  VLPR       *        *         *
 *  RUN        *        *         *
 *  HSRUN      *        *         *
 */
#define MAX_FREQ_VLPR     0U
#define MAX_FREQ_RUN      1U
#define MAX_FREQ_HSRUN    2U

#define MAX_FREQ_SYS_CLK  0U
#define MAX_FREQ_BUS_CLK  1U
#define MAX_FREQ_SLOW_CLK 2U

#define MAX_FREQ_MODES_NO 3U
#define MAX_FREQ_CLK_NO   3U

#define CLOCK_MAX_FREQUENCIES                                                                      \
{/* SYS_CLK  BUS_CLK SLOW_CLK */                                                                   \
{  4000000, 4000000, 1000000},               /*!< MAX FREQ in low power modes */                   \
{ 80000000,40000000,26670000},               /*!< MAX FREQ in normal power modes */                \
{112000000,56000000,28000000},               /*!< MAX FREQ in high speed modes */                  \
}


/*! @brief Temporary system clock source configurations.
 *         Each line represents the SYS(CORE), BUS and SLOW(FLASH) dividers
 *         for SIRC, FIRC, SOSC and SPLL clock sources.
  *
 *          SYS_CLK  BUS_CLK  SLOW_CLK
 *  SIRC       *        *         *
 *  FIRC       *        *         *
 *  SOSC       *        *         *
 *  SPLL       *        *         *
 */
#define TMP_SIRC_CLK   0U
#define TMP_FIRC_CLK   1U
#define TMP_SOSC_CLK   2U
#define TMP_SPLL_CLK   3U

#define TMP_SYS_DIV    0U
#define TMP_BUS_DIV    1U
#define TMP_SLOW_DIV   2U

#define TMP_SYS_CLK_NO 4U
#define TMP_SYS_DIV_NO 3U

#define TMP_SYSTEM_CLOCK_CONFIGS                                                                                \
{  /*       SYS_CLK                    BUS_CLK                  SLOW_CLK      */                                \
{  SCG_SYSTEM_CLOCK_DIV_BY_1, SCG_SYSTEM_CLOCK_DIV_BY_1, SCG_SYSTEM_CLOCK_DIV_BY_1},  /*!< Dividers for SIRC */ \
{  SCG_SYSTEM_CLOCK_DIV_BY_1, SCG_SYSTEM_CLOCK_DIV_BY_2, SCG_SYSTEM_CLOCK_DIV_BY_4},  /*!< Dividers for FIRC */ \
{  SCG_SYSTEM_CLOCK_DIV_BY_1, SCG_SYSTEM_CLOCK_DIV_BY_2, SCG_SYSTEM_CLOCK_DIV_BY_2},  /*!< Dividers for SOSC */ \
{  SCG_SYSTEM_CLOCK_DIV_BY_3, SCG_SYSTEM_CLOCK_DIV_BY_2, SCG_SYSTEM_CLOCK_DIV_BY_2},  /*!< Dividers for SPLL */ \
}

/* Do not use the old names of the renamed symbols */
/* #define DO_NOT_USE_DEPRECATED_SYMBOLS */

/*! START !DO_NOT_USE_DEPRECATED_SYMBOLS
 *  These symbols have been renamed.
 *  The old names (deprecated symbols)
 *  are defined for backward compatibility.
 */
#if !defined(DO_NOT_USE_DEPRECATED_SYMBOLS)
#define PCC_FLASH0_CLOCK      PCC_FTFE0_CLOCK
#define PCC_DMA_CH_MUX0_CLOCK PCC_DMAMUX0_CLOCK
#define PCC_SRTC0_CLOCK       PCC_RTC0_CLOCK
#define PCC_RGPIO0_CLOCK      PCC_PORTA_CLOCK
#define PCC_RGPIO1_CLOCK      PCC_PORTB_CLOCK
#define PCC_RGPIO2_CLOCK      PCC_PORTC_CLOCK
#define PCC_RGPIO3_CLOCK      PCC_PORTD_CLOCK
#define PCC_RGPIO4_CLOCK      PCC_PORTE_CLOCK
#define PCC_CAN0_CLOCK        PCC_FLEXCAN0_CLOCK
#define PCC_CAN1_CLOCK        PCC_FLEXCAN1_CLOCK
#define PCC_CAN2_CLOCK        PCC_FLEXCAN2_CLOCK
#define PCC_FLEXTMR3_CLOCK    PCC_FTM3_CLOCK
#define PCC_FLEXTMR0_CLOCK    PCC_FTM0_CLOCK
#define PCC_FLEXTMR1_CLOCK    PCC_FTM1_CLOCK
#define PCC_FLEXTMR2_CLOCK    PCC_FTM2_CLOCK
#define PCC_PIT0_CLOCK        PCC_LPIT0_CLOCK
#define PCC_LPTIMER0_CLOCK    PCC_LPTMR0_CLOCK
#define PCC_LPIIC0_CLOCK      PCC_LPI2C0_CLOCK
#define PCC_FTFE0_CLOCK       PCC_FTFC0_CLOCK
#endif /* !DO_NOT_USE_DEPRECATED_SYMBOLS */


/* CSEc module features */

/*! @brief CSE_PRAM offset of the page length parameter used by the following
commands: CMD_ENC_ECB, CMD_ENC_CBC, CMD_DEC_ECB, CMD_DEC_CBC, CMD_MP_COMPRESS */
#define FEATURE_CSEC_PAGE_LENGTH_OFFSET                   (0xEU)
/*! @brief CSE_PRAM offset of the message length parameter used by the following
commands: CMD_GENERATE_MAC, CMD_VERIFY_MAC (both copy and pointer methods) */
#define FEATURE_CSEC_MESSAGE_LENGTH_OFFSET                (0xCU)
/*! @brief CSE_PRAM offset of the MAC length parameter used by the following
commands: CMD_VERIFY_MAC (both copy and pointer methods) */
#define FEATURE_CSEC_MAC_LENGTH_OFFSET                    (0x8U)
/*! @brief CSE_PRAM offset of the boot size parameter used by the following
commands: CMD_BOOT_DEFINE */
#define FEATURE_CSEC_BOOT_SIZE_OFFSET                     (0x1CU)
/*! @brief CSE_PRAM offset of the boot flavor parameter used by the following
commands: CMD_BOOT_DEFINE */
#define FEATURE_CSEC_BOOT_FLAVOR_OFFSET                   (0x1BU)
/*! @brief CSE_PRAM offset of the Flash start address parameter used by the
following commands: CMD_GENERATE_MAC, CMD_VERIFY_MAC (pointer method) */
#define FEATURE_CSEC_FLASH_START_ADDRESS_OFFSET           (0x10U)
/*! @brief CSE_PRAM offset of the verification status parameter used by the
following commands: CMD_VERIFY_MAC (both copy and pointer methods) */
#define FEATURE_CSEC_VERIFICATION_STATUS_OFFSET           (0x14U)
/*! @brief CSE_PRAM offset of the error bits field contained by all commands */
#define FEATURE_CSEC_ERROR_BITS_OFFSET                    (0x4U)
/*! @brief CSE_PRAM offset of the SREG parameter used by the following commands:
CMD_GET_ID */
#define FEATURE_CSEC_SREG_OFFSET                          (0x2FU)

/*! @brief CSE_PRAM offset of page 0 */
#define FEATURE_CSEC_PAGE_0_OFFSET                        (0x0U)
/*! @brief CSE_PRAM offset of page 1 */
#define FEATURE_CSEC_PAGE_1_OFFSET                        (0x10U)
/*! @brief CSE_PRAM offset of page 2 */
#define FEATURE_CSEC_PAGE_2_OFFSET                        (0x20U)
/*! @brief CSE_PRAM offset of page 3 */
#define FEATURE_CSEC_PAGE_3_OFFSET                        (0x30U)
/*! @brief CSE_PRAM offset of page 4 */
#define FEATURE_CSEC_PAGE_4_OFFSET                        (0x40U)
/*! @brief CSE_PRAM offset of page 5 */
#define FEATURE_CSEC_PAGE_5_OFFSET                        (0x50U)
/*! @brief CSE_PRAM offset of page 6 */
#define FEATURE_CSEC_PAGE_6_OFFSET                        (0x60U)
/*! @brief CSE_PRAM offset of page 7 */
#define FEATURE_CSEC_PAGE_7_OFFSET                        (0x70U)


/* ADC module features */

/*! @brief ADC input channel mappings
 * Define some user-friendly names for ADC input channels
 */
#define ADC_INPUTCHAN_TEMP ADC_INPUTCHAN_AD26
#define ADC_INPUTCHAN_BANDGAP ADC_INPUTCHAN_AD27
#define ADC_INPUTCHAN_VREFSH ADC_INPUTCHAN_AD29
#define ADC_INPUTCHAN_VREFSL ADC_INPUTCHAN_AD30
#define ADC_INPUTCHAN_DISABLED ADC_INPUTCHAN_AD31

/* MSCM module features */

/* @brief Has interrupt router control registers (IRSPRCn). */
#define FEATURE_MSCM_HAS_INTERRUPT_ROUTER (0)
/* @brief Has directed CPU interrupt routerregisters (IRCPxxx). */
#define FEATURE_MSCM_HAS_CPU_INTERRUPT_ROUTER (0)


#endif /* S32K144_FEATURES_H */

/*******************************************************************************
 * EOF
 ******************************************************************************/
