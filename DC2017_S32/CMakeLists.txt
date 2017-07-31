cmake_minimum_required(VERSION 3.5)
project(DC2017_S32)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

link_directories(${CMAKE_SOURCE_DIR})

include(DC2017_S32.cmake)
#add_definitions(-DCMAKE_TOOLCHAIN_FILE=DC2017.cmake)

include_directories(Inc)
include_directories(src/SDK/platform/hal/inc)

include_directories(src/Generated_Code)
include_directories(src/SDK/platform/devices)
include_directories(src/SDK/platform/devices/common)
include_directories(src/SDK/platform/devices/S32K144/include)
include_directories(src/SDK/platform/devices/S32K144/startup)
include_directories(src/SDK/platform/drivers/inc)
include_directories(src/SDK/platform/drivers/src/clock/S32K144)
include_directories(src/SDK/platform/hal/inc)
include_directories(src/SDK/platform/hal/src/sim/S32K144)
include_directories(src/SDK/rtos/osif)

#file(GLOB_RECURSE USER_SOURCES "Src/*.c")
set(COMMON_USER_SOURCES
        src/startup_S32K144.S
        src/Generated_Code/canCom1.c
        src/Generated_Code/canCom2.c
        src/Generated_Code/canCom3.c
        src/Generated_Code/clockMan1.c
        src/Generated_Code/Cpu.c
        src/Generated_Code/crc1.c
        src/Generated_Code/csec1.c
        src/Generated_Code/dmaController1.c
        src/Generated_Code/flexTimer1.c
        src/Generated_Code/lpspiCom1.c
        src/Generated_Code/lpuart1.c
        src/Generated_Code/osif1.c
        src/Generated_Code/pin_mux.c
        src/Generated_Code/pwrMan1.c


        )

set (SDK_SOURCES
        src/SDK/platform/devices/S32K144/startup/system_S32K144.c
        src/SDK/platform/devices/startup.c
        src/SDK/platform/drivers/src/clock/clock_manager.c
        src/SDK/platform/drivers/src/clock/S32K144/clock_S32K144.c
        src/SDK/platform/drivers/src/crc/crc_driver.c
        src/SDK/platform/drivers/src/csec/csec_driver.c
        src/SDK/platform/drivers/src/edma/edma_common.c
        src/SDK/platform/drivers/src/edma/edma_driver.c
        src/SDK/platform/drivers/src/edma/edma_irq.c
        src/SDK/platform/drivers/src/flexcan/flexcan_common.c
        src/SDK/platform/drivers/src/flexcan/flexcan_driver.c
        src/SDK/platform/drivers/src/flexcan/flexcan_irq.c
        src/SDK/platform/drivers/src/ftm/ftm_driver.c
        src/SDK/platform/drivers/src/interrupt/interrupt_manager.c
        src/SDK/platform/drivers/src/lpspi/lpspi_irq.c
        src/SDK/platform/drivers/src/lpspi/lpspi_master_driver.c
        src/SDK/platform/drivers/src/lpspi/lpspi_shared_function.c
        src/SDK/platform/drivers/src/lpspi/lpspi_slave_driver.c
        src/SDK/platform/drivers/src/lpuart/lpuart_common.c
        src/SDK/platform/drivers/src/lpuart/lpuart_driver.c
        src/SDK/platform/drivers/src/lpuart/lpuart_irq.c
        src/SDK/platform/drivers/src/pins/pins_driver.c
        src/SDK/platform/drivers/src/power/power_manager_common.c
        src/SDK/platform/drivers/src/power/power_manager_smc.c
        src/SDK/platform/hal/src/crc/crc_hal.c
        src/SDK/platform/hal/src/csec/csec_hal.c
        src/SDK/platform/hal/src/dmamux/dmamux_hal.c
        src/SDK/platform/hal/src/edma/edma_hal.c
        src/SDK/platform/hal/src/flexcan/flexcan_hal.c
        src/SDK/platform/hal/src/ftm/ftm_hal.c
        src/SDK/platform/hal/src/lpspi/lpspi_hal.c
        src/SDK/platform/hal/src/lpuart/lpuart_hal.c
        src/SDK/platform/hal/src/pcc/pcc_hal.c
        src/SDK/platform/hal/src/pmc/pmc_hal.c
        src/SDK/platform/hal/src/port/port_hal.c
        src/SDK/platform/hal/src/rcm/rcm_hal.c
        src/SDK/platform/hal/src/scg/scg_hal.c
        src/SDK/platform/hal/src/sim/S32K144/sim_hal_S32K144.c
        src/SDK/platform/hal/src/smc/smc_hal.c
        src/SDK/rtos/osif/osif_baremetal.c

        )

set (SDK_INCLUDES
        src/SDK/platform/devices/common/s32_core_cm4.h
        src/SDK/platform/devices/devassert.h
        src/SDK/platform/devices/device_registers.h
        src/SDK/platform/devices/S32K144/include/S32K144.h
        src/SDK/platform/devices/S32K144/include/S32K144_features.h
        src/SDK/platform/devices/S32K144/startup/system_S32K144.h
        src/SDK/platform/devices/startup.h
        src/SDK/platform/devices/status.h
        src/SDK/platform/drivers/inc/clock_manager.h
        src/SDK/platform/drivers/inc/crc_driver.h
        src/SDK/platform/drivers/inc/csec_driver.h
        src/SDK/platform/drivers/inc/edma_driver.h
        src/SDK/platform/drivers/inc/flexcan_driver.h
        src/SDK/platform/drivers/inc/ftm_driver.h
        src/SDK/platform/drivers/inc/interrupt_manager.h
        src/SDK/platform/drivers/inc/lpspi_master_driver.h
        src/SDK/platform/drivers/inc/lpspi_shared_function.h
        src/SDK/platform/drivers/inc/lpspi_slave_driver.h
        src/SDK/platform/drivers/inc/lpuart_driver.h
        src/SDK/platform/drivers/inc/pins_driver.h
        src/SDK/platform/drivers/inc/power_manager.h
        src/SDK/platform/drivers/src/clock/S32K144/clock_S32K144.h
        src/SDK/platform/hal/inc/crc_hal.h
        src/SDK/platform/hal/inc/csec_hal.h
        src/SDK/platform/hal/inc/dmamux_hal.h
        src/SDK/platform/hal/inc/edma_hal.h
        src/SDK/platform/hal/inc/flexcan_hal.h
        src/SDK/platform/hal/inc/ftm_hal.h
        src/SDK/platform/hal/inc/gpio_hal.h
        src/SDK/platform/hal/inc/lpspi_hal.h
        src/SDK/platform/hal/inc/lpuart_hal.h
        src/SDK/platform/hal/inc/pcc_hal.h
        src/SDK/platform/hal/inc/pmc_hal.h
        src/SDK/platform/hal/inc/port_hal.h
        src/SDK/platform/hal/inc/rcm_hal.h
        src/SDK/platform/hal/inc/scg_hal.h
        src/SDK/platform/hal/inc/sim_hal.h
        src/SDK/platform/hal/inc/smc_hal.h
        src/SDK/platform/hal/src/sim/S32K144/sim_hal_S32K144.h
        src/SDK/rtos/osif/osif.h
        )

#file(GLOB_RECURSE USER_INCLUDES "Src/*.h")
set(COMMON_USER_INCLUDES
        Inc/MK63F12.h
        Inc/system_MK63F12.h
        src/Generated_Code/canCom1.h
        src/Generated_Code/canCom2.h
        src/Generated_Code/canCom3.h
        src/Generated_Code/clockMan1.h
        src/Generated_Code/Cpu.h
        src/Generated_Code/crc1.h
        src/Generated_Code/csec1.h
        src/Generated_Code/dmaController1.h
        src/Generated_Code/flexTimer1.h
        src/Generated_Code/lpspiCom1.h
        src/Generated_Code/lpuart1.h
        src/Generated_Code/osif1.h
        src/Generated_Code/pin_mux.h
        src/Generated_Code/pwrMan1.h

        )

set(DC2017_S32_SOURCES
        src/Sources/main.c
        )

add_executable(DC2017_S32.elf ${SDK_INCLUDES} ${COMMON_USER_INCLUDES} ${SDK_SOURCES} ${COMMON_USER_SOURCES}  ${DC2017_S32_SOURCES} ${LINKER_SCRIPT})
#add_executable(EOL.elf ${COMMON_USER_SOURCES} ${COMMON_USER_INCLUDES} ${EOL_SOURCES} ${LINKER_SCRIPT})



#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.map")
set(HEX_FILE ${PROJECT_SOURCE_DIR}/bin/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_SOURCE_DIR}/bin/${PROJECT_NAME}.bin)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMENT "Building ${HEX_FILE}"
#        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
#        COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")
)