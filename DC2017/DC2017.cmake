INCLUDE(CMakeForceCompiler)

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
CMAKE_FORCE_C_COMPILER(arm-none-eabi-gcc GNU)
CMAKE_FORCE_CXX_COMPILER(arm-none-eabi-g++ GNU)


add_definitions(-DCPU_MK63FN1M0VLQ12)
add_definitions(-D__USE_CMSIS)
add_definitions(-DCPU_MK63FN1M0VLQ12_cm4)
add_definitions(-DUSB_STACK_BM)
add_definitions(-DUSE_RTOS=1)
add_definitions(-DFSL_RTOS_FREE_RTOS)
add_definitions(-DPRINTF_ADVANCED_ENABLE=1)
add_definitions(-DAMX_TOKENTHREADING)
add_definitions(-DAMX_NOPROPLIST)
add_definitions(-DAMX_ALTCORE)
add_definitions(-DAMX_DONT_RELOCATE)
add_definitions(-DAMX_NATIVETABLE=qcm_native_table)
add_definitions(-DFLOATPOINT)

SET(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/DC2017_Debug.ld)
SET(COMMON_FLAGS "-g -mfloat-abi=softfp -specs=nosys.specs -Wall -fdata-sections -ffunction-sections -mcpu=cortex-m4 -mthumb")

SET(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++11")
SET(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections -T ${LINKER_SCRIPT}")

