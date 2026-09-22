#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 4.2)

# specify cross-compilers and tools
if(NOT DEFINED CMAKE_C_COMPILER)
    set(CMAKE_C_COMPILER arm-none-eabi-gcc)
endif()
if(NOT DEFINED CMAKE_CXX_COMPILER)
    set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
endif()
if(NOT DEFINED CMAKE_ASM_COMPILER)
    set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
endif()
if(WIN32)
    set(ARM_TOOL_SUFFIX ".exe")
else()
    set(ARM_TOOL_SUFFIX "")
endif()
if(DEFINED CMAKE_C_COMPILER AND IS_ABSOLUTE "${CMAKE_C_COMPILER}")
    get_filename_component(ARM_TOOLCHAIN_BIN "${CMAKE_C_COMPILER}" DIRECTORY)
endif()
if(NOT DEFINED CMAKE_AR)
    if(DEFINED ARM_TOOLCHAIN_BIN)
        set(CMAKE_AR "${ARM_TOOLCHAIN_BIN}/arm-none-eabi-ar${ARM_TOOL_SUFFIX}")
    else()
        set(CMAKE_AR arm-none-eabi-ar)
    endif()
endif()
if(NOT DEFINED CMAKE_RANLIB)
    if(DEFINED ARM_TOOLCHAIN_BIN)
        set(CMAKE_RANLIB "${ARM_TOOLCHAIN_BIN}/arm-none-eabi-ranlib${ARM_TOOL_SUFFIX}")
    else()
        set(CMAKE_RANLIB arm-none-eabi-ranlib)
    endif()
endif()
if(NOT DEFINED CMAKE_OBJCOPY)
    if(DEFINED ARM_TOOLCHAIN_BIN)
        set(CMAKE_OBJCOPY "${ARM_TOOLCHAIN_BIN}/arm-none-eabi-objcopy${ARM_TOOL_SUFFIX}")
    else()
        set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
    endif()
endif()
if(NOT DEFINED CMAKE_OBJDUMP)
    if(DEFINED ARM_TOOLCHAIN_BIN)
        set(CMAKE_OBJDUMP "${ARM_TOOLCHAIN_BIN}/arm-none-eabi-objdump${ARM_TOOL_SUFFIX}")
    else()
        set(CMAKE_OBJDUMP arm-none-eabi-objdump)
    endif()
endif()
if(NOT DEFINED SIZE)
    if(DEFINED ARM_TOOLCHAIN_BIN)
        set(SIZE "${ARM_TOOLCHAIN_BIN}/arm-none-eabi-size${ARM_TOOL_SUFFIX}")
    else()
        set(SIZE arm-none-eabi-size)
    endif()
endif()
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(fm_v3 C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
#add_compile_definitions(ARM_MATH_CM7;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
#add_compile_options(-mfloat-abi=hard -mfpu=fpv5-d16)
#add_link_options(-mfloat-abi=hard -mfpu=fpv5-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m7 -mthumb)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
add_compile_options(-Wall -fstack-usage --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Debug optimization and debug information")
    add_compile_options(-O0 -g3)
endif ()

include_directories(Core/Inc BSP/ETH Drivers/STM32H7xx_HAL_Driver/Inc Drivers/STM32H7xx_HAL_Driver/Inc/Legacy Drivers/CMSIS/Device/ST/STM32H7xx/Include Drivers/CMSIS/Include LWIP/App LWIP/Target Middlewares/Third_Party/LwIP/src/include Middlewares/Third_Party/LwIP/system Middlewares/Third_Party/LwIP/src/include/netif/ppp Middlewares/Third_Party/LwIP/src/include/lwip Middlewares/Third_Party/LwIP/src/include/lwip/apps Middlewares/Third_Party/LwIP/src/include/lwip/priv Middlewares/Third_Party/LwIP/src/include/lwip/prot Middlewares/Third_Party/LwIP/src/include/netif Middlewares/Third_Party/LwIP/src/include/compat/posix Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa Middlewares/Third_Party/LwIP/src/include/compat/posix/net Middlewares/Third_Party/LwIP/src/include/compat/posix/sys Middlewares/Third_Party/LwIP/src/include/compat/stdc Middlewares/Third_Party/LwIP/system/arch Middlewares/Third_Party/FreeRTOS/Source/include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F)

add_definitions(-DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -DDATA_IN_D2_SRAM)

file(GLOB_RECURSE SOURCES "BSP/*.*" "Core/*.*" "Middlewares/*.*" "LWIP/*.*" "Drivers/*.*")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32H723ZETX_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard)
add_link_options(--specs=nosys.specs --specs=nano.specs -static)
add_link_options(-Wl,--start-group -lc -lm -Wl,--end-group)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")

set(STM32_PROGRAMMER_CLI "C:/ST/STM32CubeIDE_1.10.1/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.0.301.202207041506/tools/bin/STM32_Programmer_CLI.exe" CACHE FILEPATH "STM32CubeProgrammer CLI executable")

add_custom_target(flash
        COMMAND "${STM32_PROGRAMMER_CLI}" -c port=SWD -w "${HEX_FILE}" -v -rst
        DEPENDS ${PROJECT_NAME}.elf
        USES_TERMINAL
        COMMENT "Flashing ${HEX_FILE} via ST-LINK")
