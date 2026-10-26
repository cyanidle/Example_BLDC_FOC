cmake_minimum_required(VERSION 3.22)

include(_vscode/init.cmake)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

include("cmake/gcc-arm-none-eabi.cmake")

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(vbcore C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

add_executable(vbcore)

add_subdirectory(cmake/stm32cubemx)

file(GLOB_RECURSE DSP_SRC
    Drivers/CMSIS/DSP/Source/*.c
)

add_compile_options(-Wno-sign-compare -Wno-unused-parameter -Wno-ignored-qualifiers)

add_library(DSP STATIC ${DSP_SRC})
target_include_directories(DSP PRIVATE
    Drivers/CMSIS/Include
    Drivers/CMSIS/DSP/Include
)

target_link_libraries(vbcore PRIVATE
    stm32cubemx
    DSP
)

set(VB_MOTOR_IMPL "simplefoc" CACHE STRING "Motor control implementation compiled into the firmware")
set_property(CACHE VB_MOTOR_IMPL PROPERTY STRINGS sixstep voltbro-foc simplefoc)

if(VB_MOTOR_IMPL STREQUAL "sixstep")
    set(VB_MOTOR_BACKEND_SOURCES App/backends/motor_backend_six_step.cpp)
    set(VB_MOTOR_BACKEND_DEFINE VB_MOTOR_IMPL_SIX_STEP)
elseif(VB_MOTOR_IMPL STREQUAL "voltbro-foc")
    set(VB_MOTOR_BACKEND_SOURCES App/backends/motor_backend_voltbro_foc.cpp)
    set(VB_MOTOR_BACKEND_DEFINE VB_MOTOR_IMPL_VOLTBRO_FOC)
elseif(VB_MOTOR_IMPL STREQUAL "simplefoc")
    set(VB_ARDUINO_FOC "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/Arduino-FOC")
    set(VB_MOTOR_BACKEND_SOURCES
        App/backends/motor_backend_simplefoc.cpp
        ${VB_ARDUINO_FOC}/compat/arduino_core.cpp
        ${VB_ARDUINO_FOC}/board/vbcore_g4_pwm.cpp
        ${VB_ARDUINO_FOC}/src/common/foc_utils.cpp
        ${VB_ARDUINO_FOC}/src/common/lowpass_filter.cpp
        ${VB_ARDUINO_FOC}/src/common/pid.cpp
        ${VB_ARDUINO_FOC}/src/common/time_utils.cpp
        ${VB_ARDUINO_FOC}/src/common/base_classes/CurrentSense.cpp
        ${VB_ARDUINO_FOC}/src/common/base_classes/FOCMotor.cpp
        ${VB_ARDUINO_FOC}/src/common/base_classes/Sensor.cpp
        ${VB_ARDUINO_FOC}/src/BLDCMotor.cpp
        ${VB_ARDUINO_FOC}/src/drivers/BLDCDriver3PWM.cpp
    )
    set(VB_MOTOR_BACKEND_DEFINE VB_MOTOR_IMPL_SIMPLEFOC)
else()
    message(FATAL_ERROR
        "VB_MOTOR_IMPL must be one of sixstep, voltbro-foc, simplefoc (got '${VB_MOTOR_IMPL}')")
endif()

file(GLOB USER_SOURCES "App/*.cpp")
file(GLOB_RECURSE LIBCXXCANARD_SOURCES "Drivers/libcxxcanard/cyphal/*.cpp" "Drivers/libcxxcanard/libs/*.c")
file(GLOB_RECURSE LIBVOLTBRO_SOURCES "Drivers/libvoltbro/*.cpp" "Drivers/libvoltbro/*.c")
# libvoltbro's UART configurator compiles only when HAL_UART_MODULE_ENABLED is
# set (our USART2 channel), and it needs nanoprintf, which is not vendored. The
# firmware configures itself over Cyphal, so that REPL is dead weight here.
list(FILTER LIBVOLTBRO_SOURCES EXCLUDE REGEX "/voltbro/config/serial/")
target_sources(vbcore PRIVATE
    ${USER_SOURCES}
    ${VB_MOTOR_BACKEND_SOURCES}
    ${LIBVOLTBRO_SOURCES}
    ${LIBCXXCANARD_SOURCES}
)

target_compile_options(vbcore PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-reorder>
)

target_include_directories(vbcore PRIVATE
    App
    Drivers/libcxxcanard
    # Order matters: generated DSDL headers do `#include <cyphal.h>`, which must
    # resolve to the modular cyphal/cyphal.h, not the amalgamated src/cyphal.h.
    Drivers/libcxxcanard/cyphal
    Drivers/libcxxcanard/libs
    Drivers/libcxxcanard/src
    Drivers/libvoltbro
    Drivers/CMSIS/DSP/Include
)

target_compile_definitions(vbcore PRIVATE
    -DARM_MATH_CM4
    -D__FPU_PRESENT=1U
    -DSTM32_G
    -DSTM32G
    -DSTM32G4
    -D${VB_MOTOR_BACKEND_DEFINE}
)

if(VB_MOTOR_IMPL STREQUAL "simplefoc")
    target_include_directories(vbcore PRIVATE
        ${VB_ARDUINO_FOC}/src
        ${VB_ARDUINO_FOC}/compat
    )
    target_compile_definitions(vbcore PRIVATE
        -DSIMPLEFOC_DISABLE_DEBUG
    )
endif()

add_custom_command(
    POST_BUILD
    TARGET vbcore
    COMMAND arm-none-eabi-objcopy -O ihex vbcore.elf vbcore.hex
    COMMAND arm-none-eabi-objcopy -O binary vbcore.elf vbcore.bin
)
