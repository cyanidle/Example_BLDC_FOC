cmake_minimum_required(VERSION 3.22)

include(_vscode/init.cmake)

set(CMAKE_C_STANDARD 11)
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

file(GLOB_RECURSE USER_SOURCES "App/*.cpp")
file(GLOB_RECURSE LIBCXXCANARD_SOURCES "Drivers/libcxxcanard/cyphal/*.cpp" "Drivers/libcxxcanard/libs/*.c")
file(GLOB_RECURSE LIBVOLTBRO_SOURCES "Drivers/libvoltbro/*.cpp" "Drivers/libvoltbro/*.c")
target_sources(vbcore PRIVATE
    ${USER_SOURCES}
    ${LIBVOLTBRO_SOURCES}
    ${LIBCXXCANARD_SOURCES}
)

target_compile_options(vbcore PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-reorder>
)

target_include_directories(vbcore PRIVATE
    App
    Drivers/libcxxcanard
    Drivers/libcxxcanard/libs
    Drivers/libvoltbro
    Drivers/CMSIS/DSP/Include
)

target_compile_definitions(vbcore PRIVATE
    -DARM_MATH_CM4
    -D__FPU_PRESENT=1U
)

add_custom_command(
    POST_BUILD
    TARGET vbcore
    COMMAND arm-none-eabi-objcopy -O ihex vbcore.elf vbcore.hex
    COMMAND arm-none-eabi-objcopy -O binary vbcore.elf vbcore.bin
)
