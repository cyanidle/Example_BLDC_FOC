cmake_minimum_required(VERSION 3.22)
project(six_step_host_tests LANGUAGES CXX)

enable_testing()
set(VOLTBRO "${CMAKE_CURRENT_LIST_DIR}/../Drivers/libvoltbro")
add_executable(six_step_recovery
    six_step_recovery.cpp
    ${VOLTBRO}/voltbro/encoders/hall_sensor/hall_sensor.cpp
    ${VOLTBRO}/voltbro/motors/bldc/bldc.cpp
    ${VOLTBRO}/voltbro/motors/bldc/six_step/six_step_controller.cpp
)
target_compile_features(six_step_recovery PRIVATE cxx_std_20)
target_compile_definitions(six_step_recovery PRIVATE
    STM32_G HAL_TIM_MODULE_ENABLED HAL_ADC_MODULE_ENABLED
)
target_include_directories(six_step_recovery PRIVATE hal ${VOLTBRO})
target_compile_options(six_step_recovery PRIVATE -Wall -Wextra -Wno-reorder)
add_test(NAME six_step_recovery COMMAND six_step_recovery)

add_executable(current_guard
    current_guard.cpp
)
target_compile_features(current_guard PRIVATE cxx_std_20)
target_include_directories(current_guard PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../App
    ${VOLTBRO}
)
target_compile_options(current_guard PRIVATE -Wall -Wextra -Wno-reorder)
add_test(NAME current_guard COMMAND current_guard)

add_executable(motor_limits
    motor_limits.cpp
)
target_compile_features(motor_limits PRIVATE cxx_std_20)
target_include_directories(motor_limits PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../App
)
target_compile_options(motor_limits PRIVATE -Wall -Wextra -Wno-reorder)
add_test(NAME motor_limits COMMAND motor_limits)

add_executable(config_persist
    config_persist.cpp
)
target_compile_features(config_persist PRIVATE cxx_std_20)
target_include_directories(config_persist PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../App
)
target_compile_options(config_persist PRIVATE -Wall -Wextra -Wno-reorder)
add_test(NAME config_persist COMMAND config_persist)

add_executable(cycle_counter
    cycle_counter.cpp
)
target_compile_features(cycle_counter PRIVATE cxx_std_20)
target_include_directories(cycle_counter PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../App
)
target_compile_options(cycle_counter PRIVATE -Wall -Wextra -Wno-reorder)
add_test(NAME cycle_counter COMMAND cycle_counter)

add_executable(loop_trace
    loop_trace.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../App/loop_trace.cpp
)
target_compile_features(loop_trace PRIVATE cxx_std_20)
target_include_directories(loop_trace PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../App
)
target_compile_options(loop_trace PRIVATE -Wall -Wextra -Wno-reorder)
add_test(NAME loop_trace COMMAND loop_trace)

add_executable(voltage_slew
    voltage_slew.cpp
)
target_compile_features(voltage_slew PRIVATE cxx_std_20)
target_include_directories(voltage_slew PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../App
)
target_compile_options(voltage_slew PRIVATE -Wall -Wextra -Wno-reorder)
add_test(NAME voltage_slew COMMAND voltage_slew)

add_executable(serial_line
    serial_line.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../App/serial_line.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../App/loop_trace.cpp
)
target_compile_features(serial_line PRIVATE cxx_std_20)
target_compile_definitions(serial_line PRIVATE
    STM32_G HAL_TIM_MODULE_ENABLED HAL_ADC_MODULE_ENABLED HAL_UART_MODULE_ENABLED
    VB_FIRMWARE_COMMIT=\"host-test\")
target_include_directories(serial_line PRIVATE
    hal
    ${CMAKE_CURRENT_LIST_DIR}/../App
    ${VOLTBRO}
)
target_compile_options(serial_line PRIVATE -Wall -Wextra -Wno-reorder)
add_test(NAME serial_line COMMAND serial_line)
