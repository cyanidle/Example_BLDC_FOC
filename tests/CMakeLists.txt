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
