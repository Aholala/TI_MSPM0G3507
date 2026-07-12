cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME provincial_firstprize)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    User/App/app_chassis.c
    User/App/app_icm45686.c
    User/App/app_line_follow.c
    User/App/app_race_config.c
    User/App/icm45686_port.c
    User/App/task/chassis_task.c
    User/App/task/control_task.c
    User/App/task/imu_task.c
    User/App/task/line_follow_task.c
    User/Bsp/bsp_encoder.c
    User/Bsp/bsp_buzzer.c
    User/Bsp/bsp_key.c
    User/Bsp/bsp_led.c
    User/Bsp/bsp_icm45686.c
    User/Bsp/bsp_line_sensor.c
    User/Bsp/bsp_oled.c
    User/Bsp/bsp_oled_data.c
    User/Bsp/bsp_tb6612.c
    User/Lib/lib_attitude.c
	User/Lib/lib_adaptive_mahony.c
    User/Lib/lib_kalman.c
    User/Lib/lib_pid.c
    User/Lib/lib_rate_filter.c
    User/Module/inv_imu_driver.c
    User/Module/inv_imu_transport.c
    User/Module/module_buzzer.c
    User/Module/module_icm45686.c
    User/Module/module_key.c
    User/Module/module_led.c
    User/Module/module_motor.c
    User/Module/module_tracker.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    User/App
    User/App/task
    User/Bsp
    User/Lib
    User/Module
    User/Module/Motor
    User/Module/Pid
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
