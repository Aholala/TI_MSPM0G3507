cmake_minimum_required(VERSION 3.22)

project(model LANGUAGES C)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(SYSCFG_SCRIPT "${PROJECT_ROOT}/empty.syscfg")
set(SYSCFG_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/syscfg")

set(MSPM0_STARTUP_SOURCE
    "${MSPM0_SDK_ROOT}/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c"
)
set(FREERTOS_ROOT "${MSPM0_SDK_ROOT}/kernel/freertos")
set(FREERTOS_PORT_DIR "${FREERTOS_ROOT}/Source/portable/TI_ARM_CLANG/ARM_CM0")
set(CMSIS_RTOS_V2_DIR
    "${MSPM0_SDK_ROOT}/examples/nortos/LP_MSPM0G3507/driverlib/Ark_Plan/SUPERARK_MilkForg/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2"
)

if(NOT EXISTS "${MSPM0_STARTUP_SOURCE}")
    message(FATAL_ERROR
        "MSPM0 startup source not found: ${MSPM0_STARTUP_SOURCE}\n"
        "Set MSPM0_SDK_ROOT to the installed MSPM0 SDK path."
    )
endif()

if(NOT EXISTS "${SYSCONFIG_CLI}")
    message(FATAL_ERROR
        "SysConfig CLI not found: ${SYSCONFIG_CLI}\n"
        "Set SYSCONFIG_CLI to sysconfig_cli.bat."
    )
endif()

set(SYSCFG_OUTPUTS
    "${SYSCFG_OUTPUT_DIR}/device_linker.cmd"
    "${SYSCFG_OUTPUT_DIR}/device.opt"
    "${SYSCFG_OUTPUT_DIR}/device.cmd.genlibs"
    "${SYSCFG_OUTPUT_DIR}/ti_msp_dl_config.c"
    "${SYSCFG_OUTPUT_DIR}/ti_msp_dl_config.h"
    "${SYSCFG_OUTPUT_DIR}/Event.dot"
)

add_custom_command(
    OUTPUT ${SYSCFG_OUTPUTS}
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${SYSCFG_OUTPUT_DIR}"
    COMMAND "${SYSCONFIG_CLI}"
            -s "${MSPM0_SDK_ROOT}/.metadata/product.json"
            --script "${SYSCFG_SCRIPT}"
            -o "${SYSCFG_OUTPUT_DIR}"
            --compiler ticlang
    DEPENDS "${SYSCFG_SCRIPT}"
    COMMENT "Generating MSPM0 SysConfig files"
    VERBATIM
)

add_custom_target(generate_syscfg DEPENDS ${SYSCFG_OUTPUTS})

file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    "${PROJECT_ROOT}/User/Bsp/*.c"
    "${PROJECT_ROOT}/User/Module/*.c"
    "${PROJECT_ROOT}/User/App/*.c"
    "${PROJECT_ROOT}/User/Lib/*.c"
)

file(GLOB_RECURSE USER_HEADERS CONFIGURE_DEPENDS
    "${PROJECT_ROOT}/User/*.h"
)

set(USER_INCLUDE_DIRS
    "${PROJECT_ROOT}/User/Bsp"
    "${PROJECT_ROOT}/User/Module"
    "${PROJECT_ROOT}/User/App"
    "${PROJECT_ROOT}/User/Lib"
)

foreach(USER_HEADER ${USER_HEADERS})
    get_filename_component(USER_HEADER_DIR "${USER_HEADER}" DIRECTORY)
    list(APPEND USER_INCLUDE_DIRS "${USER_HEADER_DIR}")
endforeach()

list(REMOVE_DUPLICATES USER_INCLUDE_DIRS)

add_library(freertos_kernel STATIC
    "${FREERTOS_ROOT}/Source/event_groups.c"
    "${FREERTOS_ROOT}/Source/list.c"
    "${FREERTOS_ROOT}/Source/queue.c"
    "${FREERTOS_ROOT}/Source/stream_buffer.c"
    "${FREERTOS_ROOT}/Source/tasks.c"
    "${FREERTOS_ROOT}/Source/timers.c"
    "${FREERTOS_ROOT}/Source/portable/MemMang/heap_4.c"
    "${FREERTOS_PORT_DIR}/port.c"
    "${FREERTOS_PORT_DIR}/portasm.c"
)

add_dependencies(freertos_kernel generate_syscfg)

target_include_directories(freertos_kernel PUBLIC
    "${PROJECT_ROOT}/User/App"
    "${SYSCFG_OUTPUT_DIR}"
    "${MSPM0_SDK_ROOT}/source/third_party/CMSIS/Core/Include"
    "${MSPM0_SDK_ROOT}/source"
    "${FREERTOS_ROOT}/Source/include"
    "${FREERTOS_PORT_DIR}"
    "${CMSIS_RTOS_V2_DIR}"
)

target_compile_options(freertos_kernel PRIVATE
    @${SYSCFG_OUTPUT_DIR}/device.opt
    -march=thumbv6m
    -mcpu=cortex-m0plus
    -mfloat-abi=soft
    -mlittle-endian
    -mthumb
    -O0
    -gdwarf-3
    -Wall
)

add_executable(${PROJECT_NAME}
    "${PROJECT_ROOT}/main.c"
    "${SYSCFG_OUTPUT_DIR}/ti_msp_dl_config.c"
    "${MSPM0_STARTUP_SOURCE}"
    ${USER_SOURCES}
)

add_dependencies(${PROJECT_NAME} generate_syscfg)

target_include_directories(${PROJECT_NAME} PRIVATE
    "${PROJECT_ROOT}"
    "${SYSCFG_OUTPUT_DIR}"
    "${MSPM0_SDK_ROOT}/source/third_party/CMSIS/Core/Include"
    "${MSPM0_SDK_ROOT}/source"
    "${FREERTOS_ROOT}/Source/include"
    "${FREERTOS_PORT_DIR}"
    "${CMSIS_RTOS_V2_DIR}"
    ${USER_INCLUDE_DIRS}
)

target_compile_options(${PROJECT_NAME} PRIVATE
    @${SYSCFG_OUTPUT_DIR}/device.opt
    -march=thumbv6m
    -mcpu=cortex-m0plus
    -mfloat-abi=soft
    -mlittle-endian
    -mthumb
    -O0
    -gdwarf-3
    -Wall
)

target_link_options(${PROJECT_NAME} PRIVATE
    @${SYSCFG_OUTPUT_DIR}/device.opt
    -march=thumbv6m
    -mcpu=cortex-m0plus
    -mfloat-abi=soft
    -mlittle-endian
    -mthumb
    -O0
    -gdwarf-3
    -Wall
    "SHELL:-Wl,-m${PROJECT_NAME}.map"
    "SHELL:-Wl,-i${MSPM0_SDK_ROOT}/source"
    "SHELL:-Wl,-i${PROJECT_ROOT}"
    "SHELL:-Wl,-i${SYSCFG_OUTPUT_DIR}"
    "SHELL:-Wl,-i${TI_ARM_CLANG_ROOT}/lib"
    "SHELL:-Wl,--diag_wrap=off"
    "SHELL:-Wl,--display_error_number"
    "SHELL:-Wl,--warn_sections"
    "SHELL:-Wl,--xml_link_info=${PROJECT_NAME}_linkInfo.xml"
    "SHELL:-Wl,--rom_model"
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    freertos_kernel
    "-Wl,-l${SYSCFG_OUTPUT_DIR}/device_linker.cmd"
    "-Wl,-l${SYSCFG_OUTPUT_DIR}/device.cmd.genlibs"
    "-Wl,-llibc.a"
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    OUTPUT_NAME "${PROJECT_NAME}"
    SUFFIX ".out"
)
