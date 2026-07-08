set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m0plus)

set(TI_ARM_CLANG_ROOT "C:/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS" CACHE PATH
    "TI ARM Clang compiler root")
set(MSPM0_SDK_ROOT "C:/ti/mspm0_sdk_2_10_00_04" CACHE PATH
    "TI MSPM0 SDK root")
set(SYSCONFIG_CLI "C:/ti/sysconfig_1.26.2/sysconfig_cli.bat" CACHE FILEPATH
    "TI SysConfig command line tool")

set(CMAKE_C_COMPILER "${TI_ARM_CLANG_ROOT}/bin/tiarmclang.exe" CACHE FILEPATH
    "TI ARM Clang C compiler")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
