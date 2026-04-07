cmake_minimum_required(VERSION 3.22)

# 项目基本配置
set(CMAKE_PROJECT_NAME G4_FRAMEWORK)
project(${CMAKE_PROJECT_NAME} C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

# 自动搜索源文件
file(GLOB_RECURSE USER_SOURCES
        CONFIGURE_DEPENDS
        "User/*.c" "User/*.cpp" "User/*.s"
)

add_executable(${CMAKE_PROJECT_NAME} ${USER_SOURCES}
        User/UI/RM_UI_Adapter.h
        User/UI/RM_UI_Adapter.c
        User/UI/RMUI.h
        User/UI/RMUI.c)

# 自动递归包含头文件
file(GLOB_RECURSE ALL_H_FILES CONFIGURE_DEPENDS "User/*.h" "User/*.hpp" "Core/*.h")
set(ALL_INCLUDE_DIRS "")
foreach(FILE_PATH ${ALL_H_FILES})
    get_filename_component(DIR_PATH ${FILE_PATH} DIRECTORY)
    list(APPEND ALL_INCLUDE_DIRS ${DIR_PATH})
endforeach()
list(REMOVE_DUPLICATES ALL_INCLUDE_DIRS)

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
        ${ALL_INCLUDE_DIRS}
        Core/Inc
        Drivers/CMSIS/Include
        Drivers/CMSIS/DSP/Include
        Drivers/CMSIS/Device/ST/STM32G4xx/Include
)

# 编译选项与宏定义
target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC
        STM32G473xx USE_HAL_DRIVER ARM_MATH_CM4 __FPU_PRESENT=1U
        HAL_ASSERT_DISABLE=1 ARM_MATH_MATRIX_CHECK=0 ARM_MATH_ROUNDING=0
        $<$<CONFIG:Debug>:DEBUG_MODE> $<$<CONFIG:Release>:RELEASE_MODE>
)

target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC
        ${CPU_FLAGS}
        -Wall -Wextra -Wno-comment -Wno-unused-parameter -Wno-unused-but-set-variable
        -Werror=implicit-function-declaration -Werror=incompatible-pointer-types
        -fdata-sections -ffunction-sections
        $<$<CONFIG:Debug>:-Og -g3 -fno-omit-frame-pointer -fno-lto>
        $<$<CONFIG:Release>:-O3 -ffast-math -funroll-loops -flto=auto>
)

# 链接配置
target_link_options(${CMAKE_PROJECT_NAME} PUBLIC
        ${CPU_FLAGS}
        -Wl,--gc-sections
        -Wl,--print-memory-usage
        -Wl,--relax
        -Wl,--sort-common=descending
        $<$<CONFIG:Release>:-flto=auto>
)

# 链接库
add_subdirectory(cmake/stm32cubemx)

set(DSP_LIB "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC/libarm_cortexM4lf_math.a")

target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        m
        "${DSP_LIB}"
)

# 后处理
set(HEX_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> "${HEX_FILE}"
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> "${BIN_FILE}"
        COMMENT "Generating Output Files..."
)