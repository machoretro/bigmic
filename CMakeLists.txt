cmake_minimum_required(VERSION 3.13)

# Set the Pico SDK path if not already defined
if (NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-sdk)
    message(STATUS "Using PICO_SDK_PATH from current directory: ${PICO_SDK_PATH}")
endif()

# Include the Pico SDK cmake initialization
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

# Set project name and C/C++ standards
project(bigmic C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Add the executable
add_executable(bigmic
    main.c
)

# Enable USB, disable UART
pico_enable_stdio_usb(bigmic 1)
pico_enable_stdio_uart(bigmic 0)

# Set the Pico target board (if different from default)
# Uncomment and modify if needed:
# set(PICO_BOARD pico_w)

# Link against required libraries
target_link_libraries(bigmic
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_irq
    hardware_clocks
    pico_multicore
)

# Generate additional output files (.uf2, .bin, etc.)
pico_add_extra_outputs(bigmic)
