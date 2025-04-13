cmake_minimum_required(VERSION 3.13)

# Set project name and C/C++ standards
project(bigmic-rp2040 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Define the path to the Pico SDK
if (NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-sdk)
endif()

# Pull in Pico SDK and TinyUSB
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

# Initialize SDK
pico_sdk_init()

# Add the executable
add_executable(bigmic
    bigmic-rp2040-v10.c
)

# Link against stdlib
target_link_libraries(bigmic pico_stdlib)

# Optional: enable USB stdio
pico_enable_stdio_usb(bigmic 1)
pico_enable_stdio_uart(bigmic 0)

# Generate UF2 and bin
pico_add_extra_outputs(bigmic)
