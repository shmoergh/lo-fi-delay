cmake_minimum_required(VERSION 3.22)

# Default board/platform. Can be overridden at configure time with:
#   -DPICO_BOARD=<board>
#   -DPICO_PLATFORM=<platform>
if(NOT DEFINED PICO_BOARD)
	set(PICO_BOARD pico2)
endif()

if(NOT DEFINED PICO_PLATFORM)
	set(PICO_PLATFORM rp2350-arm-s)
endif()

# Include Pico SDK (from brain-sdk subdirectory)
include(brain-sdk/pico_sdk_import.cmake)

project(lo-fi-delay C CXX ASM)

# Initialize the Pico SDK
pico_sdk_init()

# Add brain-sdk which includes brain libraries
add_subdirectory(brain-sdk)

add_executable(lo-fi-delay
	main.cpp
)

target_link_libraries(lo-fi-delay
	pico_stdlib)

pico_enable_stdio_usb(lo-fi-delay 1)
pico_enable_stdio_uart(lo-fi-delay 1)
pico_add_extra_outputs(lo-fi-delay)
