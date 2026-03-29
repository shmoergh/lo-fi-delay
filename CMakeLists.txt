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

# Reserve SDK-managed storage sectors (app blob + calibration [+ RP2350 guard])
# before Pico SDK initialization so linker flash size is adjusted safely.
include(brain-sdk/cmake/brain-storage-reserve-flash.cmake)
brain_storage_configure_flash_reservation()

# Initialize the Pico SDK
pico_sdk_init()

# Add brain-sdk which includes brain libraries
add_subdirectory(brain-sdk)

add_executable(lo-fi-delay
	main.cpp
	src/audio_input_dma.cpp
	src/fast_dac_out.cpp
	src/delay_engine.cpp
	src/delay_app.cpp
)

target_include_directories(lo-fi-delay PRIVATE
	src
)

target_link_libraries(lo-fi-delay
	pico_stdlib
	brain-common
	brain-ui
	brain-io
	brain-utils
	hardware_adc
	hardware_dma)

pico_enable_stdio_usb(lo-fi-delay 1)
pico_enable_stdio_uart(lo-fi-delay 1)
pico_add_extra_outputs(lo-fi-delay)
