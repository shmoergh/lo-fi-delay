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

option(LFD_ENABLE_CCARD_SPIKE "Enable ComputerCard-style A/B spike path" ON)

# Reserve SDK-managed storage sectors (app blob + calibration [+ RP2350 guard])
# before Pico SDK initialization so linker flash size is adjusted safely.
set(BRAIN_STORAGE_ENABLE_FLASH_RESERVATION ON CACHE BOOL "" FORCE)
set(BRAIN_STORAGE_ALLOW_UNPROTECTED_LAYOUT OFF CACHE BOOL "" FORCE)
include(brain-sdk/cmake/brain-storage-reserve-flash.cmake)
brain_storage_configure_flash_reservation()

# Initialize the Pico SDK
pico_sdk_init()

# Add brain-sdk which includes brain libraries
add_subdirectory(brain-sdk)

add_executable(lo-fi-delay
	main.cpp
	src/audio_input_dma.cpp
	src/unified_adc_dma.cpp
	src/fast_dac_out.cpp
	src/delay_engine.cpp
	src/delay_app.cpp
)

target_include_directories(lo-fi-delay PRIVATE
	src
)

target_compile_definitions(lo-fi-delay PRIVATE
	BRAIN_USE_LEDS=1
	BRAIN_USE_BUTTONS=1
	$<$<BOOL:${LFD_ENABLE_CCARD_SPIKE}>:LFD_ENABLE_CCARD_SPIKE=1>
	$<$<NOT:$<BOOL:${LFD_ENABLE_CCARD_SPIKE}>>:LFD_ENABLE_CCARD_SPIKE=0>
)

target_link_libraries(lo-fi-delay
	pico_stdlib
	brain
	hardware_adc
	hardware_dma
	hardware_gpio
	hardware_spi
	hardware_sync
)

pico_enable_stdio_usb(lo-fi-delay 1)
pico_enable_stdio_uart(lo-fi-delay 1)
pico_add_extra_outputs(lo-fi-delay)
