cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk)
include(pico_sdk_import.cmake)

project(my_firmware C CXX ASM)

pico_sdk_init()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/pico-ice-sdk)

add_executable(my_firmware
    main.c
    usb_descriptors.c
)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/generated)

# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(my_firmware ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

target_link_libraries(my_firmware
    pico_ice_sdk
    pico_ice_usb
    pico_stdio_usb
    pico_stdlib 
    hardware_i2c
    hardware_pio
)
target_include_directories(my_firmware PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)
pico_enable_stdio_usb(my_firmware 1)
pico_add_extra_outputs(my_firmware)

# Additionally generate python and hex pioasm outputs for inclusion in the RP2040 datasheet
add_custom_target(my_firmware_datasheet DEPENDS ${CMAKE_CURRENT_LIST_DIR}/generated/ws2812.py)
add_custom_command(OUTPUT ${CMAKE_CURRENT_LIST_DIR}/generated/ws2812.py
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio
        COMMAND Pioasm -o python ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio ${CMAKE_CURRENT_LIST_DIR}/generated/ws2812.py
        VERBATIM)
add_dependencies(my_firmware my_firmware_datasheet)