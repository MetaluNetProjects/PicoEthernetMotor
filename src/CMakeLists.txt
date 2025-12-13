set(TARGET_NAME picoethmotor)
set(LWIP_PATH ${PICO_SDK_PATH}/lib/lwip)
set(TARGET_FILES    
    wiznet.c
    main.cpp
    fraise.c
    version.cpp
    )

add_executable(${TARGET_NAME} ${TARGET_FILES})

target_include_directories(${TARGET_NAME} PUBLIC
        ${LWIP_PATH}/src/include
        )

# This is required to include lwipopts.h
target_include_directories(pico_lwip INTERFACE        
        ${PORT_DIR}/lwip        
        )

# force recompile version.cpp each time for updating __TIME__
add_custom_command(TARGET ${TARGET_NAME}
        PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E touch_nocreate ${CMAKE_CURRENT_SOURCE_DIR}/version.cpp
        )

target_link_libraries(${TARGET_NAME} PRIVATE
        pico_stdlib
        hardware_spi
        hardware_dma
        hardware_pwm
        ETHERNET_FILES
        IOLIBRARY_FILES
        LWIP_FILES
        )

pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)

pico_add_extra_outputs(${TARGET_NAME})
