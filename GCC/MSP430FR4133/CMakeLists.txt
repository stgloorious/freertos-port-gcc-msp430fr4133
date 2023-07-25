add_library(freertos_kernel_port STATIC)

target_sources(
    freertos_kernel_port
    PRIVATE
    port.c
    portmacro.h
)

target_include_directories(
    freertos_kernel_port
    PUBLIC
    .
)
target_link_libraries(
    freertos_kernel_port
    PRIVATE
    freertos_kernel_include
)
