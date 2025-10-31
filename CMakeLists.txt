cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(skripsie)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_sources(app PRIVATE 
src/main.c
src/uart_comm.c
src/GPIO.c
src/mesh.c
src/sensors.c
src/initdata.c
)
