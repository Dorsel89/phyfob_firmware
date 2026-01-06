cmake_minimum_required(VERSION 3.20.0)


#list(APPEND ZEPHYR_EXTRA_MODULES "${CMAKE_CURRENT_SOURCE_DIR}/drivers")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})



#add_subdirectory(drivers)

project(phyfob)
target_sources(app PRIVATE src/ble.c)
target_sources(app PRIVATE src/hdc.c)
target_sources(app PRIVATE src/bmp5.c)
target_sources(app PRIVATE src/bmpZephyr.c)
target_sources(app PRIVATE src/event.c)
target_sources(app PRIVATE src/lsm6dsr.c)
target_sources(app PRIVATE src/lsm6dsr_reg.c)

target_sources(app PRIVATE src/stcc4Zephyr.c)
target_sources(app PRIVATE src/sensirion_i2c.c)
target_sources(app PRIVATE src/stcc4_i2c.c)
target_sources(app PRIVATE src/sensirion_i2c_hal.c)
target_sources(app PRIVATE src/sensirion_common.c)
target_sources(app PRIVATE src/bmv080Zephyr.c)


set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
add_library(lib_bmv080 STATIC IMPORTED GLOBAL)
add_library(lib_postProcessor STATIC IMPORTED GLOBAL)
set_target_properties(lib_bmv080 PROPERTIES IMPORTED_LOCATION ${LIB_DIR}/lib_bmv080.a)
set_target_properties(lib_postProcessor PROPERTIES IMPORTED_LOCATION ${LIB_DIR}/lib_postProcessor.a)


target_link_libraries(app PRIVATE lib_bmv080)
target_link_libraries(app PRIVATE lib_postProcessor)

#target_sources(app PRIVATE src/bas.c)
target_sources(app PRIVATE src/main.c)
