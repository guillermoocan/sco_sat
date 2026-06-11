cmake_minimum_required(VERSION 3.10)

project(SAT C)

set(CMAKE_C_STANDARD 99)

add_library(mpu6050 src/mpu6050.c)

target_include_directories(mpu6050 PUBLIC ${PROJECT_SOURCE_DIR}/inc)

add_executable(sat src/main.c)

target_link_libraries(sat PRIVATE mpu6050)