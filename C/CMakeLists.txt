cmake_minimum_required(VERSION 3.10)

project(SAT C)

set(CMAKE_C_STANDARD 99)

add_executable(sat
    src/main.c
    src/mpu6050.c
    src/qmc5883l.c
    src/icm20948.c
    src/sco.c
)

target_include_directories(sat PRIVATE ${PROJECT_SOURCE_DIR}/inc)

target_link_libraries(sat PRIVATE m)