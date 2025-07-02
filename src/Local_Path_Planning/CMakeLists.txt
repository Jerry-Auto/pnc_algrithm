cmake_minimum_required(VERSION 3.10)
project(auto_driving_algorithm)


set(PACKAGE_NAME auto_driving_algorithm)

set(CMAKE_CXX_STANDARD 14)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# 设置编译模式Debug Realease
set(CMAKE_BUILD_TYPE Debug)

# 创建输出目录
set(LIB_OUTPUT_DIR ${CMAKE_SOURCE_DIR}/src/libs/Local_Path_Planning)
set(BIN_OUTPUT_DIR ${CMAKE_SOURCE_DIR}/bin/Local_Path_Planning/example)

file(MAKE_DIRECTORY 
    ${LIB_OUTPUT_DIR}
    ${BIN_OUTPUT_DIR}
    ${CMAKE_SOURCE_DIR}/bin/Local_Path_Planning/example/image
)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIB_OUTPUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_OUTPUT_DIR})

#包含公共头文件目录
# 查找 Python 3
find_package(Python3 REQUIRED COMPONENTS Development)
message(STATUS "Python3 include dirs: ${Python3_INCLUDE_DIRS}")
message(STATUS "Python3 libraries: ${Python3_LIBRARIES}")

# 包含目录
include_directories(
    ${CMAKE_SOURCE_DIR}/src/libs/common/include
    ${Python3_INCLUDE_DIRS}
    /usr/include/python3.10
    /usr/local/lib/python3.10/dist-packages/numpy/core/include
)

find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
    ${OSQPEIGEN_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)


# Vehicle类的编译
set(VEHICLE_LIB_NAME Vehicle)
add_library(${VEHICLE_LIB_NAME} SHARED src/Vehicle.cpp)  


target_include_directories(${VEHICLE_LIB_NAME}  
    PUBLIC
    include 
    PRIVATE 
    auto_driving_algorithm 
)


set(VEHICLE_DEMO_NAME Vehicle_demo)
add_executable(${VEHICLE_DEMO_NAME} example/Vehicle_demo.cpp)  

target_link_libraries(${VEHICLE_DEMO_NAME} ${VEHICLE_LIB_NAME})  


# 多项式曲线（poly_algorithm）类的编译
set(POLY_LIB_NAME PolynomialAlgorithm)
add_library(${POLY_LIB_NAME} SHARED src/PolynomialAlgorithm.cpp)  


target_include_directories(${POLY_LIB_NAME}  
    PUBLIC
    include 
    PRIVATE 
    auto_driving_algorithm 
)


set(POLY_DEMO_NAME PolynomialAlgorithm_demo)
add_executable(${POLY_DEMO_NAME} example/PolynomialAlgorithm_demo.cpp)  

target_link_libraries(${POLY_DEMO_NAME} ${POLY_LIB_NAME})  

