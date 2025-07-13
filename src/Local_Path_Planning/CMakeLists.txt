cmake_minimum_required(VERSION 3.10)
project(auto_driving_algorithm)


set(PACKAGE_NAME auto_driving_algorithm)

set(CMAKE_CXX_STANDARD 17)

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

#set(CMAKE_AUTOMOC ON)


# Vehicle_model类的编译
set(VEHICLE_LIB_NAME vehicle_model)
add_library(${VEHICLE_LIB_NAME} SHARED)  
target_sources(${VEHICLE_LIB_NAME}  
    PRIVATE 
    src/vehicle_model.cpp 
)
target_include_directories(${VEHICLE_LIB_NAME}  
    PUBLIC include 
    PRIVATE 
    auto_driving_algorithm 
)
set(VEHICLE_DEMO_NAME vehicle_model_demo)
add_executable(${VEHICLE_DEMO_NAME} example/vehicle_model_demo.cpp)  
target_link_libraries(${VEHICLE_DEMO_NAME} ${VEHICLE_LIB_NAME} Python3::Python Python3::NumPy)  

# World_map类so库的编译
set(WORLD_MAP_LIB_NAME World_map)
add_library(${WORLD_MAP_LIB_NAME} SHARED)  
target_sources(${WORLD_MAP_LIB_NAME}  
    PRIVATE 
    src/World_map.cpp 
)
target_link_libraries(${WORLD_MAP_LIB_NAME} ${VEHICLE_LIB_NAME} Python3::Python ) 
target_include_directories(${WORLD_MAP_LIB_NAME}  
    PUBLIC include 
    PRIVATE 
    auto_driving_algorithm 
)


# 多项式曲线（poly_algorithm）类的编译
set(POLY_LIB_NAME PolynomialAlgorithm)
add_library(${POLY_LIB_NAME} SHARED src/PolynomialAlgorithm.cpp)  
target_include_directories(${POLY_LIB_NAME}  
    PUBLIC include 
    PRIVATE 
    auto_driving_algorithm 
)
target_link_libraries(${POLY_LIB_NAME} Python3::Python) 
set(POLY_DEMO_NAME PolynomialAlgorithm_demo)
add_executable(${POLY_DEMO_NAME} example/PolynomialAlgorithm_demo.cpp)  
target_link_libraries(${POLY_DEMO_NAME} ${POLY_LIB_NAME} ${WORLD_MAP_LIB_NAME} Python3::Python Python3::NumPy)
#target_link_directories(${POLY_DEMO_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/libs/Local_Path_Planning)   


#World_map_demo的编译
set(WORLD_MAP_DEMO_NAME World_map_demo)
add_executable(${WORLD_MAP_DEMO_NAME} example/World_map_demo.cpp)  
target_link_libraries(${WORLD_MAP_DEMO_NAME} ${WORLD_MAP_LIB_NAME} ${POLY_LIB_NAME} Python3::Python Python3::NumPy)  


#用作测试的cpp文件
add_executable(test example/test.cpp) 
target_link_libraries(test ${WORLD_MAP_LIB_NAME} Python3::Python Python3::NumPy) 



# B样条曲线（B_spline）类的编译
set(B_SPLINE_LIB_NAME B_spline)
add_library(${B_SPLINE_LIB_NAME} SHARED src/B_spline.cpp)  
target_include_directories(${B_SPLINE_LIB_NAME}  
    PUBLIC include 
    PRIVATE 
    auto_driving_algorithm 
)
target_link_libraries(${B_SPLINE_LIB_NAME} Python3::Python) 
set(B_SPLINE_DEMO_NAME B_spline_demo)
add_executable(${B_SPLINE_DEMO_NAME} example/B_spline_demo.cpp)  
target_link_libraries(${B_SPLINE_DEMO_NAME} ${B_SPLINE_LIB_NAME} ${WORLD_MAP_LIB_NAME} Python3::Python Python3::NumPy)

# DWA类的编译
set(DWA_LIB_NAME DWA)
add_library(${DWA_LIB_NAME} SHARED src/DWA_planner.cpp)  
target_include_directories(${DWA_LIB_NAME}  
    PUBLIC include 
    PRIVATE 
    auto_driving_algorithm 
)
target_link_libraries(${DWA_LIB_NAME} ${WORLD_MAP_LIB_NAME} Python3::Python) 
set(DWA_DEMO_NAME DWA_demo)
add_executable(${DWA_DEMO_NAME} example/DWA_planner_demo.cpp)  
target_link_libraries(${DWA_DEMO_NAME} ${DWA_LIB_NAME} ${WORLD_MAP_LIB_NAME} Python3::Python Python3::NumPy)

# DWA动力学模型类的编译
set(DWA_DY_LIB_NAME DWA_DY)
add_library(${DWA_DY_LIB_NAME} SHARED src/DWA_dy_planner.cpp)  
target_include_directories(${DWA_DY_LIB_NAME}  
    PUBLIC include 
    PRIVATE 
    auto_driving_algorithm 
)
target_link_libraries(${DWA_DY_LIB_NAME} ${WORLD_MAP_LIB_NAME} Python3::Python) 
set(DWA_DY_DEMO_NAME DWA_dy_demo)
add_executable(${DWA_DY_DEMO_NAME} example/DWA_dy_planner_demo.cpp)  
target_link_libraries(${DWA_DY_DEMO_NAME} ${DWA_DY_LIB_NAME} ${WORLD_MAP_LIB_NAME} Python3::Python Python3::NumPy)


