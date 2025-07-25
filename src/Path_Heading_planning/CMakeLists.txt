cmake_minimum_required(VERSION 3.10)
project(auto_driving_algorithm)


set(PACKAGE_NAME auto_driving_algorithm)

set(CMAKE_CXX_STANDARD 17)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# 设置编译模式Debug Realease
set(CMAKE_BUILD_TYPE Debug)

# 创建输出目录
set(LIB_OUTPUT_DIR ${CMAKE_SOURCE_DIR}/src/libs/Path_Heading_planning)
set(BIN_OUTPUT_DIR ${CMAKE_SOURCE_DIR}/bin/Path_Heading_planning/example)

file(MAKE_DIRECTORY 
    ${LIB_OUTPUT_DIR}
    ${BIN_OUTPUT_DIR}
    ${CMAKE_SOURCE_DIR}/bin/Path_Heading_planning/example/image
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
    ${CMAKE_SOURCE_DIR}/src/Local_Path_Planning/include
    ${CMAKE_SOURCE_DIR}/src/Global_Path_planning/include
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

# 使用 link_directories() 指定库搜索路径
link_directories(${CMAKE_SOURCE_DIR}/src/libs/common)

#set(CMAKE_AUTOMOC ON)

#Dubins_Curve类的编译
set(Dubins_Curve_LIB_NAME Dubins_Curve)
add_library(${Dubins_Curve_LIB_NAME} SHARED)  
target_sources(${Dubins_Curve_LIB_NAME}  
    PRIVATE 
    src/Dubins_Curve.cpp 
)
target_include_directories(${Dubins_Curve_LIB_NAME}  
    PUBLIC include 
    PRIVATE 
    auto_driving_algorithm 
)
target_link_libraries(${Dubins_Curve_LIB_NAME} World_map Python3::Python Python3::NumPy) 
set(Dubins_Curve_DEMO_NAME Dubins_Curve_demo)
add_executable(${Dubins_Curve_DEMO_NAME} example/Dubins_Curve_demo.cpp)  
target_link_libraries(${Dubins_Curve_DEMO_NAME} ${Dubins_Curve_LIB_NAME} Python3::Python Python3::NumPy)  


#rs曲线类的编译
set(ReedsShepp_LIB_NAME ReedsShepp)
add_library(${ReedsShepp_LIB_NAME} SHARED)  
target_sources(${ReedsShepp_LIB_NAME}  
    PRIVATE 
    src/ReedsShepp.cpp 
)
target_include_directories(${ReedsShepp_LIB_NAME}  
    PUBLIC include 
    PRIVATE 
    auto_driving_algorithm 
)
target_link_libraries(${ReedsShepp_LIB_NAME} World_map Python3::Python Python3::NumPy) 
set(ReedsShepp_DEMO_NAME ReedsShepp_demo)
add_executable(${ReedsShepp_DEMO_NAME} example/ReedsShepp_demo.cpp)  
target_link_libraries(${ReedsShepp_DEMO_NAME} ${ReedsShepp_LIB_NAME} Python3::Python Python3::NumPy)  


#正确版本的RS曲线类编译
set(RS_CURVE_LIB_NAME rs_curve)
add_library(${RS_CURVE_LIB_NAME} SHARED)  
target_sources(${RS_CURVE_LIB_NAME}  
    PRIVATE 
    src/rs_curve.cpp 
)
target_include_directories(${RS_CURVE_LIB_NAME}  
    PUBLIC include 
    PRIVATE 
    auto_driving_algorithm 
)
target_link_libraries(${RS_CURVE_LIB_NAME} World_map Python3::Python Python3::NumPy) 
set(RS_CURVE_DEMO_NAME rs_curve_demo)
add_executable(${RS_CURVE_DEMO_NAME} example/rs_curve_demo.cpp)  
target_link_libraries(${RS_CURVE_DEMO_NAME} ${RS_CURVE_LIB_NAME} Python3::Python Python3::NumPy)  


#混合A*类的编译
# 使用 link_directories() 指定库搜索路径
link_directories(${CMAKE_SOURCE_DIR}/src/libs/Global_Path_planning)

set(HY_A_S_LIB_NAME HY_A_S)
add_library(${HY_A_S_LIB_NAME} SHARED)  
target_sources(${HY_A_S_LIB_NAME}  
    PRIVATE 
    src/H_A_S.cpp 
)
target_include_directories(${HY_A_S_LIB_NAME}  
    PUBLIC include 
    ${CMAKE_SOURCE_DIR}/src/Global_Path_planning/include
    PRIVATE 
    auto_driving_algorithm 
)
target_link_libraries(${HY_A_S_LIB_NAME} ObstacleGridMap rs_curve Dijkstra node3d Python3::Python Python3::NumPy) 
set(HY_A_S_DEMO_NAME HY_A_S_demo)
add_executable(${HY_A_S_DEMO_NAME} example/H_A_Star_demo.cpp)  
target_link_libraries(${HY_A_S_DEMO_NAME} ${HY_A_S_LIB_NAME} Python3::Python Python3::NumPy)  

add_executable(test_map example/test.cpp)
target_link_libraries(test_map ObstacleGridMap Python3::Python Python3::NumPy)


