cmake_minimum_required(VERSION 3.10)
project(auto_driving_algorithm)


set(PACKAGE_NAME auto_driving_algorithm)

set(CMAKE_CXX_STANDARD 17)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# 设置编译模式Debug Realease
set(CMAKE_BUILD_TYPE Debug)

# 创建输出目录
set(LIB_OUTPUT_DIR ${CMAKE_SOURCE_DIR}/src/libs/Randomize_Samples_planning)
set(BIN_OUTPUT_DIR ${CMAKE_SOURCE_DIR}/bin/Randomize_Samples_planning/example)

file(MAKE_DIRECTORY 
    ${LIB_OUTPUT_DIR}
    ${BIN_OUTPUT_DIR}
    ${CMAKE_SOURCE_DIR}/bin/Randomize_Samples_planning/example/image
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
link_directories(${CMAKE_SOURCE_DIR}/src/libs/Local_Path_Planning)


#set(CMAKE_AUTOMOC ON)

#PRM类的编译
set(PRM_LIB_NAME PRM)
add_library(${PRM_LIB_NAME} SHARED)  
target_sources(${PRM_LIB_NAME}  
    PRIVATE 
    src/Probability_Road_map.cpp 
)
target_include_directories(${PRM_LIB_NAME}  
    PUBLIC include 
    PRIVATE 
    auto_driving_algorithm 
)
target_link_libraries(${PRM_LIB_NAME} World_map Python3::Python Python3::NumPy) 
set(PRM_DEMO_NAME PRM_demo)
add_executable(${PRM_DEMO_NAME} example/Probability_Road_map_demo.cpp)  
target_link_libraries(${PRM_DEMO_NAME} ${PRM_LIB_NAME} Python3::Python Python3::NumPy)  

#RRT类的编译
set(RRT_LIB_NAME RRT)
add_library(${RRT_LIB_NAME} SHARED)  
target_sources(${RRT_LIB_NAME}  
    PRIVATE 
    src/Rapidly_exploring_Random_Tree.cpp 
)
target_include_directories(${RRT_LIB_NAME}  
    PUBLIC include 
    PRIVATE 
    auto_driving_algorithm 
)
target_link_libraries(${RRT_LIB_NAME} World_map Python3::Python Python3::NumPy) 
set(RRT_DEMO_NAME RRT_demo)
add_executable(${RRT_DEMO_NAME} example/Rapidly_exploring_Random_Tree_demo.cpp)  
target_link_libraries(${RRT_DEMO_NAME} ${RRT_LIB_NAME} Python3::Python Python3::NumPy) 

#RRT_star类的编译
set(RRT_star_LIB_NAME RRT_star)
add_library(${RRT_star_LIB_NAME} SHARED)  
target_sources(${RRT_star_LIB_NAME}  
    PRIVATE 
    src/RRT_star.cpp 
)
target_include_directories(${RRT_star_LIB_NAME}  
    PUBLIC include 
    PRIVATE 
    auto_driving_algorithm 
)
target_link_libraries(${RRT_star_LIB_NAME} ${RRT_LIB_NAME} World_map Python3::Python Python3::NumPy) 
set(RRT_star_DEMO_NAME RRT_star_demo)
add_executable(${RRT_star_DEMO_NAME} example/RRT_star_demo.cpp)  
target_link_libraries(${RRT_star_DEMO_NAME} ${RRT_star_LIB_NAME} Python3::Python Python3::NumPy) 


#RRT_connect类的编译
set(RRT_connect_LIB_NAME RRT_connect)
add_library(${RRT_connect_LIB_NAME} SHARED)  
target_sources(${RRT_connect_LIB_NAME}  
    PRIVATE 
    src/RRT_connect.cpp 
)
target_include_directories(${RRT_connect_LIB_NAME}  
    PUBLIC include 
    PRIVATE 
    auto_driving_algorithm 
)
target_link_libraries(${RRT_connect_LIB_NAME}  World_map Python3::Python Python3::NumPy) 
set(RRT_connect_DEMO_NAME RRT_connect_demo)
add_executable(${RRT_connect_DEMO_NAME} example/RRT_connect_demo.cpp)  
target_link_libraries(${RRT_connect_DEMO_NAME} ${RRT_connect_LIB_NAME} Python3::Python Python3::NumPy) 