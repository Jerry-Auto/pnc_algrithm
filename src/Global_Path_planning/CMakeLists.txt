cmake_minimum_required(VERSION 3.10)
project(auto_driving_algorithm)


set(PACKAGE_NAME auto_driving_algorithm)

set(CMAKE_CXX_STANDARD 14)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# 设置编译模式Debug Realease
set(CMAKE_BUILD_TYPE Debug)

# 创建输出目录
set(LIB_OUTPUT_DIR ${CMAKE_SOURCE_DIR}/src/libs/Global_Path_planning)
set(BIN_OUTPUT_DIR ${CMAKE_SOURCE_DIR}/bin/Global_Path_planning/example)

file(MAKE_DIRECTORY 
    ${LIB_OUTPUT_DIR}
    ${BIN_OUTPUT_DIR}
    ${CMAKE_SOURCE_DIR}/bin/Global_Path_planning/example/image
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


# graph邻接矩阵类的编译
set(GRAPH_LIB_NAME graph)
add_library(${GRAPH_LIB_NAME} SHARED)  

target_sources(${GRAPH_LIB_NAME}  
    PRIVATE 
    src/graph.cpp
)
target_include_directories(${GRAPH_LIB_NAME}  
    PUBLIC
    include 
    PRIVATE 
    auto_driving_algorithm 
)
# 设置库输出目录
#set_target_properties(${GRAPH_LIB_NAME} PROPERTIES
#    LIBRARY_OUTPUT_DIRECTORY ${LIB_OUTPUT_DIR}
#)

set(GRAPH_DEMO_NAME graph_demo)
add_executable(${GRAPH_DEMO_NAME})  

target_sources(${GRAPH_DEMO_NAME} 
 PRIVATE 
    example/graph_demo.cpp 
)
target_link_libraries(${GRAPH_DEMO_NAME} ${GRAPH_LIB_NAME})  

# 设置可执行文件输出目录（修正拼写错误）
#set_target_properties(${GRAPH_DEMO_NAME} PROPERTIES
#    RUNTIME_OUTPUT_DIRECTORY ${BIN_OUTPUT_DIR}  
#)



# 障碍物栅格地图类的编译
set(OBSTACLE_GRID_MAP_LIB_NAME ObstacleGridMap)
add_library(${OBSTACLE_GRID_MAP_LIB_NAME} SHARED)  

target_sources(${OBSTACLE_GRID_MAP_LIB_NAME}  
    PRIVATE 
    src/ObstacleGridMap.cpp
)

target_include_directories(${OBSTACLE_GRID_MAP_LIB_NAME}  
    PUBLIC
        include 
    PRIVATE 
    auto_driving_algorithm 
)

target_link_libraries(${OBSTACLE_GRID_MAP_LIB_NAME} PRIVATE Python3::Python)

# 设置库输出目录
# set_target_properties(${OBSTACLE_GRID_MAP_LIB_NAME} PROPERTIES
#     LIBRARY_OUTPUT_DIRECTORY ${LIB_OUTPUT_DIR}
# )

set(OBSTACLE_GRID_MAP_DEMO_NAME ObstacleGridMap_demo)
add_executable(${OBSTACLE_GRID_MAP_DEMO_NAME})  

target_sources(${OBSTACLE_GRID_MAP_DEMO_NAME} 
    PRIVATE 
    example/ObstacleGridMap_demo.cpp 
)

target_link_libraries(${OBSTACLE_GRID_MAP_DEMO_NAME} 
    ${OBSTACLE_GRID_MAP_LIB_NAME}
    Python3::Python
)

# 设置可执行文件输出目录
# set_target_properties(${OBSTACLE_GRID_MAP_DEMO_NAME} PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${BIN_OUTPUT_DIR}  
# )


# matplot类的编译
set(MATPLOT_DEMO_NAME matplot_demo)
add_executable(${MATPLOT_DEMO_NAME})  

target_sources(${MATPLOT_DEMO_NAME} 
    PRIVATE 
    example/matplot_demo.cpp 
)
target_link_libraries(${MATPLOT_DEMO_NAME} 
    Python3::Python
)
# 设置可执行文件输出目录
# set_target_properties(${MATPLOT_DEMO_NAME} PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${BIN_OUTPUT_DIR}  
# )



# dijkstra算法类的编译
set(DIJKSTRA_LIB_NAME Dijkstra)
add_library(${DIJKSTRA_LIB_NAME} SHARED)  

target_sources(${DIJKSTRA_LIB_NAME}  
    PRIVATE 
    src/Dijkstra.cpp
)

target_include_directories(${DIJKSTRA_LIB_NAME}  
    PUBLIC
        include 
    PRIVATE 
    auto_driving_algorithm 
)

target_link_libraries(${DIJKSTRA_LIB_NAME} PRIVATE Python3::Python ${OBSTACLE_GRID_MAP_LIB_NAME})



set(DIJKSTRA_DEMO_NAME Dijkstra_demo)
add_executable(${DIJKSTRA_DEMO_NAME})  

target_sources(${DIJKSTRA_DEMO_NAME} 
    PRIVATE 
    example/Dijkstra_demo.cpp 
)
target_link_libraries(${DIJKSTRA_DEMO_NAME} 
    ${DIJKSTRA_LIB_NAME}
    ${OBSTACLE_GRID_MAP_LIB_NAME}
)


# 蚁群算法类的编译
set(ANTCOLONY_LIB_NAME AntColony)
add_library(${ANTCOLONY_LIB_NAME} SHARED)  

target_sources(${ANTCOLONY_LIB_NAME}  
    PRIVATE 
    src/AntColony.cpp
)

target_include_directories(${ANTCOLONY_LIB_NAME}  
    PUBLIC
        include 
    PRIVATE 
    auto_driving_algorithm 
)

target_link_libraries(${ANTCOLONY_LIB_NAME} PRIVATE Python3::Python ${OBSTACLE_GRID_MAP_LIB_NAME})



set(ANTCOLONY_DEMO_NAME AntColony_demo)
add_executable(${ANTCOLONY_DEMO_NAME})  

target_sources(${ANTCOLONY_DEMO_NAME} 
    PRIVATE 
    example/AntColony_demo.cpp
)
target_link_libraries(${ANTCOLONY_DEMO_NAME} 
    ${ANTCOLONY_LIB_NAME}
    ${OBSTACLE_GRID_MAP_LIB_NAME}
)

# add_executable(ant_colony2_demo example/antcolony_demo2.cpp)

# 动态规划算法类的编译
set(DynamicProgram_LIB_NAME DynamicProgram)
add_library(${DynamicProgram_LIB_NAME} SHARED)  

target_sources(${DynamicProgram_LIB_NAME}  
    PRIVATE 
    src/DynamicProgram.cpp
)

target_include_directories(${DynamicProgram_LIB_NAME}  
    PUBLIC
        include 
    PRIVATE 
    auto_driving_algorithm 
)

target_link_libraries(${DynamicProgram_LIB_NAME} PRIVATE Python3::Python ${OBSTACLE_GRID_MAP_LIB_NAME})



set(DynamicProgram_DEMO_NAME DynamicProgram_demo)
add_executable(${DynamicProgram_DEMO_NAME})  

target_sources(${DynamicProgram_DEMO_NAME} 
    PRIVATE 
    example/DynamicProgram_demo.cpp 
)
target_link_libraries(${DynamicProgram_DEMO_NAME} 
    ${DynamicProgram_LIB_NAME}
    ${OBSTACLE_GRID_MAP_LIB_NAME}
)



# AStar算法类的编译
set(AStar_LIB_NAME AStar)
add_library(${AStar_LIB_NAME} SHARED)  

target_sources(${AStar_LIB_NAME}  
    PRIVATE 
    src/AStar.cpp
)

target_include_directories(${AStar_LIB_NAME}  
    PUBLIC
        include 
    PRIVATE 
    auto_driving_algorithm 
)

target_link_libraries(${AStar_LIB_NAME} PRIVATE Python3::Python ${OBSTACLE_GRID_MAP_LIB_NAME})



set(AStar_DEMO_NAME AStar_demo)
add_executable(${AStar_DEMO_NAME})  

target_sources(${AStar_DEMO_NAME} 
    PRIVATE 
    example/AStar_demo.cpp 
)
target_link_libraries(${AStar_DEMO_NAME} 
    ${AStar_LIB_NAME}
    ${OBSTACLE_GRID_MAP_LIB_NAME}
)
