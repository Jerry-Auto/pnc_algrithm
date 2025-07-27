

set(LIB_OUTPUT_DIR ${LIB_PRE_DIR}/Global_Path_planning)
set(BIN_OUTPUT_DIR ${BIN_PRE_DIR}/Global_Path_planning)

file(MAKE_DIRECTORY 
    ${LIB_OUTPUT_DIR}
    ${BIN_OUTPUT_DIR}
    ${BIN_OUTPUT_DIR}/image
)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIB_OUTPUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_OUTPUT_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)



# dijkstra算法类的编译
set(DIJKSTRA_LIB_NAME Dijkstra)
add_library(${DIJKSTRA_LIB_NAME} SHARED)  

target_sources(${DIJKSTRA_LIB_NAME}  
    PRIVATE 
    src/Dijkstra.cpp
)

target_include_directories(${DIJKSTRA_LIB_NAME}  
    PRIVATE 
    auto_driving_algorithm 
)

target_link_libraries(${DIJKSTRA_LIB_NAME} PRIVATE Python3::Python ObstacleGridMap)



set(DIJKSTRA_DEMO_NAME Dijkstra_demo)
add_executable(${DIJKSTRA_DEMO_NAME})  

target_sources(${DIJKSTRA_DEMO_NAME} 
    PRIVATE 
    example/Dijkstra_demo.cpp 
)
target_link_libraries(${DIJKSTRA_DEMO_NAME} 
    ${DIJKSTRA_LIB_NAME}
    ObstacleGridMap
)


# 蚁群算法类的编译
set(ANTCOLONY_LIB_NAME AntColony)
add_library(${ANTCOLONY_LIB_NAME} SHARED)  

target_sources(${ANTCOLONY_LIB_NAME}  
    PRIVATE 
    src/AntColony.cpp
)

target_include_directories(${ANTCOLONY_LIB_NAME}  
    PRIVATE 
    auto_driving_algorithm 
)

target_link_libraries(${ANTCOLONY_LIB_NAME} PRIVATE Python3::Python ObstacleGridMap)



set(ANTCOLONY_DEMO_NAME AntColony_demo)
add_executable(${ANTCOLONY_DEMO_NAME})  

target_sources(${ANTCOLONY_DEMO_NAME} 
    PRIVATE 
    example/AntColony_demo.cpp
)
target_link_libraries(${ANTCOLONY_DEMO_NAME} 
    ${ANTCOLONY_LIB_NAME}
    ObstacleGridMap
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
    PRIVATE 
    auto_driving_algorithm 
)

target_link_libraries(${DynamicProgram_LIB_NAME} PRIVATE Python3::Python ObstacleGridMap)



set(DynamicProgram_DEMO_NAME DynamicProgram_demo)
add_executable(${DynamicProgram_DEMO_NAME})  

target_sources(${DynamicProgram_DEMO_NAME} 
    PRIVATE 
    example/DynamicProgram_demo.cpp 
)
target_link_libraries(${DynamicProgram_DEMO_NAME} 
    ${DynamicProgram_LIB_NAME}
    ObstacleGridMap
)



# AStar算法类的编译
set(AStar_LIB_NAME AStar)
add_library(${AStar_LIB_NAME} SHARED)  

target_sources(${AStar_LIB_NAME}  
    PRIVATE 
    src/AStar.cpp
)

target_include_directories(${AStar_LIB_NAME}  
    PRIVATE 
    auto_driving_algorithm 
)

target_link_libraries(${AStar_LIB_NAME} PRIVATE Python3::Python ObstacleGridMap)



set(AStar_DEMO_NAME AStar_demo)
add_executable(${AStar_DEMO_NAME})  

target_sources(${AStar_DEMO_NAME} 
    PRIVATE 
    example/AStar_demo.cpp 
)
target_link_libraries(${AStar_DEMO_NAME} 
    ${AStar_LIB_NAME}
    ObstacleGridMap
)
