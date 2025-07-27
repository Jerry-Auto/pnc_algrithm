

set(LIB_OUTPUT_DIR ${LIB_PRE_DIR}/Randomize_Samples_planning)
set(BIN_OUTPUT_DIR ${BIN_PRE_DIR}/Randomize_Samples_planning)

file(MAKE_DIRECTORY 
    ${LIB_OUTPUT_DIR}
    ${BIN_OUTPUT_DIR}
    ${BIN_OUTPUT_DIR}/image
)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIB_OUTPUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_OUTPUT_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


#PRM类的编译
set(PRM_LIB_NAME PRM)
add_library(${PRM_LIB_NAME} SHARED)  
target_sources(${PRM_LIB_NAME}  
    PRIVATE 
    src/Probability_Road_map.cpp 
)
target_include_directories(${PRM_LIB_NAME}  
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
    PRIVATE 
    auto_driving_algorithm 
)
target_link_libraries(${RRT_connect_LIB_NAME}  World_map Python3::Python Python3::NumPy) 
set(RRT_connect_DEMO_NAME RRT_connect_demo)
add_executable(${RRT_connect_DEMO_NAME} example/RRT_connect_demo.cpp)  
target_link_libraries(${RRT_connect_DEMO_NAME} ${RRT_connect_LIB_NAME} Python3::Python Python3::NumPy) 