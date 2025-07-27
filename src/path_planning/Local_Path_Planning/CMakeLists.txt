

set(LIB_OUTPUT_DIR ${LIB_PRE_DIR}/Local_Path_Planning)
set(BIN_OUTPUT_DIR ${BIN_PRE_DIR}/Local_Path_Planning)

file(MAKE_DIRECTORY 
    ${LIB_OUTPUT_DIR}
    ${BIN_OUTPUT_DIR}
    ${BIN_OUTPUT_DIR}/image
)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIB_OUTPUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_OUTPUT_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)



# 多项式曲线（poly_algorithm）类的编译
set(POLY_LIB_NAME PolynomialAlgorithm)
add_library(${POLY_LIB_NAME} SHARED src/PolynomialAlgorithm.cpp)  
target_include_directories(${POLY_LIB_NAME}  
    PRIVATE 
    auto_driving_algorithm 
)
target_link_libraries(${POLY_LIB_NAME} Python3::Python) 
set(POLY_DEMO_NAME PolynomialAlgorithm_demo)
add_executable(${POLY_DEMO_NAME} example/PolynomialAlgorithm_demo.cpp)  
target_link_libraries(${POLY_DEMO_NAME} ${POLY_LIB_NAME} World_map Python3::Python Python3::NumPy)




# B样条曲线（B_spline）类的编译
set(B_SPLINE_LIB_NAME B_spline)
add_library(${B_SPLINE_LIB_NAME} SHARED src/B_spline.cpp)  
target_include_directories(${B_SPLINE_LIB_NAME}  
    PRIVATE 
    auto_driving_algorithm 
)
target_link_libraries(${B_SPLINE_LIB_NAME} Python3::Python) 
set(B_SPLINE_DEMO_NAME B_spline_demo)
add_executable(${B_SPLINE_DEMO_NAME} example/B_spline_demo.cpp)  
target_link_libraries(${B_SPLINE_DEMO_NAME} ${B_SPLINE_LIB_NAME} World_map Python3::Python Python3::NumPy)

# DWA类的编译
set(DWA_LIB_NAME DWA)
add_library(${DWA_LIB_NAME} SHARED src/DWA_planner.cpp)  
target_include_directories(${DWA_LIB_NAME}  
    PRIVATE 
    auto_driving_algorithm 
)
target_link_libraries(${DWA_LIB_NAME} World_map Python3::Python) 
set(DWA_DEMO_NAME DWA_demo)
add_executable(${DWA_DEMO_NAME} example/DWA_planner_demo.cpp)  
target_link_libraries(${DWA_DEMO_NAME} ${DWA_LIB_NAME} World_map Python3::Python Python3::NumPy)

# DWA动力学模型类的编译
set(DWA_DY_LIB_NAME DWA_DY)
add_library(${DWA_DY_LIB_NAME} SHARED src/DWA_dy_planner.cpp)  
target_include_directories(${DWA_DY_LIB_NAME}  
    PRIVATE 
    auto_driving_algorithm 
)
target_link_libraries(${DWA_DY_LIB_NAME} World_map Python3::Python) 
set(DWA_DY_DEMO_NAME DWA_dy_demo)
add_executable(${DWA_DY_DEMO_NAME} example/DWA_dy_planner_demo.cpp)  
target_link_libraries(${DWA_DY_DEMO_NAME} ${DWA_DY_LIB_NAME} World_map Python3::Python Python3::NumPy)

# 人工势场法类的编译
set(APF_LIB_NAME APF)
add_library(${APF_LIB_NAME} SHARED src/Artificial_potential_field.cpp)  
target_include_directories(${APF_LIB_NAME}  
    PRIVATE 
    auto_driving_algorithm 
)
target_link_libraries(${APF_LIB_NAME} World_map Python3::Python) 
set(APF_DEMO_NAME APF_demo)
add_executable(${APF_DEMO_NAME} example/Artificial_potential_field_demo.cpp)  
target_link_libraries(${APF_DEMO_NAME} ${APF_LIB_NAME} Python3::Python Python3::NumPy)


