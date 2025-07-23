cmake_minimum_required(VERSION 3.10)
project(auto_driving_algorithm)

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

#include(GNUInstallDirs)
set(PACKAGE_NAME auto_driving_algorithm)


# Library target
add_library(auto_driving_algorithm INTERFACE)


# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(auto_driving_algorithm INTERFACE
        Python3::Python
        Python3::Module
        )
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
    target_link_libraries(auto_driving_algorithm INTERFACE
            Python3::NumPy
            )
else()
    target_compile_definitions(auto_driving_algorithm INTERFACE WITHOUT_NUMPY)
endif()
install(
        TARGETS auto_driving_algorithm
        EXPORT install_targets
)



#包含公共头文件目录
include_directories(/usr/include/python3.10)
include_directories(/usr/local/lib/python3.10/dist-packages/numpy/core/include)
include_directories(${CMAKE_SOURCE_DIR}/src/libs/common/include)

find_package(OsqpEigen REQUIRED)
include_directories(${OSQPEIGEN_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


add_executable(main src/main/main.cpp)
target_link_libraries(main PRIVATE auto_driving_algorithm)


# 添加子目录
add_subdirectory(src/Global_Path_planning)

add_subdirectory(src/Local_Path_Planning)

add_subdirectory(src/Randomize_Samples_planning)

add_subdirectory(src/Path_Heading_planning)