cmake_minimum_required(VERSION 3.10)

# 基础项目设置
set(PACKAGE_NAME auto_driving_algorithm)
project(${PACKAGE_NAME} VERSION 1.0.0 LANGUAGES CXX)

# 输出目录设置
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# 编译标准设置
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译选项
add_compile_options(-Wall -Wextra -Wpedantic)

# 依赖查找
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

# 主接口库定义
add_library(${PACKAGE_NAME} INTERFACE)

# 包含目录设置
target_include_directories(${PACKAGE_NAME} INTERFACE
    ${CMAKE_SOURCE_DIR}/src/path_planning/common/include
    ${Python3_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OSQPEIGEN_INCLUDE_DIR}
)

# 链接库设置
target_link_libraries(${PACKAGE_NAME} INTERFACE
    Eigen3::Eigen
    Python3::Python
    Python3::Module
    OsqpEigen::OsqpEigen
)

# NumPy处理
if(Python3_NumPy_FOUND)
    target_include_directories(${PACKAGE_NAME} INTERFACE
        ${Python3_NumPy_INCLUDE_DIRS}
    )
    target_link_libraries(${PACKAGE_NAME} INTERFACE Python3::NumPy)
else()
    target_compile_definitions(${PACKAGE_NAME} INTERFACE WITHOUT_NUMPY)
endif()

# 安装配置
install(TARGETS ${PACKAGE_NAME} EXPORT ${PACKAGE_NAME}-targets)
install(DIRECTORY include/ DESTINATION include)

# 主可执行文件
add_executable(main src/main/main.cpp)
target_link_libraries(main PRIVATE ${PACKAGE_NAME})


# 添加子目录
#add_subdirectory(src/path_planning)

add_subdirectory(src/path_Tracking)

