
set(CMAKE_CXX_STANDARD 17)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# 设置编译模式Debug Realease
set(CMAKE_BUILD_TYPE Debug)

# 包含目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include
)
link_directories(${LIB_PRE_DIR}/common)
link_libraries(${PACKAGE_NAME})

set(BIN_PRE_DIR ${CMAKE_SOURCE_DIR}/bin/path_planning)
set(LIB_PRE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs/path_planning)




# 添加子目录
add_subdirectory(Global_Path_planning)

add_subdirectory(Local_Path_Planning)

add_subdirectory(Randomize_Samples_planning)

add_subdirectory(Path_Heading_planning)

add_subdirectory(common)