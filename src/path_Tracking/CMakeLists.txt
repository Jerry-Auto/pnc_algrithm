set(LIB_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/path_Tracking)
set(BIN_OUTPUT_DIR ${CMAKE_SOURCE_DIR}/bin/path_Tracking)

file(MAKE_DIRECTORY 
    ${LIB_OUTPUT_DIR}
    ${BIN_OUTPUT_DIR}
)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIB_OUTPUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_OUTPUT_DIR})


# 控制算法演示程序函数定义
function(add_control_demo name)
    add_executable(${name}_demo ${ARGN})
    target_link_libraries(${name}_demo PRIVATE ${PACKAGE_NAME})
    target_include_directories(${name}_demo PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/utils
    )
endfunction()

# PID控制
add_control_demo(pid 
    PID/main.cpp 
    PID/PID_controller.cpp 
    utils/KinematicModel.cpp
)

# Pure Pursuit
add_control_demo(pure_pursuit 
    Pure_Pursuit/main.cpp  
    Pure_Pursuit/PurePursuit.cpp 
    utils/KinematicModel.cpp
)

# Stanley控制
add_control_demo(stanley 
    Stanley/main.cpp  
    Stanley/Stanley.cpp 
    utils/KinematicModel.cpp
)

# 后轮反馈控制
add_control_demo(rear_wheel_feedback 
    Rear_Wheel_Feedback/main.cpp  
    Rear_Wheel_Feedback/RearWheelFeedback.cpp 
    utils/MyReferencePath.cpp 
    utils/KinematicModel.cpp
)

# LQR控制
add_control_demo(lqr 
    LQR/main.cpp  
    LQR/LQRControl.cpp 
    utils/MyReferencePath.cpp 
    utils/KinematicModel.cpp
)

# MPC控制 (OSQP)
add_control_demo(mpc 
    MPC/main.cpp  
    MPC/MPCControl.cpp 
    utils/KinematicModel.cpp 
    utils/MyReferencePath.cpp
)
target_link_libraries(mpc_demo PRIVATE OsqpEigen::OsqpEigen)

# MPC控制 (IPOPT)
add_executable(mpc_speed_steel_demo 
    Model_Predictive_Speed_Steel_Control/main.cpp 
    Model_Predictive_Speed_Steel_Control/ModelPredictiveControl.cpp
)
target_link_libraries(mpc_speed_steel_demo PRIVATE 
    ipopt  
    ${PACKAGE_NAME}
)