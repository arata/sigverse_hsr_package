cmake_minimum_required(VERSION 2.8.3)
project(sigverse_hsr_package)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  hardware_interface
  transmission_interface
  controller_manager
  actionlib
  message_generation
  actionlib_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sigverse_hsr_arm_controller src/hsr_arm_controller.cpp)
target_link_libraries(sigverse_hsr_arm_controller ${catkin_LIBRARIES})

add_executable(sigverse_hsr_gripper_controller src/hsr_gripper_controller.cpp)
target_link_libraries(sigverse_hsr_gripper_controller ${catkin_LIBRARIES})

add_executable(sigverse_hsr_head_controller src/hsr_head_controller.cpp)
target_link_libraries(sigverse_hsr_head_controller ${catkin_LIBRARIES})

add_executable(sigverse_hsr_omni_controller src/hsr_omni_controller.cpp)
target_link_libraries(sigverse_hsr_omni_controller ${catkin_LIBRARIES})

add_executable(sigverse_hsr_list_controllers src/list_controllers.cpp)
target_link_libraries(sigverse_hsr_list_controllers ${catkin_LIBRARIES})
