cmake_minimum_required(VERSION 2.8.3)
project(remy)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  
  moveit_commander moveit_core
  moveit_fake_controller_manager
  moveit_msgs
  moveit_planners_ompl
  moveit_ros_control_interface
  moveit_ros_manipulation
  moveit_ros_move_group
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_robot_interaction
  moveit_ros_visualization
  moveit_ros_warehouse
  moveit_setup_assistant
  moveit_simple_controller_manager
  rviz_visual_tools
  moveit_visual_tools
  rqt_moveit
  roscpp
  rospy
  interactive_markers
  actionlib_msgs
  message_generation
  geometry_msgs
)


add_service_files(
  FILES
  ObjectDetection.srv
)

add_action_files(
  DIRECTORY action
  FILES
  Pick.action
  Plan.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_executable(cook src/cook.cpp)
add_executable(chef src/chef.cpp)
add_executable(object_detection src/object_detection.cpp)

add_dependencies(cook ${pick_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})
add_dependencies(chef ${pick_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})
add_dependencies(object_detection ${catkin_EXPORTED_TARGETS})

target_link_libraries(cook ${catkin_LIBRARIES})
target_link_libraries(chef ${catkin_LIBRARIES})
target_link_libraries(object_detection ${catkin_LIBRARIES})
