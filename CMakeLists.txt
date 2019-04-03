cmake_minimum_required(VERSION 2.8.3)
project(remy)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# find_package(Eigen3 REQUIRED)

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
  TwoInts.srv
  ObjectDetection.srv
)

add_action_files(
  DIRECTORY action
  FILES
  Fibonacci.action
  Pick.action
)


generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)


# add_executable(planner src/robot_planner.cpp)
# add_executable(action_server src/action_server.cpp)
add_executable(pick src/pick.cpp)
# add_executable(action_client src/action_client.cpp)
# add_executable(object_detection src/object_detection.cpp)
# add_executable(srv_client src/srv_client.cpp)
# add_executable(my_planning_scene_api src/my_planning_scene_api)
# add_executable(my_planning src/my_planning)

# add_dependencies(planner ${catkin_EXPORTED_TARGETS})
# add_dependencies(action_server ${catkin_EXPORTED_TARGETS})
add_dependencies(pick ${catkin_EXPORTED_TARGETS})
# add_dependencies(action_client ${catkin_EXPORTED_TARGETS})
# add_dependencies(object_detection ${catkin_EXPORTED_TARGETS})
# add_dependencies(srv_client ${catkin_EXPORTED_TARGETS})
# add_dependencies(my_planning_scene_api ${catkin_EXPORTED_TARGETS})
# add_dependencies(my_planning ${catkin_EXPORTED_TARGETS})

# target_link_libraries(planner ${catkin_LIBRARIES})
# target_link_libraries(action_server ${catkin_LIBRARIES})
target_link_libraries(pick ${catkin_LIBRARIES})
# target_link_libraries(action_client ${catkin_LIBRARIES})
# target_link_libraries(object_detection ${catkin_LIBRARIES})
# target_link_libraries(srv_client ${catkin_LIBRARIES})
# target_link_libraries(my_collision ${catkin_LIBRARIES})
# target_link_libraries(my_planning_scene_api ${catkin_LIBRARIES})
# target_link_libraries(my_planning ${catkin_LIBRARIES})