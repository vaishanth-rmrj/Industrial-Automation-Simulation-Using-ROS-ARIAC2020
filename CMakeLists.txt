cmake_minimum_required(VERSION 3.0.2)
project(group2_rwa4)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nist_gear
  roscpp
  rospy
  sensor_msgs
  std_srvs
  trajectory_msgs
  message_generation
  control_msgs
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools

)

find_package(Boost REQUIRED system filesystem date_time thread)


add_message_files(
  FILES
  Order.msg
  KittingShipment.msg
  AssemblyShipment.msg
  Product.msg
  IncompleteOrder.msg  
  KittingLocation.msg
  FaultyPartPose.msg
  AgvKittingShipmentDetails.msg
  Task.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  check_exists.srv
  check_agv_faulty_parts.srv
  agv_location.srv
  list_order_details.srv
  list_all_parts.srv
  check_part_pose.srv  
  order_kitting_shipment_details.srv
  kitting_part_details.srv
  dispose_faulty_part.srv
  order_completion_status.srv
  empty_slot_pose.srv

  # testing
  assembly_part_details.srv
  order_assembly_shipment_details.srv
  assembly_task.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES group2_rwa1
#  CATKIN_DEPENDS nist_gear roscpp rospy sensor_msgs std_srvs trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)




# add_executable(rwa4_node src/nodes/rwa4_node.cpp src/main_handler.cpp src/order_handler.cpp src/bins_handler.cpp src/agv.cpp src/kitting_handler.cpp src/kitting_robot.cpp src/conveyor.cpp src/gantry_robot.cpp src/utils.cpp)
add_executable(rwa4_node 
                  src/nodes/rwa4_node.cpp 
                  src/main_handler.cpp 
                  src/order_handler.cpp 
                  src/bins_handler.cpp 
                  src/agv.cpp 
                  src/kitting_handler.cpp 
                  src/kitting_robot.cpp 
                  src/conveyor.cpp 
                  src/gantry_robot.cpp 
                  src/utils.cpp 
                  src/agv.cpp 
                  src/assembly_handler.cpp)

add_dependencies(rwa4_node group2_rwa4_gencpp)


target_link_libraries(rwa4_node
${catkin_LIBRARIES}
)




#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  # FILES_MATCHING PATTERN "*.h"
  # PATTERN ".svn" EXCLUDE
)


