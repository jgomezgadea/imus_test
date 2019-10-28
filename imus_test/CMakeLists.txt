cmake_minimum_required(VERSION 2.8.3)
project(imus_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  dynamixel_controllers
)


catkin_python_setup()

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#)

## Generate services in the 'srv' folder
#add_service_files(
#   FILES  
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   imus_test_msgs
   dynamixel_controllers
)

catkin_package(

)

