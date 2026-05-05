cmake_minimum_required(VERSION 2.8.3)
project(migbot_gazebo)



find_package(catkin REQUIRED COMPONENTS
  xacro
  roscpp
)


catkin_package(
 # INCLUDE_DIRS include 
  CATKIN_DEPENDS xacro roscpp 
)

# Plugins require c++11
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()



include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
)
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}  
  ${GAZEBO_LIBRARY_DIRS}    
)



set(XACRO_INORDER)
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "kinetic")
    set(XACRO_INORDER INORDER)
  endif()
endif()

# Generate urdf files from xacro and install
#xacro_add_files(
#  urdf/migbot_gazebo_ardupilot_scaled.urdf.xacro
#  ${XACRO_INORDER} INSTALL DESTINATION urdf
#)

# Install meshes and textures
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

# Install xacro files / macros
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
