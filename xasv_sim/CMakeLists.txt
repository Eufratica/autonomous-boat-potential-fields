cmake_minimum_required(VERSION 3.0.2)
project(xasv_sim)

#############################
## Catkin (ROS) dependency ##
#############################

find_package(catkin REQUIRED COMPONENTS
  roscpp
  xacro
)

#################################
## Gazebo / Eigen / Protobuf   ##
#################################

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)   # para Hydrodynamics / Wavefield

# PROTOBUF_IMPORT_DIRS must be defined before PROTOBUF_GENERATE_CPP
# (it is used internally by the FindProtobuf module)
set(PROTOBUF_IMPORT_DIRS "")
set(GAZEBO_MSG_INCLUDE_DIRS "")

# Discover Gazebo proto (.proto) and header (.pb.h) directories
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  # On Ubuntu with Gazebo classic we typically see something like /usr/include/gazebo-11
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

# Also let protoc find our local .proto files under plugins/msg
list(APPEND PROTOBUF_IMPORT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/plugins/msg")

message(STATUS "Gazebo msgs proto dir(s): ${PROTOBUF_IMPORT_DIRS}")
message(STATUS "Gazebo msgs (pb.h) dir: ${GAZEBO_MSG_INCLUDE_DIRS}")

find_package(Protobuf REQUIRED)

##################
## Build config ##
##################

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(${GAZEBO_CXX_FLAGS})

######################################
## catkin_package (export to ROS)   ##
######################################

catkin_package(
  CATKIN_DEPENDS roscpp xacro
  # Gazebo plugin libs are loaded by Gazebo at runtime,
  # not exported as catkin libraries.
)

##########################
## Protobuf: .proto -> .pb
##########################

# .proto files live in xasv_sim/plugins/msg/
set(PROTO_FILES
  plugins/msg/CommandMotorSpeed.proto
  plugins/msg/Wind.proto
  plugins/msg/MotorSpeed.proto
  plugins/msg/Float.proto
)

# This macro uses PROTOBUF_IMPORT_DIRS internally
set(Protobuf_IMPORT_DIRS ${PROTOBUF_IMPORT_DIRS})
set(PROTOBUF_IMPORT_DIRS ${PROTOBUF_IMPORT_DIRS})

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# Library with the messages used by the plugins (CommandMotorSpeed, Wind, etc.)
add_library(gazebo_plugin_msgs SHARED ${PROTO_SRCS})
target_link_libraries(gazebo_plugin_msgs ${Protobuf_LIBRARIES})
target_include_directories(gazebo_plugin_msgs PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}   # CommandMotorSpeed.pb.h, Wind.pb.h, etc.
  ${PROTOBUF_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}    # Gazebo’s own .pb.h (e.g. vector3d.pb.h)
)

#########################
## Includes / Link dirs ##
#########################

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}                    # generated .pb.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/plugins/include   # plugin headers (ArduPilotPlugin.hh, Wavefield*.hh, etc.)
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}                    # vector3d.pb.h
  ${PROTOBUF_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_LIBRARY_DIRS}
)

############################
## Hydrodynamics library  ##
############################
# Este bloco replica o que você tinha no CMake “migbot_plugins”:
# fornece as implementações de asv::Utilities, Wavefield, etc.

add_library(Hydrodynamics
  SHARED
    plugins/src/Gazebo.cc
    plugins/src/Geometry.cc
    plugins/src/PhysicalConstants.cc
    plugins/src/Physics.cc
    plugins/src/Utilities.cc
    plugins/src/Wavefield.cc
    plugins/src/WavefieldEntity.cc
)

target_link_libraries(Hydrodynamics
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

# Lista auxiliar, igual ao CMake antigo
set(WAVE_GAZEBO_LIBRARIES_LIST Hydrodynamics)

#########################
## Robot-level plugins ##
#########################

# Motor model  -> libgazebo_motor_model.so
add_library(gazebo_motor_model SHARED
  plugins/src/gazebo_motor_model.cpp
)
target_link_libraries(gazebo_motor_model
  ${GAZEBO_LIBRARIES}
  gazebo_plugin_msgs
  ${catkin_LIBRARIES}
)

# USV dynamics -> libgazebo_usv_dynamics_plugin.so
add_library(gazebo_usv_dynamics_plugin SHARED
  plugins/src/gazebo_usv_dynamics_plugin.cpp
)
target_link_libraries(gazebo_usv_dynamics_plugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Buoyancy     -> libbuoyancy_gazebo_plugin.so
add_library(buoyancy_gazebo_plugin SHARED
  plugins/src/buoyancy_gazebo_plugin.cc
  plugins/src/shape_volume.cc
  plugins/src/polyhedron_volume.cc
)
target_link_libraries(buoyancy_gazebo_plugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# waypoint_markers helper lib -> libwaypoint_markers.so
add_library(waypoint_markers SHARED
  plugins/src/waypoint_markers.cc
)
target_link_libraries(waypoint_markers
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Follow plugin -> libfollow_plugin.so
add_library(follow_plugin SHARED
  plugins/src/follow_plugin.cc
)
target_link_libraries(follow_plugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  waypoint_markers
)
add_dependencies(follow_plugin waypoint_markers)

# Wind plugin -> libgazebo_wind_plugin.so
add_library(gazebo_wind_plugin SHARED
  plugins/src/gazebo_wind_plugin.cpp
)
target_link_libraries(gazebo_wind_plugin
  ${GAZEBO_LIBRARIES}
  gazebo_plugin_msgs
  ${catkin_LIBRARIES}
)

# ArduPilotPlugin -> libArduPilotPlugin.so
add_library(ArduPilotPlugin SHARED
  plugins/src/ArduPilotPlugin.cc
)
target_link_libraries(ArduPilotPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)

#########################
## World-level plugins ##
#########################

# TrunkScaler -> libTrunkScaler.so
add_library(TrunkScaler SHARED
  plugins/src/TrunkScaler.cc
)
target_link_libraries(TrunkScaler
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)

# BuoySpawner -> libBuoySpawner.so
add_library(BuoySpawner SHARED
  plugins/src/BuoySpawner.cc
)
target_link_libraries(BuoySpawner
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)

# WorldGpsAlignMavlink -> libWorldGpsAlignMavlink.so
add_library(WorldGpsAlignMavlink SHARED
  plugins/src/WorldGpsAlignMavlink.cc
)
target_link_libraries(WorldGpsAlignMavlink
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)

# WavefieldModelPlugin -> libWavefieldModelPlugin.so
add_library(WavefieldModelPlugin SHARED
  plugins/src/WavefieldModelPlugin.cc
)
target_link_libraries(WavefieldModelPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${WAVE_GAZEBO_LIBRARIES_LIST}
)

# WavefieldVisualPlugin -> libWavefieldVisualPlugin.so
add_library(WavefieldVisualPlugin SHARED
  plugins/src/WavefieldVisualPlugin.cc
)
target_link_libraries(WavefieldVisualPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${WAVE_GAZEBO_LIBRARIES_LIST}
)

######################################
## Gazebo plugin install directory  ##
######################################

# Detect a reasonable install directory for Gazebo plugins.
# If GAZEBO_PLUGIN_PATH is set, use its first entry; otherwise fall back
# to ${CMAKE_INSTALL_PREFIX}/lib (or adjust to ${CATKIN_PACKAGE_LIB_DESTINATION}
# if you prefer everything inside the catkin install space).
if(NOT DEFINED GAZEBO_PLUGIN_INSTALL_DIR)
  if(DEFINED ENV{GAZEBO_PLUGIN_PATH} AND NOT "$ENV{GAZEBO_PLUGIN_PATH}" STREQUAL "")
    string(REPLACE ":" ";" GZ_PLUG_PATH_LIST "$ENV{GAZEBO_PLUGIN_PATH}")
    list(GET GZ_PLUG_PATH_LIST 0 GAZEBO_PLUGIN_INSTALL_DIR)
  else()
    set(GAZEBO_PLUGIN_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib")
  endif()
endif()

message(STATUS "Gazebo plugin install dir: ${GAZEBO_PLUGIN_INSTALL_DIR}")

install(
  TARGETS
    gazebo_plugin_msgs
    Hydrodynamics
    gazebo_motor_model
    gazebo_usv_dynamics_plugin
    buoyancy_gazebo_plugin
    waypoint_markers
    follow_plugin
    gazebo_wind_plugin
    ArduPilotPlugin
    TrunkScaler
    BuoySpawner
    WorldGpsAlignMavlink
    WavefieldModelPlugin
    WavefieldVisualPlugin
  LIBRARY DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR}
)

##########################################
## Install models / URDF / config / launch
##########################################

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

