cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(trunk_plugins)

#######################
## Find Dependencies ##
#######################

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
# PROTOBUF_IMPORT_DIRS precisa estar definido antes de PROTOBUF_GENERATE_CPP
# (ele é usado internamente pelo módulo FindProtobuf)
set(PROTOBUF_IMPORT_DIRS "")
set(GAZEBO_MSG_INCLUDE_DIRS "")

# Descobre os diretórios de proto (.proto) e de headers (.pb.h) do Gazebo
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  # Em Ubuntu com Gazebo classic costuma aparecer algo tipo /usr/include/gazebo-11
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

# Também queremos que o protoc ache os .proto locais (msg/)
list(APPEND PROTOBUF_IMPORT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/msg")

message(STATUS "Gazebo msgs proto dir: ${PROTOBUF_IMPORT_DIRS}")
message(STATUS "Gazebo msgs (pb.h) dir: ${GAZEBO_MSG_INCLUDE_DIRS}")

find_package(Protobuf REQUIRED)

##################
## Build config ##
##################

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(${GAZEBO_CXX_FLAGS})

##########################
## Protobuf: .proto -> .pb
##########################

# Seus .proto estão em xasv_sim/plugins/msg/
set(PROTO_FILES
  msg/CommandMotorSpeed.proto
  msg/Wind.proto
  msg/MotorSpeed.proto
  msg/Float.proto
)

# Essa macro usa PROTOBUF_IMPORT_DIRS internamente
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# Lib com as mensagens usadas pelos plugins (CommandMotorSpeed, Wind)
add_library(gazebo_plugin_msgs SHARED ${PROTO_SRCS})
target_link_libraries(gazebo_plugin_msgs ${PROTOBUF_LIBRARIES})
target_include_directories(gazebo_plugin_msgs PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}   # onde ficam CommandMotorSpeed.pb.h, Wind.pb.h
  ${PROTOBUF_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}    # onde fica vector3d.pb.h e cia
)

#########################
## Includes / Link dirs ##
#########################

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}      # .pb.h gerados
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}       # vector3d.pb.h
  ${PROTOBUF_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

###############################
## Plugins de robô (robot-level)
###############################

# Motor model  -> libgazebo_motor_model.so
add_library(gazebo_motor_model SHARED
  src/gazebo_motor_model.cpp
)
target_link_libraries(gazebo_motor_model
  ${GAZEBO_LIBRARIES}
  gazebo_plugin_msgs
)

# USV dynamics -> libgazebo_usv_dynamics_plugin.so
add_library(gazebo_usv_dynamics_plugin SHARED
  src/gazebo_usv_dynamics_plugin.cpp
)
target_link_libraries(gazebo_usv_dynamics_plugin
  ${GAZEBO_LIBRARIES}
)

# Buoyancy     -> libbuoyancy_gazebo_plugin.so
add_library(buoyancy_gazebo_plugin SHARED
  src/buoyancy_gazebo_plugin.cc
)
target_link_libraries(buoyancy_gazebo_plugin
  ${GAZEBO_LIBRARIES}
)

# Follow plugin -> libfollow_plugin.so
add_library(follow_plugin SHARED
  src/follow_plugin.cc
)
target_link_libraries(follow_plugin
  ${GAZEBO_LIBRARIES}
)

# Wind plugin -> libgazebo_wind_plugin.so
add_library(gazebo_wind_plugin SHARED
  src/gazebo_wind_plugin.cpp
)
target_link_libraries(gazebo_wind_plugin
  ${GAZEBO_LIBRARIES}
  gazebo_plugin_msgs
)

# ArduPilotPlugin -> libArduPilotPlugin.so
add_library(ArduPilotPlugin SHARED
  src/ArduPilotPlugin.cc
)
target_link_libraries(ArduPilotPlugin
  ${GAZEBO_LIBRARIES}
)

################################
## Plugins de mundo (world-level)
################################

# TrunkScaler -> libTrunkScaler.so
add_library(TrunkScaler SHARED
  src/TrunkScaler.cc
)
target_link_libraries(TrunkScaler
  ${GAZEBO_LIBRARIES}
)

# BuoySpawner -> libBuoySpawner.so
add_library(BuoySpawner SHARED
  src/BuoySpawner.cc
)
target_link_libraries(BuoySpawner
  ${GAZEBO_LIBRARIES}
)

# WorldGpsAlignMavlink -> libWorldGpsAlignMavlink.so
add_library(WorldGpsAlignMavlink SHARED
  src/WorldGpsAlignMavlink.cc
)
target_link_libraries(WorldGpsAlignMavlink
  ${GAZEBO_LIBRARIES}
)

# WavefieldModelPlugin -> libWavefieldModelPlugin.so
add_library(WavefieldModelPlugin SHARED
  src/WavefieldModelPlugin.cc
)
target_link_libraries(WavefieldModelPlugin
  ${GAZEBO_LIBRARIES}
)

# WavefieldVisualPlugin -> libWavefieldVisualPlugin.so
add_library(WavefieldVisualPlugin SHARED
  src/WavefieldVisualPlugin.cc
)
target_link_libraries(WavefieldVisualPlugin
  ${GAZEBO_LIBRARIES}
)

######################################
## Diretório de instalação dos plugins
######################################

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
    gazebo_motor_model
    gazebo_usv_dynamics_plugin
    buoyancy_gazebo_plugin
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

