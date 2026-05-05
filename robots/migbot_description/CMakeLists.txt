cmake_minimum_required(VERSION 3.0.2)
project(migbot_description)

find_package(catkin REQUIRED COMPONENTS
  xacro
)

catkin_package(
  CATKIN_DEPENDS xacro
)

# Keep compatibility with older ROS distros where xacro requires the INORDER flag.
set(XACRO_INORDER)
if(DEFINED ENV{ROS_DISTRO})
  if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
    set(XACRO_INORDER INORDER)
  endif()
endif()

# If you want to generate URDF files at build time from Xacro,
# you can uncomment and adjust the following block:
#
# xacro_add_files(
#   urdf/migbot1_ardupilot_scaled.urdf.xacro
#   ${XACRO_INORDER}
#   INSTALL DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
# )

# Install description files (Xacro/URDF) so they can be found via
# `roslaunch` and other tools.
install(DIRECTORY urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)

# If you have meshes or launch files associated with this robot, you can
# also install them here:
#
# install(DIRECTORY meshes
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#         USE_SOURCE_PERMISSIONS)
#
# install(DIRECTORY launch
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#         USE_SOURCE_PERMISSIONS)

