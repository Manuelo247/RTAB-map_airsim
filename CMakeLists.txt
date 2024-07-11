cmake_minimum_required(VERSION 3.0.2)
project(drone_slam_simulation)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
  rtabmap_ros
)

catkin_package(
	#INCLUDE_DIRS include
	LIBRARIES drone_slam_simulation connect_API_src
	CATKIN_DEPENDS cv_bridge rospy sensor_msgs std_msgs rtabmap_ros
	#DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Instala todos los scripts en la carpeta scripts/
file(GLOB SCRIPTS scripts/*.py)
file(GLOB SCRIPTS2 src/drone_slam_simulation/*.py)

catkin_install_python(PROGRAMS ${SCRIPTS} ${SCRIPTS2}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_python_setup()