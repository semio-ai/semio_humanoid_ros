cmake_minimum_required(VERSION 2.8.3)
project(semio_humanoid_ros)

# find catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  semio_msgs_ros
)

# find system dependencies
FIND_PACKAGE(PkgConfig REQUIRED)
pkg_check_modules(OpenNI2 REQUIRED libopenni2)
pkg_check_modules(Eigen3 REQUIRED eigen3)
pkg_check_modules(GSL REQUIRED gsl)

# define semio_LIBRARIES so dependent packages automatically link against it
set(semio_LIBRARIES semio)
set(semio_FOUND TRUE)

# remove drivers from OpenNI2_LIBRARIES
list(GET OpenNI2_LIBRARIES 0 OpenNI2_LIBRARIES)

# declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS semio_msgs_ros
  DEPENDS semio OpenNI2 Eigen3 GSL
)

# add include directores of dependencies to header search path
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${OpenNI2_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${GSL_INCLUDE_DIRS}
)

# mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# build libraries
add_subdirectory(src)

# build nodes
add_subdirectory(nodes)
