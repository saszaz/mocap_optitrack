cmake_minimum_required(VERSION 2.8.3)
project(mocap_optitrack)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs tf tf_conversions roslaunch message_generation cmake_modules)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime
  DEPENDS Eigen
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}  
  )

add_subdirectory(src)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Check the validity of the supplied launch files.
roslaunch_add_file_check(launch)
