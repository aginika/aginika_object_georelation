cmake_minimum_required(VERSION 2.8.3)
project(aginika_object_georelation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  pcl_msgs
)
generate_messages(DEPENDENCIES pcl_msgs)
find_package(PCL 1.7 REQUIRED)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES aginika_object_georelation
  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs pcl_msgs
#  DEPENDS pcl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a cpp executable
add_library(in_relation src/in_relation.cpp)
add_library(over_relation src/over_relation.cpp)
add_library(on_relation src/on_relation.cpp)
add_library(aginika_object_georelation src/aginika_object_georelation.cpp)
add_library(aginika_object_georelation_solver src/aginika_object_georelation_solver.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(in_relation aginika_object_georelation_generate_messages_cpp)
add_dependencies(over_relation aginika_object_georelation_generate_messages_cpp)
add_dependencies(on_relation aginika_object_georelation_generate_messages_cpp)
add_dependencies(aginika_object_georelation aginika_object_georelation_generate_messages_cpp)
add_dependencies(aginika_object_georelation_solver aginika_object_georelation_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(in_relation
  ${catkin_LIBRARIES}
  ${pcl_ros_LIBRARIES}
  ${PCL_LIBRARIES}
  )

target_link_libraries(over_relation
  ${catkin_LIBRARIES}
  ${pcl_ros_LIBRARIES}
  ${PCL_LIBRARIES}
  )

target_link_libraries(on_relation
  ${catkin_LIBRARIES}
  ${pcl_ros_LIBRARIES}
  ${PCL_LIBRARIES}
  )

target_link_libraries(aginika_object_georelation
  ${catkin_LIBRARIES}
  ${pcl_ros_LIBRARIES}
  ${PCL_LIBRARIES}
  )

target_link_libraries(aginika_object_georelation_solver
  ${catkin_LIBRARIES}
  ${pcl_ros_LIBRARIES}
  ${PCL_LIBRARIES}
  )

add_executable(test_aginika_object_georelation_solver test/test_aginika_object_georelation_solver.cpp)
add_dependencies(test_aginika_object_georelation_solver aginika_object_georelation_generate_messages_cpp)

target_link_libraries(test_aginika_object_georelation_solver
  ${catkin_LIBRARIES}
  ${pcl_ros_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  in_relation
  over_relation
  on_relation
  aginika_object_georelation
  aginika_object_georelation_solver
  )

