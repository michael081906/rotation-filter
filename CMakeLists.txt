cmake_minimum_required(VERSION 2.8.3)
project(rotation_filter)

#--------------------------2020/8/8/Mkam--------------------------------------
option(COVERALLS "Turn on coveralls support" OFF)
option(COVERALLS_UPLOAD "Upload the generated coveralls json" ON)

if (COVERALLS)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} 
	${PROJECT_SOURCE_DIR}/coveralls-cmake/cmake)
endif()

if (COVERALLS)
	include(Coveralls)
	coveralls_turn_on_coverage()
endif()

#---------------------2020/8/8 Mkam--------------------------------
if (COVERALLS)
  set(LIB_SRC)
  file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  list(APPEND LIB_SRC ${SRCS})
  file(GLOB_RECURSE HDRS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp)
  list(APPEND LIB_SRC ${HDRS})
endif()
#-------------------------------------------------------------------


if (COVERALLS)
	coveralls_setup(
        "${LIB_SRC}" 
        ${COVERALLS_UPLOAD}                 
        "${PROJECT_SOURCE_DIR}/coveralls-cmake/cmake")
endif()


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  roslib
  pcl_conversions
  message_generation
)

 add_service_files(
   DIRECTORY
   srv
   FILES
   rtf.srv
 )

 generate_messages(
   DEPENDENCIES
   geometry_msgs
   sensor_msgs
   std_msgs
 )

 generate_dynamic_reconfigure_options(
   cfg/rotation.cfg
 )

catkin_package()


include_directories(
# include
  src/
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/rotation_filter.cpp src/rotation_filter.hpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencpp ${PCL_LIBRARIES} ${PROJECT_NAME}_gencfg)


if (CATKIN_ENABLE_TESTING)
 find_package(rostest REQUIRED)
 find_package(PCL 1.2 REQUIRED)
 include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
 link_directories(${PCL_LIBRARY_DIRS})
 add_definitions(${PCL_DEFINITIONS})

 catkin_add_gtest(
   test_rotation_filter
   test/rotation_filter_test.cpp
 )
add_dependencies(test_rotation_filter ${PROJECT_NAME}_gencpp ${PCL_LIBRARIES} ${PROJECT_NAME}_gencfg)
target_link_libraries(test_rotation_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
endif()

