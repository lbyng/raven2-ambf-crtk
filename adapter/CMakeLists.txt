cmake_minimum_required(VERSION 3.0.2)
project(raven2_ambf_crtk_adapter)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs ambf_msgs raven_2)
find_package(Eigen3 REQUIRED)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME}_kinematics)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# RAVEN-II kinematics, kept free of ROS so it can be tested on its own
add_library(${PROJECT_NAME}_kinematics src/kinematics.cpp)

add_executable(adapter src/adapter.cpp)
add_dependencies(adapter ${catkin_EXPORTED_TARGETS})
target_link_libraries(adapter ${PROJECT_NAME}_kinematics ${catkin_LIBRARIES})

install(TARGETS adapter ${PROJECT_NAME}_kinematics
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
