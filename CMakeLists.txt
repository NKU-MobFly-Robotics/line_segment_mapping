cmake_minimum_required(VERSION 3.0.2)
project(line_segment_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Add new CMake Modules
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  message_filters
  nav_msgs
  open_karto
  roscpp
  sensor_msgs
  tf
  visualization_msgs
)

# Eigen
find_package(Eigen3 REQUIRED)

# g2o  
find_package(G2O REQUIRED)   

# suitesparse   
find_package(SUITESPARSE REQUIRED) 

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  ${SUITESPARSE_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/g2o_solver.cpp
  src/line_segment_extractor.cpp
  src/line_segment_map.cpp
  src/line_segment_mapper.cpp
  src/slam_karto_g2o.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  ${G2O_LIBRARIES}
  ${SUITESPARSE_LIBRARIES} 
)