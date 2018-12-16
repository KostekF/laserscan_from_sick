#cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.5)
project(laserscan_from_sick)

set (CMAKE_CXX_STANDARD 11)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  camera_info_manager
  std_msgs
  message_runtime
  cv_bridge 
  sensor_msgs
  message_generation
    image_transport 
  )
find_package(PCL 1.3 REQUIRED COMPONENTS common io

)
find_package(OpenCV REQUIRED) 
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)




###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laserscan_from_sick
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(SOURCES ${PROJECT_SOURCE_DIR}/src/main.cpp ${PROJECT_SOURCE_DIR}/src/laserscanFromSick.cpp ${PROJECT_SOURCE_DIR}/include/laserscan_from_sick)

add_executable(laserscan_from_sick ${SOURCES})



#add_executable(pcl2depth src/main.cpp)
target_include_directories(laserscan_from_sick PRIVATE ${PROJECT_SOURCE_DIR}/include/laserscan_from_sick)
target_link_libraries(laserscan_from_sick ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${OpenCV_LIBRARIES})

