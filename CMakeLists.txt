cmake_minimum_required(VERSION 2.8.3)
project(grid_world_conversion)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  angles
  cmake_modules
  roscpp
)

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(SBPL REQUIRED sbpl)
#include_directories(${SBPL_INCLUDE_DIRS})
#link_directories(${SBPL_LIBRARY_DIRS})

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grid_world_conversion
  CATKIN_DEPENDS angles
    cmake_modules
    roscpp
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(grid_world_conversion
    src/grid_world_conversion.cpp
)
#add_dependencies(navigation_trajectory_planner navigation_trajectory_msgs_generate_messages_cpp)
target_link_libraries(grid_world_conversion
    ${catkin_LIBRARIES}
#    ${SBPL_LIBRARIES} 
#    ${Boost_LIBRARIES}
)

