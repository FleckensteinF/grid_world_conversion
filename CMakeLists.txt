cmake_minimum_required(VERSION 2.8.3)
project(grid_world_conversion)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED COMPONENTS
#  angles
#  cmake_modules
#)

set(CMAKE_MODULE_PATH
    "${CMAKE_MODULE_PATH}"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules")

# Activate compiler warnings.
SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wall -W")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -W")

find_package(angles)
find_package(cmake_modules)

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(SBPL REQUIRED sbpl)
#include_directories(${SBPL_INCLUDE_DIRS})
#link_directories(${SBPL_LIBRARY_DIRS})

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)


####################################
### catkin specific configuration ##
####################################
### The catkin_package macro generates cmake config files for your package
### Declare things to be passed to dependent projects
### INCLUDE_DIRS: uncomment this if you package contains header files
### LIBRARIES: libraries you create in this project that dependent projects also need
### CATKIN_DEPENDS: catkin_packages dependent projects also need
### DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES grid_world_conversion
#  CATKIN_DEPENDS angles
#    cmake_modules
#  DEPENDS Eigen
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
 # ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
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

INSTALL(TARGETS grid_world_conversion
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

# install headers
FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/include/grid_world_conversion/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/include/grid_world_conversion/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})
