################################################################################
### Find include directories of grid_world_conversion and its dependencies.
################################################################################

# Try to find the header files.
FIND_PATH(GRID_WORLD_CONVERSION_INCLUDE_DIRECTORY
  NAMES grid_world_conversion/grid_world_conversion.h
  PATHS
    ${CMAKE_SOURCE_DIR}/../grid_world_conversion/include
    ${GRID_WORLD_CONVERSION_ROOT}/include
    $ENV{GRID_WORLD_CONVERSION_ROOT}/include
    /usr/local/include
    /usr/include
    /opt/local/include
    /sw/local/include
    /sw/include
  PATH_SUFFIXES grid_world_conversion
)

IF (NOT GRID_WORLD_CONVERSION_INCLUDE_DIRECTORY)
  MESSAGE(FATAL_ERROR "Cannot find grid world conversion include path! ${GRID_WORLD_CONVERSION_INCLUDE_DIRECTORY}")
ELSE ()
  MESSAGE(STATUS "Found include dirs: ${GRID_WORLD_CONVERSION_INCLUDE_DIRECTORY}")
ENDIF ()

# set includes
SET(GRID_WORLD_CONVERSION_INCLUDE_DIRECTORIES ${GRID_WORLD_CONVERSION_INCLUDE_DIRECTORY})

# Find libraries
FIND_LIBRARY(GRID_WORLD_CONVERSION_LIBRARY
  NAMES "libgrid_world_conversion.so"
  PATHS
    ${CMAKE_CURRENT_BINARY_DIR}/../
  PATH_SUFFIXES grid_world_conversion
)

IF (NOT GRID_WORLD_CONVERSION_LIBRARY)
  MESSAGE(FATAL_ERROR "Cannot find grid world conversion with name libgrid_world_conversion.so!")
ENDIF ()


SET(GRID_WORLD_CONVERSION_LIBRARIES 
  ${GRID_WORLD_CONVERSION_LIBRARY}
)

MESSAGE(STATUS "Found grid world conversion libraries!")
SET(GRID_WORLD_CONVERSION_FOUND "YES")

# Handle REQUIRED, QUIET, and version arguments
# and set the <packagename>_FOUND variable.
FIND_PACKAGE_HANDLE_STANDARD_ARGS(grid_world_conversion
  FOUND_VAR GRID_WORLD_CONVERSION_FOUND
  REQUIRED_VARS
    GRID_WORLD_CONVERSION_LIBRARIES
    GRID_WORLD_CONVERSION_INCLUDE_DIRECTORIES
)
