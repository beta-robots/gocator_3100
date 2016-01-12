cmake_minimum_required(VERSION 2.8.3)

# Cmake policy
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0005 NEW) 
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Project name and the type of project
PROJECT(gocator_3100)
message(STATUS "Executig CMakeLists of project  ${PROJECT_NAME}.")

# DEBUG/RELEASE
IF (NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "RELEASE") 
ENDIF (NOT CMAKE_BUILD_TYPE)
message(STATUS "Configured to compile in ${CMAKE_BUILD_TYPE} mode.")

#Set Flags
SET(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -D_REENTRANT")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -D_REENTRANT")

#Set compiler according C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++0x support.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
find_package(
    catkin REQUIRED 
    COMPONENTS 
        roscpp 
        rospy 
        dynamic_reconfigure 
        sensor_msgs 
)

# Find point cloud Library
FIND_PACKAGE(PCL 1.3 REQUIRED COMPONENTS)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

# Set GO_SDK include paths (That could be part of a FindGocator.cmake)
SET(GO_SDK_4 /home/andreu/dev/GO_SDK) # TODO: Should be an env variable!!
FIND_PATH(
    GOCATOR_INCLUDES
    NAMES GoSdk/GoSdk.h
    PATHS ${GO_SDK_4}/Gocator/GoSdk)
FIND_PATH(
    KAPI_INCLUDES
    NAMES kApi/kApi.h
    PATHS ${GO_SDK_4}/Platform/kApi)
INCLUDE_DIRECTORIES(${GOCATOR_INCLUDES} ${KAPI_INCLUDES})

# Set GO_SDK libs (That could be part of a FindGocator.cmake)
FIND_LIBRARY(
    GOCATOR_LIBRARIES
    NAMES GoSdk
    PATHS ${GO_SDK_4}/lib/gnud/) 
FIND_LIBRARY(
    KAPI_LIBRARIES
    NAMES kApi
    PATHS ${GO_SDK_4}/lib/gnud/) 
message(STATUS "Gocator Headers found at ${GOCATOR_INCLUDES}.")
message(STATUS "Gocator Libraries found at ${GOCATOR_LIBRARIES}.")

# add other necessary include directories
include_directories(src/ src/lib/src/ ${catkin_INCLUDE_DIRS})

# Set source files 
SET(SRCS 
    src/lib/src/gocator3100.cpp)

# Set header files 
SET(HDRS 
    src/lib/src/gocator3100.h)

#dynamic reconfigure cfg file
# generate_dynamic_reconfigure_options(cfg/gocator_3100_params.cfg)

## The catkin_package macro generates cmake config files for your package
catkin_package(
    CATKIN_DEPENDS 
        roscpp 
        rospy 
        dynamic_reconfigure 
        sensor_msgs 
#       DEPENDS 
#         PCL #??
)

#Build library (TODO: Should call a CMakeLists.txt at src/lib/)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SRCS})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${GOCATOR_LIBRARIES} ${KAPI_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

#Build examples TODO: Should be called from a CMakeLists at at src/lib/
ADD_SUBDIRECTORY(src/lib/src/examples)

## Declare a cpp executable
# add_executable(btr_point_tracker_node ${SRCS} src/btr_point_tracker_main.cpp)
# add_dependencies(btr_point_tracker_node ${PROJECT_NAME}_EXPORTED_TARGETS)
# target_link_libraries(btr_point_tracker_node ${OpenCV_LIBS} ${catkin_LIBRARIES})


