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
#         message_generation
        std_msgs
        sensor_msgs 
        pcl_ros
        pcl_conversions
)

# Find point cloud Library
FIND_PACKAGE(PCL 1.3 REQUIRED COMPONENTS)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

INCLUDE_DIRECTORIES(go_sdk/)
# Gocator
SET(SDK_SRCS
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/GoLayout.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/GoMaterial.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/GoMultiplexBank.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/GoPartDetection.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/GoPartMatching.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/GoPartModel.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/GoProfileGeneration.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/GoSdkDef.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/GoSdkLib.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/GoSensor.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/GoSensorInfo.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/GoSetup.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/GoSurfaceGeneration.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/GoSystem.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/GoTransform.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/GoUtils.c

    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Internal/GoControl.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Internal/GoDiscovery.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Internal/GoReceiver.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Internal/GoSerializer.c

    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Messages/GoDataSet.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Messages/GoDataTypes.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Messages/GoHealth.c

    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Outputs/GoAnalog.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Outputs/GoDigital.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Outputs/GoEthernet.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Outputs/GoOutput.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Outputs/GoSerial.c

    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Tools/GoExtParam.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Tools/GoExtTool.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Tools/GoMeasurement.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Tools/GoMeasurements.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Tools/GoProfileTools.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Tools/GoProfileToolUtils.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Tools/GoRangeTools.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Tools/GoSurfaceTools.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Tools/GoSurfaceToolUtils.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Tools/GoTool.c
    ${PROJECT_SOURCE_DIR}/go_sdk/GoSdk/Tools/GoTools.c
)

# Platform
SET(KAPI_SRCS
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/kAlloc.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/kApiDef.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/kApiLib.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/kAssembly.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/kObject.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/kType.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/kValue.c

    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Data/kArray1.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Data/kArray2.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Data/kArray3.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Data/kArrayList.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Data/kBox.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Data/kBytes.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Data/kCollection.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Data/kImage.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Data/kList.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Data/kMap.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Data/kMath.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Data/kQueue.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Data/kString.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Data/kXml.c

    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Io/kDat5Serializer.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Io/kDat6Serializer.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Io/kDirectory.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Io/kFile.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Io/kHttpServer.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Io/kHttpServerChannel.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Io/kHttpServerRequest.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Io/kHttpServerResponse.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Io/kMemory.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Io/kNetwork.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Io/kPath.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Io/kSerializer.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Io/kSocket.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Io/kStream.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Io/kTcpClient.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Io/kTcpServer.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Io/kUdpClient.c

    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Threads/kAtomic.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Threads/kLock.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Threads/kMsgQueue.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Threads/kPeriodic.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Threads/kSemaphore.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Threads/kThread.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Threads/kTimer.c

    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kBackTrace.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kBackTrace.h
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kBackTrace.x.h
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kDebugAlloc.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kDebugAlloc.h
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kDebugAlloc.x.h
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kDynamicLib.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kDynamicLib.h
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kDynamicLib.x.h
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kEvent.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kObjectPool.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kObjectPool.h
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kObjectPool.x.h
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kPlugin.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kPoolAlloc.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kPoolAlloc.h
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kPoolAlloc.x.h
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kUserAlloc.c
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kUserAlloc.h
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kUserAlloc.x.h
    ${PROJECT_SOURCE_DIR}/go_sdk/kApi/Utils/kUtils.c
)

# add other necessary include directories
include_directories(src/ src/lib/src/ ${catkin_INCLUDE_DIRS})

# Set source files for the library
SET(SRCS_LIB 
    src/lib/src/gocator3100.cpp)
    
# Set header files for the library
SET(HDRS_LIB 
    src/lib/src/gocator3100.h)

# Set source files for the ROS node
SET(SRCS_NODE 
    src/gocator_3100_node.cpp
    src/gocator_3100_node_main.cpp
)
    
#dynamic reconfigure cfg file
generate_dynamic_reconfigure_options(cfg/gocator_3100_params.cfg)

## The catkin_package macro generates cmake config files for your package
catkin_package(
    CATKIN_DEPENDS 
        roscpp 
        rospy 
        dynamic_reconfigure 
#         message_runtime 
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
)

# Libraries
ADD_LIBRARY(kApi SHARED
    ${KAPI_SRCS}
)
TARGET_LINK_LIBRARIES(kApi
    pthread
    rt
    m
    ${CMAKE_DL_LIBS}
)
SET_TARGET_PROPERTIES(kApi PROPERTIES LINKER_LANGUAGE CXX)

ADD_LIBRARY(GoSdk SHARED
    ${SDK_SRCS}
)
TARGET_LINK_LIBRARIES(GoSdk
    kApi
    pthread
    rt
    m
)
ADD_DEPENDENCIES(GoSdk kApi)
SET_TARGET_PROPERTIES(GoSdk PROPERTIES LINKER_LANGUAGE CXX)

ADD_LIBRARY(${PROJECT_NAME}_LIB SHARED 
    ${SRCS_LIB}
)
ADD_DEPENDENCIES(${PROJECT_NAME}_LIB GoSdk)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_LIB GoSdk kApi ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

# Executable
ADD_EXECUTABLE(${PROJECT_NAME} ${SRCS_NODE})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PROJECT_NAME}_LIB GoSdk kApi ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${catkin_LIBRARIES})
ADD_DEPENDENCIES(${PROJECT_NAME} ${PROJECT_NAME}_LIB ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

# Examples TODO: Should be called from a CMakeLists at at src/lib/
ADD_SUBDIRECTORY(src/lib/src/examples)
