# Install script for directory: C:/Users/hcdce/Desktop/OpenSteer-master

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/OpenSteer")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenSteer" TYPE FILE FILES
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/AbstractVehicle.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/Annotation.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/Camera.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/Clock.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/Color.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/Draw.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/LocalSpace.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/lq.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/Obstacle.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/OldPathway.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/OpenSteerDemo.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/Path.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/Pathway.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/PlugIn.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/PolylineSegmentedPath.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/PolylineSegmentedPathwaySegmentRadii.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/PolylineSegmentedPathwaySingleRadius.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/Proximity.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/QueryPathAlike.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/QueryPathAlikeBaseDataExtractionPolicies.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/QueryPathAlikeMappings.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/QueryPathAlikeUtilities.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/SegmentedPath.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/SegmentedPathAlikeUtilities.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/SegmentedPathway.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/SharedPointer.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/SimpleVehicle.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/StandardTypes.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/SteerLibrary.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/UnusedParameter.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/Utilities.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/Vec3.h"
    "C:/Users/hcdce/Desktop/OpenSteer-master/include/OpenSteer/Vec3Utilities.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("C:/Users/hcdce/Desktop/OpenSteer-master/third-party/glfw/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "C:/Users/hcdce/Desktop/OpenSteer-master/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
