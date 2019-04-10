cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)
set(ORB_SLAM2_DIR ~/REU_Workspace/ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.4.1 REQUIRED)
find_package(Eigen3 3.3.4 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
${ORB_SLAM2_DIR}
${ORB_SLAM2_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/lib)

add_library(${PROJECT_NAME} SHARED
${ORB_SLAM2_DIR}/src/System.cc
${ORB_SLAM2_DIR}/src/Tracking.cc
${ORB_SLAM2_DIR}/src/LocalMapping.cc
${ORB_SLAM2_DIR}/src/LoopClosing.cc
${ORB_SLAM2_DIR}/src/ORBextractor.cc
${ORB_SLAM2_DIR}/src/ORBmatcher.cc
${ORB_SLAM2_DIR}/src/FrameDrawer.cc
${ORB_SLAM2_DIR}/src/Converter.cc
${ORB_SLAM2_DIR}/src/MapPoint.cc
${ORB_SLAM2_DIR}/src/KeyFrame.cc
${ORB_SLAM2_DIR}/src/Map.cc
${ORB_SLAM2_DIR}/src/MapDrawer.cc
${ORB_SLAM2_DIR}/src/Optimizer.cc
${ORB_SLAM2_DIR}/src/PnPsolver.cc
${ORB_SLAM2_DIR}/src/Frame.cc
${ORB_SLAM2_DIR}/src/KeyFrameDatabase.cc
${ORB_SLAM2_DIR}/src/Sim3Solver.cc
${ORB_SLAM2_DIR}/src/Initializer.cc
${ORB_SLAM2_DIR}/src/Viewer.cc
${PROJECT_SOURCE_DIR}/camera_utilities.cpp
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${ORB_SLAM2_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${ORB_SLAM2_DIR}/Thirdparty/g2o/lib/libg2o.so
)

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/)

add_executable(orb_live
${PROJECT_SOURCE_DIR}/orb_live.cpp)
target_link_libraries(orb_live ${PROJECT_NAME})