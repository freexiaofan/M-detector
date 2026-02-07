cmake_minimum_required(VERSION 3.1.0)
project(m_detector)

SET(CMAKE_BUILD_TYPE "Release")


set(CMAKE_C_COMPILER "/usr/bin/gcc-9")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-9")

ADD_COMPILE_OPTIONS(-std=c++17 )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -ltbb" ) 

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -ltbb -pthread -std=c++1z -std=c++17 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  # if(N GREATER 15)
  #   add_definitions(-DMP_EN)
  #   add_definitions(-DMP_PROC_NUM=14)
  #   message("core for MP:  4")
  # elseif(N GREATER 5)
  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message("core for MP:  3")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message("core for MP:  ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# Use local oneTBB build
# set(TBB_DIR "/home/xf/Desktop/catkin_ws/M-detector0206/src/M-detector/oneTBB-2019_U8/")
# include("/home/xf/Desktop/catkin_ws/M-detector0206/src/M-detector/oneTBB-2019_U8/cmake/TBBBuild.cmake")
# tbb_build(TBB_ROOT "/home/xf/Desktop/catkin_ws/M-detector0206/src/M-detector/oneTBB-2019_U8" CONFIG_DIR TBB_DIR)

# ubuntu 22.04 with tbb installed via apt
# find_package(TBB REQUIRED)
# Use system TBB library
set(TBB_LIBRARIES /usr/local/lib/libtbb.so /usr/local/lib/libtbbmalloc.so)
set(TBB_INCLUDE_DIRS /usr/local/tbb-2019_U8/include)
include_directories(${TBB_INCLUDE_DIRS})
# target_link_libraries(xxx ${TBB_LIBRARIES})


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  message_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
# Force use system PCL 1.12 (ROS version) instead of custom PCL 1.15
# 必须使用系统安装的PCL 1.12版本，否则会出现链接错误，无法找到PCL库中的符号
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl" CACHE PATH "" FORCE)
find_package(PCL REQUIRED)
# 打印PCL配置信息用于调试
message(STATUS "PCL_VERSION: ${PCL_VERSION}")
message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_LIBRARY_DIRS: ${PCL_LIBRARY_DIRS}")
message(STATUS "TBB_LIBRARIES: ${TBB_LIBRARIES}")


find_package(OpenCV REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
  include)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)


add_executable(dynfilter src/dynfilter_with_odom.cpp src/DynObjFilter.cpp src/DynObjCluster.cpp)
target_link_libraries(dynfilter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} ${TBB_LIBRARIES})
target_include_directories(dynfilter PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(display_prediction src/display_prediction.cpp )
target_link_libraries(display_prediction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} ${TBB_LIBRARIES})
target_include_directories(display_prediction PRIVATE ${PYTHON_INCLUDE_DIRS})


add_executable(cal_recall src/cal_recall_multi.cpp)
target_link_libraries(cal_recall ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} ${TBB_LIBRARIES})
# target_include_directories(cal_recall PRIVATE ${PYTHON_INCLUDE_DIRS})


add_executable(remove_dynamic src/remove_dynamic.cpp src/DynObjFilter.cpp src/DynObjCluster.cpp)
target_link_libraries(remove_dynamic ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} ${TBB_LIBRARIES})