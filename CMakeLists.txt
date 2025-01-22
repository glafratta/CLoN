cmake_minimum_required(VERSION 3.1.0)

project(CloCK LANGUAGES CXX)

include(CTest)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(Threads)
find_package(PkgConfig)
find_package(OpenCV REQUIRED)
find_package(box2d REQUIRED)


add_subdirectory(src "${CMAKE_CURRENT_BINARY_DIR}/src")



if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
  find_package(iir)
  pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)
  include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/.. ${LIBCAMERA_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
  include(GNUInstallDirs)
  add_executable(demo custom.h demo.cpp)
  target_link_libraries(demo ${OpenCV_LIBS} iir::iir_static box2d alphabot a1lidarrpi CloCK pigpio PkgConfig::LIBCAMERA cam2opencv)
  message("detected architecture aarch64, building executables")
endif()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_compile_options(-Wall -pedantic)

enable_testing()
