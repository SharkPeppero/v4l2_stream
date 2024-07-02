cmake_minimum_required(VERSION 3.8)
project(v4l2_stream)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 没有指定CMAKE_BUILD_TYPE 默认为 Release Debug
set(CMAKE_BUILD_TYPE Debug)
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif ()

# 指定g++的版本 以及 Release编译时代码优化的程度
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

# 设置colcon build过程打印消息的级别
set(ENV{COLCON_LOG_LEVEL} "info")

########################################## 设置自定义第三方功能包的cmake ##########################################
get_filename_component(PACKAGESPACE ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
get_filename_component(WORKSPACE ${PACKAGESPACE} DIRECTORY)
message(STATUS "功能包路径: " ${PACKAGESPACE})
message(STATUS "工作空间路径: " ${WORKSPACE})

# 定义输出的assets的路径参考
add_definitions(-DROOT_DIR=\"${WORKSPACE}/\")
message(STATUS "ROOT_DIR: " ${WORKSPACE}/)

include_directories(
        "${PROJECT_SOURCE_DIR}/include/"
        "${PROJECT_SOURCE_DIR}/3rdparty/agora_rtc_sdk/include/"
)

find_package(Threads REQUIRED)

# yaml-cpp
find_package(yaml-cpp REQUIRED)
include_directories(${yaml-cpp_INCLUDE_DIRS})
link_libraries(${yaml-cpp_LIBRARY_DIRS})
if (${yaml-cpp_FOUND})
  message(STATUS "yaml-cpp version: ${yaml-cpp_VERSION}")
endif ()


# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBRARY_DIRS})
if (${OpenCV_FOUND})
  message(STATUS "OpenCV's Version is:${OpenCV_VERSION}")
endif ()

# 设置动态库的路径，log系统
add_library(wlbylog SHARED IMPORTED)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  message(STATUS "current computer architecture is x86_64")
  set_target_properties(wlbylog PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/wlbylog.so)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "current computer architecture is arm")
  set_target_properties(wlbylog PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/wlbylog_arm.so)
else()
  message(STATUS "unknown architecture")
endif()

################### 配置可执行文件 ######################
add_executable(${PROJECT_NAME}_node
        src/main.cpp
        include/v4l2_capture/v4l2_capture.cpp
        include/mix_helper/mix_buffer.cpp
        include/v4l2_stream_object/v4l2_stream_object.h
        include/v4l2_stream_object/v4l2_stream_object.cpp
)

target_link_libraries(${PROJECT_NAME}_node
        ${OpenCV_LIBRARIES}
        yaml-cpp
        wlbylog
        Threads::Threads
)


if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_include_directories(${PROJECT_NAME}_node PUBLIC ${SOURCE_DIR}/v4l2_stream/3rdparty/agora_rtc_sdk/aarch64-linux-gnu_include)
  target_link_directories(${PROJECT_NAME}_node PUBLIC ${SOURCE_DIR}/v4l2_stream/3rdparty/agora_rtc_sdk/aarch64-linux-gnu_libs)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_include_directories(${PROJECT_NAME}_node PUBLIC ${SOURCE_DIR}/v4l2_stream/3rdparty/agora_rtc_sdk/x86_64-linux-gnu_include)
  target_link_directories(${PROJECT_NAME}_node PUBLIC ${SOURCE_DIR}/v4l2_stream/3rdparty/agora_rtc_sdk/x86_64-linux-gnu_libs)
else()
  message(STATUS "There is no agora_rtc_sdk which matches HOST_SYSTEM_PROCESSOR!")
endif()

install(TARGETS
        ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})