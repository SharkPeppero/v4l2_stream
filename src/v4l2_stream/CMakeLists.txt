cmake_minimum_required(VERSION 3.8)
project(v4l2_stream)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

get_filename_component(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
get_filename_component(WORKSPACE_DIR ${SOURCE_DIR} DIRECTORY)
message("SOURCE_DIR's Version is:${SOURCE_DIR}")
message("WORKSPACE_DIR's Version is:${WORKSPACE_DIR}")

# 没有指定CMAKE_BUILD_TYPE 默认为 Release
set(CMAKE_BUILD_TYPE Release)
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
message("功能包路径: " ${PACKAGESPACE})
message("工作空间路径: " ${WORKSPACE})

# 定义输出的assets的路径参考
add_definitions(-DROOT_DIR=\"${WORKSPACE}/\")
message("ROOT_DIR: " ${WORKSPACE}/)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

set(dependencies
        ament_cmake
        rclcpp
        std_msgs
        sensor_msgs
        cv_bridge
        yaml_cpp_vendor
)

include_directories(
        "${PROJECT_SOURCE_DIR}/include/"
        "${PROJECT_SOURCE_DIR}/3rdparty/agora_rtc_sdk/include/"
)

set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBRARY_DIRS})
if (${OpenCV_FOUND})
  message("OpenCV's Version is:${OpenCV_VERSION}")
endif ()

# 设置动态库的路径，log系统
add_library(wlbylog SHARED IMPORTED)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  message("current computer architecture is x86_64")
  set_target_properties(wlbylog PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/wlbylog.so)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("current computer architecture is arm")
  set_target_properties(wlbylog PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/wlbylog_arm.so)
else()
  message("unknown architecture")
endif()

################### 配置可执行文件 ######################
set(EXE_NAME ${PROJECT_NAME}_node)
add_executable(${EXE_NAME}
        src/v4l2_stream_node.cpp
        include/v4l2_stream/v4l2_stream.cpp
        include/v4l2_capture/v4l2_capture.cpp
        include/mix_helper/mix_buffer.cpp
)

ament_target_dependencies(${EXE_NAME}
        ${dependencies}
)

target_link_libraries(${EXE_NAME}
        ${OpenCV_LIBRARIES}
        wlbylog
)


if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_include_directories(${EXE_NAME} PUBLIC ${SOURCE_DIR}/v4l2_stream/3rdparty/agora_rtc_sdk/aarch64-linux-gnu_include)
  target_link_directories(${EXE_NAME} PUBLIC ${SOURCE_DIR}/v4l2_stream/3rdparty/agora_rtc_sdk/aarch64-linux-gnu_libs)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_include_directories(${EXE_NAME} PUBLIC ${SOURCE_DIR}/v4l2_stream/3rdparty/agora_rtc_sdk/x86_64-linux-gnu_include)
  target_link_directories(${EXE_NAME} PUBLIC ${SOURCE_DIR}/v4l2_stream/3rdparty/agora_rtc_sdk/x86_64-linux-gnu_libs)
else()
  message("There is no agora_rtc_sdk which matches HOST_SYSTEM_PROCESSOR!")
endif()

target_link_libraries(${EXE_NAME} yuv)


install(TARGETS
        ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()