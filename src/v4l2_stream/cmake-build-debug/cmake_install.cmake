# Install script for directory: /home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
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

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/v4l2_stream/v4l2_stream_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/v4l2_stream/v4l2_stream_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/v4l2_stream/v4l2_stream_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/v4l2_stream" TYPE EXECUTABLE FILES "/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/cmake-build-debug/v4l2_stream_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/v4l2_stream/v4l2_stream_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/v4l2_stream/v4l2_stream_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/v4l2_stream/v4l2_stream_node"
         OLD_RPATH "/home/xu/3rdparty/cuda_11.8/lib64:/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/3rdparty/agora_rtc_sdk/x86_64-linux-gnu_libs:/home/xu/ros2_humble/install/cv_bridge/lib:/usr/local/opencv452/lib:/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/lib:/home/xu/ros2_humble/install/rclcpp/lib:/home/xu/ros2_humble/install/libstatistics_collector/lib:/home/xu/ros2_humble/install/rcl/lib:/home/xu/ros2_humble/install/rmw_implementation/lib:/home/xu/ros2_humble/install/ament_index_cpp/lib:/home/xu/ros2_humble/install/rcl_logging_spdlog/lib:/home/xu/ros2_humble/install/rcl_logging_interface/lib:/home/xu/ros2_humble/install/rcl_interfaces/lib:/home/xu/ros2_humble/install/rcl_yaml_param_parser/lib:/home/xu/ros2_humble/install/libyaml_vendor/lib:/home/xu/ros2_humble/install/rosgraph_msgs/lib:/home/xu/ros2_humble/install/statistics_msgs/lib:/home/xu/ros2_humble/install/tracetools/lib:/home/xu/ros2_humble/install/sensor_msgs/lib:/home/xu/ros2_humble/install/geometry_msgs/lib:/home/xu/ros2_humble/install/std_msgs/lib:/home/xu/ros2_humble/install/builtin_interfaces/lib:/home/xu/ros2_humble/install/rosidl_typesupport_fastrtps_c/lib:/home/xu/ros2_humble/install/rosidl_typesupport_fastrtps_cpp/lib:/home/xu/ros2_humble/install/fastcdr/lib:/home/xu/ros2_humble/install/rmw/lib:/home/xu/ros2_humble/install/rosidl_typesupport_introspection_cpp/lib:/home/xu/ros2_humble/install/rosidl_typesupport_introspection_c/lib:/home/xu/ros2_humble/install/rosidl_typesupport_cpp/lib:/home/xu/ros2_humble/install/rosidl_typesupport_c/lib:/home/xu/ros2_humble/install/rosidl_runtime_c/lib:/home/xu/ros2_humble/install/rcpputils/lib:/home/xu/ros2_humble/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/v4l2_stream/v4l2_stream_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/v4l2_stream" TYPE DIRECTORY FILES "/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/launch")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/v4l2_stream" TYPE DIRECTORY FILES "/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/config")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/v4l2_stream" TYPE DIRECTORY FILES "/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/config")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/cmake-build-debug/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/v4l2_stream")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/cmake-build-debug/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/v4l2_stream")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/v4l2_stream/environment" TYPE FILE FILES "/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/v4l2_stream/environment" TYPE FILE FILES "/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/cmake-build-debug/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/v4l2_stream/environment" TYPE FILE FILES "/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/v4l2_stream/environment" TYPE FILE FILES "/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/cmake-build-debug/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/v4l2_stream" TYPE FILE FILES "/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/cmake-build-debug/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/v4l2_stream" TYPE FILE FILES "/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/cmake-build-debug/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/v4l2_stream" TYPE FILE FILES "/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/cmake-build-debug/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/v4l2_stream" TYPE FILE FILES "/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/cmake-build-debug/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/v4l2_stream" TYPE FILE FILES "/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/cmake-build-debug/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/cmake-build-debug/ament_cmake_index/share/ament_index/resource_index/packages/v4l2_stream")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/v4l2_stream/cmake" TYPE FILE FILES
    "/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/cmake-build-debug/ament_cmake_core/v4l2_streamConfig.cmake"
    "/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/cmake-build-debug/ament_cmake_core/v4l2_streamConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/v4l2_stream" TYPE FILE FILES "/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/xu/ZME/zme_BirdView/v4l2_stream/src/v4l2_stream/cmake-build-debug/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
