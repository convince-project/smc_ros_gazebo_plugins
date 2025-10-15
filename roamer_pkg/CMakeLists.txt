cmake_minimum_required(VERSION 3.16)
project(roamer_pkg)

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" PKGXML)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Roamer.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(roamer_server_node
  src/roamer_server_main.cpp
)

target_include_directories(roamer_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(roamer_server_node
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  nav_msgs
)

add_dependencies(roamer_server_node
  ${PROJECT_NAME}__rosidl_generator_cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(roamer_server_node "${cpp_typesupport_target}")

install(TARGETS roamer_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/worlds)
  install(DIRECTORY worlds
    DESTINATION share/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
