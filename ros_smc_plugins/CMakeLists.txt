cmake_minimum_required(VERSION 3.8)
project(ros_smc_plugins)

# C++
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sim_handling_interfaces REQUIRED)
find_package(roamer_pkg REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# smc_verifiable_plugins (header-only, fetched)
include(FetchContent)
FetchContent_Declare(
  smc_verifiable_plugins
  GIT_REPOSITORY https://github.com/convince-project/smc_verifiable_plugins.git
  GIT_TAG 0.0.2
)
FetchContent_MakeAvailable(smc_verifiable_plugins)

# Plugins
set(SMC_PLUGINS
  handle_simulation_smc_plugin
  roamer_smc_plugin
)

foreach(PLUGIN_NAME ${SMC_PLUGINS})
  add_library(${PLUGIN_NAME} SHARED src/${PLUGIN_NAME}.cpp)

  # export your own headers
  target_include_directories(${PLUGIN_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # ROS deps — this sets include dirs + link libs and ensures build order
  ament_target_dependencies(${PLUGIN_NAME}
    rclcpp
    rclcpp_action
    tf2
    rosgraph_msgs
    geometry_msgs
    std_msgs
    sim_handling_interfaces
    roamer_pkg
  )

  # header-only fetched lib: plain signature (no PUBLIC/PRIVATE)
  target_link_libraries(${PLUGIN_NAME} smc_verifiable_plugins)
endforeach()

# Install
install(TARGETS ${SMC_PLUGINS}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
