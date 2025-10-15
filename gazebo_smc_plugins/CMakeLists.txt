cmake_minimum_required(VERSION 3.8)
project(gazebo_smc_plugins)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# Some more CXX flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(sim_handling_interfaces REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# smc_verifiable_plugins library (header only)
include(FetchContent)
FetchContent_Declare(
    smc_verifiable_plugins
    GIT_REPOSITORY https://github.com/convince-project/smc_verifiable_plugins.git
    GIT_TAG 0.0.2
)
FetchContent_MakeAvailable(smc_verifiable_plugins)

include_directories(include ${EIGEN3_INCLUDE_DIR})

set(SMC_PLUGINS
  gazebo_step_smc_plugin
  gazebo_drive_smc_plugin
  vel_to_goal_smc_plugin
)

foreach(PLUGIN_NAME ${SMC_PLUGINS})
  add_library(${PLUGIN_NAME} SHARED ${PROJECT_SOURCE_DIR}/src/${PLUGIN_NAME}.cpp)
  ament_target_dependencies(${PLUGIN_NAME} SYSTEM
    Eigen3
    rclcpp
    tf2
    sim_handling_interfaces
    ros_gz_interfaces
    rosgraph_msgs
    geometry_msgs
  )
  target_link_libraries(${PLUGIN_NAME} smc_verifiable_plugins)
endforeach()

# Install the executable
install(TARGETS
  ${SMC_PLUGINS}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()