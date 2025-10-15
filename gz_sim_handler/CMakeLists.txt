cmake_minimum_required(VERSION 3.8)
project(gz_sim_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(scan_processor_component SHARED src/scan_processor_node.cpp)
ament_target_dependencies(scan_processor_component SYSTEM rclcpp rclcpp_components sensor_msgs)

rclcpp_components_register_node(
    scan_processor_component
    PLUGIN "gz_sim_handler::ScanProcessorNode"
    EXECUTABLE scan_processor_node
)

install(TARGETS scan_processor_component
        EXPORT export_scan_processor_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(PROGRAMS
  scripts/spawn_simulation_ros.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch worlds jani config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_scan_processor_component)

ament_package()