cmake_minimum_required(VERSION 3.22)
project(dummy_perception_example)

# ============================================================================
# Dependencies
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_schema2param REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()



# ============================================================================
# Node Executable
# ============================================================================
add_executable(dummy_perception_node
    src/dummy_perception_node.cpp
)

# ============================================================================
# Generate Parameter Loader from Schema
# ============================================================================
ros2_generate_parameter_loader(
    TARGET dummy_perception_node
    SCHEMA schema/dummy_perception_publisher.schema.json
    OUTPUT_DIR generated
    NAMESPACE dummy_perception
    CLASS_NAME DummyPerceptionParams
)

# Optional: Add schema validation
ros2_add_parameter_validation(
    TARGET dummy_perception_node
    SCHEMA schema/dummy_perception_publisher.schema.json
)

target_include_directories(dummy_perception_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(dummy_perception_node
    rclcpp
)


# ============================================================================
# Installation
# ============================================================================
install(TARGETS dummy_perception_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config)

# ament_package()
