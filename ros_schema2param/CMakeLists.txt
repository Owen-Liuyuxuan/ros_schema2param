cmake_minimum_required(VERSION 3.22)
project(ros_schema2param)

# ============================================================================
# Dependencies
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Check Python dependencies
execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import jinja2, jsonschema"
    RESULT_VARIABLE PYTHON_DEPS_CHECK
    OUTPUT_QUIET ERROR_QUIET
)

if(NOT PYTHON_DEPS_CHECK EQUAL 0)
    message(FATAL_ERROR 
        "Required Python packages not found. Install with:\n"
        "  pip3 install jinja2 jsonschema"
    )
endif()

# ============================================================================
# Install CMake Functions
# ============================================================================
install(
    FILES cmake/ros_schema2param.cmake
    DESTINATION share/${PROJECT_NAME}/cmake
)

# ============================================================================
# Install Generator Script and Templates
# ============================================================================
install(
    PROGRAMS scripts/generate_param_loader.py
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY templates/
    DESTINATION share/${PROJECT_NAME}/templates
)

# ============================================================================
# Install Header-Only Utilities
# ============================================================================
install(
    DIRECTORY include/
    DESTINATION include
)

# ============================================================================
# Export Configuration
# ============================================================================
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

# Export CMake functions for downstream packages
ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ros_schema2param-extras.cmake.in"
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package(
    CONFIG_EXTRAS cmake/ros_schema2param.cmake
)
