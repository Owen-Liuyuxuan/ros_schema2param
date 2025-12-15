#
# Generate ROS2 parameter loader from JSON Schema
#
# Usage:
#   ros2_generate_parameter_loader(
#     TARGET <target_name>
#     SCHEMA <schema_file>
#     OUTPUT_DIR <output_directory>
#     [NAMESPACE <cpp_namespace>]
#     [CLASS_NAME <class_name>]
#     [HEADER_ONLY]
#   )
#
# Arguments:
#   TARGET: Name of the target that will use the generated code
#   SCHEMA: Path to JSON Schema file
#   OUTPUT_DIR: Directory for generated files (relative to CMAKE_CURRENT_BINARY_DIR)
#   NAMESPACE: C++ namespace (default: extracted from schema title)
#   CLASS_NAME: C++ class name (default: ParameterLoader)
#   HEADER_ONLY: Generate header-only implementation
#

function(ros2_generate_parameter_loader)
    # Parse arguments
    set(options HEADER_ONLY)
    set(oneValueArgs TARGET SCHEMA OUTPUT_DIR NAMESPACE CLASS_NAME)
    set(multiValueArgs)
    cmake_parse_arguments(
        ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
    )

    # Validate required arguments
    if(NOT ARG_TARGET)
        message(FATAL_ERROR "ros2_generate_parameter_loader: TARGET is required")
    endif()
    
    if(NOT ARG_SCHEMA)
        message(FATAL_ERROR "ros2_generate_parameter_loader: SCHEMA is required")
    endif()
    
    if(NOT ARG_OUTPUT_DIR)
        message(FATAL_ERROR "ros2_generate_parameter_loader: OUTPUT_DIR is required")
    endif()

    # Set defaults
    if(NOT ARG_CLASS_NAME)
        set(ARG_CLASS_NAME "ParameterLoader")
    endif()

    # Resolve paths
    if(NOT IS_ABSOLUTE ${ARG_SCHEMA})
        set(ARG_SCHEMA "${CMAKE_CURRENT_SOURCE_DIR}/${ARG_SCHEMA}")
    endif()

    if(NOT IS_ABSOLUTE ${ARG_OUTPUT_DIR})
        set(ARG_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/${ARG_OUTPUT_DIR}")
    endif()

    # Ensure output directory exists
    file(MAKE_DIRECTORY ${ARG_OUTPUT_DIR})

    # Find generator script
    find_package(ros_schema2param REQUIRED)
    set(GENERATOR_SCRIPT "${ros_schema2param_DIR}/../../../lib/ros_schema2param/generate_param_loader.py")
    set(TEMPLATE_DIR "${ros_schema2param_DIR}/../../../share/ros_schema2param/templates")

    # Output files
    set(GENERATED_HEADER "${ARG_OUTPUT_DIR}/${ARG_CLASS_NAME}.hpp")
    if(ARG_HEADER_ONLY)
        set(GENERATED_FILES ${GENERATED_HEADER})
    else()
        set(GENERATED_SOURCE "${ARG_OUTPUT_DIR}/${ARG_CLASS_NAME}.cpp")
        set(GENERATED_FILES ${GENERATED_HEADER} ${GENERATED_SOURCE})
    endif()

    # Build generator command
    set(GENERATOR_CMD
        ${Python3_EXECUTABLE} ${GENERATOR_SCRIPT}
        --schema ${ARG_SCHEMA}
        --output-dir ${ARG_OUTPUT_DIR}
        --template-dir ${TEMPLATE_DIR}
        --class-name ${ARG_CLASS_NAME}
    )

    if(ARG_NAMESPACE)
        list(APPEND GENERATOR_CMD --namespace ${ARG_NAMESPACE})
    endif()

    if(ARG_HEADER_ONLY)
        list(APPEND GENERATOR_CMD --header-only)
    endif()

    # Custom command to generate code
    add_custom_command(
        OUTPUT ${GENERATED_FILES}
        COMMAND ${GENERATOR_CMD}
        DEPENDS ${ARG_SCHEMA} ${GENERATOR_SCRIPT}
        COMMENT "Generating parameter loader from ${ARG_SCHEMA}"
        VERBATIM
    )

    # Create custom target
    set(GENERATOR_TARGET "${ARG_TARGET}_generate_params")
    add_custom_target(${GENERATOR_TARGET}
        DEPENDS ${GENERATED_FILES}
    )

    # Add dependency to main target
    if(TARGET ${ARG_TARGET})
        add_dependencies(${ARG_TARGET} ${GENERATOR_TARGET})
        
        # Add generated directory to include paths
        target_include_directories(${ARG_TARGET} PRIVATE ${ARG_OUTPUT_DIR})
        
        # Add generated source to target if not header-only
        if(NOT ARG_HEADER_ONLY)
            target_sources(${ARG_TARGET} PRIVATE ${GENERATED_SOURCE})
        endif()
    else()
        message(WARNING "Target ${ARG_TARGET} does not exist yet. "
                        "Make sure to call add_executable/add_library before "
                        "ros2_generate_parameter_loader or add dependencies manually.")
    endif()

    # Set output variables for parent scope
    set(${ARG_TARGET}_PARAM_HEADER ${GENERATED_HEADER} PARENT_SCOPE)
    if(NOT ARG_HEADER_ONLY)
        set(${ARG_TARGET}_PARAM_SOURCE ${GENERATED_SOURCE} PARENT_SCOPE)
    endif()

endfunction()

#
# Validate ROS2 parameters against JSON Schema at runtime
#
# Usage:
#   ros2_add_parameter_validation(
#     TARGET <target_name>
#     SCHEMA <schema_file>
#   )
#
function(ros2_add_parameter_validation)
    set(oneValueArgs TARGET SCHEMA)
    cmake_parse_arguments(ARG "" "${oneValueArgs}" "" ${ARGN})

    if(NOT ARG_TARGET)
        message(FATAL_ERROR "ros2_add_parameter_validation: TARGET is required")
    endif()

    if(NOT ARG_SCHEMA)
        message(FATAL_ERROR "ros2_add_parameter_validation: SCHEMA is required")
    endif()

    # Install schema file
    install(FILES ${ARG_SCHEMA}
        DESTINATION share/${PROJECT_NAME}/schema
    )

    # Add compile definition with schema path
    target_compile_definitions(${ARG_TARGET} PRIVATE
        PARAM_SCHEMA_PATH="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/schema/${ARG_SCHEMA}"
    )

endfunction()
