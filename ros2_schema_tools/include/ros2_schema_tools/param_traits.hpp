#pragma once

#include <rclcpp/rclcpp.hpp>
#include <type_traits>
#include <vector>
#include <array>

namespace ros2_schema_tools {

/**
 * @brief Type traits for ROS2 parameter types
 */
template<typename T>
struct ParameterTypeTraits {
  static constexpr bool is_supported = false;
};

template<>
struct ParameterTypeTraits<bool> {
  static constexpr bool is_supported = true;
  static constexpr auto type = rclcpp::ParameterType::PARAMETER_BOOL;
  static bool get(const rclcpp::Parameter & p) { return p.as_bool(); }
};

template<>
struct ParameterTypeTraits<int64_t> {
  static constexpr bool is_supported = true;
  static constexpr auto type = rclcpp::ParameterType::PARAMETER_INTEGER;
  static int64_t get(const rclcpp::Parameter & p) { return p.as_int(); }
};

template<>
struct ParameterTypeTraits<double> {
  static constexpr bool is_supported = true;
  static constexpr auto type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  static double get(const rclcpp::Parameter & p) { return p.as_double(); }
};

template<>
struct ParameterTypeTraits<std::string> {
  static constexpr bool is_supported = true;
  static constexpr auto type = rclcpp::ParameterType::PARAMETER_STRING;
  static std::string get(const rclcpp::Parameter & p) { return p.as_string(); }
};

/**
 * @brief Safe parameter getter with validation
 */
template<typename T>
T get_parameter_safe(
  rclcpp::Node * node,
  const std::string & name,
  const T & default_value)
{
  static_assert(
    ParameterTypeTraits<T>::is_supported,
    "Unsupported parameter type"
  );
  
  if (!node->has_parameter(name)) {
    return node->declare_parameter<T>(name, default_value);
  }
  
  return node->get_parameter(name).get_value<T>();
}

/**
 * @brief Validate parameter against constraints
 */
template<typename T>
void validate_range(
  const std::string & name,
  const T & value,
  const T & min,
  const T & max)
{
  if (value < min || value > max) {
    throw std::runtime_error(
      "Parameter '" + name + "' out of range [" +
      std::to_string(min) + ", " + std::to_string(max) + "], got " +
      std::to_string(value)
    );
  }
}

}  // namespace ros2_schema_tools
