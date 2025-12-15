#include <rclcpp/rclcpp.hpp>
#include "DummyPerceptionParams.hpp"  // Auto-generated

class DummyPerceptionPublisherNode : public rclcpp::Node {
public:
  DummyPerceptionPublisherNode()
  : Node("dummy_perception_publisher")
  {
    // Load parameters using generated loader
    params_.declare_and_load(this);
    params_.load(this);
    
    // Print loaded parameters
    params_.print(this->get_logger());
    
    // Access parameters with type safety
    RCLCPP_INFO(
      this->get_logger(),
      "Visible range: %.2f m", 
      params_.visible_range
    );
    
    RCLCPP_INFO(
      this->get_logger(),
      "Detection rate: %.2f", 
      params_.detection_successful_rate
    );
    
    // Access nested parameters
    RCLCPP_INFO(
      this->get_logger(),
      "Vehicle max remapping distance: %.2f m",
      params_.vehicle.max_remapping_distance
    );
    
    RCLCPP_INFO(
      this->get_logger(),
      "Vehicle path selection: %s",
      params_.vehicle.path_selection_strategy.c_str()
    );
  }

private:
  dummy_perception::DummyPerceptionParams params_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<DummyPerceptionPublisherNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("dummy_perception"), 
                 "Failed to start node: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
