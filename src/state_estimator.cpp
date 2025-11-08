#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace aerosimian
{

class AeroSimianStateEstimatorComponent : public rclcpp::Node
{
public:
  explicit AeroSimianStateEstimatorComponent(const rclcpp::NodeOptions & options)
  : Node("aerosimian_state_estimator_component", options)
  {
    subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "pendulum_markers",
      10,
      std::bind(&AeroSimianStateEstimatorComponent::topic_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "AeroSimianStateEstimatorComponent constructed");
  }

private:
  void topic_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received MarkerArray with %zu markers", msg->markers.size());
  }

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
};

}  // namespace aerosimian

// Register component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(aerosimian::AeroSimianStateEstimatorComponent)

