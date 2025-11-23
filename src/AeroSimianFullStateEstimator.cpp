#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "aerosimian/msg/aero_simian_state.hpp"

#include "moteus.h"  // <-- IMPORTANT

using namespace std::chrono_literals;

class AeroSimianPhiStateNode : public rclcpp::Node {
public:
  AeroSimianPhiStateNode()
  : rclcpp::Node("aerosimian_full_state_node"),
    controller_(make_controller_options())
  {
    // Clear any existing faults.
    controller_.SetStop();

    // Subscribe to theta/theta_dot
    theta_sub_ = this->create_subscription<aerosimian::msg::AeroSimianState>(
      "/aerosimian/theta_state",
      10,
      std::bind(&AeroSimianPhiStateNode::thetaCallback, this, std::placeholders::_1));

    // Publish full state
    state_pub_ = this->create_publisher<aerosimian::msg::AeroSimianState>(
      "/aerosimian/state", 10);

    // Timer to poll moteus and publish combined state
    timer_ = this->create_wall_timer(
      20ms,  // 50 Hz
      std::bind(&AeroSimianPhiStateNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "AeroSimianPhiStateNode started");


  }

private:
  // Helper to build Options for member initialization
  static mjbots::moteus::Controller::Options make_controller_options() {
    mjbots::moteus::Controller::Options opt;
    opt.id = 1;   // set your servo id
    return opt;
  }

  void thetaCallback(const aerosimian::msg::AeroSimianState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(theta_mutex_);
    latest_theta_state_ = *msg;
    have_theta_ = true;
  }

  void timerCallback() {
    // Query moteus: leave fields as NaN to avoid changing command
    mjbots::moteus::PositionMode::Command cmd;
    cmd.position = std::numeric_limits<double>::quiet_NaN();
    cmd.velocity = std::numeric_limits<double>::quiet_NaN();
    cmd.feedforward_torque = std::numeric_limits<double>::quiet_NaN();

    const auto maybe_result = controller_.SetPosition(cmd);
    if (!maybe_result) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "No response from moteus controller");
      return;
    }

    const auto & r = maybe_result->values;

    constexpr double TWO_PI = 2.0 * M_PI;
    float phi     = static_cast<float>(r.position * TWO_PI);  // [rad]
    float phi_dot = static_cast<float>(r.velocity * TWO_PI);  // [rad/s]

    aerosimian::msg::AeroSimianState out_msg;
    out_msg.header.stamp = this->now();

    {
      std::lock_guard<std::mutex> lock(theta_mutex_);
      if (have_theta_) {
        out_msg.header.frame_id = latest_theta_state_.header.frame_id;
        out_msg.theta           = latest_theta_state_.theta;
        out_msg.theta_dot       = latest_theta_state_.theta_dot;
      } else {
        out_msg.header.frame_id = "world";
        out_msg.theta           = 0.0f;
        out_msg.theta_dot       = 0.0f;
      }
    }

    out_msg.phi     = phi;
    out_msg.phi_dot = phi_dot;

    state_pub_->publish(out_msg);

    // Store last full state safely
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_state_ = out_msg;
    }


    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "moteus: mode=%d, phi=%.4f rad, phi_dot=%.4f rad/s, torque=%.4f Nm",
      static_cast<int>(r.mode),
      phi,
      phi_dot,
      static_cast<double>(r.torque));
  }

  void driveMoteus(float position) {
    mjbots::moteus::PositionMode::Command cmd;
    cmd.position = position;     
    cmd.velocity = 1.0;            // commanded velocity [rev/s]

    const auto maybe_result = controller_.SetPosition(cmd);

    if (!maybe_result) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Failed to send drivePhiToPosition command to moteus");
      return;
    }

    const auto & r = maybe_result->values;
    RCLCPP_DEBUG(
      this->get_logger(),
      "drivePhiToPosition → target=%.4f  | current=%.4f rev | vel=%.4f",
      position,
      r.position,
      r.velocity);
 }


  // ROS bits
  rclcpp::Subscription<aerosimian::msg::AeroSimianState>::SharedPtr theta_sub_;
  rclcpp::Publisher<aerosimian::msg::AeroSimianState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // theta storage
  std::mutex theta_mutex_;
  aerosimian::msg::AeroSimianState latest_theta_state_;
  bool have_theta_ = false;

  // moteus controller (note full namespace)
  mjbots::moteus::Controller controller_;
   aerosimian::msg::AeroSimianState last_state_;
  std::mutex state_mutex_;  
};

int main(int argc, char ** argv) {
  // Let moteus parse its CLI args (transport, etc.)
  mjbots::moteus::Controller::DefaultArgProcess(argc, argv);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<AeroSimianPhiStateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

