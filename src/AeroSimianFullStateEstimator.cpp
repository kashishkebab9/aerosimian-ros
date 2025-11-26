#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "aerosimian/msg/aero_simian_state.hpp"
#include "aerosimian/msg/aero_simian_control.hpp"

#include "moteus.h"  // <-- IMPORTANT
#include "aerosimian/vertiq.hpp"

using namespace std::chrono_literals;

class AeroSimianPhiStateNode : public rclcpp::Node {
public:
  AeroSimianPhiStateNode()
  : rclcpp::Node("aerosimian_phi_state_node"),
    controller_(make_controller_options())
  {
    vertiq_port_ = this->declare_parameter("vertiq_port", "/dev/ttyUSB0");
    vertiq_baud_ = this->declare_parameter("vertiq_baud", 115200);
    k_p_bottom_half_ = this->declare_parameter<double>("k_p_bottom_half", 0.0);
    k_d_bottom_half_ = this->declare_parameter<double>("k_d_bottom_half", 0.0);
    k_i_bottom_half_ = this->declare_parameter<double>("k_i_bottom_half", 0.0);
    RCLCPP_INFO_STREAM(get_logger(), "P gain for bottom half controller: " << k_p_bottom_half_);
    RCLCPP_INFO_STREAM(get_logger(), "I gain for bottom half controller: " << k_i_bottom_half_);
    RCLCPP_INFO_STREAM(get_logger(), "D gain for bottom half controller: " << k_d_bottom_half_);
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

    vertiq_ = std::make_unique<VertiqHeartbeat>(vertiq_port_, vertiq_baud_);

    if (vertiq_->start()) {
      RCLCPP_INFO(get_logger(), "Vertiq heartbeat started");
    } else {
      RCLCPP_WARN(get_logger(), "Failed to open Vertiq at %s", vertiq_port_.c_str());
    }

    // === Hardware test: single drive to phi = 0 ===
    // drive_test_timer_ = this->create_wall_timer(
    //   2s,
    //   [this]() {
    //     if (test_drive_done_) {
    //       return;
    //     }
    //     RCLCPP_INFO(this->get_logger(),
    //                 "Hardware test: calling driveMoteus(0.0 rad)");
    //     driveMoteus(0.0f);
    //     test_drive_done_ = true;
    //   });
    // vertiq_test_timer_ = this->create_wall_timer(
    //    2s,  // run once, 2 seconds after startup
    //   [this]() {
    //     if (vertiq_ && vertiq_->isOpen() && !vertiq_test_done_) {
    //       RCLCPP_INFO(this->get_logger(),
    //                   "Hardware test: sending SET 50,50,50,50");
    //       vertiq_->sendSet(50, 50, 50, 50);
    //       vertiq_test_done_ = true;
    //     }
    //   });

    // Closed Loop Bottom Hemisphere Control
    // 1. Create a timer that runs at 100hz
    // 2. Write a service client that writes to member variables that initialize as nan

    control_timer_ = this->create_wall_timer(
        10ms,   // 100 Hz period
        std::bind(&AeroSimianPhiStateNode::control_loop_bottom_half, this)
    );




    RCLCPP_INFO(this->get_logger(), "AeroSimianPhiStateNode started");
  }

  ~AeroSimianPhiStateNode() override {
    if (vertiq_) vertiq_->stop();
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

  void driveMoteus(float phi) {
    // phi in radians
    constexpr double TWO_PI = 2.0 * M_PI;
    float moteus_center_rads = moteus_center_revs_  * TWO_PI;
    mjbots::moteus::PositionMode::Command cmd;
    cmd.position = phi + moteus_center_rads ;
    cmd.position = cmd.position/TWO_PI;
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
      phi,
      r.position,
      r.velocity);
  }

  void control_loop_bottom_half() {
    RCLCPP_INFO_STREAM(get_logger(), "here 1");
    // Don’t do anything until someone sets desired angles
    if (std::isnan(this->theta_des_) &&  std::isnan(this->phi_des_)) {
        RCLCPP_INFO_STREAM(get_logger(), "here 2");
      return;
    }
        RCLCPP_INFO_STREAM(get_logger(), "here 3");

    // === 1. Get current theta (optionally under a mutex) ===
    // If you want to be strict about thread-safety:
    aerosimian::msg::AeroSimianState state_copy;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_copy = last_state_;
    }
    float theta = state_copy.theta;

    // === 2. Error term ===
    float e_theta;
    if (this->theta_des_ < 0) {
            if (theta < 0) {
                    e_theta = -1 *(theta_des_ - theta);
            } else {
                    e_theta = std::abs(theta_des_ - theta);
            }
    } else {
            e_theta = theta_des_ - theta;
            }
    RCLCPP_INFO_STREAM(get_logger(), "e_theta: " << e_theta);

    // Assume control_timer_ is 100 Hz → dt = 0.01 s
    constexpr float dt = 0.01f;

    // === 3. Integral of error ===
    theta_error_int_ += e_theta * dt;

    // (Optional) anti-windup on integral term
    const float I_MAX = 100.0f;  // tune this
    if (theta_error_int_ > I_MAX)  theta_error_int_ = I_MAX;
    if (theta_error_int_ < -I_MAX) theta_error_int_ = -I_MAX;

    // === 4. Derivative of error ===
    float e_theta_dot = 0.0f;
    if (have_prev_error_) {
      e_theta_dot = (e_theta - prev_theta_error_) / dt;
    }
    prev_theta_error_ = e_theta;
    have_prev_error_ = true;

    // === 5. PID control law ===
    float u = 0.0f;
    u += this->k_p_bottom_half_ * e_theta;
    u += this->k_i_bottom_half_ * theta_error_int_;
    u += this->k_d_bottom_half_ * e_theta_dot;

    RCLCPP_INFO_STREAM(get_logger(), "u_output before offset " << u);
    float u_offset = 56 * sin(theta_des_);
    if (u_offset < 0) {
            u_offset = -1 * u_offset;
    }
    u += u_offset;
    RCLCPP_INFO_STREAM(get_logger(), "u_output after offset " << u);
    // === 6. Clamp u to be within [-62, 62] ===
    const float U_MIN = 0.0f;
    const float U_MAX = 62.0f;
    if (u > U_MAX)  u = U_MAX;
    if (u < U_MIN) u = U_MIN;

    RCLCPP_INFO_STREAM(get_logger(), "u_output after clamping " << u);

    if (vertiq_ && vertiq_->isOpen() && !vertiq_test_done_) {
      RCLCPP_INFO(this->get_logger(),
                  "Hardware test: sending SET 50,50,50,50");
      vertiq_->sendSet(u, u, u, u);
      this->phi_des_ = -1 * theta_des_;
      driveMoteus(phi_des_);

    }

    return;

  }



  // ROS bits
  rclcpp::Subscription<aerosimian::msg::AeroSimianState>::SharedPtr theta_sub_;
  rclcpp::Publisher<aerosimian::msg::AeroSimianState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string vertiq_port_;
  int vertiq_baud_;
  std::unique_ptr<VertiqHeartbeat> vertiq_;

  // theta storage
  std::mutex theta_mutex_;
  aerosimian::msg::AeroSimianState latest_theta_state_;
  bool have_theta_ = false;

  // moteus controller (note full namespace)
  mjbots::moteus::Controller controller_;
  aerosimian::msg::AeroSimianState last_state_;
  std::mutex state_mutex_;
  float moteus_center_revs_ = .38;

  // TEST
  rclcpp::TimerBase::SharedPtr drive_test_timer_;
  bool test_drive_done_ = false;
  rclcpp::TimerBase::SharedPtr vertiq_test_timer_;
  bool vertiq_test_done_ = false;

  float theta_des_ = -1.57;
  float phi_des_ = std::numeric_limits<float>::quiet_NaN();
  float k_p_bottom_half_ = 0.0;
  float k_d_bottom_half_ = 0.0;
  float k_i_bottom_half_ = 0.0;
  rclcpp::TimerBase::SharedPtr control_timer_;
  // PID state (bottom hemisphere)
  float theta_error_int_ = 0.0f;   // integral of error
  float prev_theta_error_ = 0.0f;  // previous error
  bool  have_prev_error_ = false;  // for first-iteration handling

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

