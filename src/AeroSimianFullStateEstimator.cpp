#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <Eigen/Dense>

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
    k_p_phi_ = this->declare_parameter<double>("k_p_phi", 0.0);
    k_d_phi_ = this->declare_parameter<double>("k_d_phi", 0.0);

    grav_normalization_term_ = this->declare_parameter<double>("grav_normalization_term", 1.0);
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
    RCLCPP_INFO_STREAM(get_logger(), "here 1");
    float phi     = static_cast<float>((r.position- moteus_center_revs_) * TWO_PI);  // [rad]
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
    if (std::isnan(this->theta_des_) || !have_theta_ ) {
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
    float phi = state_copy.phi;
    float theta_dot = state_copy.theta_dot;
    float phi_dot = state_copy.phi_dot;


    // Assume control_timer_ is 100 Hz → dt = 0.01 s
    constexpr float dt = 0.01f;
    const double length  = .3175; // m
    const double gravity  = 9.81; // m/s^2

    Eigen::Vector2d current_state;      // 2x1 vector of doubles
    current_state << length *cos(theta), length * sin(theta);
    Eigen::Vector2d desired_state;      // 2x1 vector of doubles
    desired_state << length *cos(theta_des_), length * sin(theta_des_);

    double a_x = desired_state(0) - current_state(0);
    double a_y =   desired_state(1) - current_state(1)  ;
    RCLCPP_INFO_STREAM(get_logger(), "ax: " << a_x);
    RCLCPP_INFO_STREAM(get_logger(), "ay: " << a_y);

    double grav_term = sin(theta) * gravity;
    a_x *= grav_normalization_term_;
    a_y *= grav_normalization_term_;
    RCLCPP_INFO_STREAM(get_logger(), "ax: " << a_x);
    RCLCPP_INFO_STREAM(get_logger(), "ay: " << a_y);
    a_y += std::abs(grav_term);

    RCLCPP_INFO_STREAM(get_logger(), "grav_term: " << grav_term);
    RCLCPP_INFO_STREAM(get_logger(), "ay: " << a_y);
    // get desired phi angle

    Eigen::Vector2d acceleration_vec;      // 2x1 vector of doubles
    acceleration_vec << a_x, a_y;

    double phi_des = acceleration_vec.dot(current_state) ;
    phi_des = std::acos(phi_des / (acceleration_vec.norm() * current_state.norm()));
    RCLCPP_INFO_STREAM(get_logger(), "phi des: " << phi_des);
    phi_des = phi_des - M_PI/2;
    RCLCPP_INFO_STREAM(get_logger(), "phi des after removing pi/2: " << phi_des);

    // get thrust from linear gain on acceleration vector
    double thrust_gain = .75;
    double thrust = sqrt(std::pow(a_x, 2.0) + std::pow(a_y, 2.0));
    thrust = thrust * thrust_gain;

    RCLCPP_INFO_STREAM(get_logger(), "Theta: " << theta);
    RCLCPP_INFO_STREAM(get_logger(), "desired_state: " << desired_state);
    RCLCPP_INFO_STREAM(get_logger(), "current_state: " << current_state);
    RCLCPP_INFO_STREAM(get_logger(), "Thrust: " << thrust);

    // phi controller now
    double phi_error = phi_des - phi;
    double phi_double_dot = k_p_phi_ * phi_error - k_d_phi_ * phi_dot;
    const double I = .006; // wild estimate
    double torque_des = I * phi_double_dot;

    const double rotor_arm_length = 0.094; // m
    double thrust_left = thrust/2 - (torque_des/(2*rotor_arm_length));
    double thrust_right = thrust - thrust_left;

    RCLCPP_INFO_STREAM(get_logger(), "phi_des: " << phi_des);
    RCLCPP_INFO_STREAM(get_logger(), "phi: " << phi);
    RCLCPP_INFO_STREAM(get_logger(), "phi_error: " << phi_error);
    RCLCPP_INFO_STREAM(get_logger(), "k_p_phi_: " << k_p_phi_);
    RCLCPP_INFO_STREAM(get_logger(), "phi_double_dot: " << phi_double_dot);

    RCLCPP_INFO_STREAM(get_logger(), "torque_des: " << torque_des);
    RCLCPP_INFO_STREAM(get_logger(), "thrust_left: " << thrust_left);
    RCLCPP_INFO_STREAM(get_logger(), "thrust_right: " << thrust_right);

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

  float theta_des_ = -M_PI/2;
  float k_p_phi_ = 0.0;
  float k_d_phi_ = 0.0;
  rclcpp::TimerBase::SharedPtr control_timer_;
  // PID state (bottom hemisphere)
  float theta_error_int_ = 0.0f;   // integral of error
  float prev_theta_error_ = 0.0f;  // previous error
  bool  have_prev_error_ = false;  // for first-iteration handling

  float grav_normalization_term_ = 1.0;
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
