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
    k_p_theta_ = this->declare_parameter<double>("k_p_theta", 1.0);
    k_d_theta_ = this->declare_parameter<double>("k_d_theta", 1.0);
    k_p_phi_ = this->declare_parameter<double>("k_p_phi", 1.0);
    k_d_phi_ = this->declare_parameter<double>("k_d_phi", 1.0);
    thrust_gain_ = this->declare_parameter<double>("thrust_gain", 0.5);
    I_term_ = this->declare_parameter<double>("I_term", 0.005);

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


    const auto maybe_result = controller_.SetQuery();
    if (!maybe_result) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "No response from moteus controller");
      return;
    }

    const auto & r = maybe_result->values;

    constexpr double TWO_PI = 2.0 * M_PI;
    constexpr double moteus_center = 0.88;  // bottom in moteus frame

    float phi     = static_cast<float>((r.position + (1-moteus_center ))*TWO_PI); // a* TWO_PI/ [rad]
    float phi_dot = static_cast<float>(r.velocity * -TWO_PI);  // [rad/s]

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

    const auto maybe_result = controller_.SetQuery();

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
    // Don’t do anything until someone sets desired angles
    if (std::isnan(this->theta_des_) || !have_theta_ ) {
      return;
    }

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

    Eigen::Vector2d e = desired_state - current_state;

    double theta_dot_des = 0.0;   // unless you have a trajectory

    // Compute current Cartesian velocity
    Eigen::Vector2d xdot;
    xdot << -length * std::sin(theta) * theta_dot,
             length * std::cos(theta) * theta_dot;

    // Compute desired Cartesian velocity
    Eigen::Vector2d xdot_des;
    xdot_des << -length * std::sin(theta_des_) * theta_dot_des,
                 length * std::cos(theta_des_) * theta_dot_des;

    // Error rate: e_dot = xdot_des - xdot
    Eigen::Vector2d e_dot = xdot_des - xdot;

    RCLCPP_INFO_STREAM(get_logger(), "e x: " << e(0)
                                                 << "  e y: " << e(1));
    RCLCPP_INFO_STREAM(get_logger(), "e_dot x: " << e_dot(0)
                                                 << "  e_dot y: " << e_dot(1));
    Eigen::Vector2d acceleration_vec =
      k_p_theta_ * e
    + k_d_theta_ * e_dot;

    RCLCPP_INFO_STREAM(get_logger(), "acceleration_vec x: " << acceleration_vec(0)
                                                 << "  acceleration_vec y: " << acceleration_vec(1));
    // gravity comp
    double grav_term = gravity * std::sin(theta);
    acceleration_vec(0) -= std::abs(grav_term);
    RCLCPP_INFO_STREAM(get_logger(), "acceleration_vec after grav x: " << acceleration_vec(0)
                                                 << "  acceleration_vec after grav y: " << acceleration_vec(1));


    // get desired phi angle
    // === Signed angle from current_state -> acceleration_vec ===
    // double dot   = current_state.dot(acceleration_vec);
    // double cross = current_state(0) * acceleration_vec(1)
    //              - current_state(1) * acceleration_vec(0);

    constexpr double TWO_PI = 2.0 * M_PI;
    // double phi_des = std::atan2(cross, dot);  // in (-pi,
    double phi_des = std::atan2(acceleration_vec(1), acceleration_vec(0));  // in (-pi,

    if (phi_des < 0) {
            phi_des =TWO_PI - std::abs(phi_des);
    }



    // get thrust from linear gain on acceleration vector
    double thrust = sqrt(std::pow(acceleration_vec(0), 2.0) + std::pow(acceleration_vec(1), 2.0));
    thrust = thrust * thrust_gain_;

    RCLCPP_INFO_STREAM(get_logger(), "Theta: " << theta);
    RCLCPP_INFO_STREAM(get_logger(), "Theta_dot: " << theta_dot);
    RCLCPP_INFO_STREAM(get_logger(), "desired_state: " << desired_state);
    RCLCPP_INFO_STREAM(get_logger(), "current_state: " << current_state);
    RCLCPP_INFO_STREAM(get_logger(), "Thrust: " << thrust);

    // phi controller now
    double phi_error = phi_des - phi;
    double phi_double_dot = k_p_phi_ * phi_error - k_d_phi_ * phi_dot;
    double torque_des = I_term_ * phi_double_dot;

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

    double thrust_0 = thrust_right / 2;
    double thrust_1 = thrust_left / 2;
    double thrust_3 = thrust_left / 2;
    double thrust_2 = thrust_right / 2;

    double pwm_0 = thrustToDuty(thrust_0);
    double pwm_1 = thrustToDuty(thrust_1);
    double pwm_2 = thrustToDuty(thrust_2);
    double pwm_3 = thrustToDuty(thrust_3);
    pwm_0 *= 100;
    pwm_1 *= 100;
    pwm_2 *= 100;
    pwm_3 *= 100;
    RCLCPP_INFO_STREAM(get_logger(), "pwm_0: " << pwm_0);
    RCLCPP_INFO_STREAM(get_logger(), "pwm_1: " << pwm_1);
    RCLCPP_INFO_STREAM(get_logger(), "pwm_2: " << pwm_2);
    RCLCPP_INFO_STREAM(get_logger(), "pwm_3: " << pwm_3);
    // convert to pwm

    if (vertiq_ && vertiq_->isOpen() && !vertiq_test_done_) {
      vertiq_->sendSet(pwm_0, pwm_1,pwm_2, pwm_3);
    }

    return;

  }

  double thrustToDuty(double x) {
    // Clamp input for safety
    if (x < 0.0) x = 0.0;
    if (x > 0.8) x = 0.8;

    return 0.431 + 0.0964 * x - 6.79e-03 * x * x;
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
  float moteus_center_revs_ = .88;

  rclcpp::TimerBase::SharedPtr drive_test_timer_;
  bool test_drive_done_ = false;
  rclcpp::TimerBase::SharedPtr vertiq_test_timer_;
  bool vertiq_test_done_ = false;

  float k_p_theta_ = 1.0;
  float k_d_theta_ = 1.0;
  float theta_des_ = -M_PI/2;
  float k_p_phi_ = 1.0;
  float k_d_phi_ = 1.0;
  rclcpp::TimerBase::SharedPtr control_timer_;
  float theta_error_int_ = 0.0f;   // integral of error
  float prev_theta_error_ = 0.0f;  // previous error
  bool  have_prev_error_ = false;  // for first-iteration handling

  float thrust_gain_ = 1.0;
  float I_term_ = 0.005;
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
