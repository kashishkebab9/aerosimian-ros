#include <memory>
#include <string>
#include <eigen3/Eigen/Dense>
#include <limits> 

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "aerosimian/msg/aero_simian_state.hpp"  // <-- change to your package/msg name


namespace aerosimian
{

class AeroSimianThetaStateEstimatorComponent : public rclcpp::Node
{
public:
  explicit AeroSimianThetaStateEstimatorComponent(const rclcpp::NodeOptions & options)
  : Node("aerosimian_state_estimator_component", options)
  {
    pendulum_markers_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "pendulum_markers",
      10,
      std::bind(&AeroSimianThetaStateEstimatorComponent::pendulum_markers_cb, 
                                                    this, 
                                                    std::placeholders::_1));
    state_pub_ = this->create_publisher<aerosimian::msg::AeroSimianState>(
      "/aerosimian/theta_state", 10);


    RCLCPP_INFO(this->get_logger(), "AeroSimianThetaStateEstimatorComponent constructed");
    RCLCPP_INFO(this->get_logger(), "Ensure AeroSimian is hanging at around 0 \
        degrees theta for proper initialization");

  }

private:


  void pendulum_markers_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (!initialized_) {
      msg_count_ ++;
    }
    if (!initialized_ && msg_count_ == msg_threshold_) {
      if (debug_) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Received MarkerArray with markers");
        for (auto i: msg->poses) {
          RCLCPP_INFO_STREAM(this->get_logger(), i.position.x);
          RCLCPP_INFO_STREAM(this->get_logger(), i.position.y);
          RCLCPP_INFO_STREAM(this->get_logger(), i.position.z);
          RCLCPP_INFO_STREAM(this->get_logger(), "==========");
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "%%%%%%%%%%");
      }

      // First Step: find the two highest z value points in the marker array:
      float highest_z = 0;
      float second_highest_z = 0;
      u_int8_t highest_z_ind = msg->poses.size();
      u_int8_t second_highest_z_ind = 0;
      for (u_int8_t i = 0; i < msg->poses.size(); i++) {
        float z_val = msg->poses[i].position.z;
        if (z_val > highest_z) {
          highest_z = z_val;
          highest_z_ind = i;
        }
      }
      for (u_int8_t i = 0; i < msg->poses.size(); i++) {
        float z_val = msg->poses[i].position.z;
        if (z_val > second_highest_z && z_val < highest_z) {
          second_highest_z = z_val;
          second_highest_z_ind = i;
        }
      }

      if (debug_) {
        RCLCPP_INFO_STREAM(this->get_logger(), highest_z);
        RCLCPP_INFO_STREAM(this->get_logger(), highest_z_ind);
        RCLCPP_INFO_STREAM(this->get_logger(), second_highest_z);
        RCLCPP_INFO_STREAM(this->get_logger(), second_highest_z_ind);
        RCLCPP_INFO_STREAM(this->get_logger(), "==========");
      }

      // Third Step: grab the lowest Z marker
      theta_axes_indices_.first  = static_cast<uint32_t>(highest_z_ind);
      theta_axes_indices_.second = static_cast<uint32_t>(second_highest_z_ind);

      float lowest_z = std::numeric_limits<float>::max();
      lowest_z_ind_ = msg->poses.size();
      for (u_int8_t i = 0; i < msg->poses.size(); i++) {
        float z_val = msg->poses[i].position.z;
        if (z_val < lowest_z) {
          lowest_z = z_val;
          lowest_z_ind_ = i;
        }
      }

      initialized_ = true;
    }

    if (debug_) {
      RCLCPP_INFO_STREAM(this->get_logger(), theta_axes_indices_.first );
      RCLCPP_INFO_STREAM(this->get_logger(), theta_axes_indices_.second );
      RCLCPP_INFO_STREAM(this->get_logger(), lowest_z_ind_);
      RCLCPP_INFO_STREAM(this->get_logger(), "%%%%%%%%%%%%%");
    }

    // MAIN Loop: we have to take the theta_axes_indices_ and lowest_z_ind_
    // and output our theta

    if (!initialized_) {
      return;
    }

    const size_t N = msg->poses.size();
    if (theta_axes_indices_.first >= N ||
        theta_axes_indices_.second >= N ||
        lowest_z_ind_ >= N) {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Marker indices out of range, skipping theta computation");
      return;
    }

    // Positions
    const auto &p1_pose = msg->poses[theta_axes_indices_.first].position;
    const auto &p2_pose = msg->poses[theta_axes_indices_.second].position;
    const auto &p3_pose = msg->poses[lowest_z_ind_].position;

    // find the midpoint between p1 and p2:
    float midpoint_x = (p1_pose.x + p2_pose.x) / 2.0f;
    float midpoint_y = (p1_pose.y + p2_pose.y) / 2.0f;
    float midpoint_z = (p1_pose.z + p2_pose.z) / 2.0f;

    // Find L1: "downward" reference from midpoint
    float lower_point_x = midpoint_x;
    float lower_point_y = midpoint_y;
    float lower_point_z = midpoint_z - 100.0f;

    // midpoint
    Eigen::Vector3f midpoint(midpoint_x, midpoint_y, midpoint_z);

    // define lower point
    Eigen::Vector3f lower_point(lower_point_x, lower_point_y, lower_point_z);

    // points from message
    Eigen::Vector3f p1(p1_pose.x, p1_pose.y, p1_pose.z);
    Eigen::Vector3f p2(p2_pose.x, p2_pose.y, p2_pose.z);
    Eigen::Vector3f p3(p3_pose.x, p3_pose.y, p3_pose.z);

    // Define vectors:
    // L1: midpoint → lower_point
    Eigen::Vector3f L1 = lower_point - midpoint;

    // L2: midpoint → p3
    Eigen::Vector3f L2 = p3 - midpoint;

    const float eps = 1e-6f;
    float normL1 = L1.norm();
    float normL2 = L2.norm();

    if (normL1 < eps || normL2 < eps) {
      RCLCPP_WARN_STREAM(this->get_logger(), "L1 or L2 too small to compute angle");
      return;
    }

    Eigen::Vector3f ref_axis = p2 - p1;
    Eigen::Vector3f ref_axis_normal = ref_axis.normalized();

    float term_1 = ref_axis_normal.dot(L1.cross(L2));
    float term_2 = L1.dot(L2);

    float signed_angle_rad = std::atan2(term_1, term_2);
    float signed_angle_deg = signed_angle_rad * 180.0f / M_PI;

    if (debug_) {
      RCLCPP_INFO_STREAM(this->get_logger(), "ref_axis_normal: " << ref_axis_normal);
      RCLCPP_INFO_STREAM(this->get_logger(), "signed_angle_rad: " << signed_angle_rad);
      RCLCPP_INFO_STREAM(this->get_logger(), "signed_angle_deg: " << signed_angle_deg);
      RCLCPP_INFO_STREAM(this->get_logger(), "p1_x: " << p1_pose.x);
      RCLCPP_INFO_STREAM(this->get_logger(), "p1_y: " << p1_pose.y);
      RCLCPP_INFO_STREAM(this->get_logger(), "p1_z: " << p1_pose.z);
      RCLCPP_INFO_STREAM(this->get_logger(), "p2_x: " << p2_pose.x);
      RCLCPP_INFO_STREAM(this->get_logger(), "p2_y: " << p2_pose.y);
      RCLCPP_INFO_STREAM(this->get_logger(), "p2_z: " << p2_pose.z);
      RCLCPP_INFO_STREAM(this->get_logger(), "midpoint_x: " << midpoint_x);
      RCLCPP_INFO_STREAM(this->get_logger(), "midpoint_y: " << midpoint_y);
      RCLCPP_INFO_STREAM(this->get_logger(), "midpoint_z: " << midpoint_z);
      RCLCPP_INFO_STREAM(this->get_logger(), "");
      RCLCPP_INFO_STREAM(this->get_logger(), "p3_x: " << p3_pose.x);
      RCLCPP_INFO_STREAM(this->get_logger(), "p3_y: " << p3_pose.y);
      RCLCPP_INFO_STREAM(this->get_logger(), "p3_z: " << p3_pose.z);
      RCLCPP_INFO_STREAM(this->get_logger(), "L1: " << L1);
      RCLCPP_INFO_STREAM(this->get_logger(), "L2: " << L2);
      RCLCPP_INFO_STREAM(this->get_logger(), "%%%%%%%%%%%%%");
    }

    // === Theta and theta_dot computation ===
    float theta = signed_angle_rad;

    // Use mocap time if available, otherwise fallback to node time
    rclcpp::Time stamp;
    if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) {
      stamp = msg->header.stamp;
    } else {
      stamp = this->now();
    }

    float theta_dot_raw = 0.0f;
    double dt = 0.0;

    // Finite-difference for raw theta_dot
    if (!have_prev_theta_) {
      have_prev_theta_ = true;
      prev_theta_ = theta;
      prev_stamp_ = stamp;
      theta_dot_raw = 0.0f;
      dt = 0.0;
    } else {
      dt = (stamp - prev_stamp_).seconds();
      if (dt > 1e-4) {
        // Wrap angle difference into [-pi, pi] to avoid jumps over 2*pi
        float dtheta = std::atan2(
          std::sin(theta - prev_theta_),
          std::cos(theta - prev_theta_)
        );
        theta_dot_raw = static_cast<float>(dtheta / dt);
      } else {
        theta_dot_raw = 0.0f;
      }
    }

    // --- Low-pass filter on theta_dot (first-order IIR) ---
    // Time constant (seconds) - tune this
    const float tau = 0.15f;  // ~50 ms
    float alpha = 0.0f;

    if (dt > 1e-4) {
      alpha = static_cast<float>(dt) / (tau + static_cast<float>(dt));
    } else {
      alpha = 1.0f;  // on first useful sample, just take raw
    }

    if (!have_theta_dot_filtered_) {
      theta_dot_filtered_ = theta_dot_raw;
      have_theta_dot_filtered_ = true;
    } else {
      theta_dot_filtered_ =
        theta_dot_filtered_ + alpha * (theta_dot_raw - theta_dot_filtered_);
    }

    // Clamp absurd spikes even after filtering (safety)
    const float MAX_THETA_DOT = 50.0f;  // rad/s, tune if needed
    if (theta_dot_filtered_ >  MAX_THETA_DOT) theta_dot_filtered_ =  MAX_THETA_DOT;
    if (theta_dot_filtered_ < -MAX_THETA_DOT) theta_dot_filtered_ = -MAX_THETA_DOT;

    // Save for next iteration
    prev_theta_ = theta;
    prev_stamp_ = stamp;

    float theta_dot = theta_dot_filtered_;

    // === Fill and publish state message ===
    aerosimian::msg::AeroSimianState state_msg;
    state_msg.header.stamp = stamp;
    state_msg.header.frame_id = "world";  // or your mocap frame

    state_msg.theta     = theta;      // radians
    state_msg.theta_dot = theta_dot;  // filtered rad/s
    state_msg.phi       = 0.0f;       // TODO
    state_msg.phi_dot   = 0.0f;       // TODO

    state_pub_->publish(state_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pendulum_markers_sub_;
  rclcpp::Publisher<aerosimian::msg::AeroSimianState>::SharedPtr state_pub_;
  bool debug_ = false;
  bool initialized_ = false;
  u_int8_t msg_count_ = 0;
  u_int8_t msg_threshold_ = 5; //  wait till this many msgs before starting init
  std::pair<u_int32_t, u_int32_t> theta_axes_indices_; // a pair of indices from the 
                                                    // pose array which defines
                                                    // the axis of our theta rotation

  u_int32_t lowest_z_ind_;
  // For theta_dot estimation
  bool have_prev_theta_ = false;
  float prev_theta_ = 0.0f;
  rclcpp::Time prev_stamp_;

  float theta_dot_filtered_ = 0.0f;
  bool have_theta_dot_filtered_ = false;

};

}  // namespace aerosimian

// Register component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(aerosimian::AeroSimianThetaStateEstimatorComponent)

