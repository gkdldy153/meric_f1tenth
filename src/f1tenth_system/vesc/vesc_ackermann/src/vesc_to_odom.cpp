// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
// OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-
// Copyright 2020 F1TENTH Foundation
//
// …(동일한 라이선스 블록 생략)…

#include "vesc_ackermann/vesc_to_odom.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include <algorithm>   // std::min / std::max
#include <cmath>
#include <limits>      // NaN/Inf 체크
#include <string>

namespace vesc_ackermann
{

using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

/* ────────────────────────────── 유틸 ────────────────────────────── */
static inline bool finite_num(double v) { return std::isfinite(v); }

/* ─────────────────────────── 클래스 구현 ────────────────────────── */
VescToOdom::VescToOdom(const rclcpp::NodeOptions & options)
: Node("vesc_to_odom_node", options),
  odom_frame_("odom"),
  base_frame_("base_link"),
  use_servo_cmd_(true),
  publish_tf_(true),
  x_(0.0),
  y_(0.0),
  yaw_(0.0)
{
  /* 파라미터 */
  odom_frame_    = declare_parameter("odom_frame", odom_frame_);
  base_frame_    = declare_parameter("base_frame", base_frame_);
  use_servo_cmd_ = declare_parameter(
                     "use_servo_cmd_to_calc_angular_velocity", use_servo_cmd_);

  speed_to_erpm_gain_   = declare_parameter("speed_to_erpm_gain").get<double>();
  speed_to_erpm_offset_ = declare_parameter("speed_to_erpm_offset").get<double>();

  if (use_servo_cmd_) {
    steering_to_servo_gain_   = declare_parameter(
                                  "steering_angle_to_servo_gain").get<double>();
    steering_to_servo_offset_ = declare_parameter(
                                  "steering_angle_to_servo_offset").get<double>();
    wheelbase_ = declare_parameter("wheelbase").get<double>();
  }

  publish_tf_ = declare_parameter("publish_tf", publish_tf_);

  /* 통신 설정 */
  odom_pub_ = create_publisher<Odometry>("odom", 10);

  if (publish_tf_) {
    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  vesc_state_sub_ = create_subscription<VescStateStamped>(
    "sensors/core", 10, std::bind(&VescToOdom::vescStateCallback, this, _1));

  if (use_servo_cmd_) {
    servo_sub_ = create_subscription<Float64>(
      "sensors/servo_position_command", 10,
      std::bind(&VescToOdom::servoCmdCallback, this, _1));
  }
}

/* ─────────────────────── VESC 상태 콜백 ─────────────────────── */
void VescToOdom::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  if (use_servo_cmd_ && !last_servo_cmd_) return;

  /* ① 선속도 */
  double current_speed = (state->state.speed - speed_to_erpm_offset_)
                         / speed_to_erpm_gain_;
  if (std::fabs(current_speed) < 0.05) current_speed = 0.0;

  /* ② 조향각 / 각속도 */
  double current_steering_angle  = 0.0;
  double current_angular_velocity = 0.0;

  if (use_servo_cmd_) {
    current_steering_angle =
      (last_servo_cmd_->data - steering_to_servo_offset_)
      / steering_to_servo_gain_;

    /* C++14 에서 clamp 대체 → min/max */
    constexpr double MAX_STEER = 0.6981317008;    /* 40 deg */
    if (current_steering_angle >  MAX_STEER) current_steering_angle =  MAX_STEER;
    if (current_steering_angle < -MAX_STEER) current_steering_angle = -MAX_STEER;

    current_angular_velocity =
      current_speed * std::tan(current_steering_angle) / wheelbase_;
  }

  /* ③ 첫 호출 초기화 */
  if (!last_state_) last_state_ = state;

  /* ④ Δt */
  auto dt = rclcpp::Time(state->header.stamp)
            - rclcpp::Time(last_state_->header.stamp);

  /* ⑤ 적분 */
  double x_dot = current_speed * std::cos(yaw_);
  double y_dot = current_speed * std::sin(yaw_);

  x_   += x_dot * dt.seconds();
  y_   += y_dot * dt.seconds();
  yaw_ += current_angular_velocity * dt.seconds();

  /* ⑥ NaN / Inf 보호 */
  if (!finite_num(x_) || !finite_num(y_) || !finite_num(yaw_)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "yaw_ is NaN/Inf → reset to 0");
    x_ = y_ = yaw_ = 0.0;
  }

  last_state_ = state;

  /* ⑦ Odometry 메시지 */
  Odometry odom;
  odom.header.frame_id = odom_frame_;
  odom.header.stamp    = state->header.stamp;
  odom.child_frame_id  = base_frame_;

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
  odom.pose.pose.orientation.w = std::cos(yaw_ / 2.0);

  odom.pose.covariance[0]  = 0.2;
  odom.pose.covariance[7]  = 0.2;
  odom.pose.covariance[35] = 0.4;

  odom.twist.twist.linear.x  = current_speed;
  odom.twist.twist.angular.z = current_angular_velocity;

  constexpr double sigma_vx = 0.05, sigma_vy = 0.05, sigma_wz = 0.01;
  odom.twist.covariance[0]  = sigma_vx * sigma_vx;
  odom.twist.covariance[7]  = sigma_vy * sigma_vy;
  odom.twist.covariance[35] = sigma_wz * sigma_wz;

  /* ⑧ TF 브로드캐스트 */
  if (publish_tf_) {
    TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation      = odom.pose.pose.orientation;
    tf_pub_->sendTransform(tf);
  }

  odom_pub_->publish(odom);
}

/* ─────────────────── 서보 명령 콜백 ─────────────────── */
void VescToOdom::servoCmdCallback(const Float64::SharedPtr servo)
{
  last_servo_cmd_ = servo;
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)

