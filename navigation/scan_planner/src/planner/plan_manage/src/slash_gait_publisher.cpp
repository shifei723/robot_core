#include <array>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace scan_planner
{
class SlashGaitPublisher : public rclcpp::Node
{
public:
  SlashGaitPublisher() : Node("slash_gait_publisher")
  {
    const double rate = declare_parameter<double>("rate", 60.0);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SlashGaitPublisher::odomCallback, this, std::placeholders::_1));
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, rate)),
        std::bind(&SlashGaitPublisher::timerCallback, this));

    // Slash hip joints (4 position-controlled joints)
    joint_msg_.name = {
        "lf_joint1", "lb_joint1",
        "rf_joint1", "rb_joint1",
        "l_wheel_joint", "r_wheel_joint"};
    joint_msg_.position.resize(joint_msg_.name.size(), 0.0);
    joint_msg_.velocity.resize(joint_msg_.name.size(), 0.0);

    // Default stand pose (hip joints in neutral position)
    // lf_joint1=0.5, lb_joint1=0.5, rf_joint1=0.5, rb_joint1=0.5
    // wheels=0
    stand_pose_ = {0.5, 0.5, 0.5, 0.5, 0.0, 0.0};

    RCLCPP_INFO(get_logger(), "Slash gait publisher ready");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr /*odom*/)
  {
    has_odom_ = true;
  }

  void timerCallback()
  {
    const auto stamp = now();
    for (size_t i = 0; i < stand_pose_.size(); ++i)
    {
      joint_msg_.position[i] = stand_pose_[i];
      joint_msg_.velocity[i] = 0.0;
    }
    joint_msg_.header.stamp = stamp;
    joint_pub_->publish(joint_msg_);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState joint_msg_;
  std::array<double, 6> stand_pose_;
  bool has_odom_{false};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::SlashGaitPublisher>());
  rclcpp::shutdown();
  return 0;
}