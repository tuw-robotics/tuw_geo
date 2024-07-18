#include <tuw_geo_convert/time_fix_node.hpp>

using namespace tuw_geo;
using std::placeholders::_1;

TimeFix::TimeFix(const std::string &node_name)
    : Node(node_name)
{
  declare_parameters();
  read_static_parameters();
  read_dynamic_parameters();
  pub_gps_now_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("now/gps", 10);
  sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "gps", 10, std::bind(&TimeFix::callback_gps, this, _1));

  pub_imu_now_ = this->create_publisher<sensor_msgs::msg::Imu>("now/imu", 10);
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&TimeFix::callback_imu, this, _1));
}

void TimeFix::callback_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  static unsigned long count = 0;
  RCLCPP_INFO(this->get_logger(), "callback_gps: %4lu", count++);
  sensor_msgs::msg::NavSatFix msg_now = *msg;
  msg_now.header.stamp = get_clock()->now();
  pub_gps_now_->publish(msg_now);
}

void TimeFix::callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  static unsigned long count = 0;
  RCLCPP_INFO(this->get_logger(), "callback_imu: %4lu", count++);
  sensor_msgs::msg::Imu msg_now = *msg;
  msg_now.header.stamp = get_clock()->now();
  pub_imu_now_->publish(msg_now);
}

void TimeFix::declare_parameters()
{
}
void TimeFix::read_static_parameters()
{
}
bool TimeFix::read_dynamic_parameters()
{
  return true;
}