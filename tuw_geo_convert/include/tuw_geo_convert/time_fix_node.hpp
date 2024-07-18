#ifndef TUW_GEO_CONVERT__TIME_FIX_NODE_HPP_
#define TUW_GEO_CONVERT__TIME_FIX_NODE_HPP_

#include "tuw_ros2_utils/node.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>


namespace tuw_geo
{
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TimeFix : public tuw::Node
{
  public:
    TimeFix(const std::string & node_name);

  private:

  private: 
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps_now_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_now_;

    void callback_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
    
    void declare_parameters();
    void read_static_parameters();
    bool read_dynamic_parameters();
};
}

#endif // TUW_GEO_CONVERT__TIME_FIX_NODE_HPP_
