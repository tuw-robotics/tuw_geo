#ifndef TUW_GEO_CONVERT__GEO_CONVERT_NODE_HPP_
#define TUW_GEO_CONVERT__GEO_CONVERT_NODE_HPP_

#include "tuw/node.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>


namespace tuw_geo
{
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class GeoConvert : public tuw::Node
{
  public:
    GeoConvert(const std::string & node_name);

  private:
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_gps_utm_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_gps_map_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;

  private: 
    geometry_msgs::msg::PoseWithCovarianceStamped &gps_to_utm(const sensor_msgs::msg::NavSatFix &gps, geometry_msgs::msg::PoseWithCovarianceStamped &des);
    geometry_msgs::msg::PoseWithCovarianceStamped &utm_to_map(const geometry_msgs::msg::PoseWithCovarianceStamped &gps, geometry_msgs::msg::PoseWithCovarianceStamped &des);

    void callback_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::string frame_map_;
    std::string frame_utm_;
    int utm_zone_;
    bool utm_northp_;
    bool use_msg_time_;
    void declare_parameters();
    void read_static_parameters();
    bool read_dynamic_parameters();
};
}

#endif // TUW_GEO_CONVERT__GEO_CONVERT_NODE_HPP_
