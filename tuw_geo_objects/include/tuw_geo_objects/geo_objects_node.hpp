#ifndef TUW_GEO_OBJECTS__GEO_OBJECTS_NODE_HPP_
#define TUW_GEO_OBJECTS__GEO_OBJECTS_NODE_HPP_

#include <memory>
#include <opencv2/core/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <tuw_geometry/geo_map.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace tuw_geo_map
{
  class GeoObjectsNode : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    explicit GeoObjectsNode(const std::string & node_name, bool intra_process_comms = false);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state);

  private:
    std::string topic_map_;
    std::string frame_map_;
    rclcpp::TimerBase::SharedPtr timer_loop_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_grid_;


    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // callbacks
    void callback_timer();

    // functions
    void declare_parameters();
    void read_parameters();
    void publish_transforms();
  };
}
#endif // TUW_GEO_OBJECTS__GEO_OBJECTS_NODE_HPP_
