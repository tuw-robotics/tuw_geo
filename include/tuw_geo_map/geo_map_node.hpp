#ifndef TUW_GEO_MAP__GEO_MAP_NODE_HPP_
#define TUW_GEO_MAP__GEO_MAP_NODE_HPP_

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
  class GeoMapNode : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    explicit GeoMapNode(const std::string & node_name, bool intra_process_comms = false);

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
    rclcpp::TimerBase::SharedPtr timer_loop_;
    rclcpp::TimerBase::SharedPtr timer_parameters_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_img_;

    // callbacks
    void callback_timer();
    void callback_geo_point();

    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map_img_;
    tuw::GeoMapMetaData info_;
    std::string topic_map_;
    std::string frame_map_;
    std::string frame_utm_;
    std::string mapimage_folder_;
    bool publish_utm_;
    void declare_parameters();
    void read_parameters();

    void publish_transforms();
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void read_mapimage(const std::string &mapimage);
  };
}
#endif // TUW_GEO_MAP__GEO_MAP_NODE_HPP_
