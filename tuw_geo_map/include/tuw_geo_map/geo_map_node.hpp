#ifndef TUW_GEO_MAP__GEO_MAP_NODE_HPP_
#define TUW_GEO_MAP__GEO_MAP_NODE_HPP_

#include <memory>
#include <opencv2/core/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <tuw_geometry/geo_handler.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace tuw_geo
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
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // callbacks
    void callback_timer();
    void callback_geo_point();

    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map_img_;
    tuw::GeoHdl info_;
    std::string frame_map_;
    std::string frame_utm_;
    std::string frame_relative_;
    std::string mapimage_folder_;
    int utm_zone_;
    bool utm_northp_;
    bool publish_tf_;
    int pub_interval_;
    void declare_parameters();
    void read_parameters();

    void publish_transforms();
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void read_mapimage(const std::string &mapimage);
  };
}
#endif // TUW_GEO_MAP__GEO_MAP_NODE_HPP_
