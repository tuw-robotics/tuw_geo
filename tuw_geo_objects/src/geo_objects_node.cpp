#include "tuw_geo_objects/geo_objects_node.hpp"
#include <opencv2/imgcodecs.hpp>
#include <filesystem>

using std::placeholders::_1;
using std::placeholders::_2;

using namespace tuw_geo_map;

GeoObjectsNode::GeoObjectsNode(const std::string & node_name, bool intra_process_comms)
: rclcpp_lifecycle::LifecycleNode(
    node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
  declare_parameters();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GeoObjectsNode::on_configure(const rclcpp_lifecycle::State &)
{
  //RCLCPP_INFO(get_logger(), "on_configure() is called.");
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
  read_parameters();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
GeoObjectsNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
  LifecycleNode::on_activate(state);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  using namespace std::chrono_literals;
  timer_loop_ = create_wall_timer(1000ms, std::bind(&GeoObjectsNode::callback_timer, this));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GeoObjectsNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_deactivate(state);
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
GeoObjectsNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  // In our cleanup phase, we release the shared pointers to the
  // timer and publisher. These entities are no longer available
  // and our node is "clean".

  RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
  if(timer_loop_) timer_loop_->reset();
  if(tf_broadcaster_) tf_broadcaster_.reset();
  if(pub_occupancy_grid_) pub_occupancy_grid_.reset();
  if(occupancy_grid_) occupancy_grid_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
GeoObjectsNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called from state %s.", state.label().c_str());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


void GeoObjectsNode::callback_timer()
{
  RCLCPP_INFO(this->get_logger(), "callback_timer");
  if(pub_occupancy_grid_ && occupancy_grid_){
    pub_occupancy_grid_->publish(*occupancy_grid_);
  }
}

void GeoObjectsNode::declare_parameters()
{
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "topic name for the geo map";
    this->declare_parameter<std::string>("topic_map", "geo_map", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "frame_map";
    this->declare_parameter<std::string>("frame_map", "geo_map", descriptor);
  }
}

void GeoObjectsNode::read_parameters()
{
  this->get_parameter<std::string>("topic_map", topic_map_);
  RCLCPP_INFO(this->get_logger(), "topic_map: %s", topic_map_.c_str());
  this->get_parameter<std::string>("frame_map", frame_map_);
  RCLCPP_INFO(this->get_logger(), "frame_map: %s", frame_map_.c_str());
}
