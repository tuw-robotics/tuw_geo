#include "tuw_geo_map/geo_map_node.hpp"
#include <opencv2/imgcodecs.hpp>
#include <filesystem>

using std::placeholders::_1;
using std::placeholders::_2;

using namespace tuw_geo_map;

GeoMapNode::GeoMapNode(const std::string & node_name, bool intra_process_comms)
: rclcpp_lifecycle::LifecycleNode(
    node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
  declare_parameters();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GeoMapNode::on_configure(const rclcpp_lifecycle::State &)
{
  //RCLCPP_INFO(get_logger(), "on_configure() is called.");
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
  read_parameters();
  occupancy_map_img_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
GeoMapNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
  LifecycleNode::on_activate(state);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


  if(mapimage_folder_.empty()){
    RCUTILS_LOG_ERROR_NAMED(get_name(), "mapimage_folder not definded");
  }
  pub_map_img_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("geo_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  read_mapimage(mapimage_folder_);
  RCUTILS_LOG_ERROR_NAMED(get_name(), "%s", info_.info_geo().c_str());
  

  using namespace std::chrono_literals;
  timer_loop_ = create_wall_timer(1000ms, std::bind(&GeoMapNode::callback_timer, this));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GeoMapNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_deactivate(state);

  RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
  // pub_graph_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
GeoMapNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  // In our cleanup phase, we release the shared pointers to the
  // timer and publisher. These entities are no longer available
  // and our node is "clean".

  RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
  timer_loop_->reset();
  tf_broadcaster_.reset();
  pub_map_img_.reset();
  occupancy_map_img_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
GeoMapNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called from state %s.", state.label().c_str());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


void GeoMapNode::callback_timer()
{
  RCLCPP_INFO(this->get_logger(), "callback_timer");
  publish_transforms();

  if(occupancy_map_img_ && pub_map_img_){
    pub_map_img_->publish(*occupancy_map_img_);
  }
}


void GeoMapNode::publish_transforms()
{
  if (publish_utm_)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = frame_utm_;
    tf.child_frame_id = frame_map_;
    tf.transform.translation.x = info_.utm()[0];
    tf.transform.translation.y = info_.utm()[1];
    tf.transform.translation.z = info_.utm()[2];
    RCLCPP_INFO_ONCE(this->get_logger(), "publish TF: frame_id: %s, child_frame_id: %s", tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
    tf_broadcaster_->sendTransform(tf);
  }
}

void GeoMapNode::read_mapimage(const std::string &mapimage)
{
  std::string mapimage_filename = mapimage + std::string("mapimage.jpg");
  cv::Mat map_img = cv::imread(mapimage_filename, cv::IMREAD_GRAYSCALE);
  std::string geo_info_filename = mapimage + std::string("mapimage.jgw");
  tuw::WorldFile world_file;
  world_file.read_jgw(geo_info_filename);
  info_.size = map_img.size();
  info_.resolution = world_file.resolution_x;
  int zone = 33;
  bool northp = true;
  info_.init(world_file.coordinate_x, world_file.coordinate_y, 0.0, zone, northp);

  occupancy_map_img_->header.frame_id = frame_map_;
  occupancy_map_img_->info.width = map_img.cols;
  occupancy_map_img_->info.height = map_img.rows;
  occupancy_map_img_->info.resolution = info_.resolution;
  occupancy_map_img_->info.origin.position.x = info_.origin.x();
  occupancy_map_img_->info.origin.position.y = info_.origin.y() - map_img.rows * info_.resolution;
  occupancy_map_img_->info.origin.position.z = 0;
  occupancy_map_img_->data.resize(map_img.cols * map_img.rows);
  cv::Mat_<int8_t> img_des = cv::Mat(map_img.size(), CV_8S, &occupancy_map_img_->data[0]);
  for (int r = 0; r < img_des.rows; r++)
    for (int c = 0; c < img_des.cols; c++)
      img_des(img_des.rows - r - 1, c) = map_img.at<uint8_t>(r, c); /// fix negative y
      
}


void GeoMapNode::declare_parameters()
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
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "frame_utm only need if publish_utm == true";
    this->declare_parameter<std::string>("frame_utm", "utm", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "folder with the world file mapimage.jpw and the image mapimage.jpg";
    this->declare_parameter<std::string>("mapimage_folder", "", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "on true the maps are published in relative to utm, otherwise with a zero link to frame_map";
    this->declare_parameter<bool>("publish_utm", false, descriptor);
  }
}

void GeoMapNode::read_parameters()
{
  this->get_parameter<std::string>("topic_map", topic_map_);
  RCLCPP_INFO(this->get_logger(), "topic_map: %s", topic_map_.c_str());
  this->get_parameter<std::string>("frame_map", frame_map_);
  this->get_parameter<std::string>("frame_utm", frame_utm_);
  this->get_parameter<bool>("publish_utm", publish_utm_);
  RCLCPP_INFO(this->get_logger(), "publish_utm: %s",
              (publish_utm_ ? " true -> maps are published in utm" : " false -> maps are published in map"));
  this->get_parameter<std::string>("mapimage_folder", mapimage_folder_);
  RCLCPP_INFO(this->get_logger(), "mapimage_folder: %s", mapimage_folder_.c_str());


}
