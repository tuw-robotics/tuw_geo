#include "tuw_geo_map/geo_map_node.hpp"
#include <opencv2/imgcodecs.hpp>
#include <filesystem>

using std::placeholders::_1;
using std::placeholders::_2;

using namespace tuw_geo;

void getTransformFromStamped(const geometry_msgs::msg::TransformStamped& transform_stamped, tf2::Transform& tf_transform) {
    // Set translation
    tf2::Vector3 translation;
    translation.setX(transform_stamped.transform.translation.x);
    translation.setY(transform_stamped.transform.translation.y);
    translation.setZ(transform_stamped.transform.translation.z);
    tf_transform.setOrigin(translation);

    // Set rotation
    tf2::Quaternion rotation;
    rotation.setX(transform_stamped.transform.rotation.x);
    rotation.setY(transform_stamped.transform.rotation.y);
    rotation.setZ(transform_stamped.transform.rotation.z);
    rotation.setW(transform_stamped.transform.rotation.w);
    tf_transform.setRotation(rotation);
}

void setTransformToStamped(const tf2::Transform& tf_transform, geometry_msgs::msg::TransformStamped& transform_stamped) {
    // Get translation
    tf2::Vector3 translation = tf_transform.getOrigin();
    transform_stamped.transform.translation.x = translation.x();
    transform_stamped.transform.translation.y = translation.y();
    transform_stamped.transform.translation.z = translation.z();

    // Get rotation
    tf2::Quaternion rotation = tf_transform.getRotation();
    transform_stamped.transform.rotation.x = rotation.x();
    transform_stamped.transform.rotation.y = rotation.y();
    transform_stamped.transform.rotation.z = rotation.z();
    transform_stamped.transform.rotation.w = rotation.w();
}


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

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if(mapimage_folder_.empty()){
    RCUTILS_LOG_ERROR_NAMED(get_name(), "mapimage_folder not definded");
  }
  pub_map_img_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("geo_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  read_mapimage(mapimage_folder_);
  RCUTILS_LOG_INFO_NAMED(get_name(), "%s", info_.info_geo().c_str());
  

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
  if (publish_tf_)
  {
    tf2::Transform tf_utm_map;
    geometry_msgs::msg::TransformStamped tf_msg_utm_relativ;
    tf_utm_map.setOrigin(tf2::Vector3(info_.utm()[0], info_.utm()[1], info_.utm()[2]));
    tf_utm_map.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    bool relative_frame_found = false;
    if(!frame_relative_.empty()) {
      // Look up for the transformation between map and utm frames
      try {
        tf_msg_utm_relativ = tf_buffer_->lookupTransform(frame_utm_, frame_relative_,  tf2::TimePointZero);
        relative_frame_found = true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          frame_utm_.c_str(), frame_relative_.c_str(), ex.what());
          return;
      }
    }
    if (relative_frame_found){
      /// publisch relative -> map  (without altitute)
      tf2::Transform tf_utm_relativ;
      getTransformFromStamped(tf_msg_utm_relativ, tf_utm_relativ);
      tf2::Transform tf_relativ_map = tf_utm_relativ.inverseTimes(tf_utm_map);

      geometry_msgs::msg::TransformStamped tf_msg_relativ_map;
      tf_msg_relativ_map.header.stamp = this->get_clock()->now();
      tf_msg_relativ_map.header.frame_id = frame_relative_;
      tf_msg_relativ_map.child_frame_id = frame_map_;
      setTransformToStamped(tf_relativ_map, tf_msg_relativ_map);
      tf_msg_relativ_map.transform.translation.z = 0;  /// remove altitute
      RCLCPP_INFO_ONCE(this->get_logger(), "publish TF: frame_id: %s, child_frame_id: %s", tf_msg_relativ_map.header.frame_id.c_str(), tf_msg_relativ_map.child_frame_id.c_str());
      tf_broadcaster_->sendTransform(tf_msg_relativ_map);
    } else {
      /// publisch utm -> map
      geometry_msgs::msg::TransformStamped tf_msg_utm_map;
      tf_msg_utm_map.header.stamp = this->get_clock()->now();
      tf_msg_utm_map.header.frame_id = frame_utm_;
      tf_msg_utm_map.child_frame_id = frame_map_;
      setTransformToStamped(tf_utm_map, tf_msg_utm_map);
      RCLCPP_INFO_ONCE(this->get_logger(), "publish TF: frame_id: %s, child_frame_id: %s", tf_msg_utm_map.header.frame_id.c_str(), tf_msg_utm_map.child_frame_id.c_str());
      tf_broadcaster_->sendTransform(tf_msg_utm_map);
    }
  }
}

void GeoMapNode::read_mapimage(const std::string &mapimage)
{
  std::string mapimage_filename = mapimage + std::string("mapimage.jpg");
  cv::Mat map_img = cv::imread(mapimage_filename, cv::IMREAD_GRAYSCALE);
  std::string geo_info_filename = mapimage + std::string("mapimage.jgw");
  tuw::WorldFile world_file;
  world_file.read_jgw(geo_info_filename);
  
  cv::Vec3d utm_bottom_left(0, 0, 0);
  utm_bottom_left[0] = world_file.coordinate_x;
  utm_bottom_left[1] = world_file.coordinate_y - map_img.size().height * world_file.resolution_x;

  info_.init(map_img.size(), world_file.resolution_x, tuw::MapHdl::BOTTOM_LEFT, utm_bottom_left, 33, true);

  cv::line(map_img, info_.w2m(tuw::Point2D(0,0)).p(), info_.w2m(tuw::Point2D(10, 20)).p(), cv::Scalar(0xFF), 1);

  occupancy_map_img_->header.frame_id = frame_map_;
  occupancy_map_img_->info.width = map_img.cols;
  occupancy_map_img_->info.height = map_img.rows;
  occupancy_map_img_->info.resolution = info_.resolution_x();
  occupancy_map_img_->info.origin.position.x = 0;
  occupancy_map_img_->info.origin.position.y = 0;
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
    descriptor.description = "name of the map frame, only need if publish_tf == true";
    this->declare_parameter<std::string>("frame_map", "geo_map", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "name of the utm frame, only need if publish_tf == true";
    this->declare_parameter<std::string>("frame_utm", "utm", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "If used a relative tf is published to the given frame by substracing frame_utm -> frame_map. Altitute will be set on zero";
    this->declare_parameter<std::string>("frame_relative", "", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "folder with the world file mapimage.jpw and the image mapimage.jpg";
    this->declare_parameter<std::string>("mapimage_folder", "", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "on true a tf from frame_utm to frame_map is published";
    this->declare_parameter<bool>("publish_tf", false, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "utm zone (Austria is 33)";
    this->declare_parameter<int>("utm_zone", false, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "utm northern hemisphere";
    this->declare_parameter<bool>("northp", true, descriptor);
  }
}

void GeoMapNode::read_parameters()
{
  this->get_parameter<std::string>("frame_map", frame_map_);
  this->get_parameter<std::string>("frame_utm", frame_utm_);
  this->get_parameter<std::string>("frame_relative", frame_relative_);
  this->get_parameter<int>("utm_zone", utm_zone_);
  this->get_parameter<bool>("northp", utm_northp_);
  this->get_parameter<bool>("publish_tf", publish_tf_);
  RCLCPP_INFO(this->get_logger(), "publish_tf: %s",
              (publish_tf_ ? " true: frame_utm -> frame_map is published" : " false: not tf is published"));
  this->get_parameter<std::string>("mapimage_folder", mapimage_folder_);
  RCLCPP_INFO(this->get_logger(), "mapimage_folder: %s", mapimage_folder_.c_str());
}
