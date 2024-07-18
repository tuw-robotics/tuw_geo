#include <tuw_geo_convert/geo_convert_node.hpp>
#include <GeographicLib/UTMUPS.hpp>

using namespace tuw_geo;
using std::placeholders::_1;

GeoConvert::GeoConvert(const std::string &node_name)
    : Node(node_name)
{
  declare_parameters();
  read_static_parameters();
  read_dynamic_parameters();
  using namespace std::chrono_literals;
  pub_gps_utm_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("gps_utm", 10);
  pub_gps_map_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("gps_map", 10);
  timer_ = this->create_wall_timer(
      500ms, std::bind(&GeoConvert::timer_callback, this));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "gps", 10, std::bind(&GeoConvert::callback_gps, this, _1));
}

void GeoConvert::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "timer_callback");
}

void GeoConvert::callback_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "gps: %12.10f°, %12.10f°, %12.10fm", msg->latitude, msg->longitude, msg->altitude);
  geometry_msgs::msg::PoseWithCovarianceStamped utm;
  gps_to_utm(*msg, utm);
  utm.pose.pose.orientation.x = +0.7071068;
  utm.pose.pose.orientation.y = +0.0;
  utm.pose.pose.orientation.z = -0.7071068;
  utm.pose.pose.orientation.w = +0.0;
  utm.header.frame_id = frame_utm_;
  if (use_msg_time_)
    utm.header.stamp = msg->header.stamp;
  else
    utm.header.stamp = get_clock()->now();
  pub_gps_utm_->publish(utm);
  std::string zone = GeographicLib::UTMUPS::EncodeZone(this->utm_zone_, this->utm_northp_);
  RCLCPP_INFO(this->get_logger(), "utm: %12.10fm, %12.10fm, %12.10fm, frame: %s", utm.pose.pose.position.x, utm.pose.pose.position.y, utm.pose.pose.position.z, zone.c_str());


  geometry_msgs::msg::PoseWithCovarianceStamped map = utm;
  utm_to_map(utm, map);
  RCLCPP_INFO(this->get_logger(), "map: %12.10fm, %12.10fm, %12.10fm, frame: %s", map.pose.pose.position.x, map.pose.pose.position.y, map.pose.pose.position.z, frame_map_.c_str());

  map.header.frame_id = frame_map_;
  pub_gps_map_->publish(map);
}

geometry_msgs::msg::PoseWithCovarianceStamped &GeoConvert::gps_to_utm(const sensor_msgs::msg::NavSatFix &gps, geometry_msgs::msg::PoseWithCovarianceStamped &des)
{
  
  GeographicLib::UTMUPS::Forward(
      gps.latitude,
      gps.longitude,
      this->utm_zone_,
      this->utm_northp_,
      des.pose.pose.position.x,
      des.pose.pose.position.y);
  des.pose.pose.position.z = gps.altitude;
  return des;
}

geometry_msgs::msg::PoseWithCovarianceStamped &GeoConvert::utm_to_map(const geometry_msgs::msg::PoseWithCovarianceStamped &utm, geometry_msgs::msg::PoseWithCovarianceStamped &des)
{

    bool frame_found = false;
    tf2::Transform tf_utm_map;
    geometry_msgs::msg::TransformStamped msg_tf_utm_map;
    if(!frame_map_.empty()) {
      // Look up for the transformation between map and utm frames
      try {
        msg_tf_utm_map = tf_buffer_->lookupTransform(frame_utm_, frame_map_,  tf2::TimePointZero);
        frame_found = true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          frame_utm_.c_str(), frame_map_.c_str(), ex.what());
      }
    }
    if(frame_found){
      des = utm;
      des.pose.pose.position.x -= msg_tf_utm_map.transform.translation.x;
      des.pose.pose.position.y -= msg_tf_utm_map.transform.translation.y;
      des.pose.pose.position.z -= msg_tf_utm_map.transform.translation.z;
    }
  return des;
}

void GeoConvert::declare_parameters()
{
  declare_parameters_with_description("frame_map", "map", "Name of the map frame");
  declare_parameters_with_description("frame_utm", "utm", "Name of the utm frame");
  declare_parameters_with_description("use_msg_time", false, "on false it sets the stamp on ros::now otherwise on the incomming msg timestamp");
}
void GeoConvert::read_static_parameters()
{
  get_parameter_and_log("frame_utm", frame_utm_);
  get_parameter_and_log("frame_map", frame_map_);
  get_parameter_and_log("use_msg_time", use_msg_time_);
}
bool GeoConvert::read_dynamic_parameters()
{
  return true;
}