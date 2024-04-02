#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tuw_geo_objects/geo_objects_node.hpp"

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  std::shared_ptr<tuw_geo_map::GeoObjectsNode> node =
    std::make_shared<tuw_geo_map::GeoObjectsNode>("geo_objects");
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
