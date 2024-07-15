#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tuw_geo_convert/geo_convert_node.hpp"

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  std::shared_ptr<tuw_geo::GeoConvert> node =
    std::make_shared<tuw_geo::GeoConvert>("geo_convert");
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
