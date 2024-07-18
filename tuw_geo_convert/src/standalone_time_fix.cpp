#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tuw_geo_convert/time_fix_node.hpp"

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  std::shared_ptr<tuw_geo::TimeFix> node =
    std::make_shared<tuw_geo::TimeFix>("time_fix");
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
