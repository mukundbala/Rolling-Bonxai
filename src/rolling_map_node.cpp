#include "rclcpp/rclcpp.hpp"
#include "rolling_map/rolling_map.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RM::RollingMapNode>());
    rclcpp::shutdown();
    return 0;
}