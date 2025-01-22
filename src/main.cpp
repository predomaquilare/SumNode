#include "rclcpp/rclcpp.hpp"
#include "summer_school/calculator_node.hpp"

int main(int argc, char **argv)
{
      rclcpp::init(argc, argv);
      auto node = std::make_shared<summer_school::CalculatorNode>();
      rclcpp::spin(node->get_node_base_interface());
      rclcpp::shutdown();
}
