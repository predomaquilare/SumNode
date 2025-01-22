#include "rclcpp/rclcpp.hpp"
#include "codes/calculadoranode.hpp"

int main(int argc, char **argv)
{
      rclcpp::init(argc, argv);
      auto node = std::make_shared<calculadora::MainNode>();
      rclcpp::spin(node->get_node_base_interface());
      rclcpp::shutdown();
}
