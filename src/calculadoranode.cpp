#include "codes/calculadoranode.hpp"

namespace calculadora
{

  MainNode::MainNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("MainNode", "", options)
  {
    numeros[0] = 0;
    numeros[1] = 0;
    operators = 1;
    declare_parameter("rate.pub_result", rclcpp::ParameterValue(1.0));
  }

  MainNode::~MainNode() {

  };

  CallbackReturn MainNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "configurate");
    getParameters();
    configPubSub();
    configTimers();
    configService();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MainNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "activate");
    publisher_->on_activate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MainNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "deactivate");
    publisher_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MainNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "reset");
    publisher_.reset();
    timer_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MainNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
  }

  void MainNode::getParameters()
  {
    RCLCPP_INFO(get_logger(), "configurando node");
    this->get_parameter("rate.pub_result", _rate_);
  }

  void MainNode::configPubSub()
  {
    subscriber_ = create_subscription<std_msgs::msg::Float64MultiArray>("op_numbers", 10, std::bind(&MainNode::subCallback, this, std::placeholders::_1));
    publisher_ = create_publisher<std_msgs::msg::Float64>("op_resullt", 10);
  }

  void MainNode::configTimers()
  {
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_), std::bind(&MainNode::pubCallback, this));
  }

  void MainNode::configService()
  {
    servico_ = create_service<std_srvs::srv::Trigger>("change_op", std::bind(&MainNode::servCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

  void MainNode::pubCallback()
  {
    switch (operators)
    {
    case 1:
      RCLCPP_INFO(get_logger(), "Adicao");
      result.data = numeros[0] + numeros[1];
      break;

    case 2:
      RCLCPP_INFO(get_logger(), "Subtracao");
      result.data = numeros[0] - numeros[1];
      break;

    case 3:
      RCLCPP_INFO(get_logger(), "Multiplicacao");
      result.data = numeros[0] * numeros[1];
      break;

    case 4:
      RCLCPP_INFO(get_logger(), "Divisao");
      result.data = numeros[0] / numeros[1];
      break;
    }

    publisher_->publish(result);
  };

  void MainNode::subCallback(const std_msgs::msg::Float64MultiArray::SharedPtr number)
  {
    numeros[0] = number->data[0];
    numeros[1] = number->data[1];
  }

  void MainNode::servCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    response->success = true;
    response->message = "operacao trocada";
    operators++;
    if (operators > 4)
      operators = 1;
  }

};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(calculadora::MainNode)
