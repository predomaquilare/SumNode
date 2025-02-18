#ifndef SUMMER_SCHOOL__CALCULATOR_NODE_HPP_
#define SUMMER_SCHOOL__CALCULATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64.hpp"
#include "memory"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace summer_school 
{
    class CalculatorNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        explicit CalculatorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~CalculatorNode() override;

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servico_;

        CallbackReturn on_configure(const rclcpp_lifecycle::State &);
        CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

        void getParameters();
        void configPubSub();
        void configTimers();
        void configService();

        void pubCallback();
        void subCallback(const std_msgs::msg::Float64MultiArray::SharedPtr number);
        void servCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        int operators;
        float _rate_;

        std_msgs::msg::Float64 result;
        std_msgs::msg::Float64MultiArray numbers;
        double numeros[2];
    };

} // namespace summer_school

#endif 
