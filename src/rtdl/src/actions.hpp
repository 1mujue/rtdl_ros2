#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/exceptions.h>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

class Init : public BT::SyncActionNode {
public:
    Init(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("item"),
            BT::InputPort<std::string>("room"),
            BT::OutputPort<bool>("success")
        };
    }

    BT::NodeStatus tick() override {
        auto it = getInput<std::string>("item");
        auto ro = getInput<std::string>("room");

        if (!it || !ro) {
            throw BT::RuntimeError("missing required input port");
        }

        std::cout << "At Init with item: " << it.value()
                  << ", room: " << ro.value() << '\n';

        setOutput("success", true);
        return BT::NodeStatus::SUCCESS;
    }
};

class LogMessage: public BT::SyncActionNode{
public:
    LogMessage(const std::string& name,
    const BT::NodeConfig& config,
    const rclcpp::Node::SharedPtr& ros_node)
    : BT::SyncActionNode(name, config), ros_node_(ros_node){}
    static BT::PortsList providedPorts(){
        return {
            BT::InputPort<bool>("message")
        };
    }
    BT::NodeStatus tick() override{
        auto msg = getInput<bool>("message");
        if (!msg)
        {
        throw BT::RuntimeError("missing required input [message]: ", msg.error());
        }
        if(msg){
            RCLCPP_INFO(ros_node_->get_logger(), "ROS2 node get Init successfully!");
        } else {
            RCLCPP_INFO(ros_node_->get_logger(), "ROS2 node get Init failed!");
        }
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Node::SharedPtr ros_node_;
};