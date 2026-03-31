#include "rtdl_demo_bt/nav_to_node.hpp"

namespace rtdl_demo_bt{
NavToNode::NavToNode(
    const std::string& name,
    const BT::NodeConfig& config,
    const rclcpp::Node::SharedPtr& ros_node
) : BtServiceNodeBase(name, config, ros_node){
    client_ = ros_node_->create_client<rtdl_demo_interfaces::srv::NavTo>("/nav_to");
}
BT::PortsList NavToNode::providedPorts(){
    return {
        BT::InputPort<std::string>("robot_name"),
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"),
        BT::InputPort<double>("tolerance"),
        BT::OutputPort<std::string>("message"),
        BT::OutputPort<bool>("success"),
    };
}
BT::NodeStatus NavToNode::tick() {
    auto robot_name = getInput<std::string>("robot_name");
    auto x = getInput<double>("x"), y = getInput<double>("y"), tolerance = getInput<double>("tolerance");
    if(!wait_for_service<rtdl_demo_interfaces::srv::NavTo>(client_, "/nav_to")){
        return BT::NodeStatus::FAILURE;
    }
    auto req = std::make_shared<rtdl_demo_interfaces::srv::NavTo::Request>();
    req->robot_name = robot_name.value();
    req->x = x.value(), req->y = y.value(), req->tolerance = tolerance.value();

    rtdl_demo_interfaces::srv::NavTo::Response::SharedPtr res;
    if (!call_service_sync<rtdl_demo_interfaces::srv::NavTo>(client_, req, res)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Call to /nav_to failed.");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
    ros_node_->get_logger(),
    "NavTo(x=%lf,y=%lf) -> success=%s, msg=%s",
    req->x, req->y,
    res->success ? "true" : "false",
    res->message.c_str());

    setOutput("message", res->message);
    setOutput("success", res->success);

    return res->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
};
}