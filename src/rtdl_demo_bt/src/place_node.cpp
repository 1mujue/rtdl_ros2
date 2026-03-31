#include "rtdl_demo_bt/place_node.hpp"

namespace rtdl_demo_bt
{

PlaceNode::PlaceNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & ros_node)
: BtServiceNodeBase(name, config, ros_node)
{
  client_ = ros_node_->create_client<rtdl_demo_interfaces::srv::Place>("/place");
}

BT::PortsList PlaceNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("robot_name"),
    BT::InputPort<std::string>("object_name"),
    BT::InputPort<std::string>("location_name"),
    BT::OutputPort<std::string>("message"),
    BT::OutputPort<bool>("success"),
  };
}

BT::NodeStatus PlaceNode::tick()
{
  auto robot_name = getInput<std::string>("robot_name");
  auto object_name = getInput<std::string>("object_name");
  auto location_name = getInput<std::string>("location_name");

  if (!object_name) {
    throw BT::RuntimeError("missing required input [object_name]: ", object_name.error());
  }
  if (!location_name) {
    throw BT::RuntimeError("missing required input [location_name]: ", location_name.error());
  }

  if (!wait_for_service<rtdl_demo_interfaces::srv::Place>(client_, "/place")) {
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<rtdl_demo_interfaces::srv::Place::Request>();
  request->robot_name = robot_name.value();
  request->object_name = object_name.value();
  request->location_name = location_name.value();

  rtdl_demo_interfaces::srv::Place::Response::SharedPtr response;
  if (!call_service_sync<rtdl_demo_interfaces::srv::Place>(client_, request, response)) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Call to /place failed.");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "Place(object=%s, location=%s) -> success=%s, msg=%s",
    request->object_name.c_str(),
    request->location_name.c_str(),
    response->success ? "true" : "false",
    response->message.c_str());

  setOutput("message", response->message);
  setOutput("success", response->success);

  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rtdl_demo_bt