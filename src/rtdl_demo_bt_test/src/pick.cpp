#include <rtdl_demo_bt_test/pick.hpp>
PickNode::PickNode(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params) : Base(name, conf, params){}
BT::PortsList PickNode::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<std::string>("object_name"),
        BT::InputPort<std::string>("robot_name"),
        BT::OutputPort<std::string>("message"),
        BT::OutputPort<bool>("success"),
    });
}
bool PickNode::setRequest(Request::SharedPtr& request)
{
    std::string robot_name;
    if(!getInput("robot_name", robot_name)){ return false; }
    request->robot_name = robot_name;
    std::string object_name;
    if(!getInput("object_name", object_name)){ return false; }
    request->object_name = object_name;
    return true;
}
BT::NodeStatus PickNode::onResponseReceived(const Response::SharedPtr& response)
{
    setOutput("message", response->message);
    setOutput("success", response->success);
    return response->success? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
BT::NodeStatus PickNode::onFailure(BT::ServiceNodeErrorCode err)
{
    return BT::NodeStatus::FAILURE;
}
