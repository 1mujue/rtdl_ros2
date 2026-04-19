#include <rtdl_demo_bt_test/nav_to.hpp>
NavToNode::NavToNode(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params) : Base(name, conf, params){}
BT::PortsList NavToNode::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<double>("tolerance"),
        BT::InputPort<double>("y"),
        BT::InputPort<double>("x"),
        BT::InputPort<std::string>("robot_name"),
        BT::OutputPort<std::string>("message"),
        BT::OutputPort<bool>("success"),
    });
}
bool NavToNode::setRequest(Request::SharedPtr& request)
{
    std::string robot_name;
    if(!getInput("robot_name", robot_name)){ return false; }
    request->robot_name = robot_name;
    double x;
    if(!getInput("x", x)){ return false; }
    request->x = x;
    double y;
    if(!getInput("y", y)){ return false; }
    request->y = y;
    double tolerance;
    if(!getInput("tolerance", tolerance)){ return false; }
    request->tolerance = tolerance;
    return true;
}
BT::NodeStatus NavToNode::onResponseReceived(const Response::SharedPtr& response)
{
    setOutput("message", response->message);
    setOutput("success", response->success);
    return response->success? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
BT::NodeStatus NavToNode::onFailure(BT::ServiceNodeErrorCode err)
{
    return BT::NodeStatus::FAILURE;
}
