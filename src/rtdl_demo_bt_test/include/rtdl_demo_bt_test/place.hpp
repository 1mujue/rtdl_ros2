#include <behaviortree_ros2/bt_service_node.hpp>
#include <rtdl_demo_interfaces/srv/place.hpp>
class PlaceNode: public BT::RosServiceNode<rtdl_demo_interfaces::srv::Place>
{
public:
    using ServiceType = rtdl_demo_interfaces::srv::Place;
    using Base = BT::RosServiceNode<ServiceType>;
    using Request = typename ServiceType::Request;
    using Response = typename ServiceType::Response;
    PlaceNode(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params);
    static BT::PortsList providedPorts();
    bool setRequest(typename Request::SharedPtr& req);
    BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& res);
    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode err);
};
