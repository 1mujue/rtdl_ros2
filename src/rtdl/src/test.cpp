#include "actions.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto ros_node = rclcpp::Node::make_shared("bt_ros2_demo");

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<Init>("Init");

    BT::NodeBuilder log_builder = 
    [ros_node](const std::string& name, const BT::NodeConfig& config){
        return std::make_unique<LogMessage>(name, config, ros_node);
    };

    factory.registerBuilder<LogMessage>("LogMessage", log_builder);

    auto blackboard = BT::Blackboard::create();
    blackboard->set("room", std::string(argv[1]));
    blackboard->set("item", std::string(argv[2]));
    
    try {
        const std::string pkg_share =
            ament_index_cpp::get_package_share_directory("rtdl");
        const std::string xml_path = pkg_share + "/trees/tree.xml";
        auto tree = factory.createTreeFromFile(xml_path, blackboard);
        BT::StdCoutLogger logger(tree);
        tree.tickWhileRunning();
    } catch (const std::exception& e) {
        std::cerr << "Failed: " << e.what() << '\n';
        return 1;
    }
    RCLCPP_INFO(ros_node->get_logger(), "Behavior finished.");

    rclcpp::shutdown();
    return 0;
}
