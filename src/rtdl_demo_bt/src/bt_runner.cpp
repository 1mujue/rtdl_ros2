#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "rtdl_demo_bt/nav_to_node.hpp"
#include "rtdl_demo_bt/pick_node.hpp"
#include "rtdl_demo_bt/place_node.hpp"

struct Param{
    std::string key;
    std::string value;
};
void handleParas(int argc, char** argv, std::vector<Param>& paras){
    std::cout << "argc: " << argc << '\n';
    for(int i = 1; i < argc; i += 2){
        std::cout << "argv: "  << argv[i] << '\n';
        for(auto& it: paras){
            if(argv[i] == it.key){
                if(i + 1 >= argc){
                    throw std::runtime_error("Param Error: lack value of " + it.key);
                }
                it.value = argv[i + 1];
            }
        }
    }
}

int main(int argc, char** argv){
    Param xml_name{"--xml-name","plan.xml"};
    std::vector<Param> paras({xml_name});
    handleParas(argc, argv, paras);

    rclcpp::init(argc, argv);

    auto ros_node = rclcpp::Node::make_shared("bt_runner");
    BT::BehaviorTreeFactory factory;

    factory.registerBuilder<rtdl_demo_bt::NavToNode>(
        "NavTo",
        [ros_node](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<rtdl_demo_bt::NavToNode>(name, config, ros_node);
        });

    factory.registerBuilder<rtdl_demo_bt::PickNode>(
        "Pick",
        [ros_node](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<rtdl_demo_bt::PickNode>(name, config, ros_node);
        });

    factory.registerBuilder<rtdl_demo_bt::PlaceNode>(
        "Place",
        [ros_node](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<rtdl_demo_bt::PlaceNode>(name, config, ros_node);
        });

    const std::string pkg_share =
        ament_index_cpp::get_package_share_directory("rtdl_demo_bt");
    const std::string xml_file = pkg_share + "/trees/" + xml_name.value;

    RCLCPP_INFO(ros_node->get_logger(), "Loading tree from: %s", xml_file.c_str());

    auto tree = factory.createTreeFromFile(xml_file);
    const auto status = tree.tickWhileRunning();

    if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(ros_node->get_logger(), "Behavior tree finished with SUCCESS.");
    } else if (status == BT::NodeStatus::FAILURE) {
        RCLCPP_ERROR(ros_node->get_logger(), "Behavior tree finished with FAILURE.");
    } else {
        RCLCPP_WARN(ros_node->get_logger(), "Behavior tree finished with RUNNING.");
    }
    rclcpp::shutdown();
    return 0;
}