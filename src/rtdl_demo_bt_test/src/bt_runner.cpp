#include <memory>
#include <cstdlib>
#include <string>
#include <fstream>
#include <iostream>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "rclcpp/rclcpp.hpp"
#include "rtdl_demo_bt_test/place.hpp"
#include "rtdl_demo_bt_test/pick.hpp"
#include "rtdl_demo_bt_test/nav_to.hpp"
struct Param{
    std::string key;
    std::string value;
};
void handleParas(int argc, char** argv, std::vector<Param>& paras){
    std::cout << "argc: " << argc << '\n';
    for(int i = 1; i < argc; i += 2){
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
static std::string readWholeFile(const std::string& path)
{
    std::ifstream fin(path);
    if (!fin.is_open()) {
        throw std::runtime_error("Failed to open file for reading: " + path);
    }
    std::ostringstream ss;
    ss << fin.rdbuf();
    return ss.str();
};

static void writeWholeFile(const std::string& path, const std::string& content)
{
    std::ofstream fout(path);
    if (!fout.is_open()) {
        throw std::runtime_error("Failed to open file for writing: " + path);
    }
    fout << content;
    if (!fout.good()) {
        throw std::runtime_error("Failed to write file: " + path);
    }
}
static void registerExecutionNodes(BT::BehaviorTreeFactory& factory, const rclcpp::Node::SharedPtr& ros_node)
{
    {
        BT::RosNodeParams params;
        params.nh = ros_node;
        params.default_port_value = "place";
        params.wait_for_server_timeout = std::chrono::milliseconds(3000);
        params.server_timeout = std::chrono::seconds(30);
        factory.registerNodeType<PlaceNode>("Place", params);
    }
    {
        BT::RosNodeParams params;
        params.nh = ros_node;
        params.default_port_value = "pick";
        params.wait_for_server_timeout = std::chrono::milliseconds(3000);
        params.server_timeout = std::chrono::seconds(30);
        factory.registerNodeType<PickNode>("Pick", params);
    }
    {
        BT::RosNodeParams params;
        params.nh = ros_node;
        params.default_port_value = "nav_to";
        params.wait_for_server_timeout = std::chrono::milliseconds(3000);
        params.server_timeout = std::chrono::seconds(30);
        factory.registerNodeType<NavToNode>("NavTo", params);
    }
}
static void registerVisualizationNodes(BT::BehaviorTreeFactory& factory)
{
    {
        BT::RosNodeParams params;
        factory.registerNodeType<PlaceNode>("Place", params);
    }
    {
        BT::RosNodeParams params;
        factory.registerNodeType<PickNode>("Pick", params);
    }
    {
        BT::RosNodeParams params;
        factory.registerNodeType<NavToNode>("NavTo", params);
    }
}
static void handleXMLS(BT::BehaviorTreeFactory& factory){
    const std::string pkg_share = ament_index_cpp::get_package_share_directory("rtdl_demo_bt_test");
    const std::string xml_file = pkg_share + "/trees/plan.xml";
    std::cout << "loading XML file of BT from: " << xml_file << '\n';
    factory.registerBehaviorTreeFromFile(xml_file);
    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    const std::string model_file = pkg_share + "/trees/models.xml";
    std::cout << "saving MODEL file of BT to: " << model_file << '\n';
    std::ofstream model_out(model_file); 
    model_out << xml_models;
}
static void run(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto ros_node = rclcpp::Node::make_shared("bt_runner");
    BT::BehaviorTreeFactory factory;

    BT::RosNodeParams params;
    params.nh = ros_node;
    registerExecutionNodes(factory, ros_node);
    handleXMLS(factory);

    auto tree = factory.createTree("Main");
    const auto status = tree.tickWhileRunning();
    if (status == BT::NodeStatus::SUCCESS) { RCLCPP_INFO(ros_node->get_logger(), "Behavior tree finished with SUCCESS.");}
    else if (status == BT::NodeStatus::FAILURE){ RCLCPP_ERROR(ros_node->get_logger(), "Behavior tree finished with FAILURE.");}
    else {RCLCPP_WARN(ros_node->get_logger(), "Behavior tree finished with RUNNING."); }
    rclcpp::shutdown();
}
static std::string extractTreeNodesModelBlock(const std::string& models_content)
{
    const std::string begin_tag = "<TreeNodesModel>";
    const std::string end_tag = "</TreeNodesModel>";
    std::size_t begin_pos = models_content.find(begin_tag);
    if (begin_pos == std::string::npos) {
        return models_content;
    }
    std::size_t end_pos = models_content.find(end_tag, begin_pos);
    end_pos += end_tag.size();
    return models_content.substr(begin_pos, end_pos - begin_pos);
}

static void mergePlanAndModelsToFile(const std::string& plan_xml_path,
                                     const std::string& models_xml_path,
                                     const std::string& output_xml_path)
{
    std::string plan_content = readWholeFile(plan_xml_path);
    std::string models_content = readWholeFile(models_xml_path);
    std::string tree_nodes_model = extractTreeNodesModelBlock(models_content);
    const std::string root_end = "</root>";
    std::size_t insert_pos = plan_content.rfind(root_end);
    
    std::string merged;
    merged.reserve(plan_content.size() + tree_nodes_model.size() + 32);
    merged.append(plan_content.substr(0, insert_pos));
    merged.append("\n\n");
    merged.append(tree_nodes_model);
    merged.append("\n\n");
    merged.append(plan_content.substr(insert_pos));
    writeWholeFile(output_xml_path, merged);
}
static void launchGroot2(){
    const std::string appImage("~/tools/Groot2-v1.9.0-x86_64.AppImage");
    const std::string pkg_share = ament_index_cpp::get_package_share_directory("rtdl_demo_bt_test");
    const std::string xml_file = pkg_share + "/trees/plan.xml";
    const std::string model_file = pkg_share + "/trees/models.xml";
    const std::string merged_file = pkg_share + "/trees/for_groot2.xml";
    mergePlanAndModelsToFile(xml_file, model_file, merged_file);
    std::cout << "Launching Groot2 with merged XML file: " << merged_file << '\n';
    std::string cmd =
        appImage + " --file " + merged_file +
        " >/tmp/groot2.log 2>&1 &";
    std::cout << "cmd: "  << cmd << '\n';
    std::system(cmd.c_str());
}
static void visualize(){
    BT::BehaviorTreeFactory factory;
    registerVisualizationNodes(factory);
    handleXMLS(factory);
    launchGroot2();
}
int main(int argc, char** argv)
{
    Param mode{"--mode", ""};
    std::vector<Param> paras({mode});
    handleParas(argc, argv, paras);

    if(paras[0].value == "run"){
        run(argc, argv);
    } else if(paras[0].value == "visualize"){
        visualize();
    }
    return 0;
}
