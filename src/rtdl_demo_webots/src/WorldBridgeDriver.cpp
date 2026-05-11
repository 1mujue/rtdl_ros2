#include "rtdl_demo_webots/WorldBridgeDriver.hpp"

#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <iostream>

#include "webots/robot.h"
#include "webots/supervisor.h"

#include <pluginlib/class_list_macros.hpp>

#include "rtdl_demo_interfaces/msg/object_state.hpp"

namespace rtdl_demo_webots
{
	enum class GeometryType
	{
		Unknown,
		Box,
		Cylinder,
		Sphere
	};
	struct GeometryInfo
	{
		GeometryType type = GeometryType::Unknown;
		// Box
		double size[3] = {0, 0, 0};
		// Cylinder / Sphere
		double radius = 0;
		double height = 0;
	};
	static double geometryHeight(const GeometryInfo& g)
	{
		switch (g.type)
		{
		case GeometryType::Box:
			return g.size[2];
		case GeometryType::Cylinder:
			return g.height;
		case GeometryType::Sphere:
			return 2 * g.radius;
		default:
			return 0;
		}
	}
	static bool parseGeometryNode(WbNodeRef geometry_node, GeometryInfo& info){
		if(!geometry_node) return false;
		const char *type_name = wb_supervisor_node_get_type_name(geometry_node);
		if(!type_name) return false;

		if (std::strcmp(type_name, "Box") == 0)
		{
			WbFieldRef size_field =
				wb_supervisor_node_get_field(geometry_node, "size");
			if(!size_field) return false;

			const double *size =
				wb_supervisor_field_get_sf_vec3f(size_field);

			if (!size) return false;

			info.type = GeometryType::Box;
			info.size[0] = size[0];
			info.size[1] = size[1];
			info.size[2] = size[2];
			return true;
		}

		if (std::strcmp(type_name, "Cylinder") == 0)
		{
			WbFieldRef radius_field =
				wb_supervisor_node_get_field(geometry_node, "radius");

			WbFieldRef height_field =
				wb_supervisor_node_get_field(geometry_node, "height");

			if (!radius_field || !height_field) return false;

			info.type = GeometryType::Cylinder;
			info.radius = wb_supervisor_field_get_sf_float(radius_field);
			info.height = wb_supervisor_field_get_sf_float(height_field);
			return true;
		}

		if (std::strcmp(type_name, "Sphere") == 0)
		{
			WbFieldRef radius_field =
				wb_supervisor_node_get_field(geometry_node, "radius");

			if (!radius_field) return false;

			info.type = GeometryType::Sphere;
			info.radius = wb_supervisor_field_get_sf_float(radius_field);
			return true;
		}

		return false;
	}
	static bool getGeometryInfoFromSolid(WbNodeRef solid_node, GeometryInfo &info)
	{
		if (!solid_node) return false;

		WbFieldRef children_field =
			wb_supervisor_node_get_field(solid_node, "children");

		if (!children_field) return false;

		const int child_count =
			wb_supervisor_field_get_count(children_field);

		for (int i = 0; i < child_count; ++i)
		{
			WbNodeRef child =
				wb_supervisor_field_get_mf_node(children_field, i);

			const char *child_type =
				wb_supervisor_node_get_type_name(child);

			if (!child_type) return false;

			// Usually, Solid's child is Shape
			if (std::strcmp(child_type, "Shape") == 0)
			{
				WbFieldRef geometry_field =
					wb_supervisor_node_get_field(child, "geometry");

				WbNodeRef geometry_node =
					wb_supervisor_field_get_sf_node(geometry_field);

				if (parseGeometryNode(geometry_node, info)) return true;
			}
		}

		return false;
	}
	static double dist2d(const double *a, const double *b)
	{
		const double dx = a[0] - b[0];
		const double dy = a[1] - b[1];
		return std::sqrt(dx * dx + dy * dy);
	}
	// rot3x3 是 Webots 返回的 3x3 朝向矩阵，按平面运动近似提取 yaw。
	static double yawFromOrientation(const double *rot3x3)
	{
		return std::atan2(rot3x3[3], rot3x3[0]);
	}
	static bool isObjectHorizontallyOnSupport(
		const double *object_pos,
		const ObjectEntry &support,
		const double *support_pos,
		const GeometryInfo &support_geo)
	{
		constexpr double kInsideMargin = 0.03;

		const double dx = object_pos[0] - support_pos[0];
		const double dy = object_pos[1] - support_pos[1];

		if (support_geo.type == GeometryType::Box)
		{
			double local_x = dx;
			double local_y = dy;

			// 如果支撑物发生旋转，需要把世界坐标差值转换到 support 的局部坐标系
			const double *rot = wb_supervisor_node_get_orientation(support.node);
			if (rot)
			{
				const double yaw = yawFromOrientation(rot);
				const double c = std::cos(yaw);
				const double s = std::sin(yaw);

				// 逆旋转：world -> support local
				local_x = c * dx + s * dy;
				local_y = -s * dx + c * dy;
			}

			const double half_x = support_geo.size[0] / 2.0;
			const double half_y = support_geo.size[1] / 2.0;

			const bool inside_x =
				local_x >= -half_x - kInsideMargin &&
				local_x <=  half_x + kInsideMargin;

			const bool inside_y =
				local_y >= -half_y - kInsideMargin &&
				local_y <=  half_y + kInsideMargin;

			return inside_x && inside_y;
		}

		if (support_geo.type == GeometryType::Cylinder)
		{
			const double d = std::sqrt(dx * dx + dy * dy);
			return d <= support_geo.radius + kInsideMargin;
		}

		// Sphere is not a platfrom usually.
		return false;
	}
	static bool isOnSupport(
		const ObjectEntry &object,
		const double *object_pos,
		const ObjectEntry &support,
		const double *support_pos)
	{
		if (!object_pos || !support_pos || !object.node || !support.node) return false;
		if (!support.place_target) return false;
		GeometryInfo object_geo;
		GeometryInfo support_geo;
		if (!getGeometryInfoFromSolid(object.node, object_geo)) return false;
		if (!getGeometryInfoFromSolid(support.node, support_geo))return false;

		const double object_height = geometryHeight(object_geo);
		const double support_height = geometryHeight(support_geo);

		const double support_top_z =
			support_pos[2] + support_height / 2.0;

		const double object_bottom_z =
			object_pos[2] - object_height / 2.0;

		constexpr double kHeightTolerance = 0.04;

		const bool height_ok =
			std::abs(object_bottom_z - support_top_z) <= kHeightTolerance;

		if (!height_ok) return false;

		return isObjectHorizontallyOnSupport(
			object_pos,
			support,
			support_pos,
			support_geo
		);
	}
	static bool canActAsSupport(const GeometryInfo &geo)
	{
		return geo.type == GeometryType::Box ||
			geo.type == GeometryType::Cylinder;
	}
	static bool computePlacePosition(
		WbNodeRef object_node,
		WbNodeRef support_node,
		const double *support_pos,
		double place_pos[3])
	{
		if (!object_node || !support_node || !support_pos) return false;

		GeometryInfo object_geo;
		GeometryInfo support_geo;

		if (!getGeometryInfoFromSolid(object_node, object_geo)) return false;
		if (!getGeometryInfoFromSolid(support_node, support_geo)) return false;
		if (!canActAsSupport(support_geo)) return false;

		const double object_height = geometryHeight(object_geo);
		const double support_height = geometryHeight(support_geo);

		constexpr double kEpsilon = 0.005;

		place_pos[0] = support_pos[0];
		place_pos[1] = support_pos[1];
		place_pos[2] =
			support_pos[2] +
			support_height / 2.0 +
			object_height / 2.0 +
			kEpsilon;

		return true;
	}
	static bool isWithinRobotInteractionDistance(
		const RobotEntry &robot,
		const double *robot_pos,
		const double *target_pos)
	{
		if (!robot_pos || !target_pos) {
			return false;
		}

		return dist2d(robot_pos, target_pos) <= robot.interaction_distance;
	}
	static bool saveNodePose(WbNodeRef node, PoseSnapshot &pose)
	{
		if (!node) {
			return false;
		}

		WbFieldRef translation_field =
			wb_supervisor_node_get_field(node, "translation");

		WbFieldRef rotation_field =
			wb_supervisor_node_get_field(node, "rotation");

		if (!translation_field || !rotation_field) {
			return false;
		}

		const double *translation =
			wb_supervisor_field_get_sf_vec3f(translation_field);

		const double *rotation =
			wb_supervisor_field_get_sf_rotation(rotation_field);

		if (!translation || !rotation) {
			return false;
		}

		pose.translation = {
			translation[0],
			translation[1],
			translation[2]
		};

		pose.rotation = {
			rotation[0],
			rotation[1],
			rotation[2],
			rotation[3]
		};

		return true;
	}
	static bool restoreNodePose(WbNodeRef node, const PoseSnapshot &pose)
	{
		if (!node) {
			return false;
		}

		WbFieldRef translation_field =
			wb_supervisor_node_get_field(node, "translation");

		WbFieldRef rotation_field =
			wb_supervisor_node_get_field(node, "rotation");

		if (!translation_field || !rotation_field) {
			return false;
		}

		wb_supervisor_field_set_sf_vec3f(
			translation_field,
			pose.translation.data()
		);

		wb_supervisor_field_set_sf_rotation(
			rotation_field,
			pose.rotation.data()
		);

		wb_supervisor_node_reset_physics(node);

		return true;
	}
	void WorldBridgeDriver::init(
		webots_ros2_driver::WebotsNode *node,
		std::unordered_map<std::string, std::string> &properties)
	{
		node_ = node;

		// register services.
		get_world_state_srv_ =
			node_->create_service<rtdl_demo_interfaces::srv::GetWorldState>(
				"/get_world_state",
				std::bind(
					&WorldBridgeDriver::handleGetWorldState,
					this,
					std::placeholders::_1,
					std::placeholders::_2));
		
		reset_world_srv_ = 
			node_->create_service<std_srvs::srv::Trigger>(
				"/reset_world",
				std::bind(
					&WorldBridgeDriver::handleResetWorld,
					this,
					std::placeholders::_1,
					std::placeholders::_2
				)
			);

		pick_pri_srv_ =
			node_->create_service<rtdl_demo_interfaces::srv::PickPri>(
				"/pick_pri",
				std::bind(
					&WorldBridgeDriver::handlePickPri,
					this,
					std::placeholders::_1,
					std::placeholders::_2));

		place_pri_srv_ =
			node_->create_service<rtdl_demo_interfaces::srv::PlacePri>(
				"/place_pri",
				std::bind(
					&WorldBridgeDriver::handlePlacePri,
					this,
					std::placeholders::_1,
					std::placeholders::_2));

		
		std::string entity_config_path;

		if (node_->has_parameter("entity_config")) {
			node_->get_parameter("entity_config", entity_config_path);
		} else {
			node_->declare_parameter<std::string>("entity_config", "");
			node_->get_parameter("entity_config", entity_config_path);
		}

		if (entity_config_path.empty()) {
			throw std::runtime_error("Parameter 'entity_config' is empty.");
		}
		registerEntities(entity_config_path);
		saveInitialPoses();

		RCLCPP_INFO(node_->get_logger(), "WorldBridgeDriver initialized.");
	}
	void WorldBridgeDriver::registerEntities(const std::string& entitiy_config_path)
	{
		YAML::Node config = YAML::LoadFile(entitiy_config_path);
		std::cout << "load entity config successfully.\n";

		robots_.clear();
		objects_.clear();
		
		if(config["robots"]){
			for(const auto& robot: config["robots"]){
				std::string id = robot["id"].as<std::string>();
				std::string def = robot["def"]? robot["def"].as<std::string>(): id;
				double inter_dis = robot["interaction_distance"].as<double>();
				WbNodeRef node = wb_supervisor_node_get_from_def(def.c_str());
				if(node == nullptr){
					throw std::runtime_error("Robot DEF not found: " + def);
				}

				robots_[id] = {id, node, inter_dis};

				RCLCPP_INFO(
					node_->get_logger(),
					"Registered robot: id=%s, def=%s",
					id.c_str(),
					def.c_str()
				);
			}
		}

		if(config["objects"]){
			for(const auto& object: config["objects"]){
				std::string id = object["id"].as<std::string>();
				std::string def = object["def"]? object["def"].as<std::string>(): id;

				bool is_pick_target =
					object["pick_target"] ? object["pick_target"].as<bool>() : false;

				bool is_place_target =
					object["place_target"] ? object["place_target"].as<bool>() : false;

				WbNodeRef node = wb_supervisor_node_get_from_def(def.c_str());

				if (node == nullptr) {
					throw std::runtime_error("Object DEF not found: " + def);
				}

				objects_[id] = {
					id,
					def,
					is_place_target,
					is_pick_target,
					node
				};

				RCLCPP_INFO(
					node_->get_logger(),
					"Registered object: id=%s, def=%s, pick_target=%d, place_target=%d",
					id.c_str(),
					def.c_str(),
					is_pick_target,
					is_place_target
				);
			}
		}
	}
	void WorldBridgeDriver::saveInitialPoses()
	{
		initial_robot_poses_.clear();
		initial_object_poses_.clear();

		for (const auto &[name, robot] : robots_) {
			PoseSnapshot pose;
			if (saveNodePose(robot.node, pose)) {
				initial_robot_poses_[name] = pose;
			} else {
				RCLCPP_WARN(
					node_->get_logger(),
					"Failed to save initial pose for robot %s",
					name.c_str()
				);
			}
		}

		for (const auto &[name, object] : objects_) {
			PoseSnapshot pose;
			if (saveNodePose(object.node, pose)) {
				initial_object_poses_[name] = pose;
			} else {
				RCLCPP_WARN(
					node_->get_logger(),
					"Failed to save initial pose for object %s",
					name.c_str()
				);
			}
		}
	}
	std::string WorldBridgeDriver::inferSupportName(
		const ObjectEntry &object,
		const double *object_pos)
	{
		for (const auto &kv : objects_)
		{
			if (!kv.second.place_target || !kv.second.node)
			{
				continue;
			}
			const auto &candidate = kv.second;
			const double *support_pos = wb_supervisor_node_get_position(candidate.node);
			if (!support_pos)
			{
				continue;
			}

			if (isOnSupport(object, object_pos, candidate, support_pos))
			{
				return candidate.name;
			}
		}
		return "";
	}
	bool WorldBridgeDriver::isHeldByAnyRobot(const std::string &object_name) const
	{
		for (const auto &kv : carried_by_robot_)
		{
			if (kv.second == object_name) return true;
		}
		return false;
	}

	std::string WorldBridgeDriver::holderRobotName(const std::string &object_name) const
	{
		for (const auto &kv : carried_by_robot_)
		{
			if (kv.second == object_name)
			{
				return kv.first;
			}
		}
		return "";
	}
	ObjectEntry *WorldBridgeDriver::findObject(const std::string &name)
	{
		return objects_.find(name) == objects_.end() ? nullptr : &objects_[name];
	}
	RobotEntry *WorldBridgeDriver::findRobot(const std::string &name)
	{
		return robots_.find(name) == robots_.end() ? nullptr : &robots_[name];
	}
	void WorldBridgeDriver::disablePhysics(WbNodeRef node)
	{
		if (!node) return;
		WbFieldRef physics_field = wb_supervisor_node_get_field(node, "physics");
		if (physics_field)
		{
			wb_supervisor_field_remove_sf(physics_field);
		}
	}

	void WorldBridgeDriver::enablePhysics(WbNodeRef node)
	{
		if (!node) return;
		WbFieldRef physics_field = wb_supervisor_node_get_field(node, "physics");
		if (physics_field)
		{
			wb_supervisor_field_import_sf_node_from_string(physics_field, "Physics { }");
		}
	}

	void WorldBridgeDriver::updateCachedState()
	{
		cached_state_.robots.clear();
		cached_state_.objects.clear();

		// 1. robots
		for (const auto &kv : robots_)
		{
			const auto robot = kv.second;
			if (!robot.node) continue;

			const double *pos = wb_supervisor_node_get_position(robot.node);
			const double *rot = wb_supervisor_node_get_orientation(robot.node);
			if (!pos || !rot) continue;
			rtdl_demo_interfaces::msg::RobotState robot_msg;
			robot_msg.name = robot.name;
			robot_msg.x = pos[0];
			robot_msg.y = pos[1];
			robot_msg.z = pos[2];
			robot_msg.yaw = yawFromOrientation(rot);
			robot_msg.interaction_distance = robot.interaction_distance;

			auto it = carried_by_robot_.find(robot.name);
			robot_msg.held_object_name = (it == carried_by_robot_.end()) ? "" : it->second;
			cached_state_.robots.push_back(robot_msg);
		}

		// 2. objects
		for (const auto &kv : objects_)
		{
			const auto object = kv.second;
			if (!object.node) continue;
			const double *pos = wb_supervisor_node_get_position(object.node);
			if (!pos) continue;

			rtdl_demo_interfaces::msg::ObjectState object_msg;
			object_msg.name = object.name;
			object_msg.type = object.type;
			object_msg.x = pos[0];
			object_msg.y = pos[1];
			object_msg.z = pos[2];

			object_msg.held_by_robot = isHeldByAnyRobot(object.name);
			object_msg.holder_robot_name = holderRobotName(object.name);

			if (object_msg.held_by_robot)
			{
				object_msg.support_name = "";
			}
			else
			{
				object_msg.support_name = inferSupportName(object, pos);
			}

			cached_state_.objects.push_back(object_msg);
		}
	}

	void WorldBridgeDriver::handleGetWorldState(
		const std::shared_ptr<rtdl_demo_interfaces::srv::GetWorldState::Request> req,
		std::shared_ptr<rtdl_demo_interfaces::srv::GetWorldState::Response> res)
	{
		res->success = true;
		res->message = "Real Webots world state queried successfully.";
		res->state = cached_state_;
	}

	void WorldBridgeDriver::handleResetWorld(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
	{
		(void)req;

		bool ok = true;

		carried_by_robot_.clear();

		for (auto &[name, object] : objects_) {
			auto pose_it = initial_object_poses_.find(name);
			if (pose_it == initial_object_poses_.end()) {
				ok = false;
				RCLCPP_WARN(
					node_->get_logger(),
					"No initial pose saved for object %s",
					name.c_str()
				);
				continue;
			}

			if (!restoreNodePose(object.node, pose_it->second)) {
				ok = false;
				RCLCPP_WARN(
					node_->get_logger(),
					"Failed to restore object %s",
					name.c_str()
				);
			}

			enablePhysics(object.node);
			wb_supervisor_node_reset_physics(object.node);
		}

		for (auto &[name, robot] : robots_) {
			auto pose_it = initial_robot_poses_.find(name);
			if (pose_it == initial_robot_poses_.end()) {
				ok = false;
				RCLCPP_WARN(
					node_->get_logger(),
					"No initial pose saved for robot %s",
					name.c_str()
				);
				continue;
			}

			if (!restoreNodePose(robot.node, pose_it->second)) {
				ok = false;
				RCLCPP_WARN(
					node_->get_logger(),
					"Failed to restore robot %s",
					name.c_str()
				);
			}

			wb_supervisor_node_reset_physics(robot.node);
		}

		wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_REAL_TIME);

		res->success = ok;
		res->message = ok
			? "World soft reset successfully."
			: "World soft reset completed with warnings.";
	}

	void WorldBridgeDriver::handlePickPri(
		const std::shared_ptr<rtdl_demo_interfaces::srv::PickPri::Request> req,
		std::shared_ptr<rtdl_demo_interfaces::srv::PickPri::Response> res)
	{
		const std::string robot_name = req->robot_name;
		const std::string object_name = req->object_name;

		auto robot_it = robots_.find(robot_name);
		if (robot_it == robots_.end() || !robot_it->second.node)
		{
			res->success = false;
			res->message = "Robot " + robot_name + " does not exist.";
			return;
		}

		auto object_it = objects_.find(object_name);
		if (object_it == objects_.end() || !object_it->second.node)
		{
			res->success = false;
			res->message = "Object " + object_name + " does not exist.";
			return;
		}

		auto &robot = robot_it->second;
		auto &object = object_it->second;

		if (!object.pick_target)
		{
			res->success = false;
			res->message = "Object " + object_name + " is not pickable.";
			return;
		}

		auto carry_it = carried_by_robot_.find(robot_name);
		if (carry_it != carried_by_robot_.end() && !carry_it->second.empty())
		{
			res->success = false;
			res->message = "Robot " + robot_name + " is already holding " + carry_it->second + ".";
			return;
		}

		if (isHeldByAnyRobot(object_name))
		{
			res->success = false;
			res->message =
				"Object " + object_name + " is already held by robot " + holderRobotName(object_name) + ".";
			return;
		}

		const double *robot_pos = wb_supervisor_node_get_position(robot.node);
		const double *object_pos = wb_supervisor_node_get_position(object.node);

		if (!robot_pos || !object_pos)
		{
			res->success = false;
			res->message = "Failed to read robot/object positions.";
			return;
		}

		if (!isWithinRobotInteractionDistance(robot, robot_pos, object_pos)) {
			res->success = false;
			res->message =
				"Object " + object_name +
				" is too far from robot " + robot_name +
				". Required distance <= " + std::to_string(robot.interaction_distance) + ".";
			return;
		}

		disablePhysics(object.node);
		carried_by_robot_[robot_name] = object_name;

		res->success = true;
		res->message = "Pick object " + object_name + " successfully.";
	}

	void WorldBridgeDriver::handlePlacePri(
		const std::shared_ptr<rtdl_demo_interfaces::srv::PlacePri::Request> req,
		std::shared_ptr<rtdl_demo_interfaces::srv::PlacePri::Response> res)
	{
		const std::string robot_name = req->robot_name;
		const std::string object_name = req->object_name;
		const std::string support_name = req->location_name;

		auto fail = [&](const std::string &message) {
			res->success = false;
			res->message = message;
		};

		auto robot_it = robots_.find(robot_name);
		if (robot_it == robots_.end() || !robot_it->second.node)
		{
			fail("Robot " + robot_name + " does not exist.");
			return;
		}

		auto object_it = objects_.find(object_name);
		if (object_it == objects_.end() || !object_it->second.node)
		{
			fail("Object " + object_name + " does not exist.");
			return;
		}

		auto support_it = objects_.find(support_name);
		if (support_it == objects_.end() || !support_it->second.node)
		{
			fail("Support " + support_name + " does not exist.");
			return;
		}

		if (object_name == support_name)
		{
			fail("Cannot place object " + object_name + " on itself.");
			return;
		}

		auto &robot = robot_it->second;
		auto &object = object_it->second;
		auto &support = support_it->second;

		if (!support.place_target)
		{
			fail(support_name + " is not a valid place target.");
			return;
		}

		auto carry_it = carried_by_robot_.find(robot_name);
		if (carry_it == carried_by_robot_.end() || carry_it->second != object_name)
		{
			fail("Robot " + robot_name + " is not holding object " + object_name + ".");
			return;
		}

		const double *robot_pos =
			wb_supervisor_node_get_position(robot.node);

		const double *support_pos =
			wb_supervisor_node_get_position(support.node);

		if (!robot_pos || !support_pos)
		{
			fail("Failed to read robot/support positions.");
			return;
		}

		if (!isWithinRobotInteractionDistance(robot, robot_pos, support_pos))
		{
			res->success = false;
			res->message = "Support " + support_name +
						" is too far from robot " + robot_name + ".";
			return;
		}

		double place_pos[3];

		if (!computePlacePosition(
				object.node,
				support.node,
				support_pos,
				place_pos))
		{
			fail(
				"Failed to compute place position for object " +
				object_name + " on support " + support_name + "."
			);
			return;
		}

		WbFieldRef translation_field =
			wb_supervisor_node_get_field(object.node, "translation");

		if (!translation_field)
		{
			fail("Failed to access translation field for object " + object_name + ".");
			return;
		}

		wb_supervisor_field_set_sf_vec3f(translation_field, place_pos);

		enablePhysics(object.node);
		wb_supervisor_node_reset_physics(object.node);

		carried_by_robot_.erase(robot_name);

		const double *new_object_pos =
			wb_supervisor_node_get_position(object.node);

		const double *new_support_pos =
			wb_supervisor_node_get_position(support.node);

		if (!isOnSupport(object, new_object_pos, support, new_support_pos))
		{
			fail(
				"Object " + object_name +
				" was placed, but support validation failed on " +
				support_name + "."
			);
			return;
		}

		res->success = true;
		res->message =
			"Place object " + object_name +
			" on support " + support_name +
			" successfully.";
	}
	void WorldBridgeDriver::step()
	{
		for (const auto &kv : carried_by_robot_)
		{
			const std::string &robot_name = kv.first;
			const std::string &object_name = kv.second;

			if (object_name.empty())
			{
				continue;
			}

			auto robot_it = robots_.find(robot_name);
			auto object_it = objects_.find(object_name);

			if (robot_it == robots_.end() || object_it == objects_.end())
			{
				continue;
			}
			if (!robot_it->second.node || !object_it->second.node)
			{
				continue;
			}

			const double *robot_pos = wb_supervisor_node_get_position(robot_it->second.node);
			const double *robot_rot = wb_supervisor_node_get_orientation(robot_it->second.node);
			if (!robot_pos || !robot_rot)
			{
				continue;
			}

			const double yaw = yawFromOrientation(robot_rot);

			double new_pos[3];
			new_pos[0] = robot_pos[0] + 0.08 * std::cos(yaw);
			new_pos[1] = robot_pos[1] + 0.08 * std::sin(yaw);
			new_pos[2] = 0.5;

			WbFieldRef translation_field =
				wb_supervisor_node_get_field(object_it->second.node, "translation");
			if (translation_field)
			{
				wb_supervisor_field_set_sf_vec3f(translation_field, new_pos);
			}
		}

		updateCachedState();
	}
}

PLUGINLIB_EXPORT_CLASS(
	rtdl_demo_webots::WorldBridgeDriver,
	webots_ros2_driver::PluginInterface
)