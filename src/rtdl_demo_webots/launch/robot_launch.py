import os 

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_name = "rtdl_demo_webots"
    package_dir = get_package_share_directory(package_name=package_name)

    # you can set world:=XXX to change the world you want to launch!
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="level1.wbt",
        description="Webots world file name in the worlds directory"
    )
    entity_config_arg = DeclareLaunchArgument(
        "entity_config",
        default_value="level1_en.yaml",
        description="Entity config yaml file in config directory"
    )

    world = PathJoinSubstitution([
        FindPackageShare(package_name),
        "worlds",
        LaunchConfiguration("world")
    ])
    entity_config = PathJoinSubstitution([
        FindPackageShare(package_name),
        "config",
        LaunchConfiguration("entity_config")
    ])

    car_urdf = os.path.join(package_dir, "resource", "my_robot.urdf")
    bridge_urdf = os.path.join(package_dir, "resource", "bridge_robot.urdf")

    webots = WebotsLauncher(world=world)

    # VERY IMPORTANT: we might reset the world, and if we don't set respawn=True,
    # then after reset, the controller might disconnect forever.
    car_driver = WebotsController(
        robot_name="CAR",
        parameters=[
            {"robot_description": car_urdf},
        ],
        respawn=True,
    )

    bridge_driver = WebotsController(
        robot_name="BRIDGE",
        parameters=[
            {"robot_description": bridge_urdf},
            {"entity_config": entity_config},
        ],
        respawn=True,
    )

    return LaunchDescription([
        world_arg,
        entity_config_arg,

        webots,
        car_driver,
        bridge_driver,

        RegisterEventHandler(
            OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())]
            )
        )
    ])