# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bring up the Z1 together with the keyboard teleoperation node.

The teleoperation node needs to own a terminal to read the keystrokes.  Since
ROS2 launch does not forward the standard input to the launched processes, the
node opens `/dev/tty` directly: that works only when `ros2 launch` is itself
run from an interactive shell.  Otherwise start the robot with `teleop:=false`
and then run the node by hand in a second terminal:

    ros2 run z1_examples teleop_keyboard.py

Cartesian jogging is delegated to the inverse kinematics of MoveIt, so
`move_group` is started as well unless `moveit:=false` is passed.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    nodes_to_start = list()

    sim_ignition = LaunchConfiguration("sim_ignition")
    with_gripper = LaunchConfiguration("with_gripper")
    teleop = LaunchConfiguration("teleop")
    moveit = LaunchConfiguration("moveit")
    rviz = LaunchConfiguration("rviz")

    use_sim_time = (sim_ignition.perform(context) == "true")

    # The keyboard teleoperation streams position setpoints to the
    # joint_trajectory_controller, hence it must be the controller in use.
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("z1_bringup"), "/launch/z1.launch.py"
        ], ),
        launch_arguments={
            "sim_ignition": sim_ignition,
            "with_gripper": with_gripper,
            "rviz": rviz,
            "starting_controller": "joint_trajectory_controller",
        }.items(),
    )

    # The gripper belongs to no controller of the default configuration, so it
    # gets a dedicated one, spawned once the controller_manager is up.
    gripper_controller_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gripper_controller", "-c", "/controller_manager"],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(with_gripper),
                output="screen",
            ),
        ],
    )

    # move_group only provides `/compute_ik` here: the node does not use it to
    # plan, it just asks for the joint configuration matching the pose the
    # operator is jogging to.
    move_group_node = TimerAction(
        period=8.0,
        actions=[
            _move_group_node(use_sim_time, condition=IfCondition(moveit)),
        ],
    )

    teleop_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package="z1_examples",
                executable="teleop_keyboard.py",
                output="screen",
                emulate_tty=True,
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(teleop),
            ),
        ],
    )

    nodes_to_start += [
        bringup_launch,
        gripper_controller_spawner,
        move_group_node,
        teleop_node,
    ]
    return nodes_to_start


def _move_group_node(use_sim_time, condition):
    """Build the move_group node without pulling in a second bringup."""
    from moveit_configs_utils import MoveItConfigsBuilder

    moveit_config = MoveItConfigsBuilder(
        "z1_description", package_name="z1_moveit"
    ).to_moveit_configs()

    parameters = moveit_config.to_dict()
    parameters["use_sim_time"] = use_sim_time

    return Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[parameters],
        condition=condition,
    )


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="true",
            description="Launch simulation in Ignition Gazebo?"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "with_gripper", default_value="true", description="Use the gripper?"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit",
            default_value="true",
            description="Start move_group, needed by the cartesian mode of the "
            "keyboard teleoperation. Set it to false for a lighter bringup "
            "that only supports joint jogging"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz?")
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "teleop",
            default_value="true",
            description="Start the keyboard teleoperation node as well. Set it "
            "to false when 'ros2 launch' is not run from an interactive shell "
            "and start 'ros2 run z1_examples teleop_keyboard.py' by hand"
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
