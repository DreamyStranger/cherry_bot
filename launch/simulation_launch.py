from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess

import os

rviz_config_path = os.path.join('src', "cherry_bot", "cherry_bot", "default.rviz")

def generate_launch_description():
    true_pose = Node(
        package="cherry_bot",
        executable="true_pose",
        name="ground_truth",
    )

    noisy_pose = Node(
        package="cherry_bot",
        executable="noisy_pose",
    )

    ekf_pose = Node(
        package="cherry_bot",
        executable="ekf_pose",
    )

    nav2goal = Node(
        package="cherry_bot",
        executable="nav2goal",
    )

    goals = Node(
        package="cherry_bot",
        executable="goals",
    )

    lidar = Node(
        package="cherry_bot",
        executable="lidar",
    )

    grid = Node(
        package="cherry_bot",
        executable="grid",
    )

    tf = Node(
        package="cherry_bot",
        executable="tf",
    )

    simulation = Node(
        package="cherry_bot",
        executable="simulation",
    )

    rqt_graph = Node(
        package="rqt_graph",
        executable="rqt_graph",
    )

    # RViz node to load the saved configuration
    rviz = TimerAction(
    period=2.0,  # Delay start by x seconds
    actions=[
        ExecuteProcess(
            cmd=["rviz2", "-d", rviz_config_path],
            output="screen",
        )
    ]
)

    return LaunchDescription([
        true_pose,
        noisy_pose,
        ekf_pose,
        nav2goal,
        goals,
        lidar,
        grid,
        simulation,
        rqt_graph,
        tf,
        rviz,  # Add RViz to the launch description
    ])
