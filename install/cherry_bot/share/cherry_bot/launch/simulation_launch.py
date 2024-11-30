from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    true_pose = Node(
        package="cherry_bot",          # Name of package
        executable="true_pose",        # Name of executable (defined in setup.py)
        name="ground_truth",              # Optional: ROS node name override
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

    simulation = Node(
        package="cherry_bot",        
        executable="simulation",       
    )

    rqt_graph = Node(
        package="rqt_graph",           # Name of package for rqt_graph
        executable="rqt_graph",        # Executable for rqt_graph
    )

    return LaunchDescription([
        true_pose,
        noisy_pose,
        ekf_pose,
        nav2goal,
        goals,
        lidar,
        simulation,
        rqt_graph
    ])
