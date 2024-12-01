import numpy as np

class ArtificialPotentialField:
    def __init__(self, attraction_gain=3, repulsion_gain=2, obstacle_influence_distance=0.3):
        self.attraction_gain = attraction_gain
        self.repulsion_gain = repulsion_gain
        self.obstacle_influence_distance = obstacle_influence_distance

    def compute_steering(self, ekf_pose, goal, lidar_data):
        """
        Compute the steering angle for the robot based on APF.

        :param ekf_pose: Current robot pose (x, y, theta) from EKF [m, m, rad].
        :param goal: Current goal position (x, y) [m, m].
        :param lidar_data: List of tuples representing detected obstacles [(distance, angle)].
        :return: Steering angle [rad].
        """
        # Extract robot's position and orientation
        robot_x, robot_y, robot_theta = ekf_pose

        # Adjust goal to be relative to the robot
        goal_x, goal_y = goal
        goal_x_rel = goal_x - robot_x
        goal_y_rel = goal_y - robot_y

        # Compute forces
        fx, fy = self.compute_force(ekf_pose, (goal_x_rel, goal_y_rel), lidar_data, robot_theta)

        # Normalize the force vector
        force_magnitude = np.sqrt(fx**2 + fy**2)
        if force_magnitude > 0:
            fx /= force_magnitude
            fy /= force_magnitude

        # Compute desired steering angle
        desired_angle = np.arctan2(fy, fx)

        # Compute steering direction relative to the robot's orientation
        steering_angle = desired_angle

        # Normalize the angle to the range [-pi, pi]
        steering_angle = (steering_angle + np.pi) % (2 * np.pi) - np.pi

        return steering_angle

    def compute_force(self, ekf_pose, goal, lidar_data, robot_theta):
        """
        Compute the combined attractive and repulsive forces.

        :param ekf_pose: Current robot pose (x, y, theta) .
        :param goal: Current goal position (x, y) [m, m] relative to robot.
        :param lidar_data: List of tuples [(distance, angle)] in robot frame.
        :param robot_theta: Current robot orientation [rad].
        :return: Combined force vector (fx, fy).
        """
        # Extract robot's position
        robot_x, robot_y, _ = ekf_pose

        # Compute attractive force
        fx_attr, fy_attr = self.compute_attractive_force(robot_x, robot_y, goal)

        # Compute repulsive force
        fx_rep, fy_rep = self.compute_repulsive_force(lidar_data, robot_theta)

        # Combine forces
        fx = fx_attr + fx_rep
        fy = fy_attr + fy_rep

        return fx, fy

    def compute_attractive_force(self, robot_x, robot_y, goal):
        """
        Compute the attractive force pulling the robot toward the goal.

        :param robot_x: Robot's x position [m].
        :param robot_y: Robot's y position [m].
        :param goal: Goal position (x, y) [m, m].
        :return: Attractive force vector (fx, fy).
        """
        goal_x, goal_y = goal
        fx = self.attraction_gain * (goal_x - robot_x)
        fy = self.attraction_gain * (goal_y - robot_y)
        return fx, fy

    def compute_repulsive_force(self, lidar_data, robot_theta):
        """
        Compute the repulsive force from obstacles detected by LiDAR.

        :param lidar_data: List of tuples representing detected obstacles [(distance, angle)].
        :param robot_theta: Current robot orientation [rad].
        :return: Repulsive force vector (fx, fy).
        """
        fx, fy = 0.0, 0.0
        for distance, angle in lidar_data:
            if distance < self.obstacle_influence_distance:
                # Adjust LiDAR angle to the global frame
                adjusted_angle = angle + robot_theta

                # Compute the influence factor
                influence = max(0, self.obstacle_influence_distance - distance)

                # Compute the repulsive force in polar coordinates
                force_magnitude = self.repulsion_gain * influence / (distance + 1e-6)

                # Convert to Cartesian coordinates
                fx += -force_magnitude * np.cos(adjusted_angle)
                fy += -force_magnitude * np.sin(adjusted_angle)

        return fx, fy
