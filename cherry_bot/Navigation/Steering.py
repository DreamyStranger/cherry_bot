import numpy as np
from math import atan2, sin, cos, pi

class SteeringController:
    def __init__(self, K_h=3, max_steering=pi/2):
        """
        Constructor: Initializes the steering controller with default or provided parameters.
        
        Args:
            K_h (float): Gain for the steering control.
            max_steering (float): Maximum allowable steering angle (radians).
        """
        self._K_h = K_h
        self.max_steering = max_steering
        self.goal = None

    def compute(self, pose):
        """
        Computes the steering command based on the current pose and the goal position.

        Args:
            goal (tuple): Target position as (x, y).
            pose (tuple): Current pose as (x, y, theta), where theta is the robot's orientation.

        Returns:
            float: Steering angle command (radians), constrained by max_steering.
        """
        if self.goal is None:
            return 0.0
        
        # Calculate the desired orientation towards the goal
        desired_theta = atan2(self.goal[1] - pose[1], self.goal[0] - pose[0])

        # Calculate the steering angle (error in orientation)
        steering = desired_theta - pose[2]
        steering = self._K_h * atan2(sin(steering), cos(steering))  # Normalize and scale

        # Constrain the steering angle
        steering = np.sign(steering) * min(abs(steering), self.max_steering)

        return steering
    
    def set_goal(self, goal_x, goal_y):
        self.goal = (goal_x, goal_y)

    def set_gains(self, K_steering):
        """
        Set the gain of the steering controller.

        Args:
            K_steering (float): New steering gain.
        """
        self._K_h = K_steering
