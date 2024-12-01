import matplotlib.pyplot as plt
from cherry_bot.Environment.Environment import Environment
from cherry_bot.globals import inflated_radius, path_resolution
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.heuristic import euclidean
from cherry_bot.globals import goal_x, goal_y


class AStarPathPlanner:
    def __init__(self):
        """
        Initialize the path planner.
        
        :param resolution: Grid resolution [m].
        """
        self.resolution = path_resolution
        self.matrix = self.create_grid_from_environment()
        self.grid = Grid(matrix=self.matrix)

    @staticmethod
    def inflate_obstacles(matrix, inflation_radius):
        """
        Inflate obstacles in the grid matrix to account for the robot's size.

        :param matrix: 2D grid representing the environment.
                       1 = walkable, 0 = obstacle.
        :param inflation_radius: Radius of inflation in terms of grid cells.
        :return: Inflated matrix.
        """
        inflated_matrix = [[cell for cell in row] for row in matrix]
        rows, cols = len(matrix), len(matrix[0])
        for y in range(rows):
            for x in range(cols):
                if matrix[y][x] == 0:  # If it's an obstacle
                    for dy in range(-inflation_radius, inflation_radius + 1):
                        for dx in range(-inflation_radius, inflation_radius + 1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < cols and 0 <= ny < rows:
                                inflated_matrix[ny][nx] = 0
        return inflated_matrix

    def create_grid_from_environment(self):
        """
        Create a grid matrix from the Environment class.

        :return: A 2D grid matrix with obstacles inflated by the robot's radius.
        """
        rows = int(Environment.height / self.resolution)
        cols = int(Environment.width / self.resolution)
        matrix = [[1 for _ in range(cols)] for _ in range(rows)]

        # Add walls to the matrix
        for wall in Environment.walls:
            x_start = int(wall['x'] / self.resolution)
            x_end = int((wall['x'] + wall['width']) / self.resolution)
            y_start = int(wall['y'] / self.resolution)
            y_end = int((wall['y'] + wall['height']) / self.resolution)

            for y in range(y_start, y_end + 1):
                for x in range(x_start, x_end + 1):
                    if 0 <= x < cols and 0 <= y < rows:
                        matrix[y][x] = 0  # Mark as obstacle

        # Add obstacles to the matrix
        for obs in Environment.obstacles:
            x_start = int(obs['x'] / self.resolution)
            x_end = int((obs['x'] + obs['width']) / self.resolution)
            y_start = int(obs['y'] / self.resolution)
            y_end = int((obs['y'] + obs['height']) / self.resolution)

            for y in range(y_start, y_end + 1):
                for x in range(x_start, x_end + 1):
                    if 0 <= x < cols and 0 <= y < rows:
                        matrix[y][x] = 0  # Mark as obstacle

        # Inflate obstacles for the robot's radius
        robot_radius_in_cells = int(inflated_radius / self.resolution)
        matrix = self.inflate_obstacles(matrix, robot_radius_in_cells)
        return matrix

    def find_path(self, sx, sy, gx, gy):
        """
        Find a path using A*.

        :param sx: Start x position [m].
        :param sy: Start y position [m].
        :param gx: Goal x position [m].
        :param gy: Goal y position [m].
        :return: Path as a list of (x, y) tuples in world coordinates.
        """
        start_x = int(sx / self.resolution)
        start_y = int(sy / self.resolution)
        goal_x = int(gx / self.resolution)
        goal_y = int(gy / self.resolution)

        start = self.grid.node(start_x, start_y)
        end = self.grid.node(goal_x, goal_y)

        # Use A* with diagonal movement and Euclidean heuristic
        finder = AStarFinder(diagonal_movement=DiagonalMovement.always, heuristic=euclidean)
        path, _ = finder.find_path(start, end, self.grid)

        if path:
            # Round the world coordinates to the nearest hundredth
            return [(round(node.x * self.resolution, 2), round(node.y * self.resolution, 2)) for node in path]
        else:
            return None


    def visualize_path(self, path, sx, sy, gx, gy):
        """
        Visualize the path on a plot.

        :param path: Path as a list of (x, y) tuples in world coordinates.
        :param sx: Start x position [m].
        :param sy: Start y position [m].
        :param gx: Goal x position [m].
        :param gy: Goal y position [m].
        """
        if path:
            path_x, path_y = zip(*path)
            plt.figure(figsize=(8, 6))
            plt.xlim(0, Environment.width)
            plt.ylim(0, Environment.height)
            plt.gca().invert_yaxis()  # Flip y-axis

            plt.plot(path_x, path_y, "-r", label="Path")
            plt.scatter(sx, sy, color="green", label="Start")
            plt.scatter(gx, gy, color="blue", label="Goal")

            for wall in Environment.walls:
                plt.gca().add_patch(plt.Rectangle(
                    (wall['x'], wall['y']), wall['width'], wall['height'], color='black'))
            for obs in Environment.obstacles:
                plt.gca().add_patch(plt.Rectangle(
                    (obs['x'], obs['y']), obs['width'], obs['height'], color='red'))

            plt.legend()
            plt.grid(True)
            plt.show()
        else:
            print("No path found.")


def main():
    sx, sy = 4, 3  # Start position [meters]
    gx, gy = goal_x + 4, goal_y + 3  # Goal position [meters]

    planner = AStarPathPlanner()
    path = planner.find_path(sx, sy, gx, gy)

    if path:
        print("Path found:", path)
        planner.visualize_path(path, sx, sy, gx, gy)
    else:
        print("No path found.")


if __name__ == '__main__':
    main()
