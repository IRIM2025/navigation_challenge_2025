#!/usr/bin/env python3
"""
Automatic Maze Generator for Gazebo - ROS Noetic Compatible
Generates random mazes using recursive backtracking algorithm and spawns walls in Gazebo
No text files required - completely algorithmic generation
Compatible with Python 3 and ROS Noetic
"""
import rospy
import random
import time
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler
import sys

class MazeGenerator:
    def __init__(self, width=21, height=21, wall_height=2.0, wall_thickness=0.2, cell_size=2.0):
        """
        Initialize the maze generator
        Args:
            width, height: Maze dimensions (should be odd numbers for proper maze generation)
            wall_height: Height of the walls in meters
            wall_thickness: Thickness of the walls in meters  
            cell_size: Size of each cell in meters
        """
        self.width = width if width % 2 == 1 else width + 1  # Ensure odd dimensions
        self.height = height if height % 2 == 1 else height + 1
        self.wall_height = wall_height
        self.wall_thickness = wall_thickness
        self.cell_size = cell_size

        # Initialize maze grid (True = wall, False = passage)
        self.maze = [[True for _ in range(self.width)] for _ in range(self.height)]
        self.wall_models = []

        # Wait for Gazebo services to be available
        rospy.loginfo("Waiting for Gazebo services...")
        try:
            rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=30)
            rospy.wait_for_service('/gazebo/delete_model', timeout=30)
            rospy.loginfo("Gazebo services available")
        except rospy.ROSException:
            rospy.logerr("Gazebo services not available! Make sure Gazebo is running.")
            return

        # ROS service proxies
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    def generate_maze(self):
        """Generate maze using recursive backtracking algorithm"""
        # Start with all walls
        self.maze = [[True for _ in range(self.width)] for _ in range(self.height)]

        # Stack for backtracking
        stack = []
        # Start position (always odd coordinates)
        start_x, start_y = 1, 1
        self.maze[start_y][start_x] = False  # Mark as passage
        current = (start_x, start_y)
        visited = {current}

        while len(visited) < self._count_cells():
            neighbors = self._get_unvisited_neighbors(current, visited)

            if neighbors:
                # Choose random neighbor
                next_cell = random.choice(neighbors)
                # Remove wall between current and next cell
                wall_x = (current[0] + next_cell[0]) // 2
                wall_y = (current[1] + next_cell[1]) // 2
                self.maze[wall_y][wall_x] = False

                # Mark next cell as passage
                self.maze[next_cell[1]][next_cell[0]] = False

                # Push current to stack and move to next
                stack.append(current)
                current = next_cell
                visited.add(current)
            elif stack:
                # Backtrack
                current = stack.pop()

        rospy.loginfo("Generated maze with {} cells".format(len(visited)))

    def _count_cells(self):
        """Count total number of possible cells (odd coordinates only)"""
        return ((self.width - 1) // 2) * ((self.height - 1) // 2)

    def _get_unvisited_neighbors(self, pos, visited):
        """Get unvisited neighbors at distance 2 (to maintain wall structure)"""
        x, y = pos
        neighbors = []

        # Check all four directions at distance 2
        for dx, dy in [(0, 2), (0, -2), (2, 0), (-2, 0)]:
            nx, ny = x + dx, y + dy
            if (0 < nx < self.width - 1 and 0 < ny < self.height - 1 and 
                (nx, ny) not in visited):
                neighbors.append((nx, ny))

        return neighbors

    def create_wall_sdf(self, length, is_horizontal=True):
        """Create SDF model string for a wall"""
        if is_horizontal:
            size_x = length * self.cell_size
            size_y = self.wall_thickness
        else:
            size_x = self.wall_thickness  
            size_y = length * self.cell_size

        sdf_template = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="maze_wall">
    <static>true</static>
    <link name="wall_link">
      <collision name="wall_collision">
        <geometry>
          <box>
            <size>{} {} {}</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="wall_visual">
        <geometry>
          <box>
            <size>{} {} {}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.7 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      <inertial>
        <mass>1000</mass>
        <inertia>
          <ixx>1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>1</iyy>
          <iyz>0</iyz>
          <izz>1</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>""".format(size_x, size_y, self.wall_height, size_x, size_y, self.wall_height)

        return sdf_template

    def spawn_maze_walls(self):
        """Spawn all maze walls in Gazebo"""
        rospy.loginfo("Spawning maze walls...")
        wall_count = 0

        # Find continuous wall segments and spawn them as single models
        for y in range(self.height):
            x = 0
            while x < self.width:
                if self.maze[y][x]:  # Found a wall
                    # Count consecutive horizontal walls
                    length = 0
                    while x + length < self.width and self.maze[y][x + length]:
                        length += 1

                    if length > 0:
                        # Calculate wall position (center of the segment)
                        wall_x = (x + length / 2.0 - self.width / 2.0) * self.cell_size
                        wall_y = (y - self.height / 2.0) * self.cell_size

                        # Create and spawn wall
                        wall_sdf = self.create_wall_sdf(length, is_horizontal=True)
                        wall_name = "maze_wall_h_{}".format(wall_count)

                        pose = Pose()
                        pose.position = Point(wall_x, wall_y, self.wall_height/2)
                        pose.orientation = Quaternion(0, 0, 0, 1)

                        try:
                            self.spawn_model(wall_name, wall_sdf, "", pose, "world")
                            self.wall_models.append(wall_name)
                            wall_count += 1
                        except Exception as e:
                            rospy.logerr("Failed to spawn wall {}: {}".format(wall_name, str(e)))

                        x += length
                    else:
                        x += 1
                else:
                    x += 1

        # Spawn vertical walls
        for x in range(self.width):
            y = 0
            while y < self.height:
                if self.maze[y][x]:  # Found a wall
                    # Count consecutive vertical walls
                    length = 0
                    while y + length < self.height and self.maze[y + length][x]:
                        length += 1

                    if length > 1:  # Only spawn if length > 1 to avoid duplicates
                        # Calculate wall position
                        wall_x = (x - self.width / 2.0) * self.cell_size
                        wall_y = (y + length / 2.0 - self.height / 2.0) * self.cell_size

                        # Create and spawn wall  
                        wall_sdf = self.create_wall_sdf(length, is_horizontal=False)
                        wall_name = "maze_wall_v_{}".format(wall_count)

                        pose = Pose()
                        pose.position = Point(wall_x, wall_y, self.wall_height/2)
                        pose.orientation = Quaternion(0, 0, 0, 1)

                        try:
                            self.spawn_model(wall_name, wall_sdf, "", pose, "world")
                            self.wall_models.append(wall_name)
                            wall_count += 1
                        except Exception as e:
                            rospy.logerr("Failed to spawn wall {}: {}".format(wall_name, str(e)))

                    y += length
                else:
                    y += 1

        rospy.loginfo("Spawned {} wall segments".format(wall_count))

    def clear_existing_maze(self):
        """Delete all existing maze walls"""
        rospy.loginfo("Clearing existing maze...")
        for wall_name in self.wall_models:
            try:
                self.delete_model(wall_name)
            except Exception as e:
                rospy.logwarn("Failed to delete wall {}: {}".format(wall_name, str(e)))
        self.wall_models.clear()

    def generate_and_spawn_maze(self):
        """Main function to generate and spawn a complete maze"""
        rospy.loginfo("Generating new random maze...")

        # Clear any existing maze
        self.clear_existing_maze()

        # Wait a moment for deletions to complete
        time.sleep(0.5)

        # Generate new maze layout
        self.generate_maze()

        # Spawn walls in Gazebo
        self.spawn_maze_walls()

        rospy.loginfo("Maze generation complete!")

    def print_maze(self):
        """Print maze to console for debugging"""
        for row in self.maze:
            print(''.join('█' if cell else ' ' for cell in row))

def regenerate_callback(req):
    """Service callback to regenerate maze"""
    global generator
    if generator:
        generator.generate_and_spawn_maze()
    return []  # Empty response

def main():
    """Main function"""
    global generator
    rospy.init_node('maze_generator', anonymous=True)

    # Parse command line arguments
    width = rospy.get_param('~width', 21)
    height = rospy.get_param('~height', 21) 
    wall_height = rospy.get_param('~wall_height', 2.0)
    cell_size = rospy.get_param('~cell_size', 2.0)

    rospy.loginfo("Initializing maze generator: {}x{}".format(width, height))

    try:
        # Create maze generator
        generator = MazeGenerator(width, height, wall_height=wall_height, cell_size=cell_size)

        # Generate and spawn maze
        generator.generate_and_spawn_maze()

        # Print maze layout to console
        generator.print_maze()

        # Service for regenerating maze
        rospy.Service('regenerate_maze', Empty, regenerate_callback)

        # Keep node alive to handle potential regeneration requests
        rospy.loginfo("Maze generator ready. Use 'rosservice call /regenerate_maze' to generate new maze")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Maze generator shutting down...")
    except Exception as e:
        rospy.logerr("Error in maze generator: {}".format(str(e)))
        sys.exit(1)

if __name__ == '__main__':
    generator = None
    main()
