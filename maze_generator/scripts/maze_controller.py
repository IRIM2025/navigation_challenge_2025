#!/usr/bin/env python3
"""
Maze Controller - Provides additional services for maze management
"""
import rospy
from std_srvs.srv import Empty, EmptyResponse
from gazebo_msgs.srv import GetWorldProperties, GetModelState
import subprocess
import os

class MazeController:
    def __init__(self):
        rospy.init_node('maze_controller', anonymous=True)

        # Services
        self.clear_maze_srv = rospy.Service('clear_maze', Empty, self.clear_maze_callback)
        self.regenerate_maze_srv = rospy.Service('regenerate_maze', Empty, self.regenerate_maze_callback)
        self.get_maze_info_srv = rospy.Service('get_maze_info', Empty, self.get_maze_info_callback)

        rospy.loginfo("Maze controller ready. Available services:")
        rospy.loginfo("  - /clear_maze: Remove all maze walls")
        rospy.loginfo("  - /regenerate_maze: Generate new random maze")
        rospy.loginfo("  - /get_maze_info: Get information about current maze")

    def clear_maze_callback(self, req):
        """Clear all maze walls from Gazebo"""
        try:
            # Get all models in Gazebo
            get_world_props = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            world_props = get_world_props()

            # Delete all maze wall models
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            deleted_count = 0

            for model_name in world_props.model_names:
                if model_name.startswith('maze_wall'):
                    try:
                        delete_model(model_name)
                        deleted_count += 1
                    except Exception as e:
                        rospy.logwarn("Failed to delete model {}: {}".format(model_name, str(e)))

            rospy.loginfo("Cleared {} maze walls".format(deleted_count))
            return EmptyResponse()

        except Exception as e:
            rospy.logerr("Error clearing maze: {}".format(str(e)))
            return EmptyResponse()

    def regenerate_maze_callback(self, req):
        """Regenerate maze by calling the maze generator service"""
        try:
            regenerate_srv = rospy.ServiceProxy('/regenerate_maze', Empty)
            regenerate_srv()
            rospy.loginfo("Maze regeneration requested")
            return EmptyResponse()
        except Exception as e:
            rospy.logerr("Error regenerating maze: {}".format(str(e)))
            return EmptyResponse()

    def get_maze_info_callback(self, req):
        """Get information about current maze"""
        try:
            get_world_props = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            world_props = get_world_props()

            maze_walls = [name for name in world_props.model_names if name.startswith('maze_wall')]

            rospy.loginfo("Current maze info:")
            rospy.loginfo("  - Total wall segments: {}".format(len(maze_walls)))
            rospy.loginfo("  - Horizontal walls: {}".format(len([w for w in maze_walls if '_h_' in w])))
            rospy.loginfo("  - Vertical walls: {}".format(len([w for w in maze_walls if '_v_' in w])))

            return EmptyResponse()

        except Exception as e:
            rospy.logerr("Error getting maze info: {}".format(str(e)))
            return EmptyResponse()

    def run(self):
        """Keep the node running"""
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = MazeController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Maze controller shutting down...")
