#!/usr/bin/env python

# Python 2/3 compatibility import
from __future__ import print_function

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
import geometry_msgs.msg
import time
import sys


class CollisionSceneExample(object):
    def __init__(self):
        self._scene = PlanningSceneInterface()

        # clear the scene
        self._scene.remove_world_object()

        self.robot = RobotCommander()

        # pause to wait for rviz to load
        print("============ Waiting while RVIZ displays the scene with obstacles...")

        # TODO: need to replace this sleep by explicitly waiting for the scene to be updated.
        rospy.sleep(2)

    def add_floor(self):
        box1_pose = [0.0, 0.0, -0.25, 0, 0, 0, 1]
        box1_dimensions = [2, 2, 0.5]

        self.add_box_object("floor", box1_dimensions, box1_pose)

        print("============ Added one obstacle to RViz!!")

    def add_four_walls(self):
        box1_pose = [-1, 0, 0, 0, 0, 0, 1]
        box1_dimensions = [0.2, 2, 2]

        box2_pose = [1, 0, 0, 0, 0, 0, 1]
        box2_dimensions = [0.2, 2, 2]

        box3_pose = [0, -1, 0, 0, 0, 0, 1]
        box3_dimensions = [2, 0.2, 2]

        box4_pose = [0, 1, 0, 0, 0, 0, 1]
        box4_dimensions = [2, 0.2, 2]

        self.add_box_object("wall1", box1_dimensions, box1_pose)
        self.add_box_object("wall2", box2_dimensions, box2_pose)
        self.add_box_object("wall3", box3_dimensions, box3_pose)
        self.add_box_object("wall4", box4_dimensions, box4_pose)

        print("========== Added 4 obstacles to the scene!!")

    def add_box_object(self, name, dimensions, pose):
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = pose[0]
        p.pose.position.y = pose[1]
        p.pose.position.z = pose[2]
        p.pose.orientation.x = pose[3]
        p.pose.orientation.y = pose[4]
        p.pose.orientation.z = pose[5]
        p.pose.orientation.w = pose[6]

        self._scene.add_box(name, p, (dimensions[0], dimensions[1], dimensions[2]))


if __name__ == "__main__":
    rospy.init_node("collision_scene_example_cluttered")
    while (
        not rospy.search_param("robot_description_semantic") and not rospy.is_shutdown()
    ):
        time.sleep(0.5)
    load_scene = CollisionSceneExample()

    if len(sys.argv) != 2:
        print(
            'Correct usage:: \n"rosrun moveit_tutorials collision_scene_example.py walls" OR \n"rosrun moveit_tutorials collision_scene_example.py floor"'
        )
        sys.exit()
    if sys.argv[1] == "walls":
        load_scene.add_four_walls()
    elif sys.argv[1] == "floor":
        load_scene.add_floor()
    else:
        print("Please specify correct type of scene as walls or floor")
        sys.exit()