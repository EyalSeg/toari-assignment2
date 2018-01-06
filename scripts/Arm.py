#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Point, Pose

import moveit_commander

class Arm(object):
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()

        self.group = get_group()

    def set_pose(self, point, orientation=None):
        group = self.group
        target_pose = Pose()
        if orientation is not None:
            target_pose.orientation = orientation
        else:
            target_pose.orientation.w = 1.

        target_pose.position = Point(*point)
        group.set_pose_target(target_pose)
        plan1 = group.plan()
        group.go()
        rospy.sleep(5.)


def get_group():
    group = moveit_commander.MoveGroupCommander("arm")
    group.set_max_velocity_scaling_factor(0.1)
    group.set_max_acceleration_scaling_factor(0.5)
    group.set_num_planning_attempts(500)
    group.set_planner_id("RRTConnectkConfigDefault")
    group.set_pose_reference_frame("base_footprint")
    group.set_goal_joint_tolerance(0.1)

    return group