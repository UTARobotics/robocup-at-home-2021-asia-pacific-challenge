# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

rospy.init_node("manipulation")

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
# Specify whole body control as the control target of moveit
group = moveit_commander.MoveGroupCommander("whole_body")


# Add grasping object - MESH
rospy.sleep(4)
mesh_pose = geometry_msgs.msg.PoseStamped()
mesh_pose.header.frame_id = "base_footprint"
mesh_pose.pose.orientation.w = 1.0
mesh_pose.pose.position.x = 1.0
mesh_pose.pose.position.y = 1.0
mesh_pose.pose.position.z = 0.5 

print (mesh_pose)

mesh_name = "mesh"
scene.add_mesh(mesh_name, mesh_pose, '/opt/ros/melodic/share/tmc_wrs_gazebo_worlds/models/trofast/meshes/trofast.stl')
#scene.add_mesh(mesh_name, mesh_pose, '/opt/ros/melodic/share/tmc_wrs_gazebo_worlds/models/trofast/meshes/trofast.stl', size=(1, 1, 1))


