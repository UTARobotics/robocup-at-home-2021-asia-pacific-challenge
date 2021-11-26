#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

from std_msgs.msg import String

import actionlib
import math
import moveit_commander
import os
import rospy
import subprocess
import tf
import tf2_ros
import time

# import matplotlib.pyplot as plt

from utils_v2 import *
from param import *


from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist

from deepsparse_yolo_msgs.msg import Object, Objects
from deepsparse_yolo_msgs.srv import SetDetectState, SetDetectStateResponse, RemoveObject, RemoveObjectResponse, ClearObjects, ClearObjectsResponse

# ------YOLO and shelf_gripping class & function (koksyuen)-------
class YOLO():
    object_list_topic = 'objects/list'
    yolo_detect_service = '/yolo/set_detect_state'
    object_remove_service = '/objects/remove_object'
    object_clear_service = '/objects/clear_list'
    EASY_FOOD_SAMPLE = ["tuna_fish_can", "tomato_soup_can", "strawberry", "apple", "lemon", "peach", "pear", "orange", "plum"]
    
    def __init__(self):
        self.detected_item_list = None

        # Subscribers
        rospy.Subscriber(self.object_list_topic, Objects, self.object_cb)

    def object_cb(self, msg):
        self.detected_item_list = msg.objects
        
    #msg = True start detect 
    def detect(self, msg):
        rospy.wait_for_service(self.yolo_detect_service)
        change_state = rospy.ServiceProxy(self.yolo_detect_service, SetDetectState)
        response = change_state(msg)
        print "(Triggered detection)"
        
    def clear_list(self, msg):
        rospy.wait_for_service(self.object_clear_service)
        change_state = rospy.ServiceProxy(self.object_clear_service, ClearObjects)
        response = change_state(msg)
        print "(Cleared object database)"

    #msg = True start detect 
    def object_remove(self, cls, id):
        rospy.wait_for_service(self.object_remove_service)
        change_state = rospy.ServiceProxy(self.object_remove_service, RemoveObject)
        response = change_state(cls, id)
        print("Remove {0} with id {1} from object database.".format(cls,id))
        
    def finished_detection(self):
        return ((self.detected_item_list != None) and (len(self.detected_item_list)>0))

    def detected_items(self):
        return self.detected_item_list
    
    def get_coordinate(self, item_class):
        for item in self.detected_item_list:
            if item.Class == item_class:
                return item.x, item.y, item.z
            
    def easy_item_available(self):
        easy_grab_food = self.EASY_FOOD_SAMPLE
        
        while not self.finished_detection():
            rospy.sleep(1.0)
            
        yolo_food = [item.Class for item in self.detected_item_list]
        print("YOLO: ", ' '.join(map(str, yolo_food)))
                        
        for item in self.detected_item_list:
            for food in easy_grab_food:
                if ((item.Class == food) and (item.z <= 1.0) and (item.x >=2.0) and (item.x <=2.5)):
                    return True
                
        print("No food to grab")
        return False
        
    def target_item_coordinate(self, target=None):
        EASY_FOOD = self.EASY_FOOD_SAMPLE
        detected_easy_food = []
                        
        for item in self.detected_item_list:
            for food in EASY_FOOD:
                if item.Class == food and item.z <= 1.0 and item.x >=2.0 and item.x <=2.5:
                    detected_easy_food.append(item)
                    EASY_FOOD.remove(food)
        
        detect_easy_class = [item.Class for item in detected_easy_food]
        print(' '.join(map(str, detect_easy_class)))
        
        if (target == None):
            y_loc =[item.y for item in detected_easy_food]
            min_value = min(y_loc)
            print(min_value)
            min_index = y_loc.index(min_value)
            target_food = detected_easy_food[min_index].Class
            print(target_food)
            return self.get_coordinate(target_food)
        else:
            print(target_food)
            return self.get_coordinate(target_food)
    
    def receive_food_msg(self, command_food):
        easy_food = self.EASY_FOOD_SAMPLE
        yolo_food = [item.Class for item in self.detected_item_list]
        print("YOLO: ", ' '.join(map(str, yolo_food)))
        objts = (list(set(yolo_food).intersection(easy_food)))
        print("EASY YOLO: ", ' '.join(map(str, objts)))
        objts2 = (list(set(command_food).intersection(objts)))

        if len(objts2) != 0:
            confident_object = (''.join(objts2))
            print(confident_object, " detected")
        else:
            return self.target_item_coordinate(None)
        
        confident_x, confident_y, confident_z = self.get_coordinate(confident_object)
        
        if (confident_x < 2.0) or (confident_x > 2.5):
            print(confident_object, " out of range")
            return self.target_item_coordinate(None)
        
        # check present of obstacle
        for item in self.detected_item_list:
            if item.Class != confident_object:
                if (abs(item.z - confident_z) < 0.3) and (abs(item.x - confident_x) < 0.1) and (item.y < confident_y):
                    print("dx: %f" %(abs(item.x - confident_x)))
                    print("dz: %f" %(abs(item.z - confident_z)))
                    print("target_y: %f  block_y %f" %(confident_y,item.y))
                    print(confident_object, " is blocked by", item.Class)
                    return self.target_item_coordinate(None)
        
        print(confident_object)
        return confident_x, confident_y, confident_z
                                     
class ARM():
    def __init__(self):
        base.set_planner_id("PRM")
        
    def move_base_link_pose_ik(self, ref_frame, x, y, yaw):

        p = geometry_msgs.msg.Pose()

        p = PoseStamped()

        p.header.frame_id = ref_frame

        p.pose.position.x = x 
        p.pose.position.y = y


        p.pose.orientation = quaternion_from_euler(0, 0, yaw)

        base.set_pose_target(p)

        for i in range(10):
            print('iteration %d'%i)
            success = base.go(wait=True)
            if success:
                 break
        return success
    
    def gripper_grip(self, magnitude, grip_wait=True):
        for i in range(10):
            print('gripper iteration %d'%i)
            gripper.set_joint_value_target("hand_motor_joint", magnitude)
            success = gripper.go(wait=grip_wait)
            if success:
                 return success
                        
    def grab(self, x, y, z, collision_obj):
        
        # Parameters
        CENTROID_RADIUS = 0.02
        X_OFFSET = 0.077997053445
        Y_OFFSET = {
          "shelf_level3": 0.667,
          "shelf_level2": 0.647,
          "shelf_level1": 0.647,
          "shelf_level0": 0.617,
        }
             
       # Go to predefined arm pose based on the target object's height
        
        if (z > 0.95):
            shelf_level = "shelf_level3"
        elif (z > 0.7):
            shelf_level = "shelf_level2"
        else:
            shelf_level = "shelf_level1"
            
        # Open gripper
        self.gripper_grip(1.0, grip_wait = False)
            
        arm.set_named_target(shelf_level)
        arm.go(wait=True)
               
        # IK Preparation       
        clear_octomap()
        
        # Create collision object at target item
        collision_obj.sphere(x, y, z, CENTROID_RADIUS, "map", "centroid")
        
        # Align X-AXIS
        while not self.move_base_link_pose_ik("map", x+X_OFFSET, 3.7, 90):
            clear_octomap()
        
        # Align Y-AXIS
        while not self.move_base_link_pose_ik("map", x+X_OFFSET, y - Y_OFFSET[shelf_level], 90):
            clear_octomap()
        #rospy.sleep(3)      
        
        # Close gripper
        self.gripper_grip(0.05, grip_wait=True)
        
        # Remove arm from the shelf
        #move_base_vel(-1.0,0,0)
        #arm_joints = arm.get_current_joint_values()
        #arm.set_joint_value_target("arm_lift_joint", arm_joints[0]+0.04)
        if shelf_level == "shelf_level2":
            arm.set_joint_value_target("arm_lift_joint", 0.69)
        elif shelf_level == "shelf_level1":
            arm.set_joint_value_target("arm_lift_joint", 0.4)
        arm.go(wait=True)
        
        while not self.move_base_link_pose_ik("map", x+X_OFFSET, 3.7, 90):
           clear_octomap()
        #rospy.sleep(1)
        
        arm.set_named_target("transport_object")
        arm.go(wait=False)

class Task_2(object):

    def __init__(self):

        self.food = ['master_chef_can','master_chef_coffee','cracker_box','sugar_box','tomato_soup_can'
            ,'mustard_bottle','tuna_fish_can','pudding_box','gelatin_box','potted_meat_can'
            ,'banana','strawberry','apple','lemon','peach','pear','orange','plum','chips_can'
            ,'left', 'right', 'done', 'None']

        self.command = []
        self.collision_obj = collision_object()
        self.head = moveit_commander.MoveGroupCommander("head")
        self.navclient = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.Subscriber("/message", String, self.callback)
        self.robot_arm = ARM()
        self.yolo = YOLO()

    def grip_food(self,command_list):
        
        # robot_arm = ARM()

        # Trigger YOLO detection
        self.yolo.clear_list(True)
        self.yolo.detect(True)

        # Ready grip arm pose
        arm.set_named_target("shelf_level1")
        arm.go(wait=True)

        # Wait for YOLO finished detection
        while not self.yolo.finished_detection():
            print("===Waiting===")
            rospy.sleep(0.5)

        
        if self.yolo.easy_item_available():        
            item_x, item_y, item_z = self.yolo.receive_food_msg(command_list)

            self.robot_arm.grab(item_x, item_y, item_z,self.collision_obj)   

    def give_food(self):
        arm.set_named_target("shelf_level3")
        arm.go(wait=True)
        #rospy.sleep(1)
        move_hand(1.0)
  
    def callback(self,data):
        message = data.data.lower().replace("’",'', 10).replace("'",'', 10)
        for target in self.food:
            if target in message : 
                if self.command.count(target) == 0:
                    self.command.append(target)
                    rospy.logwarn("Robot heard %s", target)

    def quaternion_from_euler(self,roll, pitch, yaw):
        q = tf.transformations.quaternion_from_euler(roll / 180.0 * math.pi,
                                                    pitch / 180.0 * math.pi,
                                                    yaw / 180.0 * math.pi, 'rxyz')
        return Quaternion(q[0], q[1], q[2], q[3])

    def move_base_goal(self,x, y, theta):

        while(not self.navclient.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo('Waiting for the move_base action server to come up')

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = self.quaternion_from_euler(0, 0, theta)

        rospy.loginfo('Sending goal location')
        self.navclient.send_goal(goal)
        self.navclient.wait_for_result()
        state = self.navclient.get_state()

        result = 'SUCCEEDED'if state == 3 else 'FAILED'
        rospy.loginfo(result)

    def move_base_vel (self,vx, vy, vw):
        twist = Twist ()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = vw / 180.0 * math.pi # Convert from "degree" to "radian"
        self.base_vel_pub.publish (twist) # Publish velocity command

    def move_head_tilt(self, v):
        self.head.set_joint_value_target("head_tilt_joint", v)
        return self.head.go()

    def obstacle_avoidance(self):
        #yolo.detect(True)
        #while not (yolo.finished_detection()):
        #    rospy.sleep(1)
        #detected_items = yolo.detected_items()
        #rospy.logwarn("Robot detected %s object", len(detected_items))

        #obs = []
        #for item in detected_items:
        #    if item.x >= 2.2 and item.x <= 2.8:
        #        if item.y >= 2.2 and item.y <= 2.8:
        #            obs.append(item)
        #if len(obs) == 0:
        #move_head_tilt(-1.0)
        #move_base_goal(2.28, 3.84, 90) # Move to front of shelf
        
        self.collision_obj.scene.remove_world_object("task_1_boundary_1")
        self.collision_obj.scene.remove_world_object("task_1_boundary_2")
        self.collision_obj.scene.remove_world_object("task_1_boundary_3")
        self.collision_obj.scene.remove_world_object("task_1_boundary_4")
        self.collision_obj.scene.remove_world_object("task_1_boundary_5")

        GROUND_LENGTH = 100.00
        GROUND_WIDTH = 100.00
        GROUND_HEIGHT = 0.001
        self.collision_obj.box(0, 0, -0.075, GROUND_LENGTH, GROUND_WIDTH, GROUND_HEIGHT, "map", "ground")

        SHELF_LENGTH = 6.0
        SHELF_WIDTH = 0.005
        SHELF_HEIGHT = 1.0

        self.collision_obj.box(-1.1, 2.05, 0.5, SHELF_LENGTH, SHELF_WIDTH, SHELF_HEIGHT, "map", "wall1")
        self.collision_obj.box(-0.8, 0.65, 0.5, SHELF_WIDTH, SHELF_LENGTH, SHELF_HEIGHT, "map", "wall2")
        self.collision_obj.box(1.5, -1.0, 0.5, SHELF_LENGTH, SHELF_WIDTH, SHELF_HEIGHT, "map", "wall3")
        self.collision_obj.box(3.2, 1.9, 0.5, SHELF_WIDTH, SHELF_LENGTH, SHELF_HEIGHT, "map", "wall4")
              
        clear_octomap()
        
        time.sleep(15.0)

        self.robot_arm.move_base_link_pose_ik("map",2.28, 3.84, 90)

    def reach_person(self):
        if self.command.count('left') == 1: 
            self.move_base_goal(0.7, 2.73, 180)
        elif self.command.count('right') == 1: 
            self.move_base_goal(0.7, 3.98, 180)
        else: 
            self.move_base_goal(0.7, 3.4, 180)
            if self.command.count('left') == 1: self.move_base_goal(0.7, 2.73, 180)
            if self.command.count('right') == 1: self.move_base_goal(0.7, 3.98, 180)

    def task_2(self):

        self.move_head_tilt(-1.0) 
        self.move_base_goal(2.6, 0, 0)
        self.move_base_goal(2.6, 1.5, 90)
        self.move_head_tilt(-0.7)
        
        # ------------ Obstacle Avoidance--------------
        self.obstacle_avoidance()
        # ---------------------------------------------

        #move_head_tilt(-1.0)
        #move_base_goal(2.28, 3.84, 90) # Move to front of shelf
        self.move_head_tilt(-0.3)
        
        # ------------Object Grasping -----------------
        self.grip_food(self.command)
        #self.move_base_vel(-1.0,0,0)
        #arm.set_named_target("transport_object")
        #arm.go(wait=False)
        #----------------------------------------------
        
        #-------------Approach person------------------
        self.reach_person()
        #----------------------------------------------
        
        #--------------Give item 2 person -------------
        self.give_food()
        #----------------------------------------------
        
        while not (self.command.count('done') == 1):
            rospy.sleep(1.0)
           
        return True
            

if __name__ == '__main__':
    rospy.logwarn('Program START')
    rospy.init_node('go_n_get_it', anonymous=True)
    t2 = Task_2()
    t2.task_2()

    
    #print('keyword heard: ', command)
    #print('DONE')
    #rospy.spin()

    
# To test the code, open a new terminal, ebter following command line,
# rostopic pub -1 /message std_msgs/String -- '<enter the target food here>'
