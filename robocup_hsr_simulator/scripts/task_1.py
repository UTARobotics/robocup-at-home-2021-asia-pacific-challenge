#! /usr/bin/env python
# -*- encoding: UTF-8 -*
from task_2 import *
# Pick & Place by using IK of whole body

import math 
from utils_v2 import *
from param import *
from deepsparse_yolo_msgs.msg import Object, Objects
from deepsparse_yolo_msgs.srv import SetDetectState, SetDetectStateResponse, RemoveObject, RemoveObjectResponse, ClearObjects, ClearObjectsResponse


# ------YOLO-------
class YOLO_t1():

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

 
    def object_remove(self, cls, id):

        rospy.wait_for_service(self.object_remove_service)
        change_state = rospy.ServiceProxy(self.object_remove_service, RemoveObject)
        response = change_state(cls, id)
        print("Remove {0} with id {1} from object database.".format(cls,id))

        
    def finished_detection(self):

        return ((self.detected_item_list != None) and (len(self.detected_item_list)>0))


    def detected_items(self):

        return self.detected_item_list


    def get_item_info(self, item_class):

        for item in self.detected_item_list:
            if item.Class == item_class:
                return item


def object_grasping():

    # Check whether the object is successfully grasped
    arm.set_named_target("neutral")
    arm.go(wait=True)

    move_head_tilt(-1.0)
    rospy.sleep(4)
    points_data = rgbd.get_points()
    pixel_between_fingers_1 = points_data['z'][160][223]
    pixel_between_fingers_2 = points_data['z'][175][223]
    pixel_between_fingers_3 = points_data['z'][196][223]
    pixel_between_fingers_4 = points_data['z'][210][223]
    pixel_between_fingers_5 = points_data['z'][220][223]
    if math.isnan(pixel_between_fingers_1) == True or math.isnan(pixel_between_fingers_2) == True or math.isnan(pixel_between_fingers_3) == True or math.isnan(pixel_between_fingers_4) == True or math.isnan(pixel_between_fingers_5) == True:
        print("grasped object")
        return True
    else:
        print("no grasped object")
        return False 


class ARM_t1():

    def __init__(self):

        self.grasp_tool = False
        self.grasp_shape_item = False
        self.check_entrance = True
        self.picked_food = 0
        self.target_item = None
        self.failed_item_list = []
        self.hand_palm_base_link_offset = 0.078
        self.hand_palm_centroid_offset = 0.04
        self.num_attempted_item = 0

        base.set_planner_id("PRM")
        base.set_goal_joint_tolerance(0.001)
        arm.set_goal_joint_tolerance(0.001)
        # gripper.set_max_velocity_scaling_factor(0.6)
        
    
    def upload_planning_scene(self):

        print("Uploading planning scene...")
        rospy.sleep(1)
        collision_object.box(0.0, 0.0, -0.075, GROUND_LENGTH, GROUND_WIDTH, GROUND_HEIGHT, "map", "ground")
        rospy.sleep(1)
        collision_object.box(-0.7, 0.8, 0.5, 0.01, 2.5, 1, "map", "task_1_boundary_1")
        collision_object.box(3.1, 0.8, 0.5, 0.01, 2.5, 1, "map", "task_1_boundary_2")
        collision_object.box(1.2, 2.1, 0.5, 3.8, 0.01, 1, "map", "task_1_boundary_3")
        collision_object.box(1.2, -0.4, 0.07, 3.8, 0.01, 0.14, "map", "task_1_boundary_4") # Deposit Area Front
        collision_object.box(0.5, 1.6, 0.24, 2.6, 0.01, 0.02, "map", "task_1_boundary_5") # Search Area Table 
        print("Completed uploading planning scene...")


    def check_entrance_floor(self):
        
        print("Checking entrance floor...")
        y_min_dis = 0.9
        # All items within "search area" are at least y distance away (wrt origin of "map" frame)?

        arm.set_named_target("neutral")
        arm.go(wait=True)
        # Head right
        move_head_pan(1.2)
        # Head down
        move_head_tilt(-0.9)
        #wait for robot to finish move to pose
        rospy.sleep(3)

        # Trigger YOLO detection
        yolo.clear_list(True)
        yolo.detect(True)
        rospy.sleep(3)
        detected_item_list = yolo.detected_items()

        # Head central
        move_head_pan(0.0)
        move_arm_init()

        for item in range(len(detected_item_list)):
            print(detected_item_list[item])
            if detected_item_list[item].y < y_min_dis:
                print("Search area has items near outermost edge, might cause collision during navigation, decided not to open drawers...")
                break
        else:
            print("Decide to open drawers...")
            self.grasp_tool = True
            self.grasp_shape_item = True
            self.open_drawers()        


    def open_drawers(self):

        print("Navigating to drawers...")

        try:
            navigate_to('Drawers')

            # Upload collision objects of drawers in Moveit Rviz
            collision_object.box((0.49 - 0.33), -0.53, 0.22, DRAWER_LENGTH, DRAWER_WIDTH, DRAWER_HEIGHT, "map", "drawer_bottom")
            collision_object.box((0.49 - 0.33), -0.53, (0.22 + 0.28), DRAWER_LENGTH, DRAWER_WIDTH, DRAWER_HEIGHT, "map", "drawer_top")
            collision_object.box(0.49, -0.53, 0.22, DRAWER_LENGTH, DRAWER_WIDTH, DRAWER_HEIGHT, "map", "drawer_left")
            clear_octomap()

            # Slightly "open" the gripper
            move_hand(0.30)
            # Allign the robot's base with knob of "drawer_bottom"
            move_base_pose_ik("map", DRAWER_BOTTOM_KNOB[0] - 0.078, (DRAWER_BOTTOM_KNOB[1] + SAFETY_PRE_GRASP_APPROACH_DIS), -90)
            # Allign the gripper with knob of "drawer_bottom" by FK
            arm.set_named_target("drawer_bottom")
            arm.go(wait=True)
            clear_octomap()
            # Moving forward to grasp
            # move_base_pose_ik("map", DRAWER_BOTTOM_KNOB[0] - 0.078, DRAWER_BOTTOM_KNOB[1], -90)
            base_variable_values = base.get_current_joint_values()
            base_variable_values[0] -= SAFETY_PRE_GRASP_APPROACH_DIS 
            base.set_joint_value_target(base_variable_values)
            for i in range(10):
                print("Moving toward drawer knob, trial " + str(i))
                if base.go(wait=True):
                    break
            # "close" the gripper
            rospy.sleep(1.0)
            move_hand(0.0)
            
            # collision_object.attach("drawer_bottom")
            # Retract the base to pull out drawer
            # move_base_vel(-0.05, 0, 0, -0.27, 0, 0)
            # move_base_pose_ik("map", DRAWER_BOTTOM_KNOB[0] - 0.078, (DRAWER_BOTTOM_KNOB[1] + 0.27), -90)
            base_variable_values = base.get_current_joint_values()
            base_variable_values[0] += 0.27  
            base.set_joint_value_target(base_variable_values)
            for i in range(10):
                print("Pulling out drawer, trial " + str(i))
                if base.go(wait=True):
                    break
            rospy.sleep(1.0)
            # Fully "open" the gripper
            move_hand(1.1)
            
            # collision_object.dettach("drawer_bottom")
            clear_octomap()
            move_base_vel(-0.06, 0, 0, -0.06, 0, 0)
            
            collision_object.box((0.49 - 0.33), (-0.53 + 0.27), 0.22, DRAWER_LENGTH, DRAWER_WIDTH, DRAWER_HEIGHT, "map", "drawer_bottom")
            move_arm_init()
            clear_octomap()

            move_hand(0.30)
            move_base_pose_ik("map", DRAWER_TOP_KNOB[0] - 0.078, (DRAWER_TOP_KNOB[1] + SAFETY_PRE_GRASP_APPROACH_DIS), -90)
            arm.set_named_target("drawer_top")
            arm.go(wait=True)
            clear_octomap()
            # move_base_pose_ik("map", DRAWER_TOP_KNOB[0] - 0.078, DRAWER_TOP_KNOB[1], -90)
            base_variable_values = base.get_current_joint_values()
            base_variable_values[0] -= SAFETY_PRE_GRASP_APPROACH_DIS 
            base.set_joint_value_target(base_variable_values)
            for i in range(10):
                print("Moving toward drawer knob, trial " + str(i))
                if base.go(wait=True):
                    break
            rospy.sleep(1.0)
            move_hand(0.0)
            
            # collision_object.attach("drawer_top")
            # move_base_vel(-0.05, 0, 0, -0.27, 0, 0)
            # move_base_pose_ik("map", DRAWER_TOP_KNOB[0] - 0.078, (DRAWER_TOP_KNOB[1] + 0.27), -90)
            base_variable_values = base.get_current_joint_values()
            base_variable_values[0] += 0.27  
            base.set_joint_value_target(base_variable_values)
            for i in range(10):
                print("Pulling out drawer, trial " + str(i))
                if base.go(wait=True):
                    break
            rospy.sleep(1.0)
            move_hand(1.1)
            
            # collision_object.dettach("drawer_top")
            clear_octomap()
            move_base_vel(-0.06, 0, 0, -0.06, 0, 0)
            
            collision_object.box((0.49 - 0.33), (-0.53 + 0.27), (0.22 + 0.28), DRAWER_LENGTH, DRAWER_WIDTH, DRAWER_HEIGHT, "map", "drawer_top")
            move_arm_init()
            clear_octomap()

            move_hand(0.30)
            move_base_pose_ik("map", DRAWER_LEFT_KNOB[0] - 0.078, (DRAWER_LEFT_KNOB[1] + SAFETY_PRE_GRASP_APPROACH_DIS), -90)
            arm.set_named_target("drawer_bottom")
            arm.go(wait=True)
            
            collision_object.box(0.49, -0.55, 0.22, DRAWER_LENGTH, DRAWER_WIDTH, DRAWER_HEIGHT, "map", "drawer_left")
            # clear_octomap()
            # move_base_pose_ik("map", DRAWER_LEFT_KNOB[0] - 0.078, DRAWER_LEFT_KNOB[1], -90)
            base_variable_values = base.get_current_joint_values()
            base_variable_values[0] -= SAFETY_PRE_GRASP_APPROACH_DIS 
            base.set_joint_value_target(base_variable_values)
            for i in range(10):
                print("Moving toward drawer knob, trial " + str(i))
                if base.go(wait=True):
                    break
            rospy.sleep(1.0)
            move_hand(0.0)
            
            # collision_object.attach("drawer_left")
            # move_base_vel(-0.05, 0, 0, -0.27, 0, 0)
            # move_base_pose_ik("map", DRAWER_LEFT_KNOB[0] - 0.078, (DRAWER_LEFT_KNOB[1] + 0.27), -90)
            base_variable_values = base.get_current_joint_values()
            base_variable_values[0] += 0.27  
            base.set_joint_value_target(base_variable_values)
            for i in range(10):
                print("Pulling out drawer, trial " + str(i))
                if base.go(wait=True):
                    break
            rospy.sleep(1.0)
            move_hand(1.1)
            
            # collision_object.dettach("drawer_left")
            clear_octomap()
            move_base_vel(-0.06, 0, 0, -0.06, 0, 0)
            
            collision_object.box(0.49, (-0.53 + 0.25), 0.22, DRAWER_LENGTH, DRAWER_WIDTH, DRAWER_HEIGHT, "map", "drawer_left")
            move_base_vel(-0.05, 0, 0, -0.05, 0, 0)
            move_arm_init()
            clear_octomap()

            # Remove drawer from moveit planning scene
            collision_object.scene.remove_world_object("drawer_left")
            rospy.sleep(1)
            collision_object.scene.remove_world_object("drawer_bottom")
            rospy.sleep(1)
            collision_object.scene.remove_world_object("drawer_top")
            rospy.sleep(1)
            
        except EOFError:
            print("Error!!!")


    def search(self):

        # Fully "open" the gripper
        move_hand(1.0)
        move_arm_init()
        
        print("Navigating to search area...")
        num_type_sequence = "even" if self.num_attempted_item % 2 == 0 else "odd"

        if num_type_sequence == "even":
            navigate_to('Search_Area_Front_Left')
        elif num_type_sequence == "odd":
            navigate_to('Search_Area_Front_Right')
        
        self.num_attempted_item += 1

        print("Calculating manipulating cost for each detected items...")
        self.manipulation_cost()
        print("after cost")
        
        collision_object.sphere(self.target_item.x, self.target_item.y, self.target_item.z, CENTROID_RADIUS, "map", "centroid")
        rospy.sleep(1.0)
        
        print("Clear octomap...")
        clear_octomap()
        move_hand(1.0)
        
        self.pick()

        picked_up = object_grasping()
        if not picked_up:
            self.failed_item_list.append(self.target_item)
            for i in range (len(self.failed_item_list)):
                print("Failed item: " + self.failed_item_list[i].Class)
        else:
            move_arm_init()
            move_arm_init()
            
            self.place() 

            move_base_vel(-0.1, 0, 0, -0.1, 0, 0)


    def manipulation_cost(self):
        
        # Total_manipulation_cost = travel_cost *  grasp_difficulty
        # Select target_item with lowest total cost    

        print("Inside cost")
        shortest_robot_item_distance = 999999999999999 # Infinity
        lowest_total_cost = 999999999999999 # Infinity
        print("shortest robot item distance: ")
        print(str(shortest_robot_item_distance))

        yolo.clear_list(True)
        yolo.detect(True)
        rospy.sleep(6)

        detected_item_list = yolo.detected_items()

        print("grasp_tool = " + str(self.grasp_tool))
        print("grasp_shape_item = " + str(self.grasp_shape_item))

        #coordinate of base_link wrt map frame
        map_base = get_relative_coordinate("map", "base_link")
        print("Looked up 'map' to 'base' transform: ")
        print(map_base)

        aborted = False

        for item in range(len(detected_item_list)):
            # if (detected_item_list[item].Class in TOOL_ITEMS and self.grasp_tool == False) or (detected_item_list[item].Class in SHAPE_ITEMS and self.grasp_shape_item == False):
            #     continue
            # if item at task 2 area: ignore them
            if (detected_item_list[item].y > 2.2):
                continue
            print(detected_item_list[item].Class + str(detected_item_list[item].id))
            # Euclidean distance
            robot_item_distance = math.sqrt( (map_base.translation.x - detected_item_list[item].x) ** 2 + (map_base.translation.y - detected_item_list[item].y) ** 2 + (map_base.translation.z - detected_item_list[item].z) )
            print("Robot-Item distance")
            print(robot_item_distance)
            if robot_item_distance < shortest_robot_item_distance:
                shortest_robot_item_distance = robot_item_distance
            print("next")
            r.sleep()
        
        print("Item with shortest distance " + str(shortest_robot_item_distance) + " is " )

        for item in range(len(detected_item_list)):
            # if (detected_item_list[item].Class in TOOL_ITEMS and self.grasp_tool == False) or (detected_item_list[item].Class in SHAPE_ITEMS and self.grasp_shape_item == False):
            #     continue
            # if item at task 2 area: ignore them
            if (detected_item_list[item].y > 2.2):
                continue
            if len(self.failed_item_list) > 0:
                for i in range(len(self.failed_item_list)):
                    if (detected_item_list[item].Class in self.failed_item_list[i].Class):
                        aborted = True
            if (detected_item_list[item].Class in HIGH_GRASP_DIFFICUTY):
                GRASP_COST = 1.0
            elif (detected_item_list[item].Class in MEDIUM_GRASP_DIFFICUTY):
                GRASP_COST = 0.7
            elif (detected_item_list[item].Class in LOW_GRASP_DIFFICUTY):
                GRASP_COST = 0.5
            
            robot_item_distance = math.sqrt( (map_base.translation.x - detected_item_list[item].x) ** 2 + (map_base.translation.y - detected_item_list[item].y) ** 2 + (map_base.translation.z - detected_item_list[item].z) )
            total_cost = (robot_item_distance / shortest_robot_item_distance) * GRASP_COST
            print("Total cost = " + str(total_cost))

            if (total_cost < lowest_total_cost) and (aborted != True):
                lowest_total_cost = total_cost
                self.target_item = detected_item_list[item]
                print("target_item = ", self.target_item)
            print("next")
            r.sleep()
        
        print("Target item to be grasped is " + self.target_item.Class + str(self.target_item.id))


    def pick(self):

        self.upload_planning_scene()
        print("Go to pick-up item...")
        print(self.target_item)

        if self.target_item.z < 0.25:
            self.sweep_floor()
        elif self.target_item.z > 0.38 and self.target_item.x > 0.35:
            self.sweep_long_table_b()
        elif self.target_item.z > 0.58:
            self.sweep_tall_table() 

            
    def move_base_forward(self, direction):

        base_variable_values = base.get_current_joint_values()
        base_variable_values[0] += direction * ( SAFETY_PRE_GRASP_APPROACH_DIS - self.hand_palm_centroid_offset ) 
        base.set_joint_value_target(base_variable_values)
        for i in range(15):
            print("Moving base forward to item, trial " + str(i))
            if base.go(wait=True):
                break
        rospy.sleep(1.0)


    def sweep_floor(self):
        
        # Allign the robot's base with localized items
        print("Sweep floor...")
        rospy.sleep(1.0)
        arm.set_named_target("sweep_floor")
        arm.go(wait=True)
        rospy.sleep(2.0)
        move_base_pose_ik("map", self.target_item.x + self.hand_palm_base_link_offset , self.target_item.y - SAFETY_PRE_GRASP_APPROACH_DIS, 90)
        # Moving base forward to pick after setting the pre-defined arm's pose
        self.move_base_forward(1)
        move_hand(0.0)


    def sweep_long_table_b(self):

        # Allign the robot's base with localized items
        print("Sweep long table b...")
        rospy.sleep(1.0)
        arm.set_named_target("sweep_long_table_b")
        arm.go(wait=True)
        rospy.sleep(2.0)
        move_base_pose_ik("map", self.target_item.x + self.hand_palm_base_link_offset , self.target_item.y - SAFETY_PRE_GRASP_APPROACH_DIS, 90)
        # Raise arm up far above table to avoid collision in next execution 
        # Moving base forward to pick after setting the pre-defined arm's pose
        self.move_base_forward(1)
        # Close the gripper
        move_hand(0.0)
        # Retract
        move_base_vel(-0.1, 0, 0, SAFETY_PRE_GRASP_APPROACH_DIS, 0, 0)


    def sweep_tall_table(self):

        # Allign the robot's base with localized items
        print("Sweep tall table...")
        rospy.sleep(1.0)
        # Raise arm up far above table to avoid collision in next execution 
        arm.set_named_target("sweep_tall_table")
        arm.go(wait=True)
        rospy.sleep(2.0)
        move_base_pose_ik("map", self.target_item.x + self.hand_palm_base_link_offset , self.target_item.y - SAFETY_PRE_GRASP_APPROACH_DIS, 90)
        # Moving base forward to pick after setting the pre-defined arm's pose
        self.move_base_forward(1)
        move_hand(0.0)
        # Retract
        move_base_vel(-0.1, 0, 0, SAFETY_PRE_GRASP_APPROACH_DIS, 0, 0)


    def place(self):

        try:
            if self.target_item.Class in FOOD:
                navigate_to('Long_Table_A_Trays')
                yolo.clear_list(True)
                yolo.detect(True)
                rospy.sleep(6)
                tray = yolo.get_item_info('tray')
                self.picked_food += 1
                num_type = "even" if self.picked_food % 2 == 0 else "odd"
                if num_type == "odd":
                    move_whole_body_pose_ik("map", tray.x, tray.y, *TRAY_A)
                elif num_type == "even":
                    move_whole_body_pose_ik("map", tray.x, tray.y, *TRAY_B)
            elif self.target_item.Class in KITCHEN_ITEMS:
                navigate_to('Long_Table_A_Containers')
                yolo.clear_list(True)
                yolo.detect(True)
                rospy.sleep(6)
                b_container = yolo.get_item_info('b_container')
                move_whole_body_pose_ik("map", b_container.x, b_container.y, *CONTAINER_B)
            elif self.target_item.Class in ORIENTATION_ITEMS:
                navigate_to('Long_Table_A_Containers')
                yolo.clear_list(True)
                yolo.detect(True)
                rospy.sleep(6)
                a_container = yolo.get_item_info('a_container')
                move_whole_body_pose_ik("map", a_container.x, a_container.y, *CONTAINER_A)        
            elif self.target_item.Class in TOOL_ITEMS:
                navigate_to('Bins')
                yolo.clear_list(True)
                yolo.detect(True)
                rospy.sleep(6)
                black_bin = yolo.get_item_info('black_bin')
                move_whole_body_pose_ik("map", black_bin.x, black_bin.y, *BIN_B)
            elif self.target_item.Class in SHAPE_ITEMS:
                navigate_to('Bins')
                yolo.clear_list(True)
                yolo.detect(True)
                rospy.sleep(6)
                green_bin = yolo.get_item_info('green_bin')
                move_whole_body_pose_ik("map", green_bin.x, green_bin.y, *BIN_A)
            elif self.target_item.Class in TASK_ITEMS:
                navigate_to('Bins')
                yolo.clear_list(True)
                yolo.detect(True)
                rospy.sleep(6)
                green_bin = yolo.get_item_info('green_bin')
                move_whole_body_pose_ik("map", green_bin.x, green_bin.y, *BIN_A)
            else:
                navigate_to('Bins')
                yolo.clear_list(True)
                yolo.detect(True)
                rospy.sleep(6)
                black_bin = yolo.get_item_info('black_bin')
                move_whole_body_pose_ik("map", black_bin.x, black_bin.y, *BIN_B)
            
            rospy.sleep(1.0)
            move_hand(1.0)
            # collision_object.dettach("approx_grasped_item")
            # collision_object.scene.remove_world_object("approx_grasped_item")

        except EOFError:
            print("Error!!!")


def navigate_to(location):

    # To scan any obstacle that is unable to be detected by laser sensor
    move_head_tilt(-1)

    success = False

    if location == 'Drawers':
        move_base_goal(0.15, 0.5, -90)
        move_head_tilt(-0.5)
    if location == 'Search_Area_Front_Left':
        move_base_goal(0.08, 0.4, 90)
        move_head_tilt(-0.55)
    if location == 'Search_Area_Front_Right':
        move_base_goal(1.0, 0.5, 90)
        move_head_tilt(-0.72)
    if location == 'Long_Table_A_Containers':
        move_base_goal(1.1148, 0.075, -90)
        move_head_tilt(-0.9)
    if location == 'Long_Table_A_Trays':
        move_base_goal(1.7348, 0.075, -90)
        move_head_tilt(-0.9)
    if location == 'Bins':
        move_base_goal(2.7048, 0.075, -90)
        move_head_tilt(-0.9)
    if location == 'Tall_Table':
        move_base_goal(0.15, 1.20, 90)
        move_head_tilt(-0.5)
    if location == 'Long_Table_B':
        move_base_goal(1.0, 0.75, 90)
        move_head_tilt(-0.5)
    if location == 'Shelf':
        move_base_goal(2.28, 3.84, 95)
        move_head_tilt(-0.3)
    if location == 'Person':
        move_base_goal(1.12, 3.46, 180)
        move_head_tilt(0.3)

    robot.upload_planning_scene()
    rospy.sleep(1)

    return success


def robot_reset():
    move_hand(1.0)
    move_arm_init()
    clear_octomap()


if __name__ == "__main__":
    
    step = -1
    rospy.init_node("main_task")
    r = rospy.Rate(10)
    collision_object = collision_object()
    #task1
    rgbd = RGBD()
    robot = ARM_t1()
    yolo = YOLO_t1()
    # task 2
    t2 = Task_2()
    print("Starting...")
    forteen_min = rospy.Duration(10*60) #change state earlier
    five_min = rospy.Duration(10*60)
    start = rospy.Time.now()
    while not rospy.is_shutdown():
        try: 
            if step == -1:
                start = rospy.Time.now()
                step += 1
            elif step == 0:
                if rospy.Time.now() >= (start + rospy.Duration(1)):
                    start = rospy.Time.now()
                    step += 1
            elif  step == 1:
                if rospy.Time.now() >= (start + forteen_min):
                    step += 1
                else:
                    #Task1 starts here
                    try:
                        robot.search()
                    except:
                        pass
            elif  step == 2:
                print("reset robot")
                robot_reset()
                start = rospy.Time.now()
                step += 1
            elif  step == 3:
                if rospy.Time.now() >= (start + five_min):
                    step += 1
                else:
                    #Task2 starts here
                    t2.task_2()
            elif  step == 4:
                print("reset robot")
                robot_reset()
                start = rospy.Time.now()
                step += 1
 
        except:
            pass
        r.sleep()   
