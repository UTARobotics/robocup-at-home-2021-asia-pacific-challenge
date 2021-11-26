#!/usr/bin/env python

# List of classified items

FOOD = ["master_chef_can", "cracker_box", "sugar_box", "tomato_soup_can", "mustard_bottle", 
"tuna_fish_can", "pudding_box", "gelatin_box", "potted_meat_can", "banana", "strawberry", 
"apple", "lemon", "peach", "pear", "orange", "plum"]

KITCHEN_ITEMS = ["pitcher_base", "bleach_cleanser", "windex_bottle", "bowl", "mug", 
"sponge", "skillet", "skillet_lid", "plate"]

ORIENTATION_ITEMS = ["fork", "spoon", "knife", "spatula", "large_marker"]

TOOL_ITEMS = ["power_drill", "wood_block", "scissors", "padlock", 
"adjustable_wrench", "phillips_screwdriver", "flat_screwdriver", "hammer", "clamp"]

SHAPE_ITEMS = [ "mini_soccer_ball", "softball", "baseball", "tennis_ball", "racquetball", 
"golf_ball", "chain", "foam_brick", "dice", "marbles", "cup"]

TASK_ITEMS = ["colored_wood_blocks", "nine_hole_peg_test", "a_toy_airplane", 
"b_toy_airplane", "c_toy_airplane", "lego_duplo", "rubiks_cube"]

UNKNOWN_ITEMS = []


LOW_GRASP_DIFFICUTY = ["potted_meat_can", "tomato_soup_can", "strawberry", "apple", "lemon", 
"peach", "pear", "orange", "plum", "mug", "padlock", "clamp", "softball", "baseball", 
"tennis_ball", "racquetball", "golf_ball", "cup", "foam_brick", "dice", "rubiks_cube", 
"colored_wood_blocks", "lego_duplo", "c_toy_airplane"] # 'cost': 0.33

MEDIUM_GRASP_DIFFICUTY = ["pudding_box", "gelatin_box", "master_chef_can", "tuna_fish_can", "banana", "sponge", "mini_soccer_ball"] # 'cost': 0.66

HIGH_GRASP_DIFFICUTY = ["cracker_box", "sugar_box", "mustard_bottle", "windex_bottle", "bleach_cleanser", "pitcher_base", "plate", "bowl", "fork", "spoon", "knife", "spatula", "skillet", "skillet_lid", "scissors", "large_marker", "hammer", "phillips_screwdriver", "flat_screwdriver", "adjustable_wrench", "wood_block", "power_drill", "marbles", "chain", "nine_hole_peg_test", "a_toy_airplane", "b_toy_airplane"] # 'cost': 1.00


# Dimension of collision objects

DRAWER_LENGTH = 0.20
DRAWER_WIDTH = 0.35
DRAWER_HEIGHT = 0.20

GROUND_LENGTH = 100.00
GROUND_WIDTH = 100.00
GROUND_HEIGHT = 0.001

FRAME_L_LENGTH = FRAME_R_LENGTH = 1
FRAME_L_WIDTH = FRAME_R_WIDTH = 0.01
FRAME_L_HEIGHT = FRAME_R_HEIGHT = 1
FRAME_B_LENGTH = 0.01
FRAME_B_WIDTH = 1.6
FRAME_B_HEIGHT = 1

CENTROID_RADIUS = 0.02


# Task 1: PLACE poses at "Deposit Area", (x, y, z, roll, pitch, yaw)

DRAWER_BOTTOM_KNOB = [0.155, -0.265, 0.28, 90, 0, 0]
DRAWER_TOP_KNOB = [0.155, -0.267, 0.54, 90, 0, 0]
DRAWER_LEFT_KNOB = [0.52, -0.269, 0.275, 90, 0, 0]

DRAWER_TOP = [0.14, -0.20, 0.70, 180, 0, 90]
DRAWER_LEFT = [0.52, -0.20, 0.43, 180, 0, 90]

# (z, roll, pitch, yaw)
CONTAINER_A = [0.70, 90, 0, 90]
CONTAINER_B = [0.70, 90, 0, 90]

PLACE_POSE = [0.70, 180, 0, 90]

TRAY_A = [0.70, 180, 0, 90]
TRAY_B = [0.70, 180, 0, 90]

BIN_A = [0.55, 180, 0, 90]
BIN_B = [0.55, 180, 0, 90]


EEF_POINT_DOWN_FLOOR_Z_MIN = 0.08
EEF_POINT_DOWN_LONG_TABLE_B_Z_MIN = (0.08 + 0.40)
EEF_POINT_DOWN_TALL_TABLE_Z_MIN = (0.08 + 0.60)

EEF_POINT_DOWN_Z_OFFSET = 0.055


SAFETY_PRE_GRASP_APPROACH_DIS = 0.2
SAFETY_POST_GRASP_RETRACT_DIS = 0.2
