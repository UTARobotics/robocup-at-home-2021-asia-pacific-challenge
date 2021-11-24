
#path to the YOLO model
MODEL_PATH = '/workspace/src/deepsparse_yolo/model/model.onnx'

#YOLO detect input size
DIM = (640,640)

#TF frame 
SENSOR_FRAME = 'head_rgbd_sensor_depth_frame'
#Topic names  
CAMERA_RGB_TOPIC = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
CAMERA_DEPTH_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw'
CAMERA_DEPTH_INFO_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/camera_info'
BBOXES_TOPIC = '/object_bbox'
YOLO_DEBUG_TOPIC = '/yolo/debug'
OBJECT_TRANSFER_TOPIC = 'yolo/object_trans'
YOLO_DETECT_SERVICE = '/yolo/set_detect_state'

OBJECTS_TOPIC = '/objects/list'
OBJECTS_REMOVE_SERVICE = '/objects/remove_object'
OBJECTS_CLEAR_SERVICE = '/objects/clear_list'

_ROBOCUP_CLASSES = ['master_chef_can',
        'cracker_box',
        'sugar_box',
        'tomato_soup_can',
        'mustard_bottle',
        'tuna_fish_can',
        'pudding_box',
        'gelatin_box',
        'potted_meat_can',
        'banana',
        'strawberry',
        'apple',
        'lemon',
        'peach',
        'pear',
        'orange',
        'plum',
        'pitcher_base',
        'bleach_cleanser',
        'windex_bottle',
        'bowl',
        'mug',
        'sponge',
        'skillet',
        'skillet_lid',
        'plate',
        'fork',
        'spoon',
        'knife',
        'spatula',
        'power_drill',
        'wood_block',
        'scissors',
        'padlock',
        'large_marker',
        'adjustable_wrench',
        'phillips_screwdriver',
        'flat_screwdriver',
        'hammer',
        'clamp',
        'mini_soccer_ball',
        'softball',
        'baseball',
        'tennis_ball',
        'racquetball',
        'golf_ball',
        'chain',
        'foam_brick',
        'dice',
        'marbles',
        'cup',
        'colored_wood_blocks',
        'nine_hole_peg_test',
        'a_toy_airplane',
        'b_toy_airplane',
        'c_toy_airplane',
        'lego_duplo',
        'rubiks_cube',
        'tray',
        'black_bin',
        'green_bin',
        'a_container',
        'b_container']