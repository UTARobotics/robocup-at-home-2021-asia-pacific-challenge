import sys
sys.path.insert(0, '/workspace/notebooks/')
import rospy
from utils import *
from deepsparse_yolo_msgs.msg import Object, Objects
from deepsparse_yolo_msgs.srv import SetDetectState, SetDetectStateResponse, RemoveObject, RemoveObjectResponse

object_list_topic = 'objects/list'
yolo_detect_service = '/yolo/set_detect_state'
object_remove_service = '/objects/remove_object'

obj_class = None
obj_id = None

got_objs = False

#msg = True start detect 
def yolo_detect(msg):
    print "(Triggered detection)"
    rospy.wait_for_service(yolo_detect_service)
    change_state = rospy.ServiceProxy(yolo_detect_service, SetDetectState)
    response = change_state(msg)
    

#msg = True start detect 
def object_remove(cls, id):
    rospy.wait_for_service(object_remove_service)
    change_state = rospy.ServiceProxy(object_remove_service, RemoveObject)
    response = change_state(cls, id)
    print("Remove {0} with id {1} from object database.".format(cls,id))

#Get object class and id 
def object_cb(msg):
    global got_objs
    # obj_class = msg.objects[0].Class
    # obj_id = msg.objects[0].id
    if len(msg.objects) > 0:
        got_objs = True


if __name__ == '__main__':
    global got_objs
    rospy.init_node('yolotest')
    loop_rate = rospy.Rate(10)
    #Subscribe to object list
    rospy.Subscriber(object_list_topic,Objects,object_cb)
    #Move to shelf
    # move_head_tilt(0)
    # move_base_goal(2.7, 2, 90)
    move_head_tilt(-1)
    move_base_goal(2, 4, 90)
    move_head_tilt(-0.5)
    #wait for robot to finish move to pose
    rospy.sleep(2)
    #Check if gotten object
    yolo_detect(True)
    while not rospy.is_shutdown():
        if got_objs: 
            print('got objects')
            got_objs = False
        loop_rate.sleep()
    #rospy.sleep(3)
    #Move to table
    move_head_tilt(-1)
    # move_base_goal(1, 0.5, 90)
    # move_head_tilt(-0.7)
    # #wait for robot to finish move to pose
    # rospy.sleep(2)
    # #Detect and wait 3 sec for detection
    # yolo_detect(True)
    # rospy.sleep(3)
    # if (obj_class is not None and obj_id is not None):
    #     object_remove(obj_class,obj_id)


    
