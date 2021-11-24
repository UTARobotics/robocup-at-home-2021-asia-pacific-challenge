#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import sys
sys.path.insert(0, '/workspace/src/ultralytics-yolo')
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
import cv2
import os
import numpy as np
import param

from deepsparse_yolo_msgs.msg import BBox, BBoxes, Object, Objects
#from deepsparse_utils import _YOLO_CLASSES
import tf
import tf2_ros
import tf_conversions
import geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist

class Depth(object):
    def __init__(self):
        rospy.loginfo("Depth initiating ... ")
        # Params
        self._bridge = CvBridge()
        self.depth_image = None
        self.width = None
        self.height = None
        self.depth_header = None
        self.record = False
        self.recorded = False
        self.obj_buff = []
        self.objs = []
        #TF broadcaster and Listener
        self.listener = tf.TransformListener()
        self._br = tf2_ros.TransformBroadcaster()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)
        #get camera info 
        camera_info = rospy.wait_for_message(param.CAMERA_DEPTH_INFO_TOPIC, CameraInfo)
        self._invK = np.linalg.inv(np.array(camera_info.K).reshape(3, 3))
        # Subscribers
        rospy.Subscriber(param.CAMERA_DEPTH_TOPIC,Image,self.depth_callback)
        rospy.Subscriber(param.BBOXES_TOPIC,BBoxes,self.object_callback)
        # Publishers
        self.pub = rospy.Publisher(param.OBJECT_TRANSFER_TOPIC, Objects, queue_size=1)
        
    #callback for depth function 
    def depth_callback(self, depth):
        try:
            depth_image = self._bridge.imgmsg_to_cv2(depth, 'passthrough')
            depth_image = depth_image.astype('float32') / 1000.0        #connvert depth image coding 
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        #get depth array parameters
        depth_array = np.array(depth_image, dtype=np.float32)
        self.width = depth.width
        self.height = depth.height
        self.depth_image = depth_array
        self.depth_header = depth.header

    #Convert the referance frame of a tf
    def convert_TF(self,parent_frame,child_frame):
        self.listener.waitForTransform(parent_frame, child_frame, rospy.Time(), rospy.Duration(4.0))
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform(parent_frame,child_frame,rospy.Time(0))
                (roll, pitch, yaw) = euler_from_quaternion(rot)
                break
            except (tf.LookupException,tf.ConnectivityException, tf.ExtrapolationException) as e :
                rospy.logerr(e)
                continue
        return {'x':trans[0],'y':trans[1],'z':trans[2],'roll':0,'pitch':0,'yaw':yaw}  

    # Callback function object bboxes     
    def object_callback(self, bboxes):
        self.recorded = False
        for bb in bboxes.bounding_boxes:

            depth_array = np.copy(self.depth_image)
            height = self.height
            width = self.width
            xmin = bb.xmin
            xmax = bb.xmax
            ymin = bb.ymin
            ymax = bb.ymax
            cat = bb.Class
            id = bb.id
    
            #crop and remove depth out of range
            depth_array[0:xmin,:] = 200
            depth_array[xmax:height,:] = 200
            depth_array[:,0:ymin] = 200
            depth_array[:,ymax:width] = 200
            
            #Get target coordinate of object
            tar_x = xmin+(xmax-xmin)/2
            tar_y = ymin+(ymax-ymin)/2
            d_test = self.depth_image[tar_x][tar_y]
            #Remove out of range element
            result = depth_array[depth_array <= d_test+0.3]
            result = result[result >= d_test-0.3]
            #Get median of depth image 
            d = np.nanmedian(result)
            #Get object TF and publish
            self.obj_buff.append({'cat':cat,'id':id,'y':tar_y,'x':tar_x,'d':d})
            #self.get_object(cat,id ,tar_y,tar_x,d,self.depth_header)
            # pose = self.convert_TF('map',cat)
            # obj = {'x':pose.x,'y':pose.y,'z':pose.z,'name':cat}
            # print(obj.name)
            # param.OBJECTS.append(obj)
        
    # Get object TF and publish TF
    def publish_object_tf(self, obj, header):
        frame = obj['cat'] + str(obj['id']) 
        image_point = np.array([obj['y'], obj['x'], 1])
        object_point = np.dot(self._invK, image_point) * obj['d']
        t = geometry_msgs.msg.TransformStamped()
        t.header = header
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = frame
        t.transform.translation.x = object_point[0] 
        t.transform.translation.y = object_point[1] 
        t.transform.translation.z = object_point[2]
        q = tf_conversions.transformations.quaternion_from_euler(0,0,0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self._br.sendTransform([t])

    # Convert object TF to map frame
    def record_object_tf(self):
        if len(self.obj_buff) > 0:
            for obj in self.obj_buff: 
                frame = obj['cat'] + str(obj['id']) 
                trans = []
                trans = self.convert_TF('map', frame)
                #update database here
                msg = Object()
                msg.Class = obj['cat']
                msg.id = obj['id']
                msg.x = trans['x']
                msg.y = trans['y']
                msg.z = trans['z']
                self.objs.append(msg)
                self.obj_buff.remove(obj) 
        if len(self.obj_buff) == 0:
            self.recorded = True
            self.record = False
            
    #Depth start function 
    def start(self):
        rospy.loginfo("Depth started")
        #rospy.spin()
        while not rospy.is_shutdown():
            if len(self.obj_buff) > 0 and not self.recorded:
                # rospy.loginfo("Publishing ...")
                for obj in self.obj_buff:
                   self.publish_object_tf(obj,self.depth_header)
                self.record = True
                # self.record_object_tf()
            if len(self.obj_buff) > 0 and self.record:
                # rospy.loginfo("Recording ...")
                self.record_object_tf()
            if self.recorded and not self.record:        
                msg = Objects()
                msg.objects = self.objs
                self.pub.publish(msg)
                self.objs = []
                self.recorded = False              
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("depth", anonymous=True)
    my_node = Depth()
    my_node.start()