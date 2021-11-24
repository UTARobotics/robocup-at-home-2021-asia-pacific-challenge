#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import os
import numpy as np
import param

from deepsparse_yolo_msgs.msg import Object, Objects
from deepsparse_yolo_msgs.srv import RemoveObject, RemoveObjectResponse, ClearObjects, ClearObjectsResponse 

#Using yolo classes now 
import tf
import tf2_ros
import tf_conversions
import geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist

class Object_Database(object):
    def __init__(self):
        rospy.loginfo("Object Database initiating ... ")
        # Params
        self.state = False
        self.objects = []
        #TF broadcaster and Listener
        self._br = tf2_ros.TransformBroadcaster()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)
        self.buffer = []
        # Publishers
        self.pub = rospy.Publisher(param.OBJECTS_TOPIC, Objects, queue_size=1) 
        # Subscribers
        rospy.Subscriber(param.OBJECT_TRANSFER_TOPIC,Objects,self.object_callback)
        #Services
        self.srv = rospy.Service(param.OBJECTS_REMOVE_SERVICE,RemoveObject,self.srv_callback)
        self.rmsrv = rospy.Service(param.OBJECTS_CLEAR_SERVICE,ClearObjects,self.rmsrv_callback)

    #Get Object Data
    def object_callback(self, objs):
        # rospy.loginfo("Recorded ...")
        self.objects = objs.objects

    #Update Object in base
    def update(self):
        for obj in self.buffer:         
             self.objects.append(obj)
        self.buffer = []
    
    #Publish TF of objects
    def publish_TF(self):
        if len(self.objects) > 0:
            for obj in self.objects:
                frame = obj.Class + str(obj.id) 
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "map"
                t.child_frame_id = frame
                t.transform.translation.x = obj.x
                t.transform.translation.y = obj.y 
                t.transform.translation.z = obj.z
                q = tf_conversions.transformations.quaternion_from_euler(0,0,0)
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                self._br.sendTransform([t])

    #Publish data of objects
    def publish_data(self):
        msg = Objects()
        msg.objects = self.objects
        self.pub.publish(msg)
       

    #Service callback function (remove one object)
    def srv_callback(self, req):
        for obj in self.objects:
            if obj.Class == req.Class and obj.id == req.id:
                self.objects.remove(obj)
                return RemoveObjectResponse(RemoveObjectResponse.OK)
        return RemoveObjectResponse(RemoveObjectResponse.ERROR)
    
    #Service callback function (remove all object)
    def rmsrv_callback(self, req):
        if req.state == req.CLEAR:
            rospy.loginfo('Clear object list ')
            self.objects = []
        return ClearObjectsResponse(ClearObjectsResponse.OK)

    #Depth start function 
    def start(self):
        rospy.loginfo("Object Database started")
        #rospy.spin()
        while not rospy.is_shutdown():
            self.publish_TF()
            self.publish_data()
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("Object Database", anonymous=True)
    my_node = Object_Database()
    my_node.start()