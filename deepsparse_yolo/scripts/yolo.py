#!/usr/bin/env python3.6
# -*- encoding: utf-8 -*-
import sys
sys.path.insert(0, '/workspace/src/ultralytics-yolo')
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Bool
import cv2
import os
import random
import numpy as np
#Deepsparse import
from deepsparse import compile_model
from deepsparse_utils import YoloPostprocessor, postprocess_nms, annotate_image
#Using YOLO classes now 
from param import _ROBOCUP_CLASSES

#self import 
import param
from deepsparse_yolo_msgs.msg import BBox, BBoxes, Object, Objects
from deepsparse_yolo_msgs.srv import SetDetectState, SetDetectStateResponse



class Yolo(object):
    def __init__(self):
        rospy.loginfo("YOLO initiating ... ")
        # Params
        self.state = False
        self.debug = True
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)
        # load model
        self.model = compile_model(param.MODEL_PATH, batch_size=1)
        self.outputs = self.model([np.uint8(np.random.randn(1, 3, 640, 640))])
        # Publishers
        self.pub = rospy.Publisher(param.YOLO_DEBUG_TOPIC, Image, queue_size=1) #(DEBUG)
        self.bbpub = rospy.Publisher(param.BBOXES_TOPIC, BBoxes, queue_size=1)
        # Subscribers
        rospy.Subscriber(param.CAMERA_RGB_TOPIC,Image,self.callback)
        rospy.Subscriber(param.OBJECTS_TOPIC,Objects,self.objs_callback)
        #Services
        self.srv = rospy.Service(param.YOLO_DETECT_SERVICE,SetDetectState,self.srv_callback)
    
    #Sensor image callback function
    def callback(self, msg):
        if self.state:
            #rospy.loginfo('Image received...')
            dat = np.frombuffer(msg.data,dtype='uint8')
            shape = (msg.height,msg.width,3)
            img = dat.reshape(shape)
            # resize image
            img_resized = cv2.resize(img, param.DIM, interpolation = cv2.INTER_AREA)
            img_transposed = img_resized[:, :, ::-1].transpose(2, 0, 1)
            sample_batch = [np.ascontiguousarray([img_transposed])]
            #predict
            self.outputs = self.model(sample_batch)
            self.outputs = YoloPostprocessor().pre_nms_postprocess(self.outputs)
            self.outputs = postprocess_nms(self.outputs)
            self.get_data(img, 0.35, param.DIM)

            if self.debug:
                #Annotate image (DEBUG)
                annotated_image = annotate_image(img, self.outputs[0], 0.35,param.DIM)     
                #prepare for publish image 
                rep_img = np.ravel(annotated_image)
                pub_img = Image()
                pub_img.data = rep_img.tolist()
                pub_img.height = msg.height
                pub_img.width =msg.width
                pub_img.encoding = msg.encoding
                pub_img.step = msg.step
                self.pub.publish(pub_img)

            self.state = False

    #Check if object found
    def objs_callback(self, msg):
        if len(msg.objects) > 0 :
            self.got_objs = True
        
    #YOLO start function
    def start(self):
        rospy.loginfo("YOLO started")
        #rospy.spin()
        while not rospy.is_shutdown():
            self.loop_rate.sleep()
            
    #Get data function
    def get_data(self, img, score_threshold, model_input_size):

        outputs = self.outputs[0]

        boxes = outputs[:, 0:4]
        scores = outputs[:, 4]
        labels = outputs[:, 5].astype(int)

        scale_y = img.shape[0] / (1.0 * model_input_size[0]) if model_input_size else 1.0
        scale_x = img.shape[1] / (1.0 * model_input_size[1]) if model_input_size else 1.0
        data = []
        
        for idx in range(boxes.shape[0]):
                label = labels[idx].item()
                if scores[idx] > score_threshold:

                    # bounding box points
                    left = boxes[idx][0] * scale_x
                    top = boxes[idx][1] * scale_y
                    right = boxes[idx][2] * scale_x
                    bottom = boxes[idx][3] * scale_y

                    msg = BBox()
                    msg.probability = scores[idx]
                    msg.xmin = int(top)
                    msg.xmax = int(bottom)
                    msg.ymin = int(left)
                    msg.ymax = int(right)
                    msg.id   = int(random.randint(0,100))
                    msg.Class = _ROBOCUP_CLASSES[label]
                    data.append(msg)

        bboxes = BBoxes()
        bboxes.bounding_boxes = data
        self.bbpub.publish(bboxes)
    
    #Service callback function
    def srv_callback(self, req):
        if req.state == req.STOPPED:
            rospy.loginfo('Change state to STOPPED')
            self.state = False
        if not self.state and req.state == req.DETECT:
            rospy.loginfo('Change state to DETECT')
            self.state = True
        return SetDetectStateResponse(SetDetectStateResponse.OK)

if __name__ == '__main__':
    rospy.init_node("yolo", anonymous=True)
    my_node = Yolo()
    my_node.start()