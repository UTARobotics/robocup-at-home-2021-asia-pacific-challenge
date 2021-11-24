#!/usr/bin/env python
import rospy
from roslib import message
import sensor_msgs.point_cloud2 as point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

data_list = []

global rospy
rospy.init_node('listen', anonymous=True)

#listener
def listen():
    global pub, data_list,sub
    pub = rospy.Publisher('hsrb/head_depth_camera/depth_registered/rectified_points', PointCloud2, queue_size=10)
    sub = rospy.Subscriber('hsrb/head_rgbd_sensor/depth_registered/points', PointCloud2, callback)

def callback(data):
    global pub, data_list, sub
    data_list = []
    data_list.append(data)


if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            listen()
            for data in data_list:
                ground = False
                for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
                    if p[2] < 0.5:
                        print(" x : %f  y: %f  z: %f" %(p[0],p[1],p[2]))
                        ground = True
                        break
                
                if ground == False:
                    pub.publish(data)
                        

            # data_list = data_list(100:)

    except rospy.ROSInterruptException:
        pass
