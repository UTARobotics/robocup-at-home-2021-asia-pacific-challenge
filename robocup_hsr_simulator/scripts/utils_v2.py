# -*- coding: utf-8 -*-

import actionlib
import cv2
import glob
import math
import sys
import moveit_commander
import moveit_msgs.msg
import numpy as np
import os
import rospy
import ros_numpy
import subprocess
import tf
import tf2_ros
import time
import geometry_msgs.msg
import copy
import enum
import time
import atexit

from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint, PositionConstraint
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
from IPython.display import Image
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header
from std_msgs.msg import String
from std_srvs.srv import Empty

moveit_commander.roscpp_initialize(sys.argv)

def screen_shot():
    u"""スクリーンショットを取得し、表示する関数"""
    cmd = "import -window root /tmp/screen.png"
    subprocess.call(cmd.split())

    with open('/tmp/screen.png', 'rb') as file:
        display(Image(data=file.read()))


def screen_cast(sec):
    u"""スクリーンキャストを取得し、表示する関数"""
    cmd = "byzanz-record -d " + str(sec) + " /tmp/screencast.gif"
    subprocess.call(cmd.split())

    with open('/tmp/screencast.gif', 'rb') as file:
        display(Image(data=file.read()))

# 速度指令のパブリッシャーを作成
base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)


def move_base_vel(vx, vy, vw, dx, dy, dw):

    # only valid to move along or rotate about 1 axis, enter velocities according to joint limits
    u"""台車を速度制御する関数
    引数:
        vx (float): 直進方向の速度指令値 [m/s]（前進が正、後進が負）
        vy (float): 横方向の速度指令値 [m/s]（左が正、右が負）
        vw (float): 回転方向の速度指令値 [deg/s]（左回転が正、右回転が負）
    """

    # 速度指令値をセットします
    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.angular.z = vw / 180.0 * math.pi  # 「度」から「ラジアン」に変換します
    
    if vy == 0 and vw == 0:
    	duration = dx / vx
    elif vx == 0 and vw == 0:
    	duration = dy / vy
    elif vx == 0 and vy == 0:
    	duration = dw / vw
    
    start_time = get_current_time_sec()
    end_time = start_time + duration
    
    while( get_current_time_sec() < end_time ):
    	base_vel_pub.publish(twist)  # 速度指令をパブリッシュします
    	
    	
def get_current_time_sec():
    u"""現在時刻を返す関数

    戻り値:
        現在時刻 [s]

    """
    return rospy.Time.now().to_sec()


class Laser():
    u"""レーザ情報を扱うクラス"""

    def __init__(self):
        # レーザースキャンのサブスクライバのコールバックに_laser_cbメソッドを登録
        self._laser_sub = rospy.Subscriber('/hsrb/base_scan',
                                           LaserScan, self._laser_cb)
        self._scan_data = None

    def _laser_cb(self, msg):
        # レーザスキャンのコールバック関数
        self._scan_data = msg

    def get_data(self):
        u"""レーザの値を取得する関数"""
        return self._scan_data


def quaternion_from_euler(roll, pitch, yaw):
    u"""オイラー角からクオータニオンに変換する関数

    引数：
        roll (float): 入力roll値 [deg]
        pitch (float): 入力pitch値 [deg]
        yaw (float): 入力yaw値 [deg]

    返り値:
        ロール、ピッチ、ヨーの順番で回転した場合のクオータニオン

    """

    # ロール、ピッチ、ヨーの順番で回転
    q = tf.transformations.quaternion_from_euler(roll / 180.0 * math.pi,
                                                 pitch / 180.0 * math.pi,
                                                 yaw / 180.0 * math.pi, 'rxyz')
    return Quaternion(q[0], q[1], q[2], q[3])


# 自律移動のゴールを送信するクライアントを作成
navclient = actionlib.SimpleActionClient('/move_base', MoveBaseAction)


def move_base_goal(x, y, theta):
    u"""台車の自律移動のゴールを指定する関数

    引数：
        x (float): 目標のx値 [m]
        y (float): 目標のy値 [m]
        theta (float): 目標の回転角度 [deg]

    返り値:
        ゴールに到達したらTrue, そうでなければFalse

    """

    goal = MoveBaseGoal()

    # "map"座標を基準座標に指定
    goal.target_pose.header.frame_id = "map"

    # ゴールのx,y座標をセットします
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # 角度はクオータニオンという形式で与えます。そのため、オイラー角からクオータニオンに変換します
    goal.target_pose.pose.orientation = quaternion_from_euler(0, 0, theta)

    # ゴールを送信
    navclient.send_goal(goal)
    navclient.wait_for_result()
    state = navclient.get_state()
    # 成功すると、3が返ってくる
    # http://docs.ros.org/fuerte/api/actionlib_msgs/html/msg/GoalStatus.html
    return True if state == 3 else False


def get_relative_coordinate(parent, child):
    u"""相対座標を取得する関数

    引数：
        parent (str): 親の座標系
        child (str): 子の座標系

    """

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    trans = TransformStamped()
    while not rospy.is_shutdown():
        try:
            # 4秒待機して各tfが存在すれば相対関係をセット
            trans = tfBuffer.lookup_transform(parent, child,
                                              rospy.Time().now(),
                                              rospy.Duration(5.0))
            break
        except (tf2_ros.ExtrapolationException):
            pass

    return trans.transform


# moveitでの制御対象として全身制御を指定
arm = moveit_commander.MoveGroupCommander("arm")
base = moveit_commander.MoveGroupCommander("base")
whole_body = moveit_commander.MoveGroupCommander("whole_body")
arm.allow_replanning(True)


def move_wholebody_position_ik(x, y, z):

    # p = PoseStamped()

    # "map"座標を基準座標に指定
    # p.header.frame_id = "ref_frame"

    # エンドエフェクタの目標位置姿勢のx,y,z座標をセットします
    # p.pose.position.x = x
    # p.pose.position.y = y
    # p.pose.position.z = z

    target_position = [x, y, z]

    # オイラー角をクオータニオンに変換します
    # p.pose.orientation = quaternion_from_euler(roll, pitch, yaw)
    # p.pose.orientation.x = qx
    # p.pose.orientation.y = qy
    # p.pose.orientation.z = qz
    # p.pose.orientation.w = qw

    # eef_link = whole_body.get_end_effector_link()

    # 目標位置姿勢をセット
    whole_body.set_position_target(target_position)

    # plan1 = whole_body.plan()
    success = whole_body.go(wait=True)

    return success


def move_arm_position_ik(x, y, z):

    # p = PoseStamped()

    # "map"座標を基準座標に指定
    # p.header.frame_id = "ref_frame"

    # エンドエフェクタの目標位置姿勢のx,y,z座標をセットします
    # p.pose.position.x = x
    # p.pose.position.y = y
    # p.pose.position.z = z

    target_position = [x, y, z]

    # オイラー角をクオータニオンに変換します
    # p.pose.orientation = quaternion_from_euler(roll, pitch, yaw)
    # p.pose.orientation.x = qx
    # p.pose.orientation.y = qy
    # p.pose.orientation.z = qz
    # p.pose.orientation.w = qw

    eef_link = arm.get_end_effector_link()

    # 目標位置姿勢をセット
    arm.set_position_target(target_position)

    # plan1 = whole_body.plan()
    success = arm.go(wait=True)

    return success


def move_arm_pose_ik(ref_frame, x, y, z, roll, pitch, yaw):

    p = PoseStamped()

    p.header.frame_id = ref_frame

    target_position = [x, y, z]

    # エンドエフェクタの目標位置姿勢のx,y,z座標をセットします
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z

    # オイラー角をクオータニオンに変換します
    p.pose.orientation = quaternion_from_euler(roll, pitch, yaw)


    eef_link = arm.get_end_effector_link()

    # 目標位置姿勢をセット
    arm.set_pose_target(p)

    # plan1 = whole_body.plan()
    success = arm.go(wait=True)

    return success


def move_whole_body_pose_ik(ref_frame, x, y, z, roll, pitch, yaw):

    p = geometry_msgs.msg.Pose()

    target_position = [x, y, z]

    p = PoseStamped()

    # "map"座標を基準座標に指定
    p.header.frame_id = ref_frame

    # エンドエフェクタの目標位置姿勢のx,y,z座標をセットします
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z


    # オイラー角をクオータニオンに変換します
    p.pose.orientation = quaternion_from_euler(roll, pitch, yaw)
    # p.orientation.w = 0.0008446 
    # p.orientation.x = 0.7071065
    # p.orientation.y = 0.0002815
    # p.orientation.z = 0.7071065


    eef_link = whole_body.get_end_effector_link()

    # 目標位置姿勢をセット
    whole_body.set_pose_target(p)

    # plan1 = whole_body.plan()
    
    for i in range(5):
	    print('iteration %d'%i)
	    success = whole_body.go(wait=True)
	    if success:
		break

    return success

	
def move_base_pose_ik(ref_frame, x, y, yaw):

    p = geometry_msgs.msg.Pose()

    # target_position = [x, y, z]

    p = PoseStamped()

    # "map"座標を基準座標に指定
    p.header.frame_id = ref_frame

    # エンドエフェクタの目標位置姿勢のx,y,z座標をセットします
    # translation of "hand_palm_link" along x-axis of "base_link" 
    translation =  0.494286
    p.pose.position.x = x - ( translation * math.cos(yaw/180.0 * math.pi) )
    p.pose.position.y = y - ( translation * math.sin(yaw/180.0 * math.pi) )
    # p.pose.position.z = z

    roll = 0
    pitch = 0

    # オイラー角をクオータニオンに変換します
    p.pose.orientation = quaternion_from_euler(roll, pitch, yaw)
    # p.orientation.w = 0.0008446 
    # p.orientation.x = 0.7071065
    # p.orientation.y = 0.0002815
    # p.orientation.z = 0.7071065


    # eef_link = whole_body.get_end_effector_link()

    # 目標位置姿勢をセット
    base.set_pose_target(p)

    # plan1 = whole_body.plan()
    for i in range(15):
        print('Base IK iteration %d'%i)
        success = base.go(wait=True)
        if success:
             break
    
    # if success:
    # 	move_base_vel(0, -0.05, 0, 0, 0.0780025485842, 0)

    return success
    
    
def move_arm_neutral():
    u"""ロボットをニュートラルの姿勢に移動

    返り値:
        正しく動作すればTrue, そうでなければFalse

    """

    arm.set_named_target('neutral')
    return arm.go()


def move_arm_init():
    u"""ロボットを初期姿勢に移動

    返り値:
        正しく動作すればTrue, そうでなければFalse

    """

    arm.set_named_target('go')
    return arm.go()


# moveitでの制御対象としてハンドを指定
gripper = moveit_commander.MoveGroupCommander("gripper")


def move_hand(v):
    u"""ハンドを制御

    引数:
        v (float): ハンドの開き具合 (0：閉じる、1:開く)

    返り値:
        正しく動作すればTrue, そうでなければFalse

    """

    gripper.set_joint_value_target("hand_motor_joint", v)
    # gripper.set_max_acceleration_scaling_factor(0.5)
    
    success = gripper.go()
    rospy.sleep(6)

    return success


# moveitでの制御対象として頭部を指定
head = moveit_commander.MoveGroupCommander("head")


def move_head_tilt(v):
    u"""ハンドを制御

    引数:
        v (float): 頭部の入力チルト角度 (マイナス:下向き、プラス:上向き)

    返り値:
        正しく動作すればTrue, そうでなければFalse

    """

    head.set_joint_value_target("head_tilt_joint", v)
    return head.go()


def move_head_pan(v):

    """
        rotate head to left or right
    """

    head.set_joint_value_target("head_pan_joint", v)
    return head.go()


#  linear interpolation (LIN) - relative along axis
def move_end_effector_by_line(xyz_axes, displacement):

    waypoints = []

    wpose = whole_body.get_current_pose().pose

    wpose.position.x += xyz_axes[0] * displacement
    wpose.position.y += xyz_axes[1] * displacement
    wpose.position.z += xyz_axes[2] * displacement

    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = whole_body.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold

    whole_body.execute(plan, wait=True)
    

def eef_map_orientation_constraint(enable, roll, pitch, yaw):

    if enable == True:
        whole_body.goal_c = Constraints()
        header = Header()
        header.frame_id = "map"
        orientation_c = OrientationConstraint()
        orientation_c.header = header
        orientation_c.link_name = "hand_palm_link"
        orientation_c.orientation = quaternion_from_euler(roll, pitch, yaw)
        # orientation_c.orientation.w = 1.0
        orientation_c.absolute_x_axis_tolerance = 0.05
        orientation_c.absolute_y_axis_tolerance = 0.05
        orientation_c.absolute_z_axis_tolerance = 0.05
        orientation_c.weight = 1.0
        whole_body.goal_c.orientation_constraints.append(orientation_c)

        whole_body.set_path_constraints(whole_body.goal_c)

    elif enable == False:
        whole_body.set_path_constraints(None)


def clear_octomap():

    rospy.wait_for_service('/clear_octomap') #this will stop your code until the clear octomap service starts running
    clear_octomap_srv = rospy.ServiceProxy('/clear_octomap', Empty)

    # rospy.sleep(1.2)

    try:
        clear_octomap_srv()
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))


def get_object_dict():
    u"""Gazeboに出現させる物体の辞書を返す関数

    返り値:
        物体の辞書

    """

    object_dict = {}
    paths = glob.glob("/opt/ros/melodic/share/tmc_wrs_gazebo_worlds/models/ycb*")
    for path in paths:
        file = os.path.basename(path)
        object_dict[file[8:]] = file

    return object_dict


def get_object_list():
    u"""Gazeboに出現させる物体のリストを返す関数

    返り値:
        物体のリスト

    """

    object_list = get_object_dict().values()
    object_list.sort()
    for i in range(len(object_list)):
        object_list[i] = object_list[i][8:]

    return object_list


def put_object(name, x, y, z):
    u"""Gazeboに物体を出現させる関数"""

    cmd = "rosrun gazebo_ros spawn_model -database " \
          + str(get_object_dict()[name]) \
          + " -sdf -model " + str(name) \
          + " -x " + str(y - 2.1) + \
          " -y " + str(-x + 1.2) \
          + " -z " + str(z)
    subprocess.call(cmd.split())


def delete_object(name):
    u"""Gazeboの物体を消す関数

    引数:
        name (str): 物体の名前

    """

    cmd = ['rosservice', 'call', 'gazebo/delete_model',
           '{model_name: ' + str(name) + '}']
    subprocess.call(cmd)


class RGBD():
    u"""RGB-Dデータを扱うクラス"""

    def __init__(self):
        self._br = tf.TransformBroadcaster()
        # ポイントクラウドのサブスクライバのコールバックに_cloud_cbメソッドを登録
        self._cloud_sub = rospy.Subscriber(
            "/hsrb/head_rgbd_sensor/depth_registered/rectified_points",
            PointCloud2, self._cloud_cb)
        self._points_data = None
        self._image_data = None
        self._h_image = None
        self._region = None
        self._h_min = 0
        self._h_max = 0
        self._xyz = [0, 0, 0]
        self._frame_name = None

    def _cloud_cb(self, msg):
        # ポイントクラウドを取得する
        self._points_data = ros_numpy.numpify(msg)

        # 画像を取得する
        self._image_data = \
            self._points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]

        # 色相画像を作成する
        hsv_image = cv2.cvtColor(self._image_data, cv2.COLOR_RGB2HSV_FULL)
        self._h_image = hsv_image[..., 0]

        # 色相の閾値内の領域を抽出する
        self._region = \
            (self._h_image > self._h_min) & (self._h_image < self._h_max)

        # 領域がなければ処理を終える
        if not np.any(self._region):
            return

        # 領域からxyzを計算する
        (y_idx, x_idx) = np.where(self._region)
        x = np.average(self._points_data['x'][y_idx, x_idx])
        y = np.average(self._points_data['y'][y_idx, x_idx])
        z = np.average(self._points_data['z'][y_idx, x_idx])
        self._xyz = [x, y, z]

        # 座標の名前が設定されてなければ処理を終える
        if self._frame_name is None:
            return

        # tfを出力する
        self._br.sendTransform(
            (x, y, z), tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs),
            self._frame_name,
            msg.header.frame_id)

    def get_image(self):
        u"""画像を取得する関数"""
        return self._image_data

    def get_points(self):
        u"""ポイントクラウドを取得する関数"""
        return self._points_data

    def get_h_image(self):
        u"""色相画像を取得する関数"""
        return self._h_image

    def get_region(self):
        u"""抽出領域の画像を取得する関数"""
        return self._region

    def get_xyz(self):
        u"""抽出領域から計算されたxyzを取得する関数"""
        return self._xyz

    def set_h(self, h_min, h_max):
        u"""色相の閾値を設定する関数"""
        self._h_min = h_min
        self._h_max = h_max

    def set_coordinate_name(self, name):
        u"""座標の名前を設定する関数"""
        self._frame_name = name


class collision_object():

    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
        
    def box(self, x, y, z, size_x, size_y, size_z, frame_id, name):

        # Add grasping object - BOX
        # rospy.sleep(2)
        box_pose = geometry_msgs.msg.PoseStamped()
        
        # quaternion_from_euler(0, 180, 90)
        box_pose.header.frame_id = frame_id
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z
        box_name = name
        self.scene.add_box(box_name, box_pose, size=(size_x, size_y, size_z))

    
    def cylinder(self, x, y, z, height, radius, frame_id, name):

        # Add grasping object - CYLINDER
        # rospy.sleep(2)
        cylinder_pose = geometry_msgs.msg.PoseStamped()
        cylinder_pose.header.frame_id = frame_id
        cylinder_pose.pose.orientation.w = 1.0
        cylinder_pose.pose.position.x = x
        cylinder_pose.pose.position.y = y 
        cylinder_pose.pose.position.z = z 
        cylinder_name = name
        self.scene.add_cylinder(cylinder_name, cylinder_pose, height, radius)

        
    def sphere(self, x, y, z, radius, frame_id, name):

        # Add grasping object - SPHERE
        # rospy.sleep(2)
        sphere_pose = geometry_msgs.msg.PoseStamped()
        sphere_pose.header.frame_id = frame_id
        sphere_pose.pose.orientation.w = 1.0
        sphere_pose.pose.position.x = x
        sphere_pose.pose.position.y = y 
        sphere_pose.pose.position.z = z 
        sphere_name = name
        self.scene.add_sphere(sphere_name, sphere_pose, radius)
        # scene.set_object_color(sphere_name, 200, 0, 0, a=0.9)
        
        
    def mesh(self, x, y, z, file_path, frame_id, name):        

        # Add grasping object - MESH
        # rospy.sleep(2)
        mesh_pose = geometry_msgs.msg.PoseStamped()
        mesh_pose.header.frame_id = frame_id
        mesh_pose.pose.orientation.w = 1.0
        mesh_pose.pose.position.x = x
        mesh_pose.pose.position.y = y
        mesh_pose.pose.position.z = z 

        print (mesh_pose)

        mesh_name = name
        self.scene.add_mesh(mesh_name, mesh_pose, file_path)
        # scene.add_mesh(mesh_name, mesh_pose, '/opt/ros/melodic/share/tmc_wrs_gazebo_worlds/models/trofast/meshes/trofast.stl')
        # scene.add_mesh(mesh_name, mesh_pose, '/opt/ros/melodic/share/tmc_wrs_gazebo_worlds/models/trofast/meshes/trofast.stl', size=(1, 1, 1))

        
    def attach(self, object_name):

        # Attach grasping object
        grasping_group = 'whole_body'
        eef_link = whole_body.get_end_effector_link()

        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(eef_link, object_name, touch_links=touch_links)
        # rospy.sleep(5)

        
    def dettach(self, object_name):

        # Detach grasping object
        eef_link = whole_body.get_end_effector_link()
        self.scene.remove_attached_object(eef_link, object_name)
        # rospy.sleep(5)

