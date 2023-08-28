#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State, PositionTarget, HomePosition
from sensor_msgs.msg import Imu, BatteryState, NavSatFix
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool, CommandHome, CommandTOL, SetMode, StreamRate, ParamGet
from math import *
import pymap3d as pm
from communication_info import *
from pathFollowing import *


class Drone(object):
    def __init__(self, message_rate=10):
        ' Stream rate service '
        rospy.wait_for_service("/mavros/set_stream_rate")
        set_stream_rate = rospy.ServiceProxy("/mavros/set_stream_rate", StreamRate)
        set_stream_rate(message_rate=message_rate, on_off=1)
        ' Cooridnate system '
        self.origin_dict = {"0": [22.9242595824972, 120.310152769089], 
                            "1": [22.9105162191694, 120.312446057796]}
        self.origin = self.origin_dict["1"]
        ' UAV info '
        self.armed = False
        self.mode = None
        self.local_pose, self.local_velo = [0, 0, 0], [0, 0, 0]
        self.gps_pose_lla = [0, 0, 0]
        self.roll, self.pitch, self.yaw = 0, 0, 0
        self.battery_volt, self.battery_perc = 0, 0
        self.home = [0, 0, 0]
        self.frame_type = None
        while not self.frame_type:
            self.uav_classifier()
        rospy.Subscriber("/mavros/state", State, self.state_callback)
        rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.gps_enu_callback)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_lla_callback)
        rospy.Subscriber("/mavros/home_position/home", HomePosition, self.home_callback)
        rospy.Subscriber("/mavros/battery", BatteryState, self.battery_callback)
        self.setpoint_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        ' Commands '
        ' => { Px, Py, Pz, Yaw } '
        self.position_cmd = PositionTarget()
        self.position_cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.position_cmd.type_mask = 0b0000101111111000
        ' => { Px, Py, Pz } '
        self.waypoint_cmd = PositionTarget()
        self.waypoint_cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.waypoint_cmd.type_mask = 0b0000111111111000
        ' => { Vx, Vy, Vz } '
        self.velocity_cmd = PositionTarget()
        self.velocity_cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.velocity_cmd.type_mask = 0b0000111111000111
        ' => { Vx, Yaw Rate } '
        self.v_body_cmd = PositionTarget()
        self.v_body_cmd.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.v_body_cmd.type_mask = 0b0000011111000111
        ' Prameters '
        self.v = 5
        self.Rmin = 10
        self.type = 2

    def state_callback(self, msg):
        self.armed = msg.armed
        self.mode = msg.mode

    def gps_lla_callback(self, msg):
        self.gps_pose_lla = [msg.latitude, msg.longitude, msg.altitude]

    def gps_enu_callback(self, msg):
        e_h, n_h, u_h = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        ' Transform to the unified ENU coordinates of UAVs and GCS '
        x, y, z = pm.enu2ecef(e_h, n_h, u_h , self.home[0], self.home[1], self.home[2])
        e, n, u = pm.ecef2enu(x, y, z, self.origin[0], self.origin[1], 0)
        self.local_pose = [e, n, u_h]
        self.local_velo = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]      
    
    def imu_callback(self, msg):
        self.imu_msg = msg
        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z # in radians

    def home_callback(self, msg):
        self.home = [msg.geo.latitude, msg.geo.longitude, msg.geo.altitude]

    def battery_callback(self, msg):
        self.battery_volt = msg.voltage
        self.battery_perc = msg.percentage * 100

    def set_home_position(self, lat, lng, alt):
        rospy.wait_for_service("/mavros/cmd/set_home")
        try:
            cmd_setHome = rospy.ServiceProxy("/mavros/cmd/set_home", CommandHome)
            responce = cmd_setHome(current_gps=False, latitude=lat, longitude=lng, altitude=alt)
            print(responce.success)
        except rospy.ServiceException as e:
            print(e)

    def takeoff(self, alt):
        rospy.wait_for_service("/mavros/cmd/takeoff")
        try:
            cmd_takeoff = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
            responce = cmd_takeoff(altitude=alt, latitude=0, longitude=0, min_pitch=0, yaw=PlusMinusPi((90 - self.heading)*pi/180))
            return responce.success
        except rospy.ServiceException as e:
            return False

    def set_mode(self, mode):
        rospy.wait_for_service("/mavros/set_mode")
        try:
            set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
            responce = set_mode(custom_mode=mode)
            return responce.mode_sent
        except rospy.ServiceException as e:
            return False

    def set_arm(self): 
        rospy.wait_for_service("/mavros/cmd/arming")
        try:
            cmd_arming = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
            cmd_arming(True)
        except rospy.ServiceException as e:
            print(e)

    def set_disarm(self):
        rospy.wait_for_service("/mavros/cmd/arming")
        try:
            cmd_arming = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
            cmd_arming(False)
        except rospy.ServiceException as e:
            print(e)

    def get_param(self, param_name): 
        rospy.wait_for_service("/mavros/param/get")
        try:
            param_get = rospy.ServiceProxy("/mavros/param/get", ParamGet)
            responce = param_get(param_id=param_name)
            if responce.success:
                return responce.value
            else:
                return False
        except rospy.ServiceException as e:
            return False
    
    def uav_classifier(self):
        servo_1 = self.get_param("SERVO1_FUNCTION")
        if servo_1:
            if servo_1.integer == 4:
                ' Aileron: 4 '
                self.frame_type = FrameType.Fixed_wing
            else:                
                self.frame_type = FrameType.Quad
    
    def origin_correction(self, origin_id):
        self.origin = self.origin_dict[str(origin_id)]

    def guide_to_waypoint(self, waypoint, yaw=None):
        ' waypoint = [Px, Py, Pz]  with yaw angle '
        x, y, z = pm.enu2ecef(waypoint[0], waypoint[1], waypoint[2], self.origin[0], self.origin[1], 0)
        e, n, u = pm.ecef2enu(x, y, z, self.home[0], self.home[1], self.home[2])
        self.position_cmd.position.x = e
        self.position_cmd.position.y = n
        self.position_cmd.position.z = waypoint[2]
        self.position_cmd.yaw = yaw if yaw else self.heading*pi/180
        self.setpoint_pub.publish(self.position_cmd)
    
    def position_control(self, wp_x, wp_y, wp_z):
        x, y, z = pm.enu2ecef(wp_x, wp_y, wp_z, self.origin[0], self.origin[1], 0)
        e, n, u = pm.ecef2enu(x, y, z, self.home[0], self.home[1], self.home[2])
        self.waypoint_cmd.position.x = e
        self.waypoint_cmd.position.y = n
        self.waypoint_cmd.position.z = wp_z
        self.setpoint_pub.publish(self.waypoint_cmd)

    def velocity_control(self, velocity):
        '''  velocity = [Vx, Vy, Vz]  '''   
        self.velocity_cmd.velocity.x = velocity[0]
        self.velocity_cmd.velocity.y = velocity[1]
        self.velocity_cmd.velocity.z = velocity[2]
        self.setpoint_pub.publish(self.velocity_cmd)

    def velocity_bodyFrame_control(self, Vx, yaw_rate, Vz=None):
        self.v_body_cmd.velocity.x = Vx
        self.v_body_cmd.velocity.y = 0
        self.v_body_cmd.velocity.z = Vz if Vz else 0
        self.v_body_cmd.yaw_rate = yaw_rate  # +-180
        self.setpoint_pub.publish(self.v_body_cmd)
        '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'

    def iteration(self, event):
        print('LLA: ', self.gps_pose_lla)
        print("local/pose:", self.local_pose, self.local_velo)
        print("yaw angle:", self.yaw, self.yaw*180/pi)
        print("roll angle:", self.roll, self.roll*180/pi)
        print("pitch angle:", self.pitch, self.pitch*180/pi)
        # print(self.position_cmd.coordinate_frame, PositionTarget.FRAME_LOCAL_NED)
        print("....................................")
        print("bat:", self.battery_perc)
        print("bat:", self.battery_volt)
        print("....................................")
        # print("home: ", self.home)
        # print("armed", self.armed)
        # print(self.frame_type)
        print("\n")


if __name__ == "__main__":
    rospy.init_node('drone', anonymous=True)
    uav_controll = Drone()

    rospy.Timer(rospy.Duration(0.5), uav_controll.iteration)
    rospy.spin()
    