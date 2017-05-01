#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState, LaserScan
from base_module_state_checker.msg import HsrStateCheck
from base_module_state_checker.srv import FloatBool, StringBool, HsrOdom, HsrOdomRequest, StringBoolResponse, FloatBoolResponse, ObstacleInfo, ObstacleInfoResponse
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf


#from people_msgs.msg import People

class HsrState(object):
    def __init__(self):
        rospy.init_node('hsrb_state_check_node')
        self.req_base = HsrOdomRequest()
        self.hand_pose  = 0
        self.hand_state = "open"
        self.check_order = False
        self.scan_dis = 0.0
        self.scan_pos_x = None
        self.scan_pos_y = None
        self.arround_state = [0,0,0]
        self.grasp_obj = [0]
        self.laser_data = []
        self.obstacle_point_cloud = PointCloud2()
        self.get_obstacle_point_flag = False
        self.xtion_most_smal_range = 10
        #self.rate = rospy.Rate(10)
        #self.people_vel = People()
        self.pub_grasp = rospy.Publisher("/hsrb/hand/grasp_obj", HsrStateCheck, queue_size = 10)
        self.pub_env = rospy.Publisher("/hsrb/base/local_env", HsrStateCheck, queue_size = 10)
        self.pub_speed_scale = rospy.Publisher("/speed_scale", Float64, queue_size = 10)
        self.pub_most_smal_range = rospy.Publisher("/most_smal_range", Float64, queue_size = 10)
        self.srv_base = rospy.ServiceProxy("hsr_base_service", HsrOdom)
        #tf
        rospy.Service("hsrb_grasp_state_check_service", StringBool, self.grab_state)
        rospy.Service("hsrb_distance_check_service", FloatBool, self.dis_check_move)
        rospy.Service("hsrb_obstacle_info_in_square", ObstacleInfo, self.obstacle_point_cloud_in_square_func)
        rospy.Service("hsrb_obstacle_info_in_circle", ObstacleInfo, self.obstacle_point_cloud_in_circle_func)
        rospy.Subscriber("/hsrb/joint_states", JointState, self.hand_state_update)
        rospy.Subscriber("/hsrb/base_scan", LaserScan, self.scan_state_update)
        rospy.Subscriber("/hsrb/base_scan2", LaserScan, self.xtion_scan_state_update)
        rospy.Subscriber("/obstacle_point_cloud", PointCloud2, self.get_obstacle_point_cloud)
        #rospy.Subscriber("/people", People, self.get_people_velocity)

    def get_obstacle_point_cloud(self, data):
        if self.get_obstacle_point_flag:
            #rospy.loginfo("get point cloud")
            self.obstacle_point_cloud = data
            rospy.logwarn( "get obstacle point")
            self.get_obstacle_point_flag = False
        else:
            pass
            #rospy.loginfo("not get point cloud")

    def obstacle_point_cloud_in_square_func(self, data):
        count = 0
        self.get_obstacle_point_flag = True
        while not rospy.is_shutdown():
            if self.get_obstacle_point_flag is False:
                break
        for point in pc2.read_points(self.obstacle_point_cloud):
            if (data.min_x < point[0] < data.max_x ) & (data.min_y < point[1] < data.max_y ):
                count += 1
        ans = ObstacleInfoResponse()
        ans.num_of_obstacle_point = count
        return ans

    def obstacle_point_cloud_in_circle_func(self, data):
        count = 0
        self.get_obstacle_point_flag = True
        while not rospy.is_shutdown():
            if self.get_obstacle_point_flag is False:
                break
        for point in pc2.read_points(self.obstacle_point_cloud):
            if  ((point[0]-data.x)**2 + (point[1]-data.y)**2 ) < data.r**2 :
                count += 1
        ans = ObstacleInfoResponse()
        ans.num_of_obstacle_point = count
        return ans


    def dis_check_move(self,data):
        if self.obj_dis > 1.5:
            return FloatBoolResponse(success=False)
        else:
            self.req_base.x = self.obj_dis - data.height
            self.req_base.command_name = "go"
            res = self.srv_base(self.req_base)
            return FloatBoolResponse(success=res.success)

    def hand_state_update(self,data):
        self.hand_pose = data.position[7]
        #print(self.hand_pose)
        grasp_env = HsrStateCheck()
        if self.hand_pose > 1.0:
            self.hand_state = "open"
            self.grasp_obj[0] = 0
        elif self.hand_pose < -0.83:
            self.hand_state = "close"
            self.grasp_obj[0] = 0
        else:
            self.hand_state = "grasp"
            self.grasp_obj[0] = 1
        grasp_env.env = self.grasp_obj
        self.pub_grasp.publish(grasp_env)
#        print(self.hand_state)

#    def obstacle_info(self,data):
#        u"""
#        |四角形領域にレーザーの値が何個入っているか返すサービス
#        """
#        x = 0
#        y = 0
#        r = 1
#        ans = ObstacleInfoResponse()
#        #ans.num_of_obstacle_point = np.count_nonzero((((data.max_y > self.scan_pos_y)&(self.scan_pos_y > data.min_y)) * 1) & (((data.max_x > self.scan_pos_x)&(self.scan_pos_x > data.min_x))*1))
#        ans = np.count_nonzero( ( (self.scan_pos_x-x)**2 + (self.scan_pos_y-y)**2 ) < r**2)
#        return ans

    def scan_state_update(self,laser_data):
        data = laser_data
        self.laser_data = laser_data
        N = len(data.ranges)
        local_env = HsrStateCheck()
        self.scan_dis = data.ranges[len(data.ranges)/2]
#        print(self.scan_dis)
        data.ranges
        angle = np.linspace(data.angle_min,data.angle_max,N)
        self.scan_pos_x = np.array(data.ranges) * np.cos(angle)
        self.scan_pos_y = np.array(data.ranges) * np.sin(angle)
#        print(self.scan_pos_y[len(data.ranges)/2])
#        print(np.count_nonzero(((0.6 > self.scan_pos_x)&(self.scan_pos_x > 0.3)) * 1))
        if 0 < np.count_nonzero((((0.6 > self.scan_pos_y)&(self.scan_pos_y > 0)) * 1) & (((0.2 > self.scan_pos_x)&(self.scan_pos_x > -0.3))*1)):
#            rospy.logwarn("left_point{0:s}".format(np.count_nonzero(((0.6 > self.scan_pos_y)&(self.scan_pos_y > 0)) * 1) and 0 < np.count_nonzero(((0.2 > self.scan_pos_x)&(self.scan_pos_x > -0.3))*1)))
            self.arround_state[0] = 1
            #rospy.loginfo("obj in left space ")
        else:
            self.arround_state[0] = 0

        if 0 < np.count_nonzero((((0.1 > self.scan_pos_y)&(self.scan_pos_y > -0.1)) * 1) & (((0.4 > self.scan_pos_x)&(self.scan_pos_x > 0.0))*1)):
            self.arround_state[1] = 1
           # rospy.logwarn("front_point{0}".format(np.count_nonzero((((0.1 > self.scan_pos_y)&(self.scan_pos_y > -0.1)) * 1) & (((0.4 > self.scan_pos_x)&(self.scan_pos_x > 0.0))*1))))
            #rospy.loginfo("obj in front space ")
        else:
            self.arround_state[1] = 0

        if 0 < np.count_nonzero((((0.0 > self.scan_pos_y)&(self.scan_pos_y > -0.6)) * 1) & (((0.2 > self.scan_pos_x)&(self.scan_pos_x > -0.3))*1)):
            self.arround_state[2] = 1
            #rospy.loginfo("obj in right space ")
        else:
            self.arround_state[2] = 0
        local_env.env = self.arround_state
        #rospy.logwarn(local_env)
        self.pub_env.publish(local_env)

        most_smal_range = np.nanmin(data.ranges)
        if self.xtion_most_smal_range < most_smal_range:
            most_smal_range = self.xtion_most_smal_range
        min_range = 0.15
        max_range = 0.5
        scale = Float64()
        scale.data = (most_smal_range - min_range ) / (max_range - min_range)
        if scale.data < 0:
            scale.data = 0
        if scale.data > 1:
            scale.data = 1.0
        self.pub_most_smal_range.publish(most_smal_range)
        self.pub_speed_scale.publish(scale)

    def xtion_scan_state_update(self,laser_data):
        most_smal_range = np.nanmin(laser_data.ranges)
        min_range = 0.15
        max_range = 0.5
        scale = Float64()
        scale.data = (most_smal_range - min_range ) / (max_range - min_range)
        if scale.data < 0:
            scale.data = 0
        if scale.data > 1:
            scale.data = 1.0
        self.xtion_most_smal_range = most_smal_range

    def grab_state(self,data):
        self.check_order = False
        if self.hand_state == data.command:
            self.check_order = True
        ans = StringBoolResponse()
        ans.success = self.check_order
        return ans

if __name__ == "__main__":
    joint_state_service = HsrState()
    rospy.loginfo("hsrb_state_checker is running")
    rospy.spin()
