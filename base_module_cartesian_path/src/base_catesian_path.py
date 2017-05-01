#!/usr/bin/env python
# coding: utf-8

import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped
from base_module_cartesian_path.msg import BaseCartesianPathAction, BaseCartesianPathResult, BaseCartesianPathFeedback, BaseCartesianPathGoal
from geometry_msgs.msg import Pose
import numpy as np
import time
import actionlib
import tf

class BaseCartesianPath(object):
    def __init__(self):
        rospy.loginfo("base_cartesian_path init")
        self.goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=10)
        rospy.Subscriber("/laser_2d_pose", PoseWithCovarianceStamped, self._get_pose, queue_size=10)
        self.action_server = actionlib.SimpleActionServer("base_catesian_path_action", 
                                                   BaseCartesianPathAction, execute_cb=self.move_cartesian_path,
                                                    auto_start=False)

        self.now_pose = BaseCartesianPathFeedback()
        self.rate = rospy.Rate(100)
        self.action_server.start()

    def _get_pose(self, data):
        """
        現在位置の情報をFeedBackに入力する
        """
        self.now_pose.now_pose = data.pose.pose
        self.now_pose.header.stamp = rospy.Time.now()

    def move_cartesian_path(self, data):
        """
        指定時間の間に閾値以上の力が力各センサに加わったかどうかを返す。
        現在の値からの変位を見ているため、開始のタイミングに注意

        Args:
            ntime  int32: 指定時間
            nforce float64: 閾値の力
        Result:
            success bool:
        Feedback:
            force float64: 現在加えられている力
        """
        goal_list = data.path_list
        threshold_x = data.threshold_x
        threshold_y = data.threshold_y
        threshold_rotate = data.threshold_rotate
        success = BaseCartesianPathResult()
        print data
        if len(goal_list) == 0:
            success.success = False
            self.action_server.set_succeeded(success)
            rospy.logwarn("not goal")

        count = 0
        goal =PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose = goal_list[0]
        self.goal_pub.publish(goal)
        self.now_pose.now_goal = goal_list[0]
        while not rospy.is_shutdown():
            now_pose = self.now_pose
            now_rotate = tf.transformations.euler_from_quaternion([now_pose.now_pose.orientation.x, now_pose.now_pose.orientation.y, now_pose.now_pose.orientation.z, now_pose.now_pose.orientation.w])
            goal_rotate = tf.transformations.euler_from_quaternion([now_pose.now_goal.orientation.x, now_pose.now_goal.orientation.y, now_pose.now_goal.orientation.z, now_pose.now_goal.orientation.w])
            if abs(now_pose.now_pose.position.x - now_pose.now_goal.position.x) < threshold_x and abs(now_pose.now_pose.position.y - now_pose.now_goal.position.y) < threshold_y:
                if abs(now_rotate[0] - goal_rotate[0]) < threshold_rotate:
                    count += 1
                    if count == len(goal_list):
                        break
                    goal.header.stamp = rospy.Time.now()
                    goal.pose = goal_list[count]
                    self.goal_pub.publish(goal)
                    self.now_pose.now_goal = goal_list[count]
                                
            self.action_server.publish_feedback(self.now_pose)
            self.rate.sleep()
        success.success = True
        self.action_server.set_succeeded(success)


if __name__ == "__main__":
    rospy.init_node("base_cartesian_path_planer")
    wrist_wrench = BaseCartesianPath()
    rospy.spin()