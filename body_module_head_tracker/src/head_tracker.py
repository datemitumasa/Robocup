#!/usr/bin/env python
# coding: utf-8

import rospy
import numpy as np
import time
import actionlib

from sensor_msgs.msg import JointState
from body_module_head_tracker.msg import BodyHeadTrackAction, BodyHeadTrackResult, BodyHeadTrackFeedback
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tf_utils import Tf


BASE = "torso_lift_link"
VEL = 0.1

class HeadTracker(object):
    def __init__(self):
        rospy.Subscriber("/hsrb/joint_states", JointState ,self.get_pose,queue_size=10)
        self.action_server = actionlib.SimpleActionServer("/body/head_tracker", 
                                                    BodyHeadTrackAction, execute_cb=self.head_track,
                                                    auto_start=False)

        self._action = actionlib.SimpleActionClient('/hsrb/head_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.head_command = FollowJointTrajectoryGoal()
        self.traj = JointTrajectory()
        self.traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        self.point =JointTrajectoryPoint()
        self.point.time_from_start = rospy.Time(VEL)

        self.rate = rospy.Rate(10)
        self.tf = Tf()
        self.action_server.start()
        self.now_pan = 0.0
        self.now_tilt = 0.0

    def get_pose(self, data):
        self.now_pan = data.position[9]
        self.now_tilt = data.position[10]

    def head_track(self, data):
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
        rospy.logwarn(data)
        tf_name = data.tf_name
        timeout = data.time_out
        if timeout == 0:
            timeout = 10 ** 10

        head_feedback = BodyHeadTrackFeedback()

        st_time = time.time()
        success =BodyHeadTrackResult()
        success.success = True
        while (time.time() - st_time) < timeout and not rospy.is_shutdown():
            if self.action_server.is_preempt_requested():
                rospy.loginfo("stop action head track")
                self.action_server.set_preempted()
                break
            xyz, q_, e_ = self.tf.lookup(BASE, tf_name)
            if xyz ==[]:
                continue
            pan = np.arctan2(xyz[1], xyz[0])
            tilt = np.arctan2(xyz[2], np.sqrt(xyz[0]**2 + xyz[1]**2))
            if pan >1.7:
                pan = 1.7
            elif pan < -3.8:
                pan = -3.8
            
            if tilt > 0.5:
                tilt = 0.5
            elif tilt < -1.5:
                tilt = -1.5
                
            self.point.positions = [pan, tilt]
            self.point.effort = [0.0,0.0]
            self.point.velocities = [0.0, 0.0]
#            self.point.velocities = [pan_vel, tilt_vel]
            traj = JointTrajectory()
            traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
            traj.points.append(self.point)
            
            self.head_command.trajectory = traj

            self._action.send_goal(self.head_command)

            head_feedback.pan = self.now_pan
            head_feedback.tilt = self.now_tilt
            self.action_server.publish_feedback(head_feedback)

            self.rate.sleep()
        
        self.action_server.set_succeeded(success)


if __name__ == "__main__":
    rospy.init_node("head_tracker")
    wrist_wrench = HeadTracker()
    rospy.spin()    
