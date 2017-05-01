#!/usr/bin/env python
# coding: utf-8

import rospy
from geometry_msgs.msg import WrenchStamped
from body_module_wrist_wrench_manager.msg import WristWrenchForceAction, WristWrenchForceResult, WristWrenchForceFeedback
import numpy as np
import time
import actionlib

class WristWrench(object):
    def __init__(self):
        rospy.Subscriber("/hsrb/wrist_wrench/compensated", WrenchStamped, self.cb_wrench_compensated,queue_size=10)
        rospy.Subscriber("/hsrb/wrist_wrench/raw", WrenchStamped, self.cb_wrench_raw, queue_size=10)
        self.action_server = actionlib.SimpleActionServer("wrist_wrench_manager_action", 
                                                    WristWrenchForceAction, execute_cb=self.force_check_compensated,
                                                    auto_start=False)

        self._wrench_raw = WrenchStamped()
        self._wrench_com = WrenchStamped()
        self.rate = rospy.Rate(100)
        self.action_server.start()

    def cb_wrench_compensated(self, data):
        self._wrench_com = data

    def cb_wrench_raw(self, data):
        self._wrench_raw = data

    def force_check_compensated(self, data):
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
        if data.force < 0.2:
            rospy.logwarn("request force is too small")
        base_wrench = self._wrench_com
        wrench_feedback = WristWrenchForceFeedback()

        flag = 0

        if data.time == 0:
            flag = 1
        st_time = time.time()
        success =WristWrenchForceResult()
        success.success = False
        print(base_wrench)
        while (time.time() - st_time) < data.time or flag and not rospy.is_shutdown():
            now_wrench = self._wrench_com
            force_x = abs(now_wrench.wrench.force.x - base_wrench.wrench.force.x)
            force_y = abs(now_wrench.wrench.force.y - base_wrench.wrench.force.y)
            force_z = abs(now_wrench.wrench.force.z - base_wrench.wrench.force.z)
            force = np.sqrt( force_x ** 2 + force_y ** 2 + force_z ** 2)
            wrench_feedback.now_force = force
            print(force_x, force_y, force_z)
            self.action_server.publish_feedback(wrench_feedback)

            if force > data.force:
                print(force)
                print success
                success.success = True
                break
            self.rate.sleep()
        if not success.success:
            rospy.logwarn("no wrench force")
        self.action_server.set_succeeded(success)


if __name__ == "__main__":
    rospy.init_node("wrist_wrench_manager")
    wrist_wrench = WristWrench()
    rospy.spin()    
