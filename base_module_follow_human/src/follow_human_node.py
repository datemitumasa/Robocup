#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import math

from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from people_msgs.msg import PositionMeasurementArray
from people_msgs.msg import PositionMeasurement
from std_msgs.msg import Float64
import tf2_geometry_msgs
import actionlib
from actionlib_msgs.msg import GoalID
from base_module_follow_human.msg import FollowHumanAction
from base_module_follow_human.msg import FollowHumanResult
from base_module_follow_human.msg import FollowHumanFeedback
from base_module_state_checker.srv import ObstacleInfo, ObstacleInfoRequest
import copy
from base_module_state_checker.msg import HsrStateCheck

class FollowHuman(object):
    """
    hsr_follow_meを元にLiProのfollow_humanを真似て作成。
    このプログラムは、汚いので、参考にするなら、
    LiProのfollow_humanを参考にすること
    """

    _TF_TIMEOUT = 5.0
    _ROBOT_FRAME = 'base_footprint'
    _DEFAULT_MAX_ANGLE = math.radians(60)
    _DEFAULT_MIN_ANGLE = math.radians(-60)
    _DEFAULT_ROT_SPEED = 0.6
    _DEFAULT_FOW_SPEED = 0.25
    _DEFAULT_CLEARNCE = 0.7
    _DEFAULT_OBSTACLE_DISTANCE = 0.5
    _DEFAULT_MIN_SPEED = 0.08
    _DEFAULT_ABOID_TIME = 8
    _DEFAULT_OBSTACLE_COUNT_THRESHOLD = 2

    def __init__(self):
        rospy.init_node('follow_human_node')
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer)
        self._goal_pub = rospy.Publisher('goal', PoseStamped, queue_size=10)
        self._goal_cancel_pub = rospy.Publisher('/move_base/move/cancel', GoalID, queue_size=10)
        self.twist_pub = rospy.Publisher('hsrb/command_velocity', Twist, queue_size=10)
        self.zero_twist = Twist()
        self.seeds_pub = rospy.Publisher("/people_tracker_filter",PositionMeasurement,queue_size=10)
        self._speed_scale = 0
        self._most_smal_range = 0
        self.seq = 0
        self.enable_avoid_obstacle = True
        self.avoid_function = self.avoidByPlannning
        self._people = []
        self.best_person = []
        self.particle_x = 0.8
        self.particle_y = 0
        self._last_person_x = self.particle_x
        self._last_person_y = self.particle_y
        self._do_action = False
        self._stop_count = 0
        self._front_state = [1,1,1]

        self.set_rosparam()
        self._lost_count = 0      # 人を見失ったカウント
        self._obstacle_count = 0  # 目の前に障害物があって動けない場合のカウント
        self._chase_count = 0     # 追跡に成功しているときのカウント
        self._num_decrease_obstacle_count = 0.1
        self._action_server = actionlib.SimpleActionServer('follow_human_action',FollowHumanAction,execute_cb=self.run,auto_start=False)
        self._action_server.start()
        rospy.Subscriber("/speed_scale", Float64, self.__get_spped_scale)
        rospy.Subscriber("/most_smal_range", Float64, self.__get_most_smal_range)
        rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, self.people_callback)
        rospy.Subscriber("/hsrb/base/local_env", HsrStateCheck, self.__get_front_envirment)

    def set_rosparam(self,):
        self._obstacle_count_threshold = rospy.get_param('~obstacle_count_threshold',self._DEFAULT_OBSTACLE_COUNT_THRESHOLD)
        self._avoid_time = rospy.get_param('~avoid_time',self._DEFAULT_ABOID_TIME)
        self._max_angle = rospy.get_param('~max_angle',self._DEFAULT_MAX_ANGLE)
        self._min_angle = rospy.get_param('~min_angle',self._DEFAULT_MIN_ANGLE)
        self._clearance = rospy.get_param('~clearance',self._DEFAULT_CLEARNCE)
        self._rot_speed = rospy.get_param('~rot_speed',self._DEFAULT_ROT_SPEED)
        self._fow_speed = rospy.get_param('~fow_speed',self._DEFAULT_FOW_SPEED)
        self._obstacle_distance = rospy.get_param('~obstacle_distance',self._DEFAULT_OBSTACLE_DISTANCE)
        self._min_speed = rospy.get_param('~min_speed',self._DEFAULT_MIN_SPEED)

    def __get_front_envirment(self,data):
        """
        Subscribeした値を_front_state入れ続ける
        """
        self._front_state = data.env

    def calc_invert_distance(self, person):
        distance = (pow(person.pos.x, 2) + pow(person.pos.y, 2))
        try:
            return 1.0 / distance
        except ZeroDivisionError:
            return 0.0

    def sendHumanTrackOrder(self, x=0.8, y=0):
        """
        seedsを送る
        関数の名前おかしい
        """
        person = PositionMeasurement()
        person.header.seq = self.seq
        self.seq += 1
        person.header.stamp = rospy.Time.now()
        person.header.frame_id = "base_range_sensor_link"
        person.name = "person1"
        #person1.object_id = "leg0|leg1"
        person.pos.x = x
        person.pos.y = y
        person.reliability = 0.8
        person.covariance = [0.16, 0.0, 0.0, 0.0, 0.16, 0.0, 0.0, 0.0, 10000.0]
        self.seeds_pub.publish(person)

    def calc_invert_distance_acording_last_person(self, person):
        distance = (pow(person.pos.x-self._last_person_x , 2) + pow(person.pos.y-self._last_person_y, 2))
        try:
            return 1.0 / distance
        except ZeroDivisionError:
            return 0.0

    def select_best_person(self, people, value_function):
        return max(people, key=value_function)

    def __get_spped_scale(self,scale):
        """
        Subscribeした値を_speed_scaleに入れる
        """
        self._speed_scale = scale.data

    def __get_most_smal_range(self,data):
        """
        Subscribeした値を_most_smai_rangeに入れる
        """
        self._most_smal_range = data.data

    def people_callback(self, data):
        """
        あまりに遠い人や、速度がおかしい人を弾くようにする
        """
        # for per in data.people:

        self._people = data.people

    def calc_target_pos_to_follow(self, person):
        if person.header.frame_id:
            person_pos = PointStamped()
            person_pos.point = person.pos
            person_pos.header = person.header
            robot_to_person = self._tf_buffer.transform(
                object_stamped=person_pos,
                target_frame=self._ROBOT_FRAME,
                timeout=rospy.Duration(self._TF_TIMEOUT))
            x = robot_to_person.point.x
            y = robot_to_person.point.y
            distance = math.sqrt(x*x + y*y)
            direction = math.atan2(y, x)
            return distance, direction, y

    def calc_target_gaol(self, person): #action feedback用 + path plan用
        if person.header.frame_id:
            person_pos = PointStamped()
            person_pos.point = person.pos
            person_pos.header = person.header
            robot_to_person = self._tf_buffer.transform(
                object_stamped=person_pos,
                target_frame=self._ROBOT_FRAME,
                timeout=rospy.Duration(self._TF_TIMEOUT))
            x = robot_to_person.point.x
            y = robot_to_person.point.y
            distance = math.sqrt(x*x + y*y)
            #unit_vec = [x/distance, y/distance]
            # clearance未満の場合は動かない
            go_disance = max(0, distance - self._clearance)
            go_pos = [x*go_disance/distance, y*go_disance/distance]
            goal = PoseStamped()
            goal.header = robot_to_person.header
            yaw = math.atan2(y,x)
            goal.pose.position.x = go_pos[0]
            goal.pose.position.y = go_pos[1]
            goal.pose.orientation.z = math.sin(yaw/2.0)
            goal.pose.orientation.w = math.cos(yaw/2.0)
            return goal

    def chaseHuman(self, distance, direction, y):
        """
        人を追跡するメソッド LipRoのパクリ、ゲインは変えてある
        :param float distance: 追跡対象との距離
        :param float direction: 追跡対象の角度
        """
        twist = Twist()
        # 曲率計算
        K = (2 * y) / (distance ** 2)

        # 前進速度と回転速度の計算
        d = [0, 0]
        if self._min_angle < direction < self._max_angle:
            if distance > self._clearance:
                d[0] = (distance - self._clearance)
                if d[0] > self._fow_speed:
                    d[0] = self._fow_speed
                d[1] = d[0] * K * 2

                twist.linear.x = d[0] * self._speed_scale

                #速度が閾値以下の場合
                if d[0] <= self._min_speed:
                    d[0] = self._min_speed

                twist.angular.z = d[1]
#                rospy.loginfo("通常の追跡")
#                rospy.loginfo(twist)

                if self._front_state[0] is 1:#左にものがある
                    #右後ろに行く
                    twist.linear.y = -0.1
                elif self._front_state[2] is 1:#右にものがある
                    #左後ろに行く
                    twist.linear.y = 0.1

                self.twist_pub.publish(twist)
            else:
#                rospy.loginfo("近いので止まる")
#                rospy.loginfo(self.zero_twist)
                self.twist_pub.publish(self.zero_twist)
        else:
            twist.angular.z = direction / abs(direction) * self._rot_speed
#            rospy.loginfo("回転量が大きいのでその場で回転")
#            rospy.loginfo(twist)
            self.twist_pub.publish(twist)

    def run(self,action_goal):
        self.set_rosparam()
        self._do_action = True
        r = rospy.Rate(10) # 10hz
        self.particle_x = action_goal.x
        self.particle_y = action_goal.y
        self._last_person_x = self.particle_x
        self._last_person_y = self.particle_y
        if action_goal.avoid_function is 0:
            #rospy.logwarn( "disableAvoid")
            self.disableAvoid()
        elif action_goal.avoid_function is 1:
            #rospy.logwarn( "switch2PlannningAvoid")
            self.switch2PlannningAvoid()
        elif action_goal.avoid_function is 2:
            #rospy.logwarn( "switch2OpenSpaceAvoid")
            self.switch2OpenSpaceAvoid()
        else:
            rospy.logwarn("avoid_function value shuld 0 or 1 or 2")
        while not rospy.is_shutdown():
            self.set_rosparam()
            r.sleep()
            try:

                # エラーカウントのチェック
                if self._lost_count >= 100:
                    rospy.logwarn("完全に人を見失いました")
                    self._lost_count = 0
                    #breakでなく、人を再探索するようにする
                    break
                elif self._lost_count >= 50:
                    if self._lost_count == 50:
                        rospy.logwarn("人を見失っています...再トライ")
                    if self._lost_count % 10 == 0:
                        self.sendHumanTrackOrder()

                # 得られたデータのチェック
                # 情報の受信
                if not self._people:
                    #rospy.logwarn("人が見つからない")
                    self._lost_count += 1
                    continue

                #actionのcancelをチェック
                if self._action_server.is_preempt_requested():
                    self._action_server.set_preempted()
                    rospy.loginfo("cancel follow_human_action")
                    break

                # 正しく情報が取得できているとここまで来る
                self._lost_count = 0

                people = self._people
                self.best_person = self.select_best_person(people, self.calc_invert_distance_acording_last_person)

                print self.best_person
                (human_distance, human_direction, y) = self.calc_target_pos_to_follow(self.best_person)
                obstacle_distance = self._most_smal_range

                # 追跡対象よりも手前に障害物があった場合は止まる
                # この状態が長く続いていた場合はOpenSpaceに動く
                if self.enable_avoid_obstacle:
                    if self._obstacle_count >= self._obstacle_count_threshold:
                        rospy.logwarn("障害物の回避行動を行います．")
                        #print self._obstacle_count
                        self.avoid_function()
                        self._obstacle_count -= self._num_decrease_obstacle_count
                        continue

                    if (human_distance - obstacle_distance) > self._obstacle_distance:
                        self._chase_count = 0
                        # followを一旦中止して障害物がさるのを待つ
                        rospy.logwarn("進行方向に障害物")
                        #print self._obstacle_count
                        self.twist_pub.publish(self.zero_twist)
                        rospy.sleep(0.5)
                        # 一番直近で追跡できていた位置にparticleを撒き直す
                        if not self._people:
                            x, y = self.particle_x, self.particle_y
                            self.sendHumanTrackOrder(x, y)
                        self._obstacle_count += 1
                        continue

                # 障害物的に動作可能な場合はカウントをリセット
                self._obstacle_count = 0

                # 追跡
                if self._chase_count >= 50:
                    self._last_person_x = self.best_person.pos.x
                    self._last_person_y = self.best_person.pos.y
                    self.sendHumanTrackOrder(self._last_person_x, self._last_person_y)
                    self._chase_count = 50
                self.chaseHuman(human_distance, human_direction,y)
                self._chase_count += 1

                #actionの処理
                goal = self.calc_target_gaol(self.best_person)
                feedback = FollowHumanFeedback(human_position = goal)
                self._action_server.publish_feedback(feedback)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr('tf error')
        result = FollowHumanResult(True)
        self._action_server.set_succeeded(result)
        self._do_action = False

    def avoidByPlannning(self):
        """
        パスプランを実行して障害物をよけるメソッド
        """
        self.twist_pub.publish(self.zero_twist)
        goal = self.calc_target_gaol(self.best_person)
        r = rospy.Rate(10)
        self._goal_pub.publish(goal)
        rospy.logwarn("send path goal")
#        while not rospy.is_shutdown():
#            r.sleep()
#            self._obstacle_count -= self._num_decrease_obstacle_count
#            rospy.logwarn(self._obstacle_count)
#            if self._obstacle_count >= 0:
#                break
#        rospy.logwarn(self._obstacle_count)
        rospy.sleep(self._avoid_time)
        self._obstacle_count=0 + self._num_decrease_obstacle_count
        #ここでゴールをキャンセルしたい
        self._goal_cancel_pub.publish(GoalID())
        rospy.logwarn("stop avoid by path plan")
        self.sendHumanTrackOrder()
        rospy.sleep(1)

    def moveToOpenSpace(self):
        twist = Twist()
        if self._front_state[1] is 1:#目の前はふさがっている
            twist.linear.x = -0.1#後ろに下がる
        elif self._front_state[0] is 1:#左にものがある
            #右後ろに行く
            twist.linear.y = -0.1
            twist.linear.x = -0.1
        elif self._front_state[2] is 1:#右にものがある
            #左後ろに行く
            twist.linear.y = 0.1
            twist.linear.x = -0.1
        else:
            #障害物はなかったので前に進む
            twist.linear.x = 0.1
        self.twist_pub.publish(twist)

    def setFollowPosition(self, x, y):
        """
        パーティクルを撒く位置をセットするメソッド
        :param int x: パーティクルを撒く位置(x)
        :param int y: パーティクルを撒く位置(y)
        """
        self.particle_x = x
        self.particle_y = y

    def switch2PlannningAvoid(self):
        """
        障害物を避ける挙動をavoidByPlannningに変えるメソッド
        """
        self.enable_avoid_obstacle = True
        self.avoid_function = self.avoidByPlannning

    def switch2OpenSpaceAvoid(self):
        """
        障害物を避ける挙動をmoveToOpenSpaceに変えるメソッド
        """
        self.enable_avoid_obstacle = True
        self.avoid_function = self.moveToOpenSpace

    def disableAvoid(self):
        """
        障害物を避けないfollowの挙動に変えるメソッド
        """
        self.enable_avoid_obstacle = False

    def setAvoidParam(self, threshold=3, num_decrease=3):
        """
        障害物があった場合の挙動に関するパラメータを変更するメソッド
        :param int threshold: 障害物回避を実施するまでのカウントの閾値
        :param int num_decrease:
        """
        self._obstacle_count_threshold = threshold
        self._num_decrease_obstacle_count= num_decrease

if __name__ == '__main__':
    follow_human = FollowHuman()
    rospy.loginfo("follow_human is ready")
    rospy.spin()
