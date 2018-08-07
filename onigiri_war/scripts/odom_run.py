#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
"""ROS関連"""
import rospy
import tf
"""ROS関連topicの型読み込み"""
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose


class OdomRun():
	def __init__(self):
		"""constructor"""
		"""移動パターン管理用"""
		self.status = 1
		"""/odomの情報を受信した時に更新するリアルタイムのロボットの座標"""		
		self.now_pose = Pose()
		"""移動パターン開始時のロボットの座標保存用"""
		self.start_pose = Pose()
		self.start_degree = math.pi

	def start(self):
		"""subscribe（受信）するトピックの登録"""
		self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=1)
		self.opt_right_sub = rospy.Subscriber('opt_right', LaserScan, self.optRightCallback)
		self.usonic_right_sub = rospy.Subscriber('us_left', LaserScan, self.usonicRightCallback)
		"""publish（送信）するトピックの登録"""
		self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    	def optRightCallback(self, data):
        	self.opt_r = data.ranges[0]
    	def usonicRightCallback(self, data):
        	self.us_r = data.ranges[0]		

	def odom_callback(self, result):
		"""/odomトピック(ロボットが認識している座標)の情報から移動パターンに沿った移動命令を送信"""
		self.now_pose = result.pose.pose
		"""リアルタイムのロボットの座標から移動パターン開始地点までの距離(進んだ距離)と角度を計算"""
		distance = math.sqrt((self.now_pose.position.x-self.start_pose.position.x)**2 + (self.now_pose.position.y-self.start_pose.position.y)**2)
		"""下記は四元数からラジアンに変換"""
		now_degree =  tf.transformations.euler_from_quaternion((self.now_pose.orientation.x, self.now_pose.orientation.y, self.now_pose.orientation.z, self.now_pose.orientation.w))
		diff_degree = now_degree[2] - self.start_degree
		"""±180°範囲外の角度は正規化"""
		if diff_degree > math.pi :
			diff_degree = diff_degree - (2*math.pi)
		elif diff_degree < -math.pi :
			diff_degree = diff_degree + (2*math.pi)

		"""速度送信用トピックの初期化"""
		vel_msg = Twist()
		vel_msg.linear.x = 0
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0

		"""1.5m直進"""
		if self.status == 1:
			if distance < 1.4:
				"""速度0.2m/s"""
				vel_msg.linear.x = 0.3
			elif distance >= 1.4:
				vel_msg.linear.x = 0
				self.start_pose = self.now_pose
				self.start_degree = now_degree[2]
				self.status = 2
		elif self.status == 2:
			if distance < 0.5:
				"""速度0.2m/s"""
				vel_msg.linear.x = -0.3
			elif distance >= 0.5:
				vel_msg.linear.x = 0
				self.start_pose = self.now_pose
				self.start_degree = now_degree[2]
				self.status = 3
		elif self.status == 3:
			"""90度回転"""
			"""目標角度90° == (1/2*PI)ラジアン"""
			if abs(diff_degree) > (math.pi/2)*0.9:
				vel_msg.angular.z = 0
				self.start_pose = self.now_pose
				self.status = 4
			else:
				vel_msg.angular.z = 0.5
		elif self.status == 4:
			"""0.5m直進"""
			if distance < 0.5:
				vel_msg.linear.x = 0.2
			elif distance >= 0.5:
				vel_msg.linear.x = 0
				self.start_pose = self.now_pose
				self.start_degree = now_degree[2]
				self.status = 5	
		elif self.status == 5:
			"""90度回転"""
			"""目標角度90° == (1/2*PI)ラジアン"""
			if abs(diff_degree) > (math.pi)*0.97:
				vel_msg.angular.z = 0
				self.start_pose = self.now_pose
				self.status = 6
			else:
				vel_msg.angular.z = -0.5	
		elif self.status == 6:
			"""1m直進"""
			if distance < 1.0:
				vel_msg.linear.x = 0.3
			elif distance >= 1.0:
				vel_msg.linear.x = 0
				self.start_pose = self.now_pose
				self.start_degree = now_degree[2]
				self.status = 7
		elif self.status == 7:
			"""90度回転"""
			"""目標角度90° == (1/2*PI)ラジアン"""
			if abs(diff_degree) > (math.pi/2):
				vel_msg.angular.z = 0
				self.start_pose = self.now_pose
				self.status = 8
			else:
				vel_msg.angular.z = 0.5
		elif self.status == 8:
			dist = 0.10
			print self.opt_r
			if self.us_r < 0.20:
				vel_msg.linear.x = 0
				vel_msg.angular.z = 1.0
			elif self.opt_r < dist:
				vel_msg.linear.x = 0.1
				vel_msg.angular.z = 0.5
			elif self.opt_r >= dist:
				vel_msg.linear.x = 0.1
				vel_msg.angular.z = -0.5
			
		"""速度指令のトピックを送信"""
		self.pub.publish(vel_msg) 			


if __name__ == "__main__":

	# initialize as ROS node.
	rospy.init_node("odom_run", anonymous=True)

	# detect result subscribe.
	idc = OdomRun()
	idc.start()
	rospy.spin()
